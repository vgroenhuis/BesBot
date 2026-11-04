// The wheel encoders: quadrature, counted by the PCNT (all four edges: 48 counts per motor turn, 1440 per wheel turn,
// 0.155 mm of travel each). From them: each wheel's speed (over the last 50 ms), odometry, and speed control.
//
// Speed control, per wheel, every 10 ms: duty = feed-forward + kp * speed error + ki * position error, where the
// feed-forward is ks (the dead zone: the duty that just keeps the wheel turning) + kv * speed + ka * acceleration, and
// the position error is how far the wheel lags the integral of the wanted speed (the integral of the speed error, but
// counted exactly in encoder steps; limited, so it can't wind up). At a standstill the wheel is let go (no humming).
//
// The calibration finds all of it, with the robot spinning in place (on the floor with half a metre around it, or
// lifted), both ways round: the encoder directions; where each wheel starts turning (the duty slowly up) and where it
// stops again (slowly down); then steps from rest to 25 .. 100 % duty, each giving a steady speed and a time constant.
// A straight line through (speed, duty) per wheel and direction gives ks and kv; the time constant gives ka and the
// PI gains (kp, ki: a closed-loop time constant of about 0.12 s). Last, a check: half speed with the new values, both
// ways, and the speed error. The result is kept in flash (NVS besbot/wheels).
#include "wheels.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"

static const char *TAG = "wheels";
static const char *SIDE[2] = {"left", "right"};

#define PARAMS_VERSION 1
#define VEL_TICKS 5                  // the speed: over this many ticks (50 ms: 3 mm/s per count)

static pcnt_unit_handle_t unit[2];
static int32_t raw[2];               // counts as the encoders give them
static float vraw[2];                // mm/s from raw (sign as counted)
static int32_t ring[VEL_TICKS + 1][2];
static int64_t ring_t[VEL_TICKS + 1];
static int ring_n, ring_head;
static struct { float x, y, th, dist; int32_t last[2]; bool started; } odo;

static wheel_params_t P;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Before any calibration: a guess for a 12 V 30:1 gear motor on a fresh battery.
static void defaults(wheel_params_t *p) {
    memset(p, 0, sizeof *p);
    p->version = PARAMS_VERSION;
    p->enc_sign[0] = p->enc_sign[1] = 1;
    for (int s = 0; s < 2; s++) for (int d = 0; d < 2; d++) {
        p->start[s][d] = 0.2f;
        p->ks[s][d] = 0.12f;
        p->kv[s][d] = 0.0008f;
    }
    p->tau[0] = p->tau[1] = 0.1f;
    p->ka[0] = p->ka[1] = 0.00008f;
    p->kp = 0.0006f;
    p->ki = 0.006f;
    p->vmax = 1100;
    p->check_err[0] = p->check_err[1] = -1;
}

static void params_save(void) {
    nvs_handle_t h;
    if (nvs_open("besbot", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, "wheels", &P, sizeof P);
    nvs_commit(h);
    nvs_close(h);
}

static void params_load(void) {
    defaults(&P);
    nvs_handle_t h;
    if (nvs_open("besbot", NVS_READONLY, &h) != ESP_OK) return;
    wheel_params_t p;
    size_t len = sizeof p;
    if (nvs_get_blob(h, "wheels", &p, &len) == ESP_OK && len == sizeof p && p.version == PARAMS_VERSION) P = p;
    nvs_close(h);
}

void wheels_init(int left_a, int left_b, int right_a, int right_b) {
    params_load();
    const int pins[2][2] = {{left_a, left_b}, {right_a, right_b}};
    for (int s = 0; s < 2; s++) {
        pcnt_unit_config_t uc = {.low_limit = -30000, .high_limit = 30000, .flags.accum_count = 1};
        ESP_ERROR_CHECK(pcnt_new_unit(&uc, &unit[s]));
        pcnt_glitch_filter_config_t f = {.max_glitch_ns = 1000};
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit[s], &f));
        // x4: A's edges count with B's level as the direction, and B's edges with A's (as on the LynXP).
        pcnt_chan_config_t ca = {.edge_gpio_num = pins[s][0], .level_gpio_num = pins[s][1]};
        pcnt_chan_config_t cb = {.edge_gpio_num = pins[s][1], .level_gpio_num = pins[s][0]};
        pcnt_channel_handle_t a, b;
        ESP_ERROR_CHECK(pcnt_new_channel(unit[s], &ca, &a));
        ESP_ERROR_CHECK(pcnt_new_channel(unit[s], &cb, &b));
        pcnt_channel_set_edge_action(a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
        pcnt_channel_set_level_action(a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
        pcnt_channel_set_edge_action(b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        pcnt_channel_set_level_action(b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
        // (accum_count folds the 16-bit counter into the total only at these watch points)
        pcnt_unit_add_watch_point(unit[s], -30000);
        pcnt_unit_add_watch_point(unit[s], 30000);
        for (int k = 0; k < 2; k++) gpio_pullup_en(pins[s][k]);      // (an unplugged encoder reads still, not noise)
        ESP_ERROR_CHECK(pcnt_unit_enable(unit[s]));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(unit[s]));
        ESP_ERROR_CHECK(pcnt_unit_start(unit[s]));
    }
    ESP_LOGI(TAG, "Encoders: left A/B GPIO%d/%d, right A/B GPIO%d/%d; %s", left_a, left_b, right_a, right_b,
             P.valid ? "calibrated" : "not calibrated yet");
}

void wheels_sample(int64_t now_us) {
    for (int s = 0; s < 2; s++) {
        int c = 0;
        pcnt_unit_get_count(unit[s], &c);
        raw[s] = c;
    }
    memcpy(ring[ring_head], raw, sizeof raw);
    ring_t[ring_head] = now_us;
    int old = (ring_head + 1) % (VEL_TICKS + 1);
    ring_head = old;
    if (ring_n < VEL_TICKS + 1) ring_n++;
    if (ring_n == VEL_TICKS + 1 && now_us > ring_t[old]) {
        float secs = (now_us - ring_t[old]) / 1e6f;
        for (int s = 0; s < 2; s++) vraw[s] = (raw[s] - ring[old][s]) * WHEEL_MM_PER_COUNT / secs;
    }
    // Odometry: counter-clockwise positive, x forward at the start.
    if (!odo.started) { memcpy(odo.last, raw, sizeof raw); odo.started = true; }
    float dl = (raw[0] - odo.last[0]) * P.enc_sign[0] * WHEEL_MM_PER_COUNT;
    float dr = (raw[1] - odo.last[1]) * P.enc_sign[1] * WHEEL_MM_PER_COUNT;
    memcpy(odo.last, raw, sizeof raw);
    float ds = (dl + dr) / 2, dth = (dr - dl) / WHEEL_BASE_MM;
    odo.x += ds * cosf(odo.th + dth / 2);
    odo.y += ds * sinf(odo.th + dth / 2);
    odo.th = remainderf(odo.th + dth, 2 * (float)M_PI);
    odo.dist += fabsf(ds);
}

float wheels_speed(int side) { return vraw[side] * P.enc_sign[side]; }

static float pos_mm(int side) { return raw[side] * P.enc_sign[side] * WHEEL_MM_PER_COUNT; }

float wheels_position(int side) { return pos_mm(side); }

void wheels_odometry(float *x, float *y, float *heading, float *dist) {
    *x = odo.x, *y = odo.y, *heading = odo.th * 180 / (float)M_PI, *dist = odo.dist;
}

const wheel_params_t *wheels_params(void) { return &P; }

void wheels_set_gains(float kp, float ki) {
    portENTER_CRITICAL(&mux);
    if (!isnan(kp) && kp >= 0 && kp < 0.05f) P.kp = kp;
    if (!isnan(ki) && ki >= 0 && ki < 0.5f) P.ki = ki;
    portEXIT_CRITICAL(&mux);
    params_save();
}

float wheels_top_speed(void) { return 0.9f * P.vmax; }

// ------------------------------------------------------------------ speed control

static struct { float pos_ref, stuck_s; } C[2];

void wheels_ctl_reset(void) {
    for (int s = 0; s < 2; s++) C[s].pos_ref = pos_mm(s), C[s].stuck_s = 0;
}

int wheels_control(const float v_ref[2], const float a_ref[2], float dt, float duty[2]) {
    int bad = -1;
    for (int s = 0; s < 2; s++) {
        float v = v_ref[s], vel = wheels_speed(s), u;
        if (v == 0 && a_ref[s] == 0) {
            // Stopping: brake on the speed alone; let go once (nearly) still.
            C[s].pos_ref = pos_mm(s);
            u = fabsf(vel) < 20 ? 0 : -P.kp * vel;
        } else {
            int d = v >= 0 ? 0 : 1;
            float ff = (v > 0 ? P.ks[s][0] : -P.ks[s][1]) + P.kv[s][d] * v + P.ka[s] * a_ref[s];
            C[s].pos_ref += v * dt;
            float e = C[s].pos_ref - pos_mm(s), lim = P.ki > 0 ? 0.3f / P.ki : 0;    // (the integral: at most 30 % duty)
            if (e > lim) C[s].pos_ref -= e - lim, e = lim;
            if (e < -lim) C[s].pos_ref += -lim - e, e = -lim;
            u = ff + P.kp * (v - vel) + P.ki * e;
        }
        u = u > 1 ? 1 : u < -1 ? -1 : u;
        duty[s] = u;
        // Plenty of duty but the wheel stands still, or turns the other way: stalled, or the encoder is wrong.
        float plenty = fmaxf(0.4f, fmaxf(P.start[s][0], P.start[s][1]) + 0.15f);
        if (fabsf(u) > plenty && (fabsf(vel) < 15 || vel * u < 0)) C[s].stuck_s += dt;
        else C[s].stuck_s = 0;
        if (C[s].stuck_s > 0.4f) bad = s;
    }
    if (bad >= 0) duty[0] = duty[1] = 0;
    return bad;
}

// ------------------------------------------------------------------ calibration

static const float LEVELS[] = {0.25f, 0.4f, 0.55f, 0.7f, 0.85f, 1.0f};
#define NLEV (int)(sizeof LEVELS / sizeof LEVELS[0])
#define STEP_TICKS 110                   // 1.1 s per step
#define CHECK_S 1.3f

enum { CAL_IDLE, CAL_RUNNING, CAL_DONE, CAL_FAILED };
enum { PH_REST, PH_BREAK, PH_HOLD, PH_DOWN, PH_STEP, PH_CHECK };

static struct {
    int state, phase, next, dir, level, n;
    bool phase_check;                    // the values are fitted, the final check runs
    float t, rest_s, u[2];
    int32_t p0[2];
    bool moved[2], stopped[2];
    int8_t sign[2][2];                   // [wheel][round]
    int32_t trace[2][STEP_TICKS];
    float start[2][2], stop[2][2];       // [wheel][forward, backward]
    float lv[2][2][NLEV][3];             // per wheel and direction: duty, steady speed, time constant
    int nlev[2][2];
    float chk_sq[2];
    int chk_n[2];
    wheel_params_t res, before;
    char msg[160];
    int64_t started_us;
} K;

// Round 0: left forward, right backward (spinning clockwise); round 1 the other way.
static int csign(int s, int round) { return (s == 0) == (round == 0) ? 1 : -1; }
static int wdir(int s, int round) { return csign(s, round) > 0 ? 0 : 1; }

static void say(const char *fmt, const char *a) {
    portENTER_CRITICAL(&mux);
    snprintf(K.msg, sizeof K.msg, fmt, a);
    portEXIT_CRITICAL(&mux);
}

static void finish(int state, const char *fmt, const char *a) {
    if (fmt) say(fmt, a);                          // (NULL: what was said last is the reason)
    if (state == CAL_FAILED && K.phase_check) {
        // Stopped during the final check: the measurements are complete, so the new values stay (unchecked).
        P = K.res;
        P.check_err[0] = P.check_err[1] = -1;
        params_save();
        char m[sizeof K.msg + 64];
        snprintf(m, sizeof m, "%s The new values are saved (without the final check).", K.msg);
        say("%s", m);
    } else if (state == CAL_FAILED) {
        P = K.before;
    }
    K.state = state;
    wheels_ctl_reset();
    ESP_LOGI(TAG, "Calibration %s: %s", state == CAL_DONE ? "done" : "ended", K.msg);
}

static void begin(int phase) {
    K.phase = phase;
    K.t = 0;
    K.n = 0;
    memcpy(K.p0, raw, sizeof raw);
    const char *way = K.dir == 0 ? "clockwise" : "anticlockwise";
    switch (phase) {
    case PH_BREAK:
        K.u[0] = K.u[1] = 0;
        K.moved[0] = K.moved[1] = false;
        say("Spinning %s: where do the wheels start turning?", way);
        break;
    case PH_HOLD:
        for (int s = 0; s < 2; s++) K.u[s] = fminf(1, K.start[s][wdir(s, K.dir)] + 0.08f);
        break;
    case PH_DOWN:
        K.stopped[0] = K.stopped[1] = false;
        say("Spinning %s: where do they stop again?", way);
        break;
    case PH_STEP: {
        float floor = fmaxf(K.stop[0][wdir(0, K.dir)], K.stop[1][wdir(1, K.dir)]) + 0.06f;
        while (K.level < NLEV - 1 && LEVELS[K.level] < floor) K.level++;
        K.u[0] = K.u[1] = LEVELS[K.level];
        char m[64];
        snprintf(m, sizeof m, "%s at %d %% power", way, (int)lroundf(LEVELS[K.level] * 100));
        say("Spinning %s: speed and response", m);
        break;
    }
    case PH_CHECK:
        wheels_ctl_reset();
        K.chk_sq[K.dir] = 0, K.chk_n[K.dir] = 0;
        say("Checking: half speed %s with the new values", way);
        break;
    }
}

static void rest(int next, float secs) {
    K.phase = PH_REST;
    K.next = next;
    K.rest_s = secs;
    K.t = 0;
    K.u[0] = K.u[1] = 0;
}

bool wheels_cal_start(void) {
    if (K.state == CAL_RUNNING) return false;
    memset(&K, 0, sizeof K);
    K.before = P;
    K.state = CAL_RUNNING;
    K.started_us = esp_timer_get_time();
    say("Starting%s", "");
    ESP_LOGI(TAG, "Calibration started");
    rest(PH_BREAK, 0.5f);
    return true;
}

void wheels_cal_abort(const char *why) {
    if (K.state == CAL_RUNNING) finish(CAL_FAILED, "Stopped: %s.", why);
}

bool wheels_cal_running(void) { return K.state == CAL_RUNNING; }

// One step's steady speed (the last 0.4 s) and time constant (63 % of it, from a centred difference: no lag).
static void analyse_step(void) {
    for (int s = 0; s < 2; s++) {
        int32_t *tr = K.trace[s];
        float vss = fabsf((float)(tr[STEP_TICKS - 1] - tr[STEP_TICKS - 41])) * WHEEL_MM_PER_COUNT / 0.4f;
        if (vss < 30) continue;                              // (barely moving: no use)
        float tau = -1, prev = 0;
        for (int i = 2; i < STEP_TICKS - 2; i++) {
            float v = fabsf((float)(tr[i + 2] - tr[i - 2])) * WHEEL_MM_PER_COUNT / 0.04f;
            if (v >= 0.632f * vss) {
                float f = v > prev ? (0.632f * vss - prev) / (v - prev) : 1;
                tau = (i - 1 + f) * 0.01f;
                break;
            }
            prev = v;
        }
        int d = wdir(s, K.dir), k = K.nlev[s][d]++;
        K.lv[s][d][k][0] = LEVELS[K.level];
        K.lv[s][d][k][1] = vss;
        K.lv[s][d][k][2] = tau;
    }
}

// From all steps: the feed-forward per wheel and direction, and the gains. False (with the reason said) if it can't.
static bool fit(void) {
    wheel_params_t r = K.before;
    r.version = PARAMS_VERSION;
    float kv_sum = 0, tau_all = 0;
    int tau_all_n = 0;
    r.vmax = 1e9f;
    for (int s = 0; s < 2; s++) {
        if (K.sign[s][0] != K.sign[s][1]) {
            say("The %s encoder counts the same way both ways round: check its A and B wires.", SIDE[s]);
            return false;
        }
        r.enc_sign[s] = K.sign[s][0];
        float tau_sum = 0;
        int tau_n = 0;
        for (int d = 0; d < 2; d++) {
            int n = K.nlev[s][d];
            if (n < 2) {
                say("The %s wheel hardly turns at full power: is the battery charged?", SIDE[s]);
                return false;
            }
            // Least squares: duty = ks + kv * speed.
            float sv = 0, su = 0, svv = 0, svu = 0;
            for (int k = 0; k < n; k++) {
                float u = K.lv[s][d][k][0], v = K.lv[s][d][k][1];
                sv += v, su += u, svv += v * v, svu += v * u;
                if (K.lv[s][d][k][2] > 0) tau_sum += K.lv[s][d][k][2], tau_n++;
            }
            float den = n * svv - sv * sv, kv = den > 0 ? (n * svu - sv * su) / den : 0, ks = (su - kv * sv) / n;
            if (kv <= 0) { say("The %s wheel's speed doesn't grow with the power: check the motor and its wires.", SIDE[s]); return false; }
            r.start[s][d] = K.start[s][d];
            r.ks[s][d] = fminf(fmaxf(ks, 0), K.start[s][d]);
            r.kv[s][d] = kv;
            r.vmax = fminf(r.vmax, (1 - r.ks[s][d]) / kv);
            kv_sum += kv;
        }
        r.tau[s] = tau_n ? tau_sum / tau_n : 0.1f;
        tau_all += r.tau[s] * tau_n, tau_all_n += tau_n;
    }
    float kv_m = kv_sum / 4, tau_m = tau_all_n ? tau_all / tau_all_n : 0.1f;
    for (int s = 0; s < 2; s++) r.ka[s] = (r.kv[s][0] + r.kv[s][1]) / 2 * r.tau[s];
    // PI for a first-order wheel (speed / duty = 1 / (kv (tau s + 1))): the zero on the pole, closed loop tau_c.
    float tau_c = fmaxf(0.12f, 0.5f * tau_m);
    r.kp = kv_m * tau_m / tau_c;
    r.ki = kv_m / tau_c;
    r.valid = 1;
    r.when = (uint32_t)time(NULL);
    r.check_err[0] = r.check_err[1] = -1;
    K.res = r;
    return true;
}

void wheels_cal_step(float dt, float duty[2]) {
    duty[0] = duty[1] = 0;
    if (K.state != CAL_RUNNING) return;
    K.t += dt;
    if (esp_timer_get_time() - K.started_us > 120 * 1000000LL) { finish(CAL_FAILED, "Took too long%s.", ""); return; }
    switch (K.phase) {
    case PH_REST:            // still for a moment (both wheels slower than 5 mm/s; 3 s at most)
        if (K.t >= K.rest_s && ((fabsf(vraw[0]) < 5 && fabsf(vraw[1]) < 5) || K.t > 3)) begin(K.next);
        return;
    case PH_BREAK:           // the duty up by 15 % a second until each wheel has turned 8 counts (1.2 mm)
        for (int s = 0; s < 2; s++) {
            if (K.moved[s]) continue;
            int32_t d = raw[s] - K.p0[s];
            if (d >= 8 || d <= -8) {
                K.moved[s] = true;
                K.start[s][wdir(s, K.dir)] = K.u[s];
                K.sign[s][K.dir] = (d > 0 ? 1 : -1) * csign(s, K.dir);
            } else if (K.u[s] >= 0.9f) {
                finish(CAL_FAILED, "The %s wheel doesn't turn, or its encoder gives no pulses (check the wires).", SIDE[s]);
                return;
            } else {
                K.u[s] += 0.15f * dt;
            }
        }
        if (K.moved[0] && K.moved[1]) begin(PH_HOLD);
        break;
    case PH_HOLD:
        if (K.t >= 0.6f) begin(PH_DOWN);
        break;
    case PH_DOWN:            // the duty down by 8 % a second until each wheel stops (slower than 8 mm/s)
        for (int s = 0; s < 2; s++) {
            if (K.stopped[s]) continue;
            K.u[s] -= 0.08f * dt;
            if ((K.t > 0.2f && fabsf(vraw[s]) < 8) || K.u[s] <= 0) {
                K.stopped[s] = true;
                K.stop[s][wdir(s, K.dir)] = fmaxf(K.u[s], 0);
                K.u[s] = 0;
            }
        }
        if (K.stopped[0] && K.stopped[1]) { K.level = 0; rest(PH_STEP, 0.4f); }
        break;
    case PH_STEP:
        for (int s = 0; s < 2; s++) K.trace[s][K.n] = raw[s];
        if (++K.n == STEP_TICKS) {
            analyse_step();
            if (++K.level < NLEV) rest(PH_STEP, 0.4f);
            else if (K.dir == 0) { K.dir = 1; rest(PH_BREAK, 0.6f); }
            else if (!fit()) { finish(CAL_FAILED, NULL, NULL); return; }
            else { P = K.res; K.phase_check = true; params_save(); K.dir = 0; rest(PH_CHECK, 0.6f); }
            return;
        }
        break;
    case PH_CHECK: {         // half speed (reached in 0.25 s) with the new values; the error after 0.5 s
        float v[2], a[2], vh = 0.5f * P.vmax;
        bool ramping = K.t < 0.25f;
        for (int s = 0; s < 2; s++) {
            float sg = csign(s, K.dir);
            v[s] = sg * vh * fminf(1, K.t / 0.25f);
            a[s] = ramping ? sg * vh / 0.25f : 0;
        }
        int bad = wheels_control(v, a, dt, duty);
        if (bad >= 0) {
            finish(CAL_FAILED, "The %s wheel didn't follow the speed control: calibrate again on a flat floor.", SIDE[bad]);
            duty[0] = duty[1] = 0;
            return;
        }
        if (K.t > 0.5f) for (int s = 0; s < 2; s++) {
            float e = v[s] - wheels_speed(s);
            K.chk_sq[K.dir] += e * e, K.chk_n[K.dir]++;
        }
        if (K.t >= CHECK_S) {
            P.check_err[K.dir] = K.chk_n[K.dir] ? sqrtf(K.chk_sq[K.dir] / K.chk_n[K.dir]) : -1;
            duty[0] = duty[1] = 0;
            if (K.dir == 0) { K.dir = 1; rest(PH_CHECK, 0.6f); }
            else {
                K.res = P;
                params_save();
                char m[48];
                snprintf(m, sizeof m, "%.0f and %.0f mm/s", P.check_err[0], P.check_err[1]);
                finish(CAL_DONE, "Done. Speed error at half speed: %s (RMS).", m);
            }
        }
        return;              // (the duty came from the speed control)
    }
    }
    for (int s = 0; s < 2; s++) duty[s] = csign(s, K.dir) * K.u[s];
}

// {"state":"idle|running|done|failed","msg","progress":%,"enc":[l,r] counts,"v":[l,r] mm/s,"odo":[x,y,deg,dist],
//  "params":{"valid","when","enc_sign","start","ks","kv","tau","ka","kp","ki","vmax","check"},
//  "steps":[[[[duty,speed,tau],..] left forward, left backward], [right ...]] (the last calibration's measurements)}
int wheels_json(char *out, size_t len) {
    static const char *STATES[] = {"idle", "running", "done", "failed"};
    wheel_params_t p;
    char msg[sizeof K.msg];
    portENTER_CRITICAL(&mux);
    p = P;
    memcpy(msg, K.msg, sizeof msg);
    portEXIT_CRITICAL(&mux);
    int progress = K.state == CAL_RUNNING ? (int)fminf(99, (esp_timer_get_time() - K.started_us) / 1e6f * 100 / 35) :
                   K.state == CAL_DONE ? 100 : 0;
    int n = snprintf(out, len, "{\"state\":\"%s\",\"msg\":\"%s\",\"progress\":%d,\"enc\":[%ld,%ld],\"v\":[%.0f,%.0f],"
                     "\"odo\":[%.0f,%.0f,%.1f,%.0f],\"params\":{\"valid\":%d,\"when\":%lu,\"enc_sign\":[%d,%d],",
                     STATES[K.state], msg, progress, (long)raw[0], (long)raw[1], wheels_speed(0), wheels_speed(1),
                     odo.x, odo.y, odo.th * 180 / (float)M_PI, odo.dist, p.valid, (unsigned long)p.when,
                     p.enc_sign[0], p.enc_sign[1]);
    const char *names[3] = {"start", "ks", "kv"};
    float (*arr[3])[2] = {p.start, p.ks, p.kv};
    for (int i = 0; i < 3; i++)
        n += snprintf(out + n, len - n, "\"%s\":[[%.4g,%.4g],[%.4g,%.4g]],", names[i], arr[i][0][0], arr[i][0][1],
                      arr[i][1][0], arr[i][1][1]);
    n += snprintf(out + n, len - n, "\"tau\":[%.3f,%.3f],\"ka\":[%.4g,%.4g],\"kp\":%.4g,\"ki\":%.4g,\"vmax\":%.0f,"
                  "\"top\":%.0f,\"check\":[%.0f,%.0f]},\"steps\":[",
                  p.tau[0], p.tau[1], p.ka[0], p.ka[1], p.kp, p.ki, p.vmax, 0.9f * p.vmax, p.check_err[0], p.check_err[1]);
    for (int s = 0; s < 2; s++) {
        n += snprintf(out + n, len - n, "%s[", s ? "," : "");
        for (int d = 0; d < 2; d++) {
            n += snprintf(out + n, len - n, "%s[", d ? "," : "");
            for (int k = 0; k < K.nlev[s][d] && n < (int)len - 40; k++)
                n += snprintf(out + n, len - n, "%s[%.2f,%.0f,%.3f]", k ? "," : "", K.lv[s][d][k][0], K.lv[s][d][k][1],
                              K.lv[s][d][k][2]);
            n += snprintf(out + n, len - n, "]");
        }
        n += snprintf(out + n, len - n, "]");
    }
    n += snprintf(out + n, len - n, "]}");
    return n;
}
