// The wheel encoders (since 1.7.0): odometry, speed control per wheel (feed-forward + PI) and the calibration that
// finds its parameters. See wheels.c.
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define WHEEL_DIAMETER_MM 71.0f
#define WHEEL_BASE_MM 262.5f
#define WHEEL_COUNTS_PER_REV (48 * 30)       // 48 counts per motor turn (both edges of A and B), 30:1 gearbox
#define WHEEL_MM_PER_COUNT ((float)M_PI * WHEEL_DIAMETER_MM / WHEEL_COUNTS_PER_REV)

// Per wheel (0 left, 1 right) and per direction (0 forward, 1 backward). Duty is the PWM fraction 0..1.
typedef struct {
    uint8_t version, valid;
    int8_t enc_sign[2];          // +1/-1: makes forward count up
    float start[2][2];           // duty at which a wheel at rest starts turning
    float ks[2][2];              // duty that just keeps it turning (the dead zone: the feed-forward's offset)
    float kv[2][2];              // duty per mm/s above that
    float tau[2];                // time constant of the speed (s)
    float ka[2];                 // duty per mm/s^2 (feed-forward for accelerating)
    float kp;                    // duty per mm/s of speed error
    float ki;                    // duty per mm of position error (the integral of the speed error)
    float vmax;                  // mm/s at full duty, the slowest wheel and direction
    uint32_t when;               // Unix time of the calibration (0: defaults)
    float check_err[2];          // after the calibration: RMS speed error (mm/s) at half speed, per direction
} wheel_params_t;

void wheels_init(int left_a, int left_b, int right_a, int right_b);
void wheels_sample(int64_t now_us);                      // every control tick, before the rest
float wheels_speed(int side);                            // mm/s, forward positive (over the last 50 ms)
float wheels_position(int side);                         // mm travelled, forward positive (since the start)
void wheels_odometry(float *x_mm, float *y_mm, float *heading_deg, float *dist_mm);
const wheel_params_t *wheels_params(void);
void wheels_set_gains(float kp, float ki);               // NAN: keep
float wheels_top_speed(void);                            // mm/s for a full command (90 % of vmax: room to correct)

// Speed control: v and a are the wanted speed (mm/s) and acceleration per wheel; duty out. -1, or the wheel whose
// encoder doesn't follow (stalled, or no pulses): then stop.
void wheels_ctl_reset(void);
int wheels_control(const float v[2], const float a[2], float dt, float duty[2]);
void wheels_ctl_terms(int side, float t[4]);              // the last duty (-1..1): feed-forward, P (speed), I (position), total

// Calibration (about 40 s): the robot spins in place, both ways.
bool wheels_cal_start(void);
void wheels_cal_abort(const char *why);
bool wheels_cal_running(void);
void wheels_cal_step(float dt, float duty[2]);           // every control tick while running
int wheels_json(char *out, size_t len);                  // GET /api/wheels
