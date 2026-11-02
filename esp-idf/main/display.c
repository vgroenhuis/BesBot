// The OLED on I2C: see display.h. Laid out like the other lab demos' displays (LynXP, SpaceMouse, HandTool):
//
//   BesBot 1.6.1                        [Wi-Fi]     <- header: name, firmware version
//   ------------------------------------------
//   Mojca_Vincent_Reel...         ch36 -56dBm     <- network (scrolls if too long), channel and signal
//   192.168.0.37                                  <- address, the largest font that fits
//   ------------------------------------------
//   Free to drive  |  Wi-Fi     ====|           <- control: free, or how it's driven and the motors (left
//                                 |==             above right)
//
// and while a control code is shown: the code as big as it fits, with a bar for the time left. u8g2 draws off-screen
// (its dummy callbacks never touch a bus); five times a second the 128-byte pages that changed go out over I2C.
#include "display.h"
#include "closer_access.h"
#include "closer_ble.h"
#include "closer_demo.h"
#include "u8g2.h"

#include <stdio.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "display";

#define W 128
#define H 64
#define PAGES (H / 8)
#ifdef CONFIG_BESBOT_OLED_SH1106
#define COL_OFFSET 2             // the SH1106 has 132 columns, the panel shows the middle 128
#else
#define COL_OFFSET 0
#endif

static i2c_master_dev_handle_t dev;
static u8g2_t u8g2;
static uint8_t sent[PAGES][W];
static bool sent_valid;
static char found[24];
static display_drive_fn drive_fn;

static int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

// ------------------------------------------------------------------ the controller

static esp_err_t commands(const uint8_t *c, size_t n) {
    uint8_t buf[32] = {0x00};    // control byte: commands follow
    memcpy(buf + 1, c, n);
    return i2c_master_transmit(dev, buf, n + 1, 100);
}

// Page addressing (the SSD1306's default, and all the SH1106 has). u8g2's buffer has the panel's layout: 8 pages of
// 128 bytes, each byte a column of 8 pixels.
static esp_err_t flush(void) {
    const uint8_t *fb = u8g2_GetBufferPtr(&u8g2);
    for (int p = 0; p < PAGES; p++) {
        const uint8_t *page = fb + p * W;
        if (sent_valid && memcmp(page, sent[p], W) == 0) continue;
        uint8_t c[3] = {0xB0 | p, COL_OFFSET & 15, 0x10 | COL_OFFSET >> 4};
        uint8_t d[W + 1] = {0x40};                 // control byte: data follows
        memcpy(d + 1, page, W);
        esp_err_t e = commands(c, sizeof c);
        if (e == ESP_OK) e = i2c_master_transmit(dev, d, sizeof d, 100);
        if (e != ESP_OK) {
            sent_valid = false;
            return e;
        }
        memcpy(sent[p], page, W);
    }
    sent_valid = true;
    return ESP_OK;
}

static esp_err_t init_panel(void) {
    static const uint8_t seq[] = {
        0xAE,                    // off
        0xD5, 0x80,              // clock
        0xA8, 0x3F,              // 64 rows
        0xD3, 0x00, 0x40,        // no offset, start line 0
        0x8D, 0x14,              // SSD1306 charge pump on (the SH1106 ignores it; its DC-DC is on from reset)
#ifdef CONFIG_BESBOT_OLED_FLIP
        0xA0, 0xC0,              // upside down
#else
        0xA1, 0xC8,              // the usual way up
#endif
        0xDA, 0x12,              // COM pins
        0x81, 0x7F,              // contrast
        0xD9, 0xF1, 0xDB, 0x40,  // pre-charge, VCOMH
        0xA4, 0xA6,              // show the RAM, not inverted
    };
    esp_err_t e = commands(seq, sizeof seq);
    u8g2_ClearBuffer(&u8g2);
    sent_valid = false;
    if (e == ESP_OK) e = flush();
    if (e == ESP_OK) e = commands((const uint8_t[]){0xAF}, 1);     // on
    return e;
}

// ------------------------------------------------------------------ drawing helpers (as in the LynXP firmware)

static void text(int x, int y, const uint8_t *font, const char *s) {
    u8g2_SetFont(&u8g2, font);
    u8g2_DrawStr(&u8g2, x, y, s);
}

static int width(const uint8_t *font, const char *s) {
    u8g2_SetFont(&u8g2, font);
    return u8g2_GetStrWidth(&u8g2, s);
}

static void text_right(int x_right, int y, const uint8_t *font, const char *s) { text(x_right - width(font, s), y, font, s); }
static void text_center(int y, const uint8_t *font, const char *s) { text((W - width(font, s)) / 2, y, font, s); }

// The largest of the fonts (largest first) in which the text fits the width.
static const uint8_t *pick_font(const uint8_t *const *fonts, size_t n, const char *s, int max_width) {
    for (size_t i = 0; i < n; i++)
        if (width(fonts[i], s) <= max_width) return fonts[i];
    return fonts[n - 1];
}

// Text in a band of the given width that scrolls back and forth when it doesn't fit.
typedef struct { char text[64]; int64_t start_ms; } marquee_t;

static void band(marquee_t *m, const uint8_t *font, const char *s, int baseline, int band_width) {
    if (strncmp(m->text, s, sizeof m->text - 1) != 0) {
        strlcpy(m->text, s, sizeof m->text);
        m->start_ms = now_ms();
    }
    int tw = width(font, s);
    if (tw <= band_width) {
        u8g2_DrawStr(&u8g2, 0, baseline, s);
        return;
    }
    const int HOLD_MS = 1500, PX_PER_S = 30;
    int overflow = tw - band_width, scroll_ms = overflow * 1000 / PX_PER_S;
    int t = (int)((now_ms() - m->start_ms) % (HOLD_MS + scroll_ms + HOLD_MS));
    int offset = t < HOLD_MS ? 0 : t < HOLD_MS + scroll_ms ? (t - HOLD_MS) * PX_PER_S / 1000 : overflow;
    u8g2_SetClipWindow(&u8g2, 0, baseline - u8g2_GetAscent(&u8g2), band_width, baseline - u8g2_GetDescent(&u8g2) + 1);
    u8g2_DrawStr(&u8g2, -offset, baseline, s);
    u8g2_SetMaxClipWindow(&u8g2);
}

static void wifi_icon(int x, int y, int rssi) {
    int bars = rssi == 0 ? 0 : rssi > -55 ? 4 : rssi > -65 ? 3 : rssi > -75 ? 2 : 1;
    for (int i = 0; i < 4; i++) {
        int h = 3 + 2 * i;
        if (i < bars) u8g2_DrawBox(&u8g2, x + 3 * i, y - h, 2, h);
        else u8g2_DrawPixel(&u8g2, x + 3 * i, y - 1);
    }
    if (!bars) {
        u8g2_DrawLine(&u8g2, x + 6, y - 9, x + 11, y - 4);
        u8g2_DrawLine(&u8g2, x + 11, y - 9, x + 6, y - 4);
    }
}

// Word-wrapped text; returns the number of lines used.
static int wrapped(int x, int y, int line_h, int max_width, int max_lines, const uint8_t *font, const char *s) {
    char line[48];
    int lines = 0;
    u8g2_SetFont(&u8g2, font);
    while (*s && lines < max_lines) {
        size_t best = 0, n = 0;
        while (s[n]) {
            size_t m = n;
            while (s[m] && s[m] != ' ') m++;
            size_t k = m < sizeof line - 1 ? m : sizeof line - 1;
            memcpy(line, s, k);
            line[k] = 0;
            if (best && u8g2_GetStrWidth(&u8g2, line) > max_width) break;
            best = k;
            if (!s[m]) break;
            n = m + 1;
        }
        memcpy(line, s, best);
        line[best] = 0;
        u8g2_DrawStr(&u8g2, x, y + lines * line_h, line);
        lines++;
        s += best;
        while (*s == ' ') s++;
    }
    return lines;
}

// The motors' outputs -1..1 as two bars (left above right) from a middle line, in x0..x1, rows y..y+10.
static void motor_bars(int x0, int x1, int y, float l, float r) {
    int mid = (x0 + x1) / 2, half = (x1 - x0) / 2 - 1;
    for (int y2 = y; y2 <= y + 10; y2 += 2) u8g2_DrawPixel(&u8g2, mid, y2);
    for (int x = x0; x <= x1; x += 8) u8g2_DrawPixel(&u8g2, x, y + 5);          // (a dotted axis between them)
    for (int i = 0; i < 2; i++) {
        float v = i ? r : l;
        int len = (int)(v * half + (v < 0 ? -0.5f : 0.5f)), top = y + i * 7;
        if (len > 0) u8g2_DrawBox(&u8g2, mid + 1, top, len, 4);
        else if (len < 0) u8g2_DrawBox(&u8g2, mid + len, top, -len, 4);
    }
}

// ------------------------------------------------------------------ the screens

static const uint8_t *const IP_FONTS[] = {u8g2_font_logisoso24_tn, u8g2_font_logisoso22_tn, u8g2_font_logisoso20_tn,
                                          u8g2_font_logisoso18_tn, u8g2_font_logisoso16_tn, u8g2_font_fub14_tn,
                                          u8g2_font_fub11_tn};
static const uint8_t *const CODE_FONTS[] = {u8g2_font_logisoso42_tn, u8g2_font_logisoso38_tn, u8g2_font_logisoso24_tn,
                                            u8g2_font_logisoso16_tn};
static marquee_t network_band;

static void header(const char *title, const closer_demo_status_t *st, bool version) {
    text(0, 9, u8g2_font_helvB08_tr, title);
    if (version) text(width(u8g2_font_helvB08_tr, title) + 4, 9, u8g2_font_helvR08_tr, closer_demo_version());
    wifi_icon(115, 9, st->connected ? (st->rssi ? st->rssi : -60) : 0);
    u8g2_DrawHLine(&u8g2, 0, 11, W);
}

// The bottom line: who controls the robot and, while it's driven, its motors.
static void control_line(const closer_demo_status_t *st) {
    u8g2_DrawHLine(&u8g2, 0, 50, W);
    float l = 0, r = 0;
    bool bluetooth = false;
    if (drive_fn) drive_fn(&l, &r, &bluetooth);
    if (closer_access_holder() == CLOSER_ACCESS_NONE && l == 0 && r == 0) {
        // (free to drive only once it can be reached: through the site, or over Bluetooth while the Wi-Fi isn't up)
        text_center(63, u8g2_font_helvB10_tr, st->connected ? "Free to drive" : closer_ble_running() ? "Bluetooth only" :
                                                                                          "Not reachable yet");
        return;
    }
    const char *via = bluetooth ? "Bluetooth" : "Wi-Fi";
    text(0, 62, u8g2_font_helvB08_tr, via);
    motor_bars(width(u8g2_font_helvB08_tr, via) + 8, W - 1, 53, l, r);
}

static void connected_screen(const closer_demo_status_t *st) {
    char s[32];
    snprintf(s, sizeof s, "ch%d %d dBm", st->channel, st->rssi);
    u8g2_SetFont(&u8g2, u8g2_font_helvR08_tr);
    int sw = st->rssi ? width(u8g2_font_helvR08_tr, s) : 0;
    if (sw) text_right(W, 22, u8g2_font_helvR08_tr, s);
    u8g2_SetFont(&u8g2, u8g2_font_helvR08_tr);
    band(&network_band, u8g2_font_helvR08_tr, st->ssid, 22, W - sw - 6);
    const uint8_t *f = pick_font(IP_FONTS, sizeof IP_FONTS / sizeof IP_FONTS[0], st->ip, W);
    text_center(46, f, st->ip);
}

static void offline_screen(const closer_demo_status_t *st, int tick) {
    char line[64], detail[48] = "";
    bool busy = strncmp(st->wifi, "Searching", 9) == 0 || strncmp(st->wifi, "Getting", 7) == 0 ||
                strncmp(st->wifi, "Starting", 8) == 0 || strncmp(st->wifi, "Joining", 7) == 0;
    snprintf(line, sizeof line, "%s%.*s", st->wifi, busy ? tick / 3 % 4 : 0, "...");
    int lines = wrapped(0, 24, 11, W - 1, 2, u8g2_font_helvB08_tr, line);
    if (st->attempts > 1) snprintf(detail, sizeof detail, "attempt %d", st->attempts);
    if (closer_ble_running()) snprintf(detail + strlen(detail), sizeof detail - strlen(detail), "%sBluetooth on",
                                       detail[0] ? " - " : "");
    text(0, 24 + lines * 11, u8g2_font_5x8_tr, detail);
}

static void code_screen(const char *code, int left, const closer_demo_status_t *st) {
    char s[16];
    header("Control code", st, false);
    snprintf(s, sizeof s, "%d s", left);
    text_right(110, 9, u8g2_font_helvR08_tr, s);
    const uint8_t *f = pick_font(CODE_FONTS, sizeof CODE_FONTS / sizeof CODE_FONTS[0], code, W);
    text_center(56, f, code);
    int bar = (W - 4) * (left > 60 ? 60 : left) / 60;
    u8g2_DrawRFrame(&u8g2, 0, 58, W, 6, 2);
    if (bar > 0) u8g2_DrawBox(&u8g2, 2, 60, bar, 2);
}

// A tick mark and a line of text in the middle, for a moment.
static void news_screen(const char *title, const char *msg, const closer_demo_status_t *st) {
    header(title, st, true);
    int x = (W - width(u8g2_font_helvB12_tr, msg) - 18) / 2;
    u8g2_DrawLine(&u8g2, x, 33, x + 4, 38);
    u8g2_DrawLine(&u8g2, x + 1, 33, x + 5, 38);
    u8g2_DrawLine(&u8g2, x + 4, 38, x + 12, 27);
    u8g2_DrawLine(&u8g2, x + 5, 38, x + 13, 27);
    text(x + 18, 38, u8g2_font_helvB12_tr, msg);
}

static void draw(int tick) {
    static int64_t news_until;
    static int last_left;
    closer_demo_status_t st;
    closer_demo_get_status(&st);
    char code[12];
    int left = closer_access_code(code, sizeof code, NULL, 0);
    // A code just used: say so for a moment (an expired one just goes).
    if (last_left > 0 && left == 0 && closer_access_holder() != CLOSER_ACCESS_NONE) news_until = now_ms() + 2500;
    last_left = left;

    u8g2_ClearBuffer(&u8g2);
    if (st.updating) {
        header("BesBot", &st, true);
        text_center(29, u8g2_font_helvB12_tr, "Updating");
        int x = (W - width(u8g2_font_helvB12_tr, "firmware")) / 2;       // (centred without the dots)
        snprintf(code, sizeof code, "firmware%.*s", tick / 3 % 4, "...");
        text(x, 45, u8g2_font_helvB12_tr, code);
        text_center(62, u8g2_font_6x10_tr, "Don't switch off");
    } else if (left > 0) {
        code_screen(code, left, &st);
    } else if (now_ms() < news_until) {
        news_screen("BesBot", "Control given", &st);
    } else {
        header("BesBot", &st, true);
        if (st.connected) connected_screen(&st);
        else offline_screen(&st, tick);
        control_line(&st);
    }
}

static void display_task(void *arg) {
    int failures = 0;
    for (int tick = 0;; tick++) {
        draw(tick);
        if (flush() == ESP_OK) failures = 0;
        else if (++failures == 5) ESP_LOGW(TAG, "the display doesn't answer");
        else if (failures % 25 == 0 && init_panel() == ESP_OK) {    // (plugged in again: set it up afresh)
            ESP_LOGI(TAG, "the display is back");
            failures = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

bool display_start(int sda, int scl, display_drive_fn drive) {
    drive_fn = drive;
    i2c_master_bus_config_t bc = {.i2c_port = -1, .sda_io_num = sda, .scl_io_num = scl,
                                  .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
                                  .flags.enable_internal_pullup = true};
    i2c_master_bus_handle_t bus;
    if (i2c_new_master_bus(&bc, &bus) != ESP_OK) return false;
    int addr = 0;
    for (int a = 0x3C; a <= 0x3D && !addr; a++)
        if (i2c_master_probe(bus, a, 50) == ESP_OK) addr = a;
    if (!addr) {
        ESP_LOGI(TAG, "no display on SDA %d / SCL %d", sda, scl);
        i2c_del_master_bus(bus);
        return false;
    }
    i2c_device_config_t dc = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = addr, .scl_speed_hz = 400000};
    if (i2c_master_bus_add_device(bus, &dc, &dev) != ESP_OK) return false;
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_dummy_cb, u8x8_dummy_cb);    // (off-screen only)
    u8g2_SetFontMode(&u8g2, 1);
    snprintf(found, sizeof found, "%s 0x%02x", COL_OFFSET ? "sh1106" : "ssd1306", addr);
    esp_err_t e = init_panel();
    ESP_LOGI(TAG, "%s on SDA %d / SCL %d: %s", found, sda, scl, esp_err_to_name(e));
    xTaskCreate(display_task, "display", 4096, NULL, 2, NULL);
    return true;
}

const char *display_found(void) { return found; }
