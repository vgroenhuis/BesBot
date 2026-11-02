// The small I2C OLED (128x64, SSD1306 or SH1106): the control code while one is shown, otherwise the robot's Wi-Fi,
// address and who drives it.
#pragma once
#include <stdbool.h>

// The motors' outputs (-1..1) and whether the driver is on Bluetooth, for the bottom line.
typedef void (*display_drive_fn)(float *left, float *right, bool *bluetooth);

bool display_start(int sda, int scl, display_drive_fn drive);  // false when no display answers (then nothing happens)
const char *display_found(void);                               // e.g. "ssd1306 0x3c", "" when none
