# BesBot firmware (ESP-IDF)

ESP-IDF 6.1 firmware for the BesBot (ESP32-C5-DevKitC-1 + L298N), replacing the Arduino code in `../src`
(which the original board ran). The ESP32 only drives IN1..IN4 of the L298N (ENA/ENB tied high); there is no display.

* **Driving:** WebSocket `/ws`: send `"stream"` for the live state, `{"x":..,"y":..,"src":".."}` (turn right +, forward +,
  -1..1) to drive, `{"stop":1}`, `{"set":{"max":100,"accel":25,"trim":0,"rev_left":0,"rev_right":0,"min":10}}`. Or HTTP:
  `GET /api/drive`, `POST /api/drive?x=..&y=..` / `?stop=1`, `POST /api/settings` (JSON).
  Dead-man stop 0.5 s after the last command; one driver at a time (lease 1.5 s after its last moving command); speed
  ramp; settings in NVS. Roaming scans, access point scans and firmware updates wait while it drives.
* **Control** (since 1.3.0): one person at a time controls the robot, everyone else watches (`closer_access` in the
  closer_demo component, `components/closer_demo/include/closer_access.h`). Control comes with a code the robot
  shows on request (`{"access":"request"}`, valid 60 s; BesBot: 2 digits, 2 tries a code; on the display and in
  the serial log), with the lab demos site's grant (an admin:
  "Take control"; a server link takes it only while nobody has it), or with the fixed Bluetooth code. A new holder takes
  over; control ends on release, 60 s after the holder's connection closed (unless it comes back with its token), or
  after 10 minutes without a command. Driving and settings need control (`{"need":1}` otherwise; HTTP: `?token=` or
  `X-Access-Token`); stop works for everyone; the fixed Bluetooth code can only be changed with the site's grant.
  The state carries `"access":"you"|"other"|"none"` and `"code"` (seconds the code is still shown).
* **Display** (since 1.4.0; this layout since 1.5.0): a 128x64 I2C OLED, SSD1306 (or SH1106:
  `CONFIG_BESBOT_OLED_SH1106`) at 0x3C/0x3D, SDA GPIO3 and SCL GPIO2 (`menuconfig`: BesBot). Laid out like the other
  demos' displays (u8g2 fonts, drawn off-screen; `main/display.c`): header with Wi-Fi bars, network, the address as
  big as it fits, and at the bottom "Free to drive" or Wi-Fi/Bluetooth with the two motors as bars. While a control
  code is shown: the code (2 digits on BesBot, `closer_access_set_digits`) as big as it fits, with a bar for the time
  left; "Control given" for a moment after it was used. The drive state's `"display"` says what was found.
* **Bluetooth LE** (since 1.1.0): the CloSer lab demo service of `closer_ble` (see `components/closer_demo/include/closer_ble.h`)
  with the `/ws` commands, for places with poor Wi-Fi. Anyone nearby may connect and stop the robot; driving and settings
  need the access code first: `code:<code>` -> `{"auth":1}`. The code (6-32 characters, in NVS) is set by an admin through
  the site (`{"set":{"ble_code":".."}}` on `/ws` or `POST /api/settings`; `""` switches Bluetooth driving off); a wrong code
  blocks code checks for 2 s. Each connection gets its own state notification (`{"drive":1,"l","r","driver","src",
  "deadman","max","auth","code"}`), losing the connection stops the robot if it drove. The site's drive page connects
  with Web Bluetooth (Chrome/Edge; not iPhone).
* **Wi-Fi:** the closer_demo component (roaming, reporting to the lab demos site, over-the-air updates with rollback) and
  the Turtle's Wi-Fi meter: the LED as a signal light, `/api/wifi`, `/api/wifi/history` (10 minutes), `/api/wifi/aps`,
  surveys (`/api/survey`).
* **Pages:** `/` (touch pad, keyboard, settings), `/wifi` (saved networks, band, roaming, Bluetooth on/off), `/udp`. The lab demos site has the full driving page with gamepad, SpaceMouse,
  HandTool and ATLAS (`labdemos/apps/besbot/drive.html` on the server).

Pins (menuconfig, *BesBot*): IN1 9, IN2 10 (left), IN3 25, IN4 26 (right), LED 27.

Module: ESP32-C5-WROOM N16R8. Since 1.2.0: 16 MB layout (`partitions.csv`: two 4 MB app slots, a core dump partition,
8 MB `storage` for later), PSRAM on (big allocations, TLS among them, go there; `psram_free` in the stats). Access
point scans only while someone looks at the list (5 min apart on a good signal, 1 min on a weak one), never while it
drives or while a Bluetooth device is connected (with Bluetooth on a scan holds the radio for ~10 s).

Moving a board from the old 4 MB layout (once, over USB; keeps the settings in NVS at 0x9000):

```
python -m esptool --chip esp32c5 -p COMx write-flash 0x2000 build/bootloader/bootloader.bin \
    0x8000 build/partition_table/partition-table.bin 0xf000 build/ota_data_initial.bin 0x20000 build/besbot.bin
```

## Build and flash

```
. ~/esp/idf-env.sh
idf.py build
python tools/provision.py besbot     # local/besbot.json (Wi-Fi) + local/besbot.key -> local/besbot-factory.bin
python -m esptool --chip esp32c5 -p COMx write-flash 0x0 besbot-factory.bin     # once, over USB
```

After that, updates go over the air: `python -m labdemos.cli publish besbot build/besbot.bin` on the server.
`local/` (Wi-Fi password, device key, images) is not in git.
