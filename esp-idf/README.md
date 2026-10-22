# BesBot firmware (ESP-IDF)

ESP-IDF 5.5 firmware for the BesBot (ESP32-C5-DevKitC-1 + L298N), replacing the Arduino code in `../src`
(which the original board ran). The ESP32 only drives IN1..IN4 of the L298N (ENA/ENB tied high); there is no display.

* **Driving:** WebSocket `/ws`: send `"stream"` for the live state, `{"x":..,"y":..,"src":".."}` (turn right +, forward +,
  -1..1) to drive, `{"stop":1}`, `{"set":{"max":70,"accel":25,"trim":0,"rev_left":0,"rev_right":0,"min":30}}`. Or HTTP:
  `GET /api/drive`, `POST /api/drive?x=..&y=..` / `?stop=1`, `POST /api/settings` (JSON).
  Dead-man stop 0.5 s after the last command; one driver at a time (lease 1.5 s after its last moving command); speed
  ramp; settings in NVS. Roaming scans, access point scans and firmware updates wait while it drives.
* **Wi-Fi:** the closer_demo component (roaming, reporting to the lab demos site, over-the-air updates with rollback) and
  the Turtle's Wi-Fi meter: the LED as a signal light, `/api/wifi`, `/api/wifi/history` (10 minutes), `/api/wifi/aps`,
  surveys (`/api/survey`).
* **Pages:** `/` (touch pad, keyboard, settings). The lab demos site has the full driving page with gamepad, SpaceMouse,
  HandTool and ATLAS (`labdemos/apps/besbot/drive.html` on the server).

Pins (menuconfig, *BesBot*): IN1 9, IN2 10 (left), IN3 25, IN4 26 (right), LED 27.

## Build and flash

```
. ~/esp/idf-env.sh
idf.py build
python tools/provision.py besbot     # local/besbot.json (Wi-Fi) + local/besbot.key -> local/besbot-factory.bin
python -m esptool --chip esp32c5 -p COMx write-flash 0x0 besbot-factory.bin     # once, over USB
```

After that, updates go over the air: `python -m labdemos.cli publish besbot build/besbot.bin` on the server.
`local/` (Wi-Fi password, device key, images) is not in git.
