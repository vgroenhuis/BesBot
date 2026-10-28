# BesBot firmware (ESP-IDF)

ESP-IDF 6.1 firmware for the BesBot (ESP32-C5-DevKitC-1 + L298N), replacing the Arduino code in `../src`
(which the original board ran). The ESP32 only drives IN1..IN4 of the L298N (ENA/ENB tied high); there is no display.

* **Driving:** WebSocket `/ws`: send `"stream"` for the live state, `{"x":..,"y":..,"src":".."}` (turn right +, forward +,
  -1..1) to drive, `{"stop":1}`, `{"set":{"max":70,"accel":25,"trim":0,"rev_left":0,"rev_right":0,"min":30}}`. Or HTTP:
  `GET /api/drive`, `POST /api/drive?x=..&y=..` / `?stop=1`, `POST /api/settings` (JSON).
  Dead-man stop 0.5 s after the last command; one driver at a time (lease 1.5 s after its last moving command); speed
  ramp; settings in NVS. Roaming scans, access point scans and firmware updates wait while it drives.
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
