# MOTION 2350 Pro — Arduino Sketches Reference

> Records the design, pin usage, and change history for the two Arduino sketches
> that run on the Cytron MOTION 2350 Pro board.
> Hardware specs → `motion_2350_pro.md` | Full wiring → `wiring_and_components.md`

---

## Sketches Overview

| Sketch | Path | Purpose |
|---|---|---|
| `motion_controller_v2` | `Code/Motion 2350 Pro_code/motion_controller_v2/` | **Production** — state-machine controller, listens for pulse from master |
| `motion_control_panel` | `Code/Motion 2350 Pro_code/motion_control_panel/` | **Debug/tuning** — WiFi web panel for manual servo control and IMU inspection |

Flash one or the other — they are not run simultaneously.

---

## Pin Assignments (as used in code)

| Signal | GPIO | Notes |
|---|---|---|
| Extender servo | GP0 | Continuous rotation — 1500=stop, 1800=extend, 1200=retract |
| Flipper R servo | GP1 | Continuous rotation — 1500=stop |
| Flipper L servo | GP3 | Same board as GP1 but **physically inverted** (back-to-back mount) — signal is always mirrored around 1500 |
| State input (from master) | GP26 | Rising-edge pulse; long ≥60 ms = state advance, short burst = servo cmd |
| Buzzer | GP22 | `tone()` — startup beep + state-change beep + background melody |
| NeoPixel RGB LED | GP28 | Adafruit NeoPixel, 1 pixel, NEO_GRB + NEO_KHZ800 |
| IMU SDA | GP20 | MPU-6050 I2C data |
| IMU SCL | GP21 | MPU-6050 I2C clock |

---

## Servo Pulse Width Reference

All three servos are **continuous rotation**. Speed is proportional to deviation from neutral.

| Constant | Value (µs) | Meaning |
|---|---|---|
| NEUTRAL / US_MID | 1500 | Stopped |
| EXT_EXTEND | 1800 | Extender — full speed extend |
| EXT_RETRACT | 1200 | Extender — full speed retract |
| FLIP_R_RAISE | **1650** | Flipper R — raise (50% speed) |
| FLIP_L_RAISE | **1350** | Flipper L — raise, inverted (50% speed) |
| FLIP_R_DUMP | **1350** | Flipper R — dump (50% speed) |
| FLIP_L_DUMP | **1650** | Flipper L — dump, inverted (50% speed) |

> Flipper values were originally ±300 µs (1800/1200) and were reduced to ±150 µs
> (1650/1350) on 2026-05-20 to halve rotation speed. Adjust further by moving
> values closer to 1500 (slower) or further from 1500 (faster).

---

## motion_controller_v2 — State Machine

### States

| # | Name | Servo action | LED colour |
|---|---|---|---|
| 0 | WAITING | All neutral | Dim white |
| 1 | CALIBRATION | All neutral | Cyan |
| 2 | FORWARD1 | All neutral | Green |
| 3 | BACKWARD | All neutral | Yellow |
| 4 | FORWARD2 | All neutral | Green |
| 5 | EXTEND | Extender extends, flipperR retracts | Purple |
| 6 | RAISEARM | Flippers raise | Magenta |
| 7 | TURN | All neutral | Blue |
| 8 | FORWARD3 | All neutral | Green-teal |
| 9 | DEPOSIT | Flippers dump | Orange |
| 10 | GOHOME | All neutral | Teal |
| 11+ | DONE | All neutral + 3× white flash | Dim white |

### Pulse Protocol (GP26)

- **Long pulse ≥60 ms** — advance to next state
- **Short pulse burst (10–59 ms each, decoded after 400 ms silence)** — servo command:

| Burst count | Command |
|---|---|
| 2 | Extender extend |
| 3 | Extender stop |
| 4 | Extender retract |
| 5 | Flippers raise |
| 6 | Flippers stop |
| 7 | Flippers dump |
| 8 | All neutral |

### Background Music
Non-blocking melody plays on the buzzer (GP22) during operation — "Samplab_Again" instrumental, looping ~10 s. Implemented in `updateMusic()` using `millis()` — does not block the state machine.

---

## motion_control_panel — WiFi Debug Panel

### WiFi Access Point
| Setting | Value |
|---|---|
| SSID | `Robot-Motion` |
| Password | `robot1234` |
| IP address | `192.168.4.1` |
| Requires | Pico 2W variant (RP2350 + CYW43439 chip) |

### HTTP Endpoints

| Endpoint | Parameters | Action |
|---|---|---|
| `GET /` | — | Serve the control panel HTML page |
| `GET /servo` | `s=ext\|flip`, `us=1000–2000` | Move extender or both flippers (GP3 auto-inverted) |
| `GET /raw` | `e=`, `r=`, `l=` (µs each) | Set all three servos independently, no auto-invert |
| `GET /neutral` | — | All servos → 1500 µs |
| `GET /led` | `r=`, `g=`, `b=` (0–255) | Set NeoPixel colour |
| `GET /imu` | — | Returns 7 newline-delimited values: aX, aY, aZ (g), gX, gY, gZ (°/s), temp (°C) |
| `GET /log` | — | Returns full action log as plain text |
| `GET /clearlog` | — | Wipes the in-memory log buffer |

### IMU Integration (MPU-6050)
- Initialised at boot via I2C on GP20/GP21, address 0x68
- Accel range: ±2g (16384 LSB/g)
- Gyro range: ±250°/s (131 LSB/°/s)
- Temperature formula: T(°C) = raw/340 + 36.53
- Live readings served at `/imu`, displayed in the webpage IMU card (auto-refresh every 2 s)

### Action Log
- Every command is timestamped (`[seconds.ms]` since boot) and appended to an in-memory string
- IMU snapshot is recorded **before** and **after** every servo command (300 ms delay between command and POST reading to capture motion)
- Buffer capped at 6 KB — oldest lines dropped when full
- Webpage auto-refreshes the log display every 3 s
- **Download button** — browser fetches `/log` and saves as `robot_actions.txt`
- Log is also mirrored to Serial at 115200 baud

Example log excerpt:
```
[   0.312] Board booted    — control panel ready at 192.168.4.1
[   0.315]   BOOT IMU  — Accel: X=+0.01g Y=-0.00g Z=+0.99g  |  Gyro: X=  +0.1 Y=  -0.0 Z=  +0.0 °/s  |  Temp: 25.3°C
[  12.001]   PRE  IMU  — Accel: X=+0.01g Y=+0.00g Z=+0.99g  |  Gyro: X=  +0.0 Y=  +0.1 Z=  -0.0 °/s  |  Temp: 25.4°C
[  12.001] CMD  ext     — Extender (GP0) → 1650µs (fwd)
[  12.302]   POST IMU  — Accel: X=+0.03g Y=+0.01g Z=+0.97g  |  Gyro: X=  +1.2 Y=  +0.3 Z=  -0.1 °/s  |  Temp: 25.4°C
```

---

## Libraries Required

| Library | Used by | Install via |
|---|---|---|
| `Servo.h` | Both sketches | Earle Philhower arduino-pico core (includes Servo for RP2350) |
| `Adafruit_NeoPixel` | Both sketches | Arduino Library Manager |
| `Wire.h` | motion_control_panel | Built into arduino-pico core |
| `WiFi.h` | motion_control_panel | Built into arduino-pico core |
| `WebServer.h` | motion_control_panel | Built into arduino-pico core |

> **Important:** `Servo.h` requires the **Earle Philhower arduino-pico core** — the official
> Arduino-Mbed RP2350 core does not include Servo support. Board manager URL:
> `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`

---

## Change Log

| Date | Change |
|---|---|
| 2026-05-20 | Created `motion_controller_v2` and `motion_control_panel` sketches |
| 2026-05-20 | Reformatted `motion_control_panel` for readability (expanded minified HTML/CSS/JS, renamed CSS classes and element IDs) |
| 2026-05-20 | Added action logging to `motion_control_panel` — timestamped log buffer, `/log` and `/clearlog` endpoints, download-as-txt button |
| 2026-05-20 | Added MPU-6050 IMU support to `motion_control_panel` — live display card, `/imu` endpoint, PRE/POST IMU snapshots in action log |
| 2026-05-20 | Reduced flipper servo speed from ±300 µs to ±150 µs (50% speed) in both sketches — values changed from 1800/1200 to 1650/1350 |
