# LilyGO T-Display S3 — Hardware Reference

## Board Overview
- **Model:** LilyGO T-Display S3
- **MCU:** ESP32-S3 (dual-core Xtensa LX7, up to 240 MHz)
- **Flash:** 16 MB
- **PSRAM:** 8 MB (OPI)
- **Display:** 1.9" ST7789 TFT, 170×320 px, 8-bit parallel interface
- **USB:** USB-C (native USB, no USB-UART chip)
- **Battery:** JST connector, onboard charging circuit (IP5306 or similar)

## Roles in This Robot
| Unit | Assigned Role |
|---|---|
| T-Display S3 #1 | Master controller - takes sensor input and coordinates slave |
| T-Display S3 #2 | Accumulation of sensor information and motor control |

## Pin Reference
| Function | GPIO | Notes |
|---|---|---|
| Display BL (backlight) | 38 | PWM controllable |
| Display CS | 6 | |
| Display DC | 7 | |
| Display RST | 5 | |
| Display WR | 8 | |
| Display RD | 9 | |
| Display D0–D7 | 39,40,41,42,45,46,47,48 | 8-bit parallel |
| Button 1 | 0 | Active LOW, boot button |
| Button 2 | 14 | Active LOW |
| I2C SDA | 43 | Shared bus |
| I2C SCL | 44 | Shared bus |
| SPI MOSI | 11 | |
| SPI MISO | 13 | |
| SPI CLK | 12 | |
| Battery ADC | 4 | Voltage divider, read × 2 |

> **Note:** Update this table to match your actual wiring. GPIO assignments above are
> T-Display S3 defaults — verify against your board revision's schematic.

## MicroPython
- **Firmware:** Use the official ESP32-S3 MicroPython build with SPIRAM support.
- **Recommended build:** `GENERIC_S3` with `idf5.x` (check https://micropython.org/download/ESP32_GENERIC_S3/)
- **Flash command:**
  ```bash
  esptool.py --chip esp32s3 --port /dev/ttyACM0 erase_flash
  esptool.py --chip esp32s3 --port /dev/ttyACM0 write_flash -z 0x0 ESP32_GENERIC_S3-*.bin
  ```

## Display Driver
The ST7789 display uses the `st7789` MicroPython driver. Typical init:
```python
import st7789
import tft_config  # board-specific pin/config helper

tft = tft_config.config(tft_config.WIDE)
tft.init()
tft.fill(st7789.BLACK)
```
- Landscape mode: 320×170 px
- Portrait mode: 170×320 px
- See `src/display/` for project display drivers.

## Power Notes
- Powered via USB-C or LiPo battery (JST 2-pin, 3.7 V).
- Do not exceed 3.3 V on GPIO pins.
- 5 V is available on the `5V` pin only when USB is connected.

## Known Quirks
- USB-C is native USB (CDC); on some hosts the port appears as `/dev/ttyACM0` not `/dev/ttyUSB0`.
- Backlight must be explicitly enabled after display init — it defaults off.
- _(add project-specific quirks here as you discover them)_
