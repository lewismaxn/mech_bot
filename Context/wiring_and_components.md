# Wiring & Components Reference

> This is the **single source of truth** for all physical connections.
> Update this file whenever you add, remove, or rewire a component.
> Then update `config/pins.py` to match.

---

## Inter-Board Communication
| From | To | Interface | Pins (From side) | Pins (To side) | Notes |
|---|---|---|---|---|---|
| T-Display S3 #1 | MOTION 2350 Pro | _(e.g. UART)_ | TX: GP_, RX: GP_ | RX: GP8, TX: GP9 | _(baud rate, protocol details)_ |
| T-Display S3 #2 | MOTION 2350 Pro | _(e.g. I2C)_ | SDA: GP43, SCL: GP44 | SDA: GP20, SCL: GP21 | _(I2C address)_ |

---

## Sensors

### Sensor 1 — _(Name / Model)_
- **Type:** _(e.g. ultrasonic distance, IMU, line sensor)_
- **Connected to board:** _(T-Display S3 #1 / #2 / MOTION 2350 Pro)_
- **Interface:** _(GPIO / I2C / SPI / UART / ADC)_
- **Pins:**
  | Signal | Board GPIO | Wire colour |
  |---|---|---|
  | VCC | 3V3 | Red |
  | GND | GND | Black |
  | _(signal)_ | GP_ | _(colour)_ |
- **Operating voltage:** 3.3 V / 5 V
- **I2C address (if applicable):** `0x__`
- **Notes / datasheet:** _(link or filename)_

### Sensor 2 — _(Name / Model)_
_(copy the block above and fill in)_

---

## Actuators

### Motor 1 — _(Name / Description)_
- **Type:** _(e.g. DC gear motor, stepper, brushless)_
- **Connected to:** MOTION 2350 Pro — Channel 1
- **Direction pin:** GP0
- **PWM pin:** GP1
- **Encoder A:** GP22 _(if fitted)_
- **Encoder B:** GP23 _(if fitted)_
- **Rated voltage:** __ V
- **Max current:** __ A
- **Gear ratio:** __ : 1
- **Notes:** _(e.g. left drive wheel)_

### Motor 2 — _(Name / Description)_
_(copy the block above and fill in)_

### Servo 1 — _(Name / Description)_
- **Connected to:** MOTION 2350 Pro — Servo output 1 (GP10)
- **Range:** 0–180°
- **Centre position:** 90°
- **Notes:** _(e.g. pan axis of camera mount)_

---

## Power Distribution
| Rail | Source | Connected to | Notes |
|---|---|---|---|
| VIN (__ V) | Battery / PSU | MOTION 2350 Pro VIN | Motor supply |
| 5 V | MOTION 2350 onboard reg | _(peripherals)_ | Max __ A |
| 3.3 V | Each board onboard reg | GPIO logic, sensors | Max 300 mA per board |
| USB 5 V | USB-C | T-Display S3 #1, #2 | Development / programming only |

---

## Wiring Diagram
_(Insert a diagram image here, or link to a KiCad / Fritzing file in the repo)_

---

## Change Log
| Date | Change | Author |
|---|---|---|
| _(date)_ | Initial wiring documented | _(you)_ |
