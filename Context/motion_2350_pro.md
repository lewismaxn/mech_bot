# Cytron MOTION 2350 Pro — Hardware Reference

## Board Overview
- **Model:** Cytron MOTION 2350 Pro
- **MCU:** Raspberry Pi RP2350 (dual Cortex-M33 or RISC-V Hazard3, up to 150 MHz)
- **Flash:** 4 MB onboard + microSD slot
- **Motor Channels:** 4× bidirectional DC motor channels (via 2× MDD3A dual drivers)
- **Servo Outputs:** 8× PWM servo outputs
- **Power Input:** 6–16 V DC (motor supply); onboard 5 V and 3.3 V regulators
- **USB:** USB-C (RP2350 USB)
- **GPIO:** 20 GPIO pins (3.3 V logic)
- **Comms:** UART, I2C, SPI, CAN (optional)

## Role in This Robot
- Primary motion controller: drives all servos.
- Receives commands from the T-Display S3 boards via _(specify: UART / I2C / SPI / WiFi)_.

## Pin Reference
| Function | GPIO / Pin | Notes |
|---|---|---|
| Motor 1A | GP0 | MDD3A channel 1 direction |
| Motor 1B | GP1 | MDD3A channel 1 PWM speed |
| Motor 2A | GP2 | MDD3A channel 2 direction |
| Motor 2B | GP3 | MDD3A channel 2 PWM speed |
| Motor 3A | GP4 | MDD3A channel 3 direction |
| Motor 3B | GP5 | MDD3A channel 3 PWM speed |
| Motor 4A | GP6 | MDD3A channel 4 direction |
| Motor 4B | GP7 | MDD3A channel 4 PWM speed |
| Servo 1–8 | GP10–GP17 | 50 Hz PWM, 1–2 ms pulse |
| I2C SDA | GP20 | |
| I2C SCL | GP21 | |
| UART TX | GP8 | To display boards |
| UART RX | GP9 | From display boards |
| Encoder A (M1) | GP22 | Interrupt-capable |
| Encoder B (M1) | GP23 | Interrupt-capable |
| User LED | GP25 | Onboard LED |
| ADC 0 | GP26 | Battery / sensor ADC |
| ADC 1 | GP27 | |
| ADC 2 | GP28 | |

> **Note:** Update these assignments to match your actual wiring before coding.
> Cytron's official pinout: https://docs.cytron.io/motion-2350-pro

## MicroPython
- **Firmware:** Use the official Raspberry Pi RP2350 MicroPython build.
- **Download:** https://micropython.org/download/RPI_PICO2/
- **Flash:** Hold BOOTSEL, plug USB-C → board appears as USB mass storage → drag-and-drop `.uf2`.
- **Thonny** or `mpremote` both work for file transfer and REPL.

## Motor Control
```python
from machine import Pin, PWM

# Example: drive motor 1 forward at 50% speed
dir_pin = Pin(0, Pin.OUT)
pwm_pin = PWM(Pin(1))
pwm_pin.freq(20000)  # 20 kHz to reduce audible whine

dir_pin.value(1)                    # set direction
pwm_pin.duty_u16(int(0.5 * 65535)) # 50% duty cycle
```

## Servo Control
```python
from machine import Pin, PWM

servo = PWM(Pin(10))
servo.freq(50)  # 50 Hz

def set_angle(servo_pwm, angle):
    """Set servo angle (0–180°)."""
    min_duty = 1638   # ~1 ms pulse at 50 Hz (16-bit)
    max_duty = 8192   # ~2 ms pulse at 50 Hz (16-bit)
    duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
    servo_pwm.duty_u16(duty)
```

## Power Notes
- Motor supply: 6–16 V DC on VIN/GND terminals.
- Logic and MCU powered from onboard 5 V / 3.3 V regulators (from VIN).
- Do not exceed 3.3 V on GPIO pins.
- Each MDD3A channel: max 3 A continuous, 5 A peak.

## Known Quirks
- RP2350 has two cores — offload encoder counting to core 1 if needed.
- PWM frequency above 20 kHz reduces motor whine but check driver compatibility.
- _(add project-specific quirks here as you discover them)_
