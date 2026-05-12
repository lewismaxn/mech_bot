"""
config/pins.py — Single source of truth for all GPIO pin assignments.

IMPORTANT: Keep this in sync with docs/hardware/wiring_and_components.md.
Import from this file in all source code — never hardcode pin numbers elsewhere.

Board targets:
  - BOARD_DISPLAY_1  : LilyGO T-Display S3 #1
  - BOARD_DISPLAY_2  : LilyGO T-Display S3 #2
  - BOARD_MOTION     : Cytron MOTION 2350 Pro
"""

# ---------------------------------------------------------------------------
# T-Display S3 — Built-in display pins (do not change)
# ---------------------------------------------------------------------------
DISPLAY_BL  = 38   # Backlight (PWM)
DISPLAY_CS  = 6
DISPLAY_DC  = 7
DISPLAY_RST = 5
DISPLAY_WR  = 8
DISPLAY_RD  = 9
DISPLAY_D0  = 39
DISPLAY_D1  = 40
DISPLAY_D2  = 41
DISPLAY_D3  = 42
DISPLAY_D4  = 45
DISPLAY_D5  = 46
DISPLAY_D6  = 47
DISPLAY_D7  = 48

# ---------------------------------------------------------------------------
# T-Display S3 — User buttons
# ---------------------------------------------------------------------------
BTN_BOOT    = 0    # Active LOW
BTN_USER    = 14   # Active LOW

# ---------------------------------------------------------------------------
# T-Display S3 — Communication with MOTION board
# ---------------------------------------------------------------------------
DISPLAY_UART_TX = None   # TODO: assign GPIO
DISPLAY_UART_RX = None   # TODO: assign GPIO
DISPLAY_I2C_SDA = 43
DISPLAY_I2C_SCL = 44

# ---------------------------------------------------------------------------
# MOTION 2350 Pro — Motor channels
# ---------------------------------------------------------------------------
MOTOR1_DIR  = 0    # Direction
MOTOR1_PWM  = 1    # Speed (PWM)
MOTOR2_DIR  = 2
MOTOR2_PWM  = 3
MOTOR3_DIR  = 4
MOTOR3_PWM  = 5
MOTOR4_DIR  = 6
MOTOR4_PWM  = 7

# ---------------------------------------------------------------------------
# MOTION 2350 Pro — Servo outputs
# ---------------------------------------------------------------------------
SERVO_1     = 10
SERVO_2     = 11
SERVO_3     = 12
SERVO_4     = 13
SERVO_5     = 14
SERVO_6     = 15
SERVO_7     = 16
SERVO_8     = 17

# ---------------------------------------------------------------------------
# MOTION 2350 Pro — Encoders
# ---------------------------------------------------------------------------
ENC1_A      = 22
ENC1_B      = 23
# ENC2_A    = None   # TODO: assign if using encoder on motor 2
# ENC2_B    = None

# ---------------------------------------------------------------------------
# MOTION 2350 Pro — Communication
# ---------------------------------------------------------------------------
MOTION_UART_TX  = 8
MOTION_UART_RX  = 9
MOTION_I2C_SDA  = 20
MOTION_I2C_SCL  = 21

# ---------------------------------------------------------------------------
# MOTION 2350 Pro — ADC / sensors
# ---------------------------------------------------------------------------
ADC_BATTERY = 26   # Battery voltage divider
ADC_1       = 27   # Spare ADC
ADC_2       = 28   # Spare ADC

# ---------------------------------------------------------------------------
# Sensors — add entries as you wire components
# ---------------------------------------------------------------------------
# Example:
# SENSOR_ULTRASONIC_TRIG = None   # TODO
# SENSOR_ULTRASONIC_ECHO = None   # TODO
