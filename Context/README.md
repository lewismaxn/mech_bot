# Robot Project

## Quick Start for Claude Code

Open Claude Code in this directory:
```bash
cd robot_project
claude
```

Claude will automatically read `CLAUDE.md` at the start of every session,
giving it context about the boards, wiring, and project structure.

## Hardware
- 2× LilyGO T-Display S3 (ESP32-S3, MicroPython)
- 1× Cytron MOTION 2350 Pro (RP2350, MicroPython)

## Documentation
| File | Purpose |
|---|---|
| `CLAUDE.md` | Agent context — rules, structure, task guide |
| `docs/hardware/lilygo_tdisplay_s3.md` | T-Display S3 specs, pins, display API |
| `docs/hardware/motion_2350_pro.md` | MOTION 2350 Pro specs, motor/servo API |
| `docs/hardware/wiring_and_components.md` | Full wiring table, sensors, actuators |
| `config/pins.py` | All GPIO pin assignments (import, don't hardcode) |

## Filling In Your Details

Before your first coding session, update these placeholders:

1. **`docs/hardware/wiring_and_components.md`** — add your actual sensors, actuators, and wiring.
2. **`docs/hardware/lilygo_tdisplay_s3.md`** — fill in the role of each T-Display S3 unit.
3. **`config/pins.py`** — set any `None` pin values once you've decided on wiring.
4. **`CLAUDE.md`** — add any project-specific rules that emerge as you develop.
