# Robot Project — Claude Code Context

## Project Overview
Python/MicroPython robotics project running across three microcontroller boards.
All hardware documentation lives in `docs/hardware/` — read the relevant file before
writing any code that touches a board, pin, sensor, or actuator.

## Hardware Summary
| Board | Qty | Role | Docs |
|---|---|---|---|
| LilyGO T-Display S3 | 2 | Display + UI | `docs/hardware/lilygo_tdisplay_s3.md` |
| Cytron MOTION 2350 Pro | 1 | Motion / motors | `docs/hardware/motion_2350_pro.md` |

Full wiring, pin assignments, sensors, actuators → `docs/hardware/wiring_and_components.md`

## Key Rules
- **Check pin assignments** in `docs/hardware/wiring_and_components.md` before any GPIO code.
- **Never hardcode pin numbers** in logic — import from `config/pins.py`.
- When adding a component, update `wiring_and_components.md` first, then `config/pins.py`.
- Prefer non-blocking patterns (asyncio / timers) — blocking loops cause display freezes.
- MicroPython, not CPython — avoid stdlib modules that don't exist on the device.

## Project Structure
```
robot_project/
├── CLAUDE.md                            ← you are here
├── config/
│   └── pins.py                          ← single source of truth for all pin numbers
├── docs/hardware/
│   ├── lilygo_tdisplay_s3.md            ← board specs, display API, flashing
│   ├── motion_2350_pro.md               ← board specs, motor API, flashing
│   └── wiring_and_components.md         ← wiring table, all sensors & actuators
└── src/
    ├── display/                          ← code for the T-Display S3 boards
    └── motion/                           ← code for the MOTION 2350 Pro
```

## Common Tasks
- **Flash a board:** see the "Flashing" section in the relevant board doc
- **Add a component:** update `wiring_and_components.md` → `config/pins.py` → write driver in `src/`
