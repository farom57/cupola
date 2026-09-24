# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Control system for an observatory dome ("coupole", 3.2 m radius). It has two parts that talk over USB serial at **1 000 000 baud**:

- `cupola/`: Arduino sketch (Arduino Uno) that drives the dome relays (rotate left/right, open/close the shutter, red/white light) and reads the local push-buttons, limit switches and two photo-interrupters used as a quadrature step encoder.
- `python/`: desktop app (Tkinter, used mainly on Windows but also working on Linux) that reads the telescope position from **PlaneWave PWI4** and slews the dome so the shutter opening stays in front of the telescopes.

The shutter ("cimier") motors are powered through 6 sliding contacts between the fixed and rotating parts. These contacts only touch at one dome azimuth, so the shutter can only open or close at that position (see the "Cimier" section of `TODO.md`).

The older BLE/IMU/RF firmware (`ble.cpp`, `imu.cpp`, `rf.cpp`…) was removed in commit `b04c57c`. Check git history if you need it.

The UI strings and many comments are in French.

## Commands

There is no build system, no test suite and no linter.

- Firmware: open `cupola/cupola.ino` in the Arduino IDE, or `arduino-cli compile --fqbn arduino:avr:uno cupola` then `arduino-cli upload -p <port> --fqbn arduino:avr:uno cupola`.
- Python app: `cd python && python main.py`. The modules import each other as siblings, so run it from `python/`. Install the dependencies with `pip install -r python/requirements.txt`. `tkinter` also has to be available, since it ships with Python rather than pip (on Linux: `sudo apt install python3-tk`). On Linux the user must be in the `dialout` group to open `/dev/ttyACM*` / `/dev/ttyUSB*`. PWI4 must be running and serving HTTP at the configured address (default `localhost:8220`).
- Firmware by hand: open a serial monitor at 1 000 000 baud and send `?` to list the single-character commands.

## Architecture

### Firmware (`cupola/cupola.cpp`)

All the logic is in one `loop()` that runs about every 1 ms:
1. Light button: a short press toggles off/red, and a long press (about 1 s) switches red to white. Buttons are active-low.
2. Quadrature decoding of `IN_PHOT_1/2` into `step_counter`, which wraps modulo `TOTAL_STEPS`. A rising transition of `IN_HOME` records `step_home`.
3. When a `target` is set, the dome stops as soon as the counter crosses it.
4. Pressing any local button (except light) cancels remote commands. Local buttons always take priority.
5. Outputs are written from `dome_cmd` / `cover_cmd`. In `*_MANUAL` state the physical buttons drive the relays directly.
6. `comm()` handles one serial byte per loop: `s` step, `h` home step, `i` input bitmask (hex), `l`/`r`/`o`/`c`/`x` motion, `b`/`n`/`w` light, `t<N>` go to step N (it takes the shortest direction; `t0` or a negative N stops).

### Python app (`python/`)

- `main.py`: builds the Tkinter UI and starts a daemon `worker()` thread. Once per second the thread polls the dome and PWI4, computes the target azimuth, and sends `goto` when tracking is on, the error exceeds the tolerance, and the last `goto` was at least 3 s ago. Shared state lives in module globals. Site and telescope configuration is at the top of the file: `mount_origin`, `dome_radius`, `opening_width`, `scope_offset` and `scope_diameter` (indexed by the scope checkboxes). `ConfigWindow` is the « Config » dialog. When saved, it reassigns the global `pwi4` and disconnects `c` so that `worker()` reconnects to the new port.
- `config.py`: user settings (dome serial port, PWI4 host and port), saved as JSON to `%APPDATA%\cupola\config.json`, or `~/.config/cupola/config.json` outside Windows. Missing keys fall back to `DEFAULTS`.
- `serial_com.py`: `list_ports()` lists the ports shown in the config dialog. It puts the stable `/dev/serial/by-id/*` links first, then USB ports, and hides the phantom Linux `/dev/ttyS*`. `DEFAULT_PORT` is `COM6` on Windows and `/dev/ttyACM0` elsewhere. The `Cupola` class wraps the serial protocol behind a `threading.Lock`, because both the worker thread and the UI callbacks send commands. The home position is tracked on the PC side (`set_home` records the current step, and `ref_azimuth` is the azimuth at home). The firmware's `step_home` is not used. Step/azimuth conversion uses `STEPS_PER_TURN`.
- `geometry.py`: `compute_azimuth(ha, de, lat, …)` rotates each telescope's optical axis from the mount frame into the dome frame (X=east, Y=north, Z=zenith) and intersects rays from the telescope aperture outline with the dome sphere. The result is the optimal dome azimuth plus a tolerance (half the overlap of the allowed ranges). Angles are in radians. `plot_accessible_range()` is a standalone visualisation helper.
- `pwi4_client.py`: the stock PlaneWave PWI4 HTTP client (vendor code). Don't modify it.

### Known firmware/client mismatches

The Python client and the firmware in this repo are currently out of sync. Keep this in mind before assuming either side is correct. `TODO.md` tracks these issues and the pending hardware checks, so update it when you resolve one:
- The client sends `k` / `k0` / `k1` (tracking flag) and `p` (outputs), but the firmware doesn't implement them. The tracking button therefore never gets a numeric reply.
- `STEPS_PER_TURN = 692` in Python, but `TOTAL_STEPS = 48` in the firmware (a comment there says `4320`).
- For `t<N>` the firmware prints `delta:<d>` before echoing the target. `Cupola.goto()` reads only the first line, so it gets a non-numeric reply.
- In firmware command `i`, the accumulator `int i` is not initialised.

## Outside this repo

The parent directory (`../`, not under git) contains related material: `Doc/` (wiring diagrams, component list, schematic/PCB PDFs, dome manuals), `3D/` (FreeCAD enclosures and the encoder wheel), `matlab/` (older IMU heading-calibration scripts), `suivi_coupole/` (a pre-repo copy of the Python app), `ASCOM/` (an Alpaca rotator demo, not integrated), and `old/`.
