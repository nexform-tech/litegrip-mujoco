# litegrip-mujoco

Official MuJoCo-based simulation environment for the **LiteGrip parallel two-finger gripper**.
API-compatible with the `litegrip` SDK — swap `LiteGrip` for `MujocoGripper` and your control
code runs identically in simulation and on hardware.

## Features

- 🔄 **Drop-in API compatibility** — all 44 public `LiteGrip` members, same names, same meanings.
- 🖥️ **Three operating modes** — Standalone simulation / Mirror tracking / Dual control.
- 🎮 **Native MuJoCo rendering** — real-time visualization of jaw motion, contact and grasp.
- 🧪 **No hardware required** — `--dry-run` runs the mirror and dual-control examples without CAN.
- 📏 **True millimetres** — the stroke basis is 85.452 mm of measured URDF geometry, not the
  SDK's nominal 120 mm scale.
- **Calibration is explicit** — anything that moves the real gripper must be given a
  calibration file first. The SDK's silent fallback to its factory constants is refused, not
  inherited. See [Calibration](#calibration).

## Installation

```bash
# Standalone simulation (no hardware needed)
pip install litegrip-mujoco

# Mirror / Dual control mode (needs the real gripper SDK)
pip install "litegrip-mujoco[mirror]"
```

Or from source:

```bash
git clone https://github.com/nexform-tech/litegrip-mujoco.git
cd litegrip-mujoco
pip install -e ".[dev]"
```

## Quick Start

### Mode 1 — Standalone Simulation

```python
from litegrip_mujoco import MujocoGripper

with MujocoGripper(render=True) as gripper:
    gripper.open(duration=1.0)
    gripper.close(duration=1.0)

    # Position control, in true millimetres. 0 = closed, 85.452 = fully open.
    gripper.goto(20.0, duration=0.5)

    # Force control: close until stall, then hold the commanded grip force.
    gripper.grasp(force_n=10.0)

    state = gripper.get_state()
    print(state.position_mm, state.force_n)
```

The demo scene adds a floor and a 20 mm workpiece held in a fixture, so grasps can be verified
end to end:

```python
from litegrip_mujoco import MujocoGripper

with MujocoGripper(model_path="scene.xml", render=True) as gripper:
    gripper.grasp(force_n=10.0)
    gripper.release_fixture()   # the part now hangs on friction alone
```

### Mode 2 — Mirror Mode (sim follows the real gripper)

```python
from litegrip_mujoco import MujocoGripper, MirrorMode, apply_calibration, require_sdk

# The file the host-side calibration tool wrote for this gripper.
CALIBRATION = "~/.litegrip/litegrip_calibration.json"

sdk = require_sdk()
real = sdk.LiteGrip(channel="can0", can_id=0x08)
real.connect()
apply_calibration(real, CALIBRATION)   # must come before enable()
real.enable()

with MujocoGripper(render=True) as sim:
    with MirrorMode(real, sim, rate_hz=50.0):
        real.goto(20.0)     # the simulation follows in real time
```

`apply_calibration()` must run after `connect()` and before `enable()`. `enable()` is the first
call that energises the motor, so a calibration that failed to load has to be caught above it.

### Mode 3 — Dual Control (one command, both grippers)

```python
from litegrip_mujoco import DualGripper

dual = DualGripper(channel="can0", can_id=0x08, render=True,
                   calibration="~/.litegrip/litegrip_calibration.json")
dual.start()

dual.open()
dual.grasp(force_n=10.0)
print(dual.compare())       # {'real_frac': …, 'sim_frac': …, 'delta': …}

dual.disconnect()           # note: close() closes the jaws, not the resources
```

## Calibration

A calibration file records the two θ endpoints of one physical gripper — `zero_position_rad`
(the closed end), `max_position_rad` (the open end) and `rad_to_mm`. The host-side calibration
tool (GUI) writes it; nothing in this package can produce one.

**Every code path that moves the real gripper requires a calibration file, chosen at run
time.** Nothing selects one for you, and the default file is never used implicitly. The reasons
are in [Why the guard exists](#why-the-guard-exists); the short version is that the SDK's
`load_calibration()` cannot tell you which file it actually read.

### Choosing a file

`04_mirror_real.py` and `05_dual_control.py` accept two forms:

| Invocation | Behaviour |
| --- | --- |
| `--calibration PATH` | Uses that file, no prompt. This is the form for scripts and for non-interactive runs. |
| no argument, interactive terminal | Prints the candidates with their endpoints and travel, then prompts for one. |
| no argument, non-interactive terminal | Exits with status 2 and prints the `--calibration` form to use instead. |

```bash
# List what is on this machine, then pick one — run from the repository root.
python3 examples/05_dual_control.py --list-calibrations
python3 examples/05_dual_control.py --calibration ~/.litegrip/litegrip_calibration.json
```

`--dry-run` moves no hardware and skips calibration selection entirely.

From Python, `calibration=` is a keyword argument, and `apply_calibration()` is the explicit
call:

```python
from litegrip_mujoco import DualGripper, MirrorMode, apply_calibration

CALIBRATION = "~/.litegrip/litegrip_calibration.json"

dual = DualGripper(channel="can0", can_id=0x08, calibration=CALIBRATION)
MirrorMode(real, sim, rate_hz=50.0, calibration=CALIBRATION)

real.connect()
apply_calibration(real, CALIBRATION)   # after connect(), before enable()
real.enable()
```

The order matters. `enable()` is the first call that energises the motor, so the calibration
must be applied and verified above it. `apply_calibration()` raises rather than warning: on an
unreadable or inconsistent file, on the SDK's factory file, and on a post-load `config` whose
endpoints differ from the file's.

### Candidate discovery

With no explicit path, the scan looks in the directory holding the SDK's default calibration
path (`$LITEGRIP_CALIB`, else `~/.litegrip/litegrip_calibration.json`) and in the working
directory, for `*.json` regular files. The SDK's bundled factory file is never offered. Valid
calibrations are listed before unusable JSON, newest first.

An explicit path always wins and is never second-guessed — including the SDK's default path.
That file is marked `⚠ SDK default path` in the list to make the choice visible, but choosing
it deliberately is allowed. It is only the *implicit* use that is refused.

### What the guard checks

Three layers, because any one of them can be defeated on its own:

1. The file is parsed and validated **before** the SDK sees it — required keys present, values
   numeric and finite, `rad_to_mm > 0`, and the closed endpoint numerically **larger** than the
   open one. A file that fails is never handed to the SDK.
2. The SDK's factory calibration is refused by `realpath` identity, so relocating or symlinking
   it does not slip through.
3. After loading, `config.pos_closed_rad` / `pos_open_rad` are compared field-by-field against
   the file. A mismatch raises and names both the requested and the observed endpoint — that is
   the signature of the SDK's silent fallback.

The provenance gate, plus a check that `config` still matches the calibration that was applied,
runs again on every `read_frac_open()` and `write_frac_open()` call and before every
`DualGripper` and `MirrorMode` motion. A device that loses its calibration mid-run is therefore
caught, not only one that never had it.

### What the guard cannot check

A well-formed calibration file for a **different gripper of the same model** is
indistinguishable from the right one. Nothing here can detect it. The one check available is
arithmetic: the picker prints the travel each file implies, so read that number before trusting
the run and compare it with the gripper's real stroke.

### Strict conversions

`read_frac_open()` and `write_frac_open()` are strict by default. On a device with no proven
calibration they raise `UncalibratedDeviceError` instead of falling back to the SDK's
millimetre path, which is defined in terms of the very endpoints that are missing.

The previous behaviour is still reachable: `read_frac_open(device, warn=True)` emits a
`DeprecationWarning`, implies `strict=False`, and keeps the old warn-and-fall-back path. Pass
`strict=False` for the same thing without the warning.

Simulated devices are exempt, because their endpoints come from the model rather than from a
calibration — `MujocoGripper` and `DryRunGripper` set `IS_SIMULATED = True`. For any other
device, `mark_calibrated(device, None, reason="…")` records a deliberate exemption.

### Why the guard exists

The SDK's `load_calibration(path)` builds `sources = [path, _FACTORY_CALIB]`, swallows
`FileNotFoundError` and `json.JSONDecodeError`, falls through to the factory file, and returns
`True` for either source. A typo in the path therefore produces a gripper that reports success
and then moves in someone else's coordinates. See [Known behaviour](#known-behaviour) for the
two further defects that path exposes.

## Architecture

```text
┌──────────────────────────────────────────────────────┐
│                  Your Python Program                  │
│                                                       │
│   g = MujocoGripper()   ← replaces litegrip.LiteGrip  │
│   g.grasp(force_n=10.0)                               │
│   g.get_state()                                       │
└──────────┬─────────────────────────┬──────────────────┘
           │                         │
    ┌──────▼───────┐         ┌───────▼────────────┐
    │  Standalone  │         │  Dual / Mirror     │
    │  simulation  │         │                    │
    │              │         │  MuJoCo + CAN      │
    │  MuJoCo      │         │  → litegrip SDK    │
    │  physics     │         │                    │
    │  θ-space PD  │         │  real + sim        │
    │  μs solver   │         │  together          │
    └──────────────┘         └────────────────────┘
```

The physics model is built from the shipped URDF and STL meshes. It models the single DM4310
motor driving both fingers through a rigid coupling, and reproduces the SDK's control
semantics — including the ones that surprise people. See **Known behaviour** below.

## Examples

| Example | Description | Needs hardware |
| --- | --- | :---: |
| `01_hello_sim.py` | Create the simulation, read state, open and close once | ❌ |
| `02_move_sim.py` | Position, speed and force control; grasp verification | ❌ |
| `03_trajectory.py` | Record, save, load and replay a position trajectory | ❌ |
| `04_mirror_real.py` | The real gripper drives the simulation | `--dry-run`, else `--calibration` |
| `05_dual_control.py` | One command drives both simulation and hardware | `--dry-run`, else `--calibration` |

Examples 04 and 05 use the real gripper unless `--dry-run` is given, so they require a
calibration file — see [Calibration](#calibration).

```bash
# Run from the repository root, after `pip install -e ".[dev]"` — the examples
# import litegrip_mujoco, and a src-layout package is not importable until it
# is installed.
python3 examples/01_hello_sim.py
python3 examples/02_move_sim.py
python3 examples/03_trajectory.py
python3 examples/04_mirror_real.py --dry-run
python3 examples/05_dual_control.py --dry-run
```

If a run that opened the viewer ends with `Segmentation fault (core dumped)` **after** printing
`✅ 完成`, the run itself succeeded — that is an upstream GL teardown crash at process exit, not a
simulation failure. `--no-render` is unaffected and exits cleanly. See the developer guide.

## API Reference

Every public member of `litegrip.LiteGrip` exists on `MujocoGripper` with the same name and
the same meaning.

| `litegrip.LiteGrip` | `MujocoGripper` | Notes |
| --- | --- | --- |
| `LiteGrip(channel, can_id)` | `MujocoGripper(render=True)` | Constructor |
| `connect()` / `disconnect()` | `connect()` / `disconnect()` | ✅ Identical |
| `enable()` / `disable()` / `clear_fault()` | `enable()` / `disable()` / `clear_fault()` | ✅ Identical |
| `open()` / `close()` | `open()` / `close()` | ✅ Identical |
| `goto(mm)` / `goto_rad(rad)` | `goto(mm)` / `goto_rad(rad)` | ✅ Identical |
| `move_to(rad)` / `move_at_speed(mm, mm_s)` | `move_to(rad)` / `move_at_speed(mm, mm_s)` | ✅ Identical |
| `grasp(force_n)` | `grasp(force_n)` | ✅ Identical, including the overshoot |
| `set_force(force_n)` | `set_force(force_n)` | ✅ Identical, target = current position |
| `home()` | `home()` | ⚠️ See *Known behaviour* |
| `get_state()` / `get_position()` / `get_position_rad()` | same | ✅ Identical (see *State freshness*) |
| `refresh_status(timeout_s)` | `refresh_status(timeout_s)` | ✅ Always `True` — sim state is always readable |
| `get_force()` / `get_torque()` / `get_error()` | same | ✅ Identical |
| `get_temperature()` / `get_info()` | same | ✅ Simulated thermal model |
| `is_moving()` / `is_grasped()` / `wait_for_ready()` | same | ✅ Identical |
| `send_mit_frame(q, kp, kd, dq, tau)` | same | ✅ Identical — a control-law update |
| `poll(timeout)` | `poll(timeout)` | ✅ No-op in simulation |
| `read_param(rid)` | `read_param(rid)` | ❌ Raises `NotImplementedError` |
| `stop()` | `stop()` | ✅ Identical |
| `enter_zero_gravity()` / `exit_zero_gravity()` | same | ✅ Identical |
| `calibrate*()` / `save_calibration()` / `load_calibration()` | same | ✅ Simulated calibration |
| `channel` / `can_id` / `mst_id` / `config` | same | ✅ Recorded, unused in simulation |

Simulation-only additions: `step()`, `settle()`, `reset()`, `release_fixture()`,
`hold_fixture()`, `gap_mm()`, `frac_open()`, `set_frac_open()`, `launch_viewer()`,
`sync_viewer()`, `model`, `data`, `model_path`.

Also exported: `DualGripper`, `MirrorMode`, `DryRunGripper`, `read_frac_open()`,
`write_frac_open()`, `constants`, `require_sdk()`, `HAS_SDK`.

Calibration exports: `apply_calibration()`, `require_calibration()`,
`select_calibration_for()`, `resolve_calibration_path()`, `discover_calibrations()`,
`load_calibration_file()`, `default_calibration_path()`, `sdk_factory_calibration_path()`,
`mark_calibrated()`, `applied_calibration()`, `is_calibrated()`, `is_simulated_device()`,
`require_usable_device()`, `format_selection()`, `describe_candidate()`, the `Calibration`
dataclass, and the exceptions `CalibrationError`, `CalibrationRequiredError`,
`CalibrationFileError`, `CalibrationVerificationError` and `UncalibratedDeviceError`.

## Known behaviour

These are **faithful reproductions of the SDK**, not simulation defects. Do not "fix" them
without changing the real gripper's behaviour too — diverging here means the simulation stops
predicting the hardware.

- **`close(force_n=…)` cannot limit force.** The target runs to the closed position, so the
  position error `kp·Δθ` overwhelms the feedforward torque and the actuator saturates. Measured
  against the 20 mm workpiece: `force_n = 0 / 5 / 10 / 20` all produce exactly 100 N. Use
  `grasp()` or `set_force()` for force control.
- **`grasp(force_n)` applies somewhat more force than requested.** The stall-confirmation
  window (5 × 10 ms) keeps the jaws advancing, so `q_target` is rewritten to the position at
  the end of the window, leaving a fixed position error. Measured: 10 N requested → 15.5 N
  applied. Force remains monotonic in `force_n`, so it works as an open-loop force controller
  with a known offset.
- **`set_force(N)` needs to be re-issued.** The target is set to the *current* position, so the
  first call carries the previous preload's position error (2 N requested → 2.88 N on the first
  call). Re-issuing converges to within 0.5%.
- **`grasp()` returns `True` on an empty gripper.** Jaws hitting the closed hard stop also count
  as "stalled". This matches the SDK.
- **`get_force()` is torque-derived, not measured.** It is `torque_nm × 10`, exactly as in the
  SDK. It reads a number even when nothing is between the jaws.
- **`home()` differs.** In the SDK, `home()` targets the constant `POS_CLOSED_RAD`, which is
  `0.0`, and `goto_rad()`'s clamp then drives the gripper **open** — the opposite of its
  docstring. The simulation goes to the closed position, as documented. This is a deliberate
  divergence: the examples should not teach the bug.

### State freshness

The SDK's `GripperState` carries `data_age_s` (how long ago the frame behind these numbers
arrived), plus `has_data` and `is_stale` (default threshold `STALE_AFTER_S = 0.5 s`). A
disabled motor does not stream status frames, so on hardware `get_state()` can hand back a
snapshot that is seconds old, or the constructor defaults from before the first enable —
which is exactly why `refresh_status()` exists.

Simulation has no such problem: every `get_state()` is computed from the physics state
*right then*, so `data_age_s` is always `0.0` and `is_stale` is always `False`. That is
deliberate — it reflects the absence of a CAN link, not a change to the SDK's semantics.
Real-machine code that uses `is_stale` to decide whether to re-read will simply always take
the "fresh" branch under simulation, which is correct.

### Two bugs found in the `litegrip` SDK
### Three bugs found in the `litegrip` SDK

Reported here for awareness. `litegrip-mujoco` does not patch the SDK. The third one it refuses
to run behind — see [Calibration](#calibration) — because the failure it produces is a moving
gripper in the wrong coordinates rather than an exception.

1. **`goto_rad()` ignores its argument.** `constants.py` sets `POS_OPEN_RAD = +1.14` and
   `POS_CLOSED_RAD = 0.0`, which violates the SDK's own documented invariant (the closed value
   should be numerically larger). `gripper.py`'s clamp `max(pos_open, min(pos_closed, x))` then
   returns `+1.14` for **every** input. `goto_rad(0.0)`, `goto_rad(0.5)`, `goto_rad(-1.14)` and
   `goto_rad(3.0)` all produce `+1.1400`. Loading a calibration JSON overwrites both endpoints
   and masks the bug.
2. **`calibrate_guided()` hard-codes the 120 mm scale.** It computes `rad_to_mm = 120.0 / travel`
   and ignores `config.max_stroke_mm`, while `calibrate()` and `calibrate_manual()` both honour
   it. Guided calibration therefore always produces a 120-scale gripper.
3. **`load_calibration()` cannot report which file it read.** It tries the given path, then the
   bundled factory calibration, swallowing `FileNotFoundError` and `json.JSONDecodeError`, and
   returns `True` for either. A misspelled path is therefore indistinguishable from success.
   Separately, when a file parses but has no `rad_to_mm`, the `KeyError` raises from outside the
   guard that handles the other missing keys.

## Development

```bash
pip install -e ".[dev]"
python -m pytest tests/ -v
```

The suite is pure simulation — no CAN interface, no hardware, no `litegrip` SDK. Cases that
need the SDK skip themselves when it is absent.

See [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md) for the model layout, the millimetre
basis, the calibration guard, and the design decisions behind the collision geometry.

## License

Proprietary

---

[中文文档](README_zh-CN.md) | [Developer Guide](docs/DEVELOPER_GUIDE.md)
