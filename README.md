# litegrip-mujoco

Official MuJoCo-based simulation environment for the **LiteGrip parallel two-finger gripper**.
API-compatible with the `litegrip` SDK — swap `LiteGrip` for `MujocoGripper` and your control
code runs identically in simulation and on hardware.

## Features

- 🔄 **Drop-in API compatibility** — all 43 public `LiteGrip` members, same names, same meanings.
- 🖥️ **Three operating modes** — Standalone simulation / Mirror tracking / Dual control.
- 🎮 **Native MuJoCo rendering** — real-time visualization of jaw motion, contact and grasp.
- 🧪 **No hardware required** — `--dry-run` runs the mirror and dual-control examples without CAN.
- 📏 **True millimetres** — the stroke basis is 85.452 mm of measured URDF geometry, not the
  SDK's nominal 120 mm scale.

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
from litegrip_mujoco import MujocoGripper, MirrorMode, require_sdk

sdk = require_sdk()
real = sdk.LiteGrip(channel="can0", can_id=0x08)
real.connect()
real.enable()

with MujocoGripper(render=True) as sim:
    with MirrorMode(real, sim, rate_hz=50.0):
        real.goto(20.0)     # the simulation follows in real time
```

### Mode 3 — Dual Control (one command, both grippers)

```python
from litegrip_mujoco import DualGripper

dual = DualGripper(channel="can0", can_id=0x08, render=True)
dual.start()

dual.open()
dual.grasp(force_n=10.0)
print(dual.compare())       # {'real_frac': …, 'sim_frac': …, 'delta': …}

dual.disconnect()           # note: close() closes the jaws, not the resources
```

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
| `04_mirror_real.py` | The real gripper drives the simulation | `--dry-run` |
| `05_dual_control.py` | One command drives both simulation and hardware | `--dry-run` |

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
| `get_state()` / `get_position()` / `get_position_rad()` | same | ✅ Identical |
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

### Two bugs found in the `litegrip` SDK

Reported here for awareness. `litegrip-mujoco` neither patches nor guards against them.

1. **`goto_rad()` ignores its argument.** `constants.py` sets `POS_OPEN_RAD = +1.14` and
   `POS_CLOSED_RAD = 0.0`, which violates the SDK's own documented invariant (the closed value
   should be numerically larger). `gripper.py`'s clamp `max(pos_open, min(pos_closed, x))` then
   returns `+1.14` for **every** input. `goto_rad(0.0)`, `goto_rad(0.5)`, `goto_rad(-1.14)` and
   `goto_rad(3.0)` all produce `+1.1400`. Loading a calibration JSON overwrites both endpoints
   and masks the bug.
2. **`calibrate_guided()` hard-codes the 120 mm scale.** It computes `rad_to_mm = 120.0 / travel`
   and ignores `config.max_stroke_mm`, while `calibrate()` and `calibrate_manual()` both honour
   it. Guided calibration therefore always produces a 120-scale gripper.

## Development

```bash
pip install -e ".[dev]"
python -m pytest tests/ -v
```

The suite is pure simulation — no CAN interface, no hardware, no `litegrip` SDK. Cases that
need the SDK skip themselves when it is absent.

See [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md) for the model layout, the millimetre
calibration basis, and the design decisions behind the collision geometry.

## License

Proprietary

---

[中文文档](README_zh-CN.md) | [Developer Guide](docs/DEVELOPER_GUIDE.md)
