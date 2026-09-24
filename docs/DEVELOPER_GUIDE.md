# Developer Guide

How the LiteGrip MuJoCo model is put together, and why it is put together that way. Read this
before changing `assets/litegrip.xml` or anything under `src/litegrip_mujoco/`.

## Package layout

```text
src/litegrip_mujoco/
├── __init__.py          Public exports
├── constants.py         Units, stroke basis, conversions. No SDK import, by design.
├── controller.py        θ-space PD + minimum-jerk / linear ramps
├── gripper.py           MujocoGripper — the LiteGrip-compatible façade
├── mirror.py            DualGripper, MirrorMode, cross-device frac_open helpers
├── dryrun.py            DryRunGripper — a LiteGrip-shaped virtual gripper
├── _litegrip/           Lazily probes the real SDK; falls back to local dataclass mirrors
└── assets/
    ├── litegrip.xml     The gripper model
    ├── scene.xml        litegrip.xml + floor + lights + a fixtured workpiece
    └── meshes/          Three STLs copied byte-for-byte from litegrip-urdf
```

## Where the numbers come from

| Source | Provides |
| --- | --- |
| `litegrip-urdf/urdf/litegrip_urdf.urdf.xacro` + 3 STLs | All geometry, inertia and joint origins |
| `lite-grip` (the SDK) | Control semantics, units, `grasp()` stall logic, error types |
| `litearm-mujoco` | Package shape, example numbering, controller structure |

The URDF is authoritative for **geometry**. The SDK is authoritative for **control semantics**.
Where they disagree, the disagreement is documented in the XML itself rather than smoothed over.

## The millimetre basis

Two different millimetre scales exist and they differ by 40%:

| Scale | Value | Where it comes from |
| --- | --- | --- |
| True stroke | **85.452 mm** | Measured jaw travel, `2 × 0.042726 m` |
| SDK nominal | 120 mm | `calibrate_guided()`'s hard-coded `120.0 / travel` |

**This package always reports true millimetres.** `constants.MM_SCALE = 85.452`, and every
`position_mm` you read is a real millimetre.

The SDK's 120 mm scale should be **eliminated on the hardware side, not compensated for on the
simulation side**. Set `GripperConfig(max_stroke_mm=85.452)` and re-calibrate; the real gripper
then reports true millimetres too, and both sides agree with zero conversion. Adding an offset
in the simulation would double-count the error and make the SDK's own bug permanent.

### θ is device-dependent, `frac_open` is not

Motor angle is a poor interchange quantity: its zero point **and** its span depend on
calibration.

| Device | θ closed | θ open | Span |
| --- | --- | --- | --- |
| Simulation | `0.0` | `-1.14` | 1.14 |
| Real gripper (calibrated) | `1.775959` | `-0.064279` | 1.8402 |

Handing the same `position_rad` to both devices puts them in completely different positions.
`frac_open ∈ [0, 1]` is calibration-independent and is therefore the only safe quantity to
exchange. `MirrorMode` and `DualGripper` exchange `frac_open`; see `mirror.py`.

### The two URDF discrepancies

Both are known, both are deliberate, both are pinned by tests:

1. **Stroke: 0.0435 m (mesh) vs 0.042726 m (measured).** The model uses the measured value,
   because that is the real mechanical hard stop. Using 0.0435 would let `close()` travel
   0.77 mm past the physical limit and offset every reported millimetre by a constant.
2. **Open gap 87.000 mm vs total stroke 85.452 mm.** These are mutually inconsistent by
   0.04 mm — the URDF comment's stated closed gap of 1.508 mm was computed from a measured open
   gap of 86.960 mm, not from the mesh. This model uses **mesh geometry**, so the closed gap is
   always 1.548 mm.

The relationship is `gap_mm = travel_mm + 1.548`. `get_position()` reports travel (0 = closed);
`gap_mm()` reports the absolute opening.

## Why collision uses boxes, not meshes

MuJoCo has **no concave mesh collision** — every collision mesh is reduced to its convex hull.
Measured on this gripper, the hull is 2.334× the mesh volume for `base_link` and 2.075× for the
fingers. Hulled, the base fills the fingers' slide channels, producing **6 contacts and 5.25 mm
of penetration at every joint value**. No parameter tuning fixes this; it is structural.

So: visual geoms use the original STL meshes with collision disabled, and collision uses two box
proxies per finger (slider body + gripping pad) derived by scanning the finger STL layer by
layer. The pad box's `+x` face sits exactly on mesh `x = 0.0235`, i.e. world `x = -0.0435` at
`q = 0` — which is what makes the 87.000 mm opening exact. The base body has no collision geometry
at all, matching `litearm7.xml`.

⚠ **Never write `pos`/`quat` on a mesh geom.** MuJoCo principal-axis-aligns meshes at compile time
and sets the geom's default `pos`/`quat` to the same transform to cancel it out. Overwriting them
un-cancels the rotation and tilts the part. A regression test guards this.

## The actuator chain

```text
ctrl  ──(gear=10)──►  qfrc_actuator  ──►  joint force  ──►  pad force
 τ [Nm]                 10·τ [N]          (per finger)
```

`gear="10"` is chosen so that `data.ctrl` is numerically the SDK's `tau` in Nm. `ctrlrange=±10`
maps to the DM4310's `TAU_MAX`, so the grip-force ceiling comes from the real motor.

Two traps:

- **`actuator_force` is pre-gear**, `qfrc_actuator` is post-gear. Mixing them is a 10× error.
- **`qfrc_actuator` is the commanded force, not a measurement.** With the jaws stalled on an
  empty clamp it reads 100 N while the true contact force is zero. Nor can `qfrc_constraint` be
  used to measure grip force: with the workpiece on the floor and `ctrl = 1.0`, it still reads
  exactly `-5.000 N` because the closed joint limit supplies the reaction.

`get_force()` therefore reproduces the SDK exactly: `force_n = torque_nm × 10`. It is an
estimate derived from torque, and it reports a value even when nothing is between the jaws.

### `armature` is not optional

The DM4310's rotor inertia is about `1.2e-5 kg·m²`. Through a 10:1 reduction that is
`1.2e-3 kg·m²`, and through the screw ratio `dθ/dx = 26.68 rad/m` it becomes about **0.85 kg of
reflected mass** — 24× the finger's own 0.0355 kg. Without `armature="0.85"`, `vmax` reaches
2.93 m/s at `dt = 2 ms` and never converges (`q` still at 0.0399 after 1 s).

`timestep="0.001"` for the same reason: at `dt = 2 ms`, `kd ≥ 5` does not converge.

## The controller

The PD loop runs in **θ space** with the SDK's `kp`/`kd`, so `kp = 150` produces the same motor
torque for the same motor-angle error on the simulation and on the hardware. No gain porting is
needed; the effective joint-space gain follows automatically (`kp_q = GEAR · R · kp`).

`<motor>` plus an external Python PD loop is used instead of `<position>` because the MIT frame
is `f(q, dq, kp, kd, tau)` — kp and kd are per-call payload in that protocol, and `<position>`
would hide them in the model.

Default gains are `kp = 100, kd = 2`, taken from `GripperParams` (not litearm's 260/5). The
θ-space damping ratio is ≈ 2.8 — overdamped, which is what a geared transmission should be.
Do not "correct" it.

`duration` is honoured in **wall-clock seconds**. The simulation runs far faster than real time,
so copying the SDK's "sleep 1 s" would make `open(duration=1.0)` return in milliseconds and
diverge from hardware timing. The loop schedules against absolute deadlines so the error does
not accumulate.

Position-family calls (`open`/`close`/`goto`/`move_to`/`move_at_speed`) use a minimum-jerk
target ramp. `grasp()` and `set_force()` use a **constant** target — a ramp would smear the
stall detection.

## Force control: the load-bearing detail

Force control only works when the position error is zero.

```text
ctrl = kp·(θ_des − θ) + kd·(−θ̇) + τ_ff
       └──────┬──────┘
        this term must vanish for ctrl to equal τ_ff
```

`grasp()` therefore does two things in sequence:

1. Close at `kp = 150` with the target at the closed position — the position error dominates and
   the actuator saturates (this is why `close(force_n=…)` cannot limit force).
2. Detect stall (5 consecutive 10 ms samples with `|Δθ| < 0.001 rad`), then **rewrite
   `q_target` to the current position** and keep only `τ_ff = force_n × 0.1`.

Step 2 is what makes the force real. `set_force()` does the same thing in one shot.

The stall threshold is a **motor-angle** quantity and must be converted, not copied:
`Δq = 0.001 / R = 3.748e-5 m`. It is counted on the 10 ms control tick, not per physics step —
at `dt = 1 ms` a per-step `Δq` is 10× smaller than the hardware's window and would report
stall while the jaws are still moving.

`grasp()` returning `True` on an empty gripper (jaws against the closed hard stop) is SDK
behaviour and is reproduced rather than improved; changing it would make the simulation and the
hardware disagree.

## The demo scene

`scene.xml` adds a floor at `z = -0.08`, lights, and a 20 × 20 × 30 mm workpiece at
`z = 0.096735` with a freejoint.

**The workpiece is held by a `<weld>` fixture rather than resting on a pedestal, and that is not
a shortcut.** The gripper is fixed to the world and cannot move to reach a part, so the part must
start between the jaws. But it cannot sit on a table: scanning the right finger's inner face
(`world_x = -0.067 + mesh_x_max + q`) shows that at full close the fingers converge to
`|x| ≈ 0.0008–0.0045` at **every** height `z ∈ [0.024, 0.109]`. There is no height at which a
support column could pass between the closing jaws. It is geometrically impossible.

So the examples `close()` first, then `release_fixture()`. After that the part hangs on friction
alone — which is what makes it a real end-to-end grasp test rather than a torque-derived number.
`open()` drops it to the floor at `z = -0.065` (floor `-0.08` + half-height 15 mm).

## Uncalibrated free parameters

**The URDF contains no friction data.** Everything below is a conservative placeholder, not a
measurement. Calibrate before trusting absolute force numbers on hardware.

| Parameter | Value | Basis |
| --- | --- | --- |
| `friction` | `0.9 0.02 0.0001` | Order-of-magnitude for silicone/TPU pads on plastic |
| `solref` | `0.002 1` | Chosen, not measured — see the table in the XML |
| `solimp` | `0.98 0.999 0.001` | MuJoCo default-ish stiff contact |
| `damping` / `frictionloss` | `0.05` / `0.02` | Placeholders |
| `armature` | `0.85` | Computed from the DM4310 rotor inertia — this one is derived, not guessed |

`solref` sets how soft the gripping pads are, which sets how far `grasp(force_n)` overshoots its
request. Measured against the demo workpiece:

| `solref` | 10 N requested | 20 N requested | 40 N requested | Pad indentation per finger |
| --- | --- | --- | --- | --- |
| `0.006 1` | 21.84 N | 29.33 N | 45.61 N | 287 / 349 / 442 µm |
| `0.002 1` | 15.51 N | 24.86 N | 43.59 N | 27 / 43 / 75 µm |
| `0.001 1` | 15.51 N | 24.86 N | 43.59 N | 27 / 43 / 75 µm |

Going stiffer than `0.002` buys nothing — `0.001` is identical, proving the remaining overshoot
is the SDK's 5 × 10 ms stall-confirmation window, not contact compliance. The hardware has the
same window, so the overshoot is faithful, not a simulation defect.

## The SDK shim

`_litegrip/` probes for the real `litegrip` package **lazily and once**. If it is installed,
its dataclasses and enums are re-exported; if not, local field-for-field mirrors in
`_fallback.py` are used so that `MujocoGripper.get_state()` still returns a real `GripperState`.

The SDK is not vendored, because importing it pulls in SocketCAN. The one intentional divergence
in `_fallback.py` is the default `GripperConfig`, which uses the simulation's self-consistent
values (`pos_closed_rad = 0.0`, `pos_open_rad = -1.14`, `max_stroke_mm = 85.452`) rather than the
SDK's defaults, which violate their own documented invariant.

## Testing

```bash
python -m pytest tests/ -v
```

Three layers:

- **Geometry and units** — reads the STL and MJCF data directly, bypassing `MujocoGripper`. These
  pin the model itself: mesh units, the 87.000 mm opening, no self-penetration anywhere in the
  stroke, the mesh-alignment invariant.
- **Behaviour** — the public API, checked against the SDK's semantics.
- **Physical fidelity** — the surprising behaviours above are frozen as tests, with the reason in
  the docstring, so nobody "tidies them up" later.

The suite needs no CAN interface, no hardware and no `litegrip` SDK. Cases that want the SDK skip
themselves when it is absent.

If you change contact parameters, `test_grasp_force_exceeds_request` will fail and point you at
the override table in `litegrip.xml`. Update both together.
