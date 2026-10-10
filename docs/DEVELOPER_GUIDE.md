# Developer Guide

How the LiteGrip MuJoCo model is put together, and why it is put together that way. Read this
before changing `assets/litegrip.xml` or anything under `src/litegrip_mujoco/`.

## Package layout

```text
src/litegrip_mujoco/
├── __init__.py          Public exports
├── constants.py         Units, stroke basis, conversions. No SDK import, by design.
├── controller.py        θ-space PD + minimum-jerk / linear ramps
├── calibration.py       Calibration provenance: discovery, validation, apply-and-verify
├── gripper.py           MujocoGripper — the LiteGrip-compatible façade
├── mirror.py            DualGripper, MirrorMode, cross-device frac_open helpers
├── dryrun.py            DryRunGripper — a LiteGrip-shaped virtual gripper
├── _litegrip/           Lazily probes the real SDK; falls back to local dataclass mirrors
└── assets/
    ├── litegrip.xml     The gripper model
    ├── scene.xml        litegrip.xml + floor + lights + a fixtured workpiece
    └── meshes/          Three STLs copied byte-for-byte from litegrip-description
```

## Where the numbers come from

| Source | Provides |
| --- | --- |
| `litegrip-description/urdf/litegrip_urdf.urdf.xacro` + 3 STLs | All geometry, inertia and joint origins |
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

The real row is one calibration taken on 2026-09-24, frozen as a reference. Recalibrating the
gripper changes both numbers, so treat them as an illustration rather than as this machine's
current endpoints. `constants.REAL_POS_CLOSED_RAD` / `REAL_POS_OPEN_RAD` carry that pair and are
what the dry-run gripper reports, so its θ arithmetic has the same shape as a real device's —
nothing else reads them.

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

## Calibration selection and provenance

Real-hardware motion requires a calibration file. With none named, that file is the SDK's
factory calibration (`factory_calibration.json`, beside the `litegrip` package), and the rule is
implemented once, in `resolve_calibration_path()`. `calibration.py` owns it; `mirror.py` and
examples 04/05 enforce it by calling into it. Nothing scans the disk for a calibration and
nothing prompts for one.

**Do not** add a scan-and-pick step back into `apply_calibration()` or the examples. The SDK
itself silently loads its factory file when a given path cannot be read, and still returns
`True`; a picker on top of that only adds a second way to drive the motor with another
machine's angles. To change which file is used, name it: `calibration=` in Python,
`--calibration` on the command line.

### The two gates

A device may move only when both hold:

| Gate | Test | Failure |
| --- | --- | --- |
| Provenance | `is_simulated_device(d)` or `_provenance(d) is not None` | `UncalibratedDeviceError` |
| Validity | `_endpoints(d) is not None` — the closed endpoint is numerically larger than the open one | `UncalibratedDeviceError` |

Gate 1 asks where the numbers came from, gate 2 whether they make sense. The SDK's factory
constants (`pos_closed_rad = 0.0`, `pos_open_rad = +1.14`) have the right *types* but the wrong
*order*, and that inversion is exactly how an uncalibrated device is recognised: `_endpoints()`
returns `None` unless `pos_closed_rad > pos_open_rad`. Every real calibration satisfies it.

Provenance is a frozen `Provenance(path, calibration, reason)` stored as
`device._litegrip_mujoco_calibration`, with a `WeakKeyDictionary` behind it for devices that
refuse `setattr` (`__slots__`). `mark_calibrated(device, None, reason="…")` is the escape hatch
for a device whose endpoints are trustworthy but which has no file; the reason is recorded so
the exemption shows up in review instead of being implied.

`require_usable_device()` runs gate 1 and then `_check_device_still_holds()`, which raises
`CalibrationVerificationError` if `config` has drifted away from the calibration that was
applied. That second check is what catches a device that *lost* its calibration mid-run, not
just one that never had it.

### The two layers

`apply_calibration(device, path)` defends in two places, because the first one alone cannot see
the SDK's fallback:

1. **Validate before the SDK sees the file.** Existence, regular file, JSON object, the three
   required keys (`zero_position_rad`, `max_position_rad`, `rad_to_mm`), finiteness,
   `rad_to_mm > 0`, `closed > open`, and a span of at least `MIN_SPAN_RAD = 1e-3`. The SDK
   raises an unguarded `KeyError` when a file parses but lacks `rad_to_mm`, and a file with
   `closed == open` divides by zero further downstream.
2. **Compare the post-load `config` endpoints to the file's, field by field.** This is the
   load-bearing layer — it is the only one that catches the SDK's fallback — and it raises with
   both the requested and the observed endpoint, which is the signature of that fallback.

The SDK's factory file gets no special treatment here: it is validated and verified like any
other file. It used to be refused by `realpath` identity (`allow_factory=True` to pass), which
made the default calibration the one file the library would not accept.

⚠ **Never move the `apply_calibration()` call below `enable()`.** `enable()` is the first call
that energises the motor. The SDK's documented order is `connect() → load_calibration() →
enable()`; `apply_calibration()` is a drop-in for the middle step, and both examples follow that
order.

### Why `read_frac_open` is strict by default

The millimetre fallback is defined in terms of the endpoints that are missing, so it returns a
number that looks like a reading and is not one. On a device that fails either gate,
`read_frac_open()` and `write_frac_open()` raise `UncalibratedDeviceError` instead.

`read_frac_open(device, warn=True)` survives as a deprecated alias: it emits a
`DeprecationWarning` and implies `strict=False`. It was kept rather than deleted because
deleting it breaks callers, and under this repository's release policy a breaking change means a
major version — not something to do as a side effect of a safety fix. `strict=False` gives the
old path without the warning.

### The `DryRunGripper` trap

`DryRunGripper.load_calibration()` and `save_calibration()` delegate to `self._inner`, so
`dry.config` never changes and the delegate's return value says nothing about the outer object.
`DryRunGripper` is exempt from gate 1 via `IS_SIMULATED`, so nothing here depends on those
delegates — but do not read `dry.config` expecting the loaded file's endpoints. Pre-existing and
unrelated to this guard; tracked rather than fixed here.

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

The four collision boxes carry `group="3"`, which is visual only: collision reads `contype` and
`conaffinity`, never the group, so no dynamics and no contact change. It is not optional. The boxes
deliberately overshoot the mesh — the pad box has to cover every vertex with `x ≥ 0.0230` — so in
the default group they poke through the white STL and render as two grey blocks above the
fingertips, which reads as a broken model. The viewer draws groups 0-2 only: `mujoco.MjvOption()`
defaults to `[1 1 1 0 0 0]` and `launch_passive` uses that default. Tick group 3 in the Rendering
panel to see the proxies.

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

### Spare slots, and what `add_box()` costs

`scene.xml` also carries four parked bodies, `spawn_box_0` … `spawn_box_3`: each is a free joint
with a single box geom, resting away from the gripper. MuJoCo compiles a model once, so a runtime
`add_box()` cannot conjure a body — it **moves** a slot to `pos` (by default `grasp_center()`,
between the jaws) and rewrites its size. `box_slots()` lists the names; `litegrip.xml` has none,
and asking it for a slot raises `IndexError`.

- **The slot is moved, not created.** Whatever occupied it — size, mass, velocity — is gone.
  Reusing a slot overwrites the previous box. `reset()` parks every slot back on the floor.
- **Resizing needs `mj_setConst`.** MuJoCo derives `body_mass` and `body_inertia` from the geom
  size at *compile* time and does not notice a runtime change, so `add_box()` writes those two
  arrays and then calls `mj_setConst(model, data)`.
- **`mj_setConst` resets `qpos` to `qpos0`.** Measured on MuJoCo 3.11: `qvel`, `ctrl` and `time`
  are left alone, but every joint position returns to the model's default. Left unhandled, adding
  a box would quietly put the gripper and the workpiece back at their default pose.
  `world.spawn_box()` therefore saves `qpos` before the resize and restores it around the
  `mj_setConst` call; the new box's pose is written *after* the restore, so the two cannot fight.

The read-only half of the same module is cheaper and needs none of this: `link_aabb()` and
`pad_aabbs()` return world-space boxes for collision geoms, `contacts()` resolves contact names
and forces, and `grasp_center()` returns the midpoint between the two finger pads. Those AABBs
are the **collision** boxes, deliberately coarse — never read the jaw opening from one. Use
`gap_mm()`.

## The viewer and the threading model

The physics runs in a background thread (`litegrip_sim`) started by `connect()`. One RLock guards
every touch of `MjData`. Three rules follow, and breaking any of them produces a failure that is
*intermittent*, which is exactly why they are written down:

**1. Create the viewer before starting the sim thread.** `launch_passive()` calls
`mj_forward(model, data)` on your `MjData` internally. If the sim thread is already stepping that
same data, two threads enter its arena at once, `mj_makeConstraint` cannot grow it, and you get
`mj_makeConstraint: nefc under-allocation` — or, more often, a segfault. This was a real bug here:
`_open_viewer()` used to call `connect()` first. It is fixed, and
`test_no_sim_thread_when_viewer_is_created` fails if it comes back.

**2. Call `sync()` inside the lock.** `sync()` copies `MjData` into the viewer's internal copy.
The main thread's `close()`/`goto()` ramp loops also step `MjData` under the lock, so a `sync()`
outside it races them — the same disease as rule 1. `litearm-mujoco` does it this way too.

**3. Do not close the viewer while the sim thread is alive.** `disconnect()` sets
`_running = False`, detaches the viewer so the loop can no longer sync it, joins the thread, and
closes the viewer **only if the thread actually exited**. If the join times out the thread may be
wedged inside `sync()`, and closing the window under it races for `MjData`. Better to leak a
window into process teardown than to segfault there.

**4. The key callback runs on the viewer's thread and may only enqueue.** `launch_passive()`
invokes `key_callback` from the viewer's own thread, at a moment when the physics thread is
inside `mj_step()`. `MujocoGripper._on_key()` therefore does exactly one thing —
`self._keys.feed(int(keycode))`, an append to a `deque` — and the main thread drains it in its
own loop with `keyboard_events()`. **Do not** touch `MjData`, open or close the viewer, or print
from that callback: an exception raised there lands on the viewer's thread, next to MuJoCo's
internal state, where nothing is watching for it. Reading input is the main thread's job, and
`pressed()` / `held()` take the list `keyboard_events()` returns.

`key_codes()` resolves the glfw key constants lazily, with a literal fallback when `glfw` cannot
be imported, so this module still imports with no window, no display and no MuJoCo viewer —
which is what `--help`, `--headless` and CI need. The callback itself is only offered by MuJoCo
≥ 3.1: `_accepts_key_callback()` inspects `launch_passive`'s signature rather than trying the
call and catching `TypeError`, because a `TypeError` from *inside* `launch_passive` would mean
the window already exists, and retrying would leave an orphan window nobody syncs. On an older
MuJoCo the window still opens; `keyboard_events()` simply stays empty.

**The overlay font is Latin-only.** `status_text()` reaches the window through `mjr_overlay`,
which draws with MuJoCo's built-in bitmap font. That font has no CJK glyphs — every Chinese
character renders as a filled rectangle — so a line of Chinese reads as a row of blocks. Measured
here through the same call (`mjFONTSCALE_150`): `'A'` is 70 ink pixels with a legible glyph, `'真'`
is a filled 12×10 rectangle, `'开'` is a filled 24×15 rectangle plus an overflow bar. Keep overlay
strings ASCII and leave the Chinese to the terminal, which has the fonts. The library does not
filter: `lines` goes to `set_texts()` unchanged, so the rule is enforced by
`tests/test_example_overlay_text.py`, which scans every `status_text()` call under `examples/` and
fails on a non-ASCII literal.

**A closed window is a disconnect.** `_sim_loop()` checks `viewer.is_running()` under the lock on
every tick. When it goes false the operator has closed the window, so the loop sets `_abort`
(which wakes a main thread blocked inside `open()` / `close()`) and breaks. It deliberately does
**not** clear `self._viewer`: closing a window and closing the viewer handle are two different
things, and detaching the reference here would remove the very evidence `disconnect()` uses to
decide whether closing the window is safe at all (rule 3). Teardown belongs to `disconnect()`.
`pump()` is the main thread's view of the same signal: it returns `False` once the connection is
gone, which is how the examples' resident loops end.

### What is *not* fixable here

On some Linux setups — Wayland with a remote-desktop session and the NVIDIA proprietary driver is
the observed one — a process that opened a MuJoCo viewer can **core-dump during interpreter
shutdown**, after all work is done and printed. This reproduces with bare MuJoCo, an inline box
model and no code from this repository, both with and without `viewer.close()`, so it is upstream
and environmental. `--no-render` is not affected and exits 0.

The symptom is confusing: the example prints `✅ 完成` and *then* the shell reports
`Segmentation fault (core dumped)`. If you see that, the run succeeded; the crash is teardown.
A non-empty `MUJOCO_GL` (`egl` or `glfw`) sometimes changes whether it happens, and a `0x502`
warning on startup is likewise only a warning.

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
its dataclasses, enums, exceptions and helpers are re-exported; if not, local field-for-field
mirrors in `_fallback.py` are used so that `MujocoGripper.get_state()` still returns a real
`GripperState`.

The SDK is not vendored, because importing it pulls in SocketCAN.

### How tightly this tracks the SDK

Coupling lives on three levels, and they do **not** all follow the SDK automatically:

1. **Data types — follows automatically.** The shim resolves each name from the SDK at import
   time, so `GripperState`, `GripperConfig`, `ErrorCode`, `DM_Motor_Type`, `Control_Mode`,
   `ERROR_DESCRIPTIONS`, `describe_error`, `STALE_AFTER_S` and the seven exception classes are
   whatever the installed SDK says they are. `isinstance(state, litegrip.GripperState)` holds.
2. **Control semantics — hand-copied, does not follow.** Controller gains, the stall window,
   the min-jerk ramp, the `grasp()` target rewrite. Those are read from the SDK source and
   reproduced by hand; an upstream change there needs a matching change here.
3. **Constants — deliberately not imported.** `constants.py` owns the simulation's numbers
   (85.452 mm, the θ endpoints) and must not inherit the SDK's defaults.

Exceptions are level 1 but with a twist worth knowing: the shim rebinds the names to the SDK's
*class objects*, not to same-named local classes. `except litegrip.LiteGripError:` has to catch
what the simulation raises, and two classes that merely share a name would not.

### The intentional divergences

`_fallback.py` diverges from the SDK in one group of values, spread over four types and
recorded in `DELIBERATE_DIVERGENCE` in the test file:

- **`GripperConfig`, `CalibrationData`, `GripperParams`, `UnitConversion`** — the same two facts
  copied into four places: `max_stroke_mm` is 85.452 (not the nominal 120), `rad_to_mm` follows
  from it, and `pos_open_rad` is `-1.14` rather than `+1.14`, because the SDK's pair violates
  its own documented invariant (the closed end must be numerically larger). See *The millimetre
  basis*. Note `pos_closed_rad` is `0.0` on both sides and so is **not** listed — the test that
  guards this list requires listed keys to actually differ.

Two naming details that are *not* divergences, listed here because they look like ones:

- **Enum class names** in `_fallback.py` are the SDK's real names — `MotorType` and
  `ControlMode` — with `DM_Motor_Type` and `Control_Mode` kept as module-level aliases. The SDK
  does exactly the same; matching it keeps `repr()` identical, and reprs of
  `GripperParams.MOTOR_TYPE` do end up in logs.
- **`home()`** targets the closed position, whereas the SDK's own clamp bug drives it *open*.
  That divergence is in `gripper.py`, not the shim, and is documented under *Known behaviour*.

Everything else is compared field-by-field against the installed SDK by
`test_mirrors_match_installed_sdk`, including field order (the dataclasses are constructible by
position) and public members such as `GripperState.is_stale`. `test_deliberate_divergences_still_diverge`
is the mirror image: it fails if someone "helpfully" aligns one of the four above with the SDK.

### State freshness

The SDK records `data_age_s` on every `GripperState` — how long ago the frame behind the
numbers arrived — and derives `has_data` and `is_stale` from it (`STALE_AFTER_S`, default
0.5 s). This matters on hardware: a disabled motor streams nothing, so `get_state()` can return
a snapshot seconds old or the pre-enable defaults, which is what `refresh_status()` is for.

Simulation has no CAN link, so both `MujocoGripper.get_state()` and `DryRunGripper.get_state()`
pass `data_age_s=0.0`: the values are computed on the spot. `is_stale` is therefore always
`False`. `refresh_status()` keeps the SDK's shape — it calls `_check_connected()` first and so
raises `NotInitializedError` when not connected — and then returns `True` without asking the
motor for anything, because there is nothing to ask.

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
- **SDK parity** — the `TestApiParity` class. `test_matches_installed_sdk` pins the 44-member
  public API snapshot, `test_mirrors_match_installed_sdk` diffs every mirrored dataclass, enum and
  constants class field-by-field against the installed SDK, and
  `test_exception_family_is_the_sdk_family` checks the seven exception names resolve to the SDK's
  *objects*. This layer is what catches upstream drift: when the SDK gains a field, a code or a
  method, these fail loudly instead of the simulation quietly missing it.
- **Calibration guard** — `tests/test_calibration.py` covers resolution (the factory file, an
  explicit path, and the refusal when neither exists), discovery, validation, the provenance
  marker and both strict conversions, against a duck-typed `LiteGrip` stand-in whose
  `load_calibration()` can be told to reproduce the SDK's silent fallback.
- **Example command-line contract** — the same file runs examples 04 and 05 as subprocesses:
  `--list-calibrations` exits 0 as a pure query, a run with no calibration to use exits 1 with
  the guidance text, and `--dry-run` never needs a calibration at all.
  Three environment details matter. `LITEGRIP_CALIB` is set to the case's own home so a
  developer's calibration cannot answer for it; `LITEGRIP_SDK_DIR` points at a stand-in SDK
  checkout; and `PYTHONPATH` points there too, because `litegrip` is a *package name* — an
  installed copy on the developer's machine would otherwise answer the library layer's
  `import litegrip` probe and supply the factory calibration the case is trying to deny
  (`no_sdk()` shadows it with a package that raises on import). The resident loops get
  `--duration`: without it they would run until Esc or a closed window, which in a test means
  forever.

The suite needs no CAN interface, no hardware and no `litegrip` SDK. Cases that want the SDK skip
themselves when it is absent.

Run it **both ways** before pushing — with the SDK installed and without. The two runs exercise
different code paths (`shim → SDK` versus `shim → _fallback`) and only the SDK-present run can see
drift:

```bash
# with the SDK
python -m pytest tests/ -q

# without it — point PYTHONPATH at src and use an interpreter that lacks litegrip
PYTHONPATH=src python3 -m pytest tests/ -q
```

If you change contact parameters, `test_grasp_force_exceeds_request` will fail and point you at
the override table in `litegrip.xml`. Update both together.
