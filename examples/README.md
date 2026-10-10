# LiteGrip MuJoCo examples

Five runnable programs that build on one another — read the model, move it in
simulation, teach the gripper a motion by hand and replay it into both sides,
then couple the two in each direction. Read this if you are about to run one of
them, or if you are looking for the checklist that has to come before a real
gripper moves.

**English** · [简体中文](README.zh-CN.md)

| File | Direction | Hardware needed? |
| --- | --- | --- |
| [`01_hello_sim.py`](01_hello_sim.py) | reads the state, moves nothing | **No** |
| [`02_move_sim.py`](02_move_sim.py) | simulation only | **No** |
| [`03_trajectory.py`](03_trajectory.py) | records on the gripper, replays into both | Yes, to record or to replay onto it (`--dry-run` runs the loop, `--play` reads a file) |
| [`04_mirror_real.py`](04_mirror_real.py) | gripper → simulation | Yes, to actually mirror (`--dry-run` substitutes a stand-in) |
| [`05_dual_control.py`](05_dual_control.py) | simulation → gripper | Yes, to actually move (`--dry-run` substitutes a stand-in) |

## Contents

- [Prerequisites](#prerequisites)
- [One opening, three notations](#one-opening-three-notations)
- [01 — hello, simulation](#01--hello-simulation)
- [02 — moving, in simulation](#02--moving-in-simulation)
- [03 — recording a motion and replaying it](#03--recording-a-motion-and-replaying-it)
- [04 — the gripper drives the simulation](#04--the-gripper-drives-the-simulation)
- [05 — simulation drives the gripper](#05--simulation-drives-the-gripper)
- [Before you drive the hardware](#before-you-drive-the-hardware)
- [Shared helpers](#shared-helpers)
- [Coming from the PyBullet examples](#coming-from-the-pybullet-examples)
- [Notes](#notes)

## Prerequisites

```bash
cd /path/to/litegrip-mujoco
python3 -m pip install -e ".[dev]"                    # examples 01–02
python3 -m pip install -e /path/to/litegrip-python    # examples 03–05
```

Run from the repository root: the package is a src layout, so
`import litegrip_mujoco` works only after the editable install. The examples
bootstrap `src/` on `sys.path` themselves, and re-exec into `./.venv/bin/python`
when one exists and `mujoco` is missing from the interpreter you started, but
neither is a substitute for the install.

The `litegrip` SDK is **not on PyPI**: `pip install litegrip` installs something
else, and no released version carries the API these examples use. All three
hardware examples take it from **one** checkout,
[`nexform-tech/litegrip-python`](https://github.com/nexform-tech/litegrip-python),
found in this order:

1. `$LITEGRIP_SDK_DIR`, pointing at the directory that *contains* the `litegrip`
  package — `<repo>/src` in a src layout, not the package directory and not the
  repository root.
2. A sibling checkout next to this repository: `../litegrip-python`, tried as `src/` and then as the repository root.
3. Whatever `litegrip` is installed in the running interpreter.

**Do not** point `LITEGRIP_SDK_DIR` at a directory that has no `litegrip`
package in it: the examples stop with an explanation rather than importing a
same-named package from somewhere else. Another repository ships a package
called `litegrip` whose `__version__` is also 2.2.0 but which has no trajectory
API, so the version number cannot tell the two apart — the startup check
(`check_sdk_api()`) can, and it names the members it needs
(`LiteGrip.record_start`, `LiteGrip.poll`, `Trajectory.load`, …) instead of
failing inside a control loop.

Bring up CAN before running 03, 04 or 05. You do not have to do it by hand: before
connecting, the three of them read the interface with `ip -details link show` and
run the privileged commands **only when its state is actually wrong** — an interface
that is already right costs no command and no password prompt. `--no-can-setup`
turns the step off.

```bash
ip -details link show can0          # what the probe reads, with -details
python3 examples/04_mirror_real.py --no-can-setup   # or manage it yourself
```

What the automatic step runs when it does have to repair, and the manual recipe:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100 fd off
sudo ip link set can0 up
ip -details link show can0
```

`restart-ms` is not optional. The kernel default is `restart-ms 0`: the controller
never leaves bus-off by itself, so one bad frame leaves the interface up, at the
right bitrate, and unable to send anything. Some USB adapters reject the option;
the examples retry that one command without it and leave the rest alone.

> **Examples 03, 04 and 05 move real hardware.** Read [Before you drive the
> hardware](#before-you-drive-the-hardware) first.

## One opening, three notations

The gripper can be described several ways, and mixing them up is the usual
source of confusion:

| Notation | Range | Meaning |
| --- | --- | --- |
| Prismatic joint | `0 … 0.042726` m | travel of *one* finger; `0` is wide open, so a larger joint value is a smaller opening |
| **Normalised opening** | `0 … 1` | **the one shared quantity**; `0` closed, `1` wide open (`frac_open()`) |
| Opening travel | `0 … 85.452` mm | this repository's millimetre scale (`get_position()`) |
| Jaw gap | `1.548 … 87` mm | physical distance between the finger faces (`gap_mm()`) |
| Simulated joint angle | `0 … -1.14` rad | what the MuJoCo model calls the joint (`get_position_rad()`) |
| Calibrated motor angle | `+1.776 … -0.064` rad | what the *hardware* reports, from the calibration file |

The last two are **not** the same scale and not a unit conversion apart: the
simulated joint runs `0 … -1.14 rad`, the real motor `+1.776 … -0.064 rad`, so
the same number on both sides means different openings. Only the normalised
opening means the same thing on both, which is why every example converts
through it.

**The examples convert between radians and the normalised opening directly, not
through millimetres**, because a calibration file carries whatever `rad_to_mm`
it was saved with. This repository's real scale is 85.452 mm over the full
stroke, while the SDK's nominal `max_stroke_mm` is 120 mm; a file written by the
host software carries neither number as a cross-check. Converting through
millimetres therefore introduces a scale that the file may not agree with. See
`_common.fraction_to_target_rad` and `_common.rad_to_fraction`.

## 01 — hello, simulation

Reads the model and moves nothing. No hardware, no CAN, no SDK — and no motion
call anywhere in the file: `sim.pump()` only pumps the window's events and
advances the clock, so the jaws stay where they are however long you leave it.
The claim is checked rather than promised:

```bash
grep -n "command_fraction\|command_joint\|set_frac_open\|settle(\|reset(\|goto(\|grasp(\|send_mit_frame" examples/01_hello_sim.py
# (no match; `frac_open()` appears, which only reads)
```

1. **The model constants** — the finger count, the per-finger stroke, the jaw
  opening the bundled MJCF actually has, and the force cap in force.
2. **The current opening, in three notations** — the normalised fraction, the millimetre travel and the jaw gap, side by side.
3. **A live status line** — with a window open it refreshes until you press **Esc** or **Q**.

```bash
python3 examples/01_hello_sim.py                # window, full demo
python3 examples/01_hello_sim.py --headless     # no window, exits 0
python3 examples/01_hello_sim.py --scene        # the demo scene instead of the bare gripper
```

## 02 — moving, in simulation

The same gripper, now driven, still with nothing but MuJoCo:

1. **Speed-limited travel** — a full stroke takes ~1.05 s, measured on this
  machine (below): each finger ramps at 42.73 mm/s, and because the two fingers
  close on each other the *gap* changes at twice that — the hardware's rated 85
  mm/s.
2. **Midpoint positioning** — command a normalised opening, and `settle()` waits for the jaws to actually arrive.

```bash
python3 examples/02_move_sim.py                 # window, full demo
python3 examples/02_move_sim.py --headless      # no window, exits 0
python3 examples/02_move_sim.py --speed 0.02    # slow: ~2 s for a full stroke
python3 examples/02_move_sim.py --force 20      # 20 N grip cap
python3 examples/02_move_sim.py --scene         # adds a part to grasp (§3)
```

`--speed` is one finger's speed in m/s, so the default `0.04273` is the rated 85
mm/s of *gap* change. The measured timings, verbatim from
`python3 examples/02_move_sim.py --headless`:

```text
   → 闭合：用掉 1.050 s 仿真时间 · 到位
   → 张开：用掉 1.057 s 仿真时间 · 到位
   命令  50.0% → 实测  50.0% · 开口 44.26 mm · 0.556 s · 到位
   命令  25.0% → 实测  25.0% · 开口 22.90 mm · 0.300 s · 到位
   命令  75.0% → 实测  75.0% · 开口 65.65 mm · 0.557 s · 到位
```

**Do not** reach for `--object-mm`, `--hold`, `--pull` or `--slip` here: this
example has no part to grasp or pull on, and argparse rejects those flags with
exit 2 rather than ignoring them. The PyBullet repository's `03_grasp.py`
carried them; this repository has no such example, and its `--scene` section is
the whole of the grasp demonstration.

**Do not** read `grasp()` as "apply this force". §3 measures what it actually
does — see [What `grasp()` really does](#what-grasp-really-does).

## 03 — recording a motion and replaying it

Teach the gripper a motion by hand, save it, and play it back into the hardware
**and** the window at the same time:

1. **Record.** The motor drops into zero gravity — its force is switched off and
  you push the fingers through the motion yourself — while the SDK samples the
  opening at 100 Hz in the background. Press **Enter** / **Space** to end the
  recording, or pass `--record 6` to stop by itself; **Esc** / **Q** means
  "discard this one" and saves nothing. The window follows your hand as you go.
2. **Save.** The recording goes to `~/.litegrip/trajectories/` as a `.lgt` named
  after the example and the time, so a second recording cannot overwrite the
  first. Nothing is written into this repository: a trajectory is data measured
  on one machine.
3. **Confirm.** The recording does not run on into the replay by itself: you
  press **Enter** / **Space** to start it, or **Esc** / **Q** to leave it
  unplayed for now. That is what gives you the moment to get your hand out of
  the travel before the motor starts.
4. **Replay.** The same trajectory drives the motor and teleports the simulated
  fingers onto its **measured** position, so what the window shows is what the
  gripper is doing. The fingers stop where the trajectory ends and do not return
  to the start by themselves.

```bash
python3 examples/03_trajectory.py --calib ~/.litegrip/litegrip_calibration.json
python3 examples/03_trajectory.py --record 6 --calib ~/.litegrip/litegrip_calibration.json
python3 examples/03_trajectory.py --dry-run --calib ~/.litegrip/litegrip_calibration.json
python3 examples/03_trajectory.py --play 03_hand_taught-20260929-120000
python3 examples/03_trajectory.py --play 03_hand_taught-20260929-120000 --real
python3 examples/03_trajectory.py --play 03_hand_taught-20260929-120000 --speed 0.5 --headless
```

`--dry-run` needs no hardware at all: the simulation stands in for the gripper,
the main loop walks a scripted opening curve, and the background recorder
samples it at 100 Hz exactly as it would sample the motor. Saving, reading back
and replaying are all exercised; only the CAN traffic is missing.

The trajectory stores the **normalised opening**, not an angle, so a motion
taught on one gripper replays on another, and the file replayed through a
different machine's calibration still means the same opening. A bare name is
resolved inside `~/.litegrip/trajectories` with `.lgt` appended; a path with a
`/` in it is used as written.

`--play` on its own touches nothing: no connection, no enable, no frame. It
reads the file and drives the window from it — the one path in this example that
runs without a gripper. Add `--real` to send the same trajectory to the hardware
as well. The replayed recording carries the `can_id` and mount it was made with,
and this example prints them and then ignores them: the conversion uses *this*
machine's calibration, because an opening means the same thing everywhere and an
angle does not.

Replay commands **position**, not force. The recorded torque is kept in the file
as a diagnostic and is never fed forward, so a squeeze that was recorded against
a part replays as a position path that presses with whatever `kp` yields — the
grip force you taught is not preserved. For a repeatable grip, replay the motion
and then call the SDK's `grasp(force_n=...)`.

**There is no mouse click here.** The PyBullet example starts the replay when
you click the window; MuJoCo's `launch_passive` delivers key callbacks and no
mouse events, so this example asks for **Enter** / **Space** instead. The
confirmation step itself — the thing that keeps the replay from following your
hand into the travel — is unchanged.

### Between the phases, this example feeds the motor itself

`record_stop()` puts the motor back under closed-loop control and then stops:
the SDK's recorder was the only thing streaming frames, and the replay has not
started yet. An enabled motor that hears nothing latches the communication-loss
fault (0xD) — see [Why "just watching" still has to send
frames](#why-just-watching-still-has-to-send-frames). So this example streams
"**locked at the measured position**" hold frames at 200 Hz through every gap:
after `enable()`, between the recording and the replay (which includes the wait
for your confirmation — the longest unfed gap in the run), and after the replay
ends until you quit. It commands no motion; the target *is* the position the
motor reports, with zero feed-forward.

**Do not** stream frames of your own while a recording or a replay is running.
Both are SDK background threads streaming the bus, and a second stream on the
same wire tears the trajectory apart. The example's hold frames are therefore
strictly between phases, never inside one.

`--dry-run` is the exception, and it is not a hold stream at all: a stand-in has
no bus, so there is nothing to keep alive and the example sends no frames.

## 04 — the gripper drives the simulation

The hardware is the source of truth; the simulation is a display. Each frame
reads the hardware position once and teleports the simulated fingers onto it
with `set_frac_open()` — pure kinematics, no dynamics — so the window shows
where the hardware is right now, with no lag and no drift of its own.

Two ways to use it:

- **Push it by hand** (`--zero-gravity`, recommended): the motor goes slack and
  you can move the fingers yourself. The window follows your hand. Press **Z**
  while running to toggle slack/enabled.
- **Watch another program drive it** (`--passive`): this example connects,
  reads, and leaves the feeding to that program — it does not enable the motor
  and sends nothing at all.

```bash
python3 examples/04_mirror_real.py --zero-gravity --calib ~/.litegrip/litegrip_calibration.json
python3 examples/04_mirror_real.py                  # mirror (hold frames keep it alive)
python3 examples/04_mirror_real.py --passive        # read only, let someone else feed it
python3 examples/04_mirror_real.py --headless       # terminal readings only
python3 examples/04_mirror_real.py --duration 10    # stop after 10 s
python3 examples/04_mirror_real.py --dry-run --duration 10   # no CAN, scripted stand-in
```

Like 05, this reads a calibration file on every run. With no `--calib` it uses
the factory calibration shipped inside the SDK package — see [Before you drive
the hardware](#before-you-drive-the-hardware).

`--dry-run` swaps in a `DryRunGripper` and drives it from a background thread
that walks `open() → close() → goto(42.7 mm) → grasp(10 N)` on a loop, so the
mirror has something to follow. The θ → opening conversion is the same code path
as with the hardware. It sends **no frames at all**, which is not the "stay
quiet" of `--passive` but a property of the stand-in: it has no bus, so it
cannot latch 0xD and nothing needs feeding. The Z toggle is therefore refused
there, with a printed reason, instead of appearing to work and doing nothing.

### Why "just watching" still has to send frames

An **enabled** motor that hears nothing for about **0.9 s** latches the
communication-loss fault (0xD) — a blinking red LED, positions that still read,
and commands that are silently ignored. The SDK measured that 0.9 s on this
hardware; the `TIMEOUT` register (RID 9) disagrees with it (8000 ms on one read,
0 on another) and is marked unresolved, so the timing here follows the
measurement. Watching the window without feeding the motor is enough to wedge
the gripper.

So the default mode is not read-only: it sends "**locked at the measured
position**" hold frames at 200 Hz — target where the fingers already are, zero
velocity, zero feed-forward. It commands no motion; it just gives the fingers
stiffness and keeps the watchdog fed. To back-drive it instead, add
`--zero-gravity` (or press Z), which streams the same way with kp/kd zeroed.

Use `--passive` when a **different program** is driving the hardware: this
example then sends no frames, does not enable the motor, and disables the Z key,
so the two never fight over the bus. The cost is that the other program has to
feed the motor itself, or it latches 0xD about 0.9 s later anyway. **Do not
combine `--passive` with running this example alone.**

`--zero-gravity` (or Z) makes the gripper *soft*, so the fingers can be
back-driven and can also sag under gravity. Support the gripper before enabling
it.

Quitting **disables** the motor (0xFD) rather than sending one last relock
frame: a single frame has nothing after it, so an enabled motor goes quiet and
latches 0xD within the second — with nobody left to clear it. The gripper
therefore goes limp on exit and the fingers may drift under their own weight.

## 05 — simulation drives the gripper

The window is two things at once: the keyboard *commands* the hardware, and the
window *shows* where the hardware is. Enabling the motor does not move anything
— the example reads the position you are already at, shows it in the window and
holds there, and the hardware only moves once you press an arrow key.

| Key | Effect |
| --- | --- |
| **←** / **→** | target opening, one step of 5 % per press (`aperture_down` / `aperture_up`) |
| **↓** / **↑** | speed limit, one step of 10 % per press |
| **-** / **=** | grip force, one step of 5 N per press |
| **Space** | stop: hold the current target, sending no further motion |
| **H** | back to fully open |
| **Esc** / **Q** | quit |

1. Press an arrow key. The keypress *is* the command: there is no send key. The
  hardware ramps toward the target at 200 Hz, and the simulated jaws mirror its
  **measured** position, so they lag the target rather than jumping to it. That
  gap is the tracking error, and it stays open while the fingers are blocked by
  a part — which is how you tell you have gripped something.
2. Change the **speed** at any time, before or mid-move. It caps how fast the
  position target may grow, as a percentage of the fingers' rated 85 mm/s. 100 %
  means "at most 85 mm/s"; values above 100 % are treated as 100 % and the
  example says so.
3. Change the **force**. It becomes the feed-forward torque sent with the frame,
  and it is only applied in the closing direction (a grip force on an opening
  move would fight the motor) and only once the target is reached.
4. Press **Esc** or **Q** to quit. Frame sending stops and the motor is
  **disabled** (0xFD): the fingers go limp and a gripped part will drop, but no
  communication-loss fault is left latched on the motor.

> **There is no confirmation step.** A keypress *is* a command, so one arrow key
> is a real command to a real motor. Expect the gripper to move the moment you
> touch it, and keep the travel clear.

```bash
python3 examples/05_dual_control.py --calib ~/.litegrip/litegrip_calibration.json
python3 examples/05_dual_control.py --dry-run         # window only, never touches CAN
python3 examples/05_dual_control.py --speed 40        # start the speed limit at 40 %
python3 examples/05_dual_control.py --force 20 --channel can1
```

Between presses the example keeps streaming the current position at 200 Hz. That
is not a keep-alive nicety: an enabled motor that hears nothing for about 0.9 s
latches a communication-loss fault (0xD), so standing still is the thing that
fails.

**This is where the MuJoCo version differs most from the PyBullet one.** That
version puts three sliders in the PyBullet window; MuJoCo's viewer has no widget
you can add to a running simulation, so the commands are discrete keypresses and
the current target, speed and force are echoed in the window's status text and
on the terminal. A press moves the target by one fixed step instead of dragging
it continuously. Note this is a deliberate divergence, not a defect.

`--dry-run` runs the same loop with no CAN traffic and no measured position: the
window follows the *commanded* opening instead of mirroring the hardware, the
status line says `dry-run` to remind you, and a stand-in `DryRunGripper` answers
in the hardware's θ scale so the conversion path is the one the real gripper
would take.

`--headless` is refused unless `--dry-run` is also given: the keyboard is the
input device, and a headless run has none. In `--dry-run --headless` the hands
are replaced by a script that walks four targets on a 10.5 s cycle, which is how
CI and this document's own transcripts exercise the drive → keep-alive handoff.

### Why it ramps

The MIT position term is `kp × (q_target − q_actual)`, and `kp` is an entry in
the calibration file rather than a constant. At the SDK's default
`kp = 100 Nm/rad`, sending the target as a step asks for
`100 × 1.845 rad ≈ 185 Nm` from a motor rated around 10 Nm; at this machine's
present `kp = 5.0` the same step
asks for about 9 Nm, inside the rating. The current saturates and the motor
latches an under-voltage/over-current fault, after which it keeps reporting its
position while ignoring every command — the blinking red LED and "it reads but I
can't control it" symptom.

The ramp is kept because it does not depend on that number: it bounds how far
the *position target* may jump in one frame, whatever `kp` is configured to.
Each frame advances one tick from where the motor *is*, so a single frame
demands well under the rated torque. The SDK's own `goto_rad` ramps for the same
reason.

This is also why the speed limit is a limit on the *target*, not on the
keyboard. Holding an arrow key down, or pressing it as fast as the terminal
delivers, still costs the motor one bounded increment per frame — the key repeat
rate never reaches the bus. The example prints that increment when it starts:

```text
   位置目标每帧最多走 0.009153 rad（= 速度 × 5 ms），200 Hz 发帧——按得再快也不会变成一条阶跃指令
```

### Why it slows down at the end

The ramp reaches the target *and then the fingers move back a little*, most
visibly when opening. The cause is the velocity field of an MIT frame: `dq` is a
**target** velocity, not a measurement, so the frame that drops it from full
speed to `0` reverses the damping term into a torque step of `kd × v`. At this
machine's `kd = 2.0` and a full-speed `1.39 rad/s` that is about `2.8 Nm`;
absorbing it needs `2.8 / kp = 0.56 rad` of position error at `kp = 5.0`, which
is 40 % of the travel — more than the position loop can find, so the mechanism
recoils to rebuild it.

Example 05 therefore decelerates before it arrives: while the remaining distance
is short, the commanded speed is `√(2·a·remaining)`, with the deceleration
`a = full speed / 0.15 s` (`RAMP_DOWN_S`). The speed is never *above* the rate
limit, so the limit
stays a hard bound; it only makes the last few frames slower, and the frame
before the stop is under 7 % of full speed — about `0.17 Nm` of reversed damping
instead of `2.8 Nm`. The whole move costs at most `RAMP_DOWN_S` more than
walking the distance at the rate limit.

The SDK's own `_move_at_speed_rad` does **not** do this — it holds `dq` at the
full speed and then sets it to `0` on the frame it stops — so the same recoil
appears when a script drives the gripper with the SDK directly.

### If a wedged gripper reads but won't move

```bash
python3 examples/05_dual_control.py --status                  # read only, no motion command
python3 examples/05_dual_control.py --status --clear-fault    # clear the latched fault
```

`--status` opens no window, does not enable the motor and **sends no motion
command at all** — it reads the error code and translates it, so it is safe to
run while the gripper holds a part or is in someone's hands. Reading the DM
registers does put read requests on the bus, which is not the same as sending no
frames: it is the *motion* commands it never sends.

An unpowered, unenabled motor sends no status frames, so `--status` there cannot
read a position at all. That is reported as "cannot tell" with **exit 0**, not
as a fault: it skips the position and error-code lines rather than printing the
SDK's `0.0` initial value as if it were a measurement, and still reads the
registers. Faults are only determinable from a status frame, so run the example
without `--status` if you need that judgement — it enables the motor, and an
enabled motor streams frames by itself.

Only `--clear-fault` sends frames, and those are all zero-torque; but the SDK's
clear sequence is disable → clear → enable, so the motor goes limp for an
instant and the fingers may drift under their own weight. Support the gripper
first.

The fault is latched: it will not clear itself until the gripper is
power-cycled.

### Two faults look identical and are not

| Code | Meaning | What triggers it |
| --- | --- | --- |
| 0x9 / 0xA | under-voltage / over-current | a step command: ~185 Nm in one frame on a ~10 Nm motor at the default kp, ~9 Nm at this machine's 5.0 |
| **0xD** | **communication loss** (`通讯丢失 (CAN 超时)`, named by the SDK's own `describe_error`) | an enabled motor left silent for about 0.9 s — including "just watching with the window open" |

0xD is the motor's CAN watchdog, and it is measured rather than configured: **an
enabled motor latches it after roughly 0.9 s of silence** (the SDK's own figure,
taken on this hardware). **Idling is itself the fault cause.** This example
therefore keeps sending hold frames at 200 Hz through its idle periods (target =
measured position, zero feed-forward, no motion commanded) — see `IdleKeeper`.
An earlier revision sent nothing, so the quiet stretch after `enable()` — its 50
ms hold stream ends, then the window starts up and the operator looks at it for
a few seconds — went past the watchdog and wedged the motor. That, not the ramp,
was the main cause of "simulation can read the gripper but not control it."

Do not derive that duration from the `TIMEOUT` register (RID 9): it reads 8000
ms on one run and 0 (no watchdog) on another, and neither agrees with the
measured 0.9 s, so the SDK marks it as unresolved. `--status` prints the
register's live value for the record and says as much next to it. Those lines
are these — transcribed from the source, since **this path has never been run
here** (no CAN hardware); the register *values* show the shape of the output and
are not a measurement:

```text
[2] 读寄存器（只发读请求）
   TIMEOUT=0 · CTRL_MODE=1 · UV_Value=15 · OC_Value=0.8 · OT_Value=100
   通信超时保护（TIMEOUT, RID 9）= 0（这个寄存器当前不生效）
   别拿这个寄存器当依据：实测**使能态**的电机静默约 0.9 s 就锁 0xD 通信丢失故障，与寄存器读数对不上（这台机器读到过 8000，也读到过 0），SDK 自己把这条标成「待查」。
      所以 04/05 空闲时照 200 Hz 持续发帧，不赌这个数字；只读不喂帧（或跑了别的只读脚本）同样会把它看哑。
```

A **second master** on the same bus makes the symptom messier still: the two
streams fight and neither side wins. Before running a hardware example, check
nothing else (say `litegrip_console --backend real`) is on the same CAN
interface:

```bash
pgrep -af python3 | grep -i litegrip        # who has CAN open
ip -details -statistics link show can0      # busy bus? counters climbing while you run nothing means someone else is talking
```

`ip link set can0 down` / `up` does **not** evict that program — its socket
survives and it resumes when the interface returns. The process has to exit.

The example drives the CAN loop itself, with the SDK's public `send_mit_frame()`
and `poll()`, rather than calling `move_to()`/`goto_rad()`. Those run
`control_mit_stream()` internally, which sleeps in its own loop and never yields
— the window would freeze and keystrokes would go unread. The public frame-level
API exists for exactly this case.

## Before you drive the hardware

All three hardware examples print a safety banner and are meant to be run with
the gripper in hand or clamped to a bench, **with the travel clear**, and the
power switch within reach. The first run of 05 should be `--dry-run`.

A first real run of 05 looks like nothing happening, and that is correct: the
motor is enabled, the window shows where the gripper already is, and it stays
there until you press an arrow key. Keep the fingers clear the whole time — the
motion starts on the keypress, not on a confirmation.

**Only run 03 with your hands where they should be.** Recording means the
motor's force is off and your hand is on the fingers, so nothing can pinch you —
the risk there is that you push the fingers somewhere they cannot go, or that
the recording starts before you are holding it. Replay is the opposite: the
motor is under closed-loop control and follows the trajectory at whatever `kp`
and `kd` the calibration carries. And the end of a recording does **not** run
into the replay: nothing moves until you press Enter / Space, which is the
moment to take your hand out of the travel — so do not press it while your
fingers are still in it. Keep clear of the travel during the replay, and
remember that the fingers stay at the end of the trajectory, at force, until you
quit or the example is stopped. **Do not** run a replay in front of someone who
is not expecting the gripper to move: the keypress you make *is* the replay, and
the window shows the motion at the same time as the hardware does it.

Confirm the CAN interface before anything moves:

```bash
ip -details link show can0
```

Read it with `-details`. The flag list alone cannot tell a working interface from a
bus-off one: both print `UP,LOWER_UP` and the right bitrate. Only `can state` tells
them apart, and a bus-off controller sends nothing at all — which is what
`使能失败: [Errno 100] Network is down` usually means. It is a host link problem,
not a gripper one: a CAN socket binds happily on a down interface, so `connect()`
succeeds and the failure only shows on the first frame. The examples now probe the
interface before connecting and repair it only when it is wrong — the recipe is near
the top of this document. With `restart-ms 0` a bus-off interface stays broken until
something reconfigures it.

Do not run the examples against an interface another program is using: the repair
step does not drive off a second master already on the bus, and the SDK does not
share it.

### Which calibration is in use

Every path that touches the hardware starts by resolving one. `--calib <path>`
uses this gripper's own file:

```bash
python3 examples/05_dual_control.py --calib ~/.litegrip/litegrip_calibration.json
python3 examples/05_dual_control.py            # no --calib: the SDK's factory file
```

With no `--calib`, the calibration shipped inside the SDK package
(`factory_calibration.json`, alongside the `litegrip` package's `__init__.py`)
is used, and the example says so when it starts. The path is resolved from the
package directory, so it follows the checkout to any machine. **That file holds
the factory's bench-fixture measurements, not measurements of your gripper**:
its travel endpoints may not match the unit in front of you. Pass `--calib` for
anything beyond a first look.

Nothing is scanned and nothing is offered to pick from. A run reads exactly one
file: the one named by `--calib`, else the SDK's factory file. When neither can
be read it stops with exit 1 and prints the `--calib` form to use — it never
guesses among the JSON files lying around on the machine.

`--list-calibrations` is how you find a file without guessing. It is a query
that prints the candidates and their key values (closed/open angles,
`rad_to_mm`, `kp`, `mst_id`), then exits 0:

```bash
python3 examples/05_dual_control.py --list-calibrations
```

The `*.sim.json` file the studio writes for its simulator backend and the
`*.bak` backups are never offered, and a `*.sim.json` named explicitly is
refused — its scale belongs to the simulated gripper.

`--dry-run` moves no hardware and does not need a calibration at all: the
stand-in reports its own angles, and what it never reads it cannot get wrong.
Pass `--calib` anyway if you want the run to say which file the real path would
have used. `03` resolves the same way; `04` and `05` share
`open_real_gripper`.

The file comes from calibrating *this* gripper in the host software
(`litegrip-studio` / `litegrip-console`, or the SDK's own
`tools/gui/litegrip_gui.py`) and saving it. That matters because the SDK's
`load_calibration` loads the shipped factory calibration *silently* when the
path it was given cannot be read, and still returns `True` — so a mistyped path
would otherwise drive the motor with another machine's angles. The examples
therefore read the file themselves and check it took effect field by field, and
they refuse a file whose `can_id`/`mst_id` name a different motor. They also
check the angles are self-consistent: the SDK's factory defaults ship an opening
angle that contradicts its own `goto()` convention, and an uncalibrated unit is
stopped with an explanation rather than driven with meaningless angles.

### Not verified

There is no CAN hardware on the machine these examples were written on.
Everything in 03, 04 and 05 that talks to the gripper — `connect()`, `enable()`,
the record and replay streams, the frame sending, `--status`'s register reads,
`--clear-fault` — is written from the SDK's own source and the PyBullet
repository's measurements, and has **not** been run against a gripper here. The
`--dry-run`, `--play` and simulation paths in this file were run, and their
transcripts come from those runs. Confirm the hardware paths on a bench before
trusting them.

Of the CAN probe, only the reading half is verified: the parser is pinned against
real `ip -details link show` transcripts and `probe_can_link("can0")` was run
read-only against a live interface. The **repair** half — the `sudo ip` sequence
that reconfigures an interface — has only ever been run against a stand-in, so
nobody has watched it bring a wrong interface up.

## Shared helpers

[`_common.py`](_common.py) is imported by all five examples and is not an
example itself. It holds the argument parsers (`add_common_args()`,
`add_hardware_args()`), the SDK discovery (`import_litegrip()`, `sdk_dir()`,
`check_sdk_api()`), the calibration resolution (`choose_calibration_file()`,
`factory_calibration_path()`, `is_sdk_factory_calibration()`, the candidate
listing, the "did the file actually
take effect" checks), the connect/enable sequence (`open_real_gripper()`),
the CAN link probe (`ensure_can_link()`), `make_sim()`, `fresh_state()`, the unit
conversions and the status line. Each
example therefore starts with `from _common import ...` *before* importing
`litegrip_mujoco`.

`ensure_can_link(channel, repair=...)` runs before the gripper is built, so a wrong
interface fails before a socket is opened. A `repair=False` call only reports:
05's `--status` and 04's `--passive` pass that, because their whole point is to
observe, and reconfiguring the interface underneath them would delete the evidence
they exist to collect. **Do not** treat a repair failure as fatal — it prints what
it read and returns `False`, and the connect attempt right after it is what actually
decides whether the link works.

`fresh_state(gripper, timeout_s=...)` is the only way to read a real position:
`poll()` returns `True` only when a status frame for our motor was decoded in
that call, so the snapshot taken straight after it is a measurement. **Do not**
read `get_state(wait=False)` directly and treat it as a position — with no frame
behind it, it returns the SDK's `0.0` initial value, which looks like a reading
and is not one. There is deliberately no second freshness gate: this SDK exposes
no public "is this snapshot backed by data" flag, and `poll()` is the whole
signal.

**Do not** wait for a status frame on a silent bus from the thread that also
sends the frames. The DM motor answers each command frame and sends nothing on
its own — the SDK waits for its own enable acknowledgement by streaming
zero-gain frames for exactly that reason. A bare `poll(timeout_s=...)` inside a
loop that stops sending while it waits is therefore waiting for a reply nobody
triggered. Example 05 did this on every keypress that changed the target: 50 ms
of silence per key, then a rejection, and the hardware never moved.
`wait_fresh_while_feeding(gripper, keeper)` in [`05_dual_control.py`](05_dual_control.py)
waits in 5 ms slices and re-sends the keepalive frame between them, which keeps
the replies coming without inventing a new target.

`make_sim(headless)` is this repository's replacement for `pkg.connect(GUI)`: it
builds a `MujocoGripper` over the bundled MJCF (`litegrip.xml`, or `scene.xml`
with `--scene`) and turns rendering off for `--headless`.

## Coming from the PyBullet examples

The two repositories tell the same five stories with different libraries —
`litegrip-pybullet` drives `pybullet`, this one drives MuJoCo through
`MujocoGripper`, which mirrors the `litegrip.LiteGrip` SDK class rather than
exposing a simulation-specific API. The scenario, the section numbering and the
terminal output match; most names do not. The mapping:

| PyBullet | MuJoCo | Note |
| --- | --- | --- |
| `GripperSim(...)` | `make_sim(headless)` → `MujocoGripper` | `connect()` / `disconnect()` still bracket the run |
| `sim.step()` | `sim.pump()` | pumps events and advances the clock; returns `False` when the window is closed |
| `sim.reset_fraction(f)` | `sim.set_frac_open(f)` | a kinematic teleport *that also sets the controller target*, so the finger is held against gravity |
| `sim.fraction()` | `sim.frac_open()` | the normalised opening |
| `sim.aperture_mm()` | `sim.gap_mm()` | the jaw gap, not the millimetre travel |
| `sim.run_for(s)` / `sim.settle()` | `sim.settle(...)` | `settle()` returns `(elapsed_s, reached)` |
| `sim.grasp_center()`, `contacts()`, `add_box()` | same names | `add_box()` reuses one of the four spare MJCF slots |
| `SliderDrive` | `KeyboardDrive` | the ramp and the `RAMP_DOWN_S` deceleration are identical |
| three sliders in the window | arrow keys and a status-text legend | MuJoCo's viewer has no widget API |
| `mouse_events()`, `clicked()` | same names | kept as dummies: they return `[]` and `False` always, because `launch_passive` delivers no mouse events (03 asks for Enter / Space instead) |
| `sim_motor_params()` | — | never added: `--status` is reachable only on the real-hardware path, so the SDK's own `read_param()` is the only thing ever called |
| `--no-mirror-first` | — | dropped: it belonged to the old dual-gripper design and has no meaning here |

Two behaviour changes to expect if you run the old files from memory:
calibration refusals now exit **1** (`SystemExit(message)`), not 2 — 2 is left
to argparse's own usage errors — and 05's `--headless` is refused unless
`--dry-run` is given, because the keyboard is the input device.

## Notes

### What `grasp()` really does

`grasp(force_n=N)` sets the **feed-forward torque** to `N`; it does not close a
force loop. The position loop stays engaged, the part pushes back, the target
and the measurement end up a little apart, and `kp × Δθ` is added on top of the
feed-forward. Measured on 2026-09-30 with
`python3 examples/02_move_sim.py --headless --scene --force 10`:

| Requested | `get_force()` reports | Overrun |
| --- | --- | --- |
| 10 N | 15.51 N | +5.5 N |
| 0 N | 6.2 N | +6.2 N |

The contact force at the finger faces reads the same 15.51 N over 4 contact
points per side, so the model and the reported number agree; it is the *request*
that is a baseline rather than a setpoint. Whether the hardware overshoots the
same way is **not verified** — there is no CAN hardware on this machine.

### Where the grasp centre is

`sim.grasp_center()` returns `[0.0, 0.0, 0.096735]` m in the bare gripper model
— the midpoint between the two finger faces, and the point a part should be
placed at. Measured with the bundled `litegrip.xml`; the number moves with the
model, so read it rather than hard-coding it.

**Do not** read the jaw opening from `link_aabb()` or `pad_aabbs()`: those are
*collision* boxes, deliberately coarse (see "Why collision uses boxes, not
meshes" in the developer guide), and they are not the gap. Use `gap_mm()` for
the opening and `contacts()` for what is touching what.

### 03 connects through its own helper

`_common.open_real_gripper()` imports the SDK itself, but 03 imports it in
`main()` first so it can run the API check before anything else happens, and
then passes that module to its own `open_gripper()`. The calibration, connect,
verify and enable steps are the same ones; only the module's origin differs.

### Why the window text is ASCII

The viewer draws `status_text()` with MuJoCo's built-in bitmap font, and that
font has no CJK glyphs: each Chinese character comes out as a solid box, so a
line of Chinese is a row of blocks. Measured on this machine through the same
path (`mjr_overlay`, `mjFONTSCALE_150`): `'A'` is 70 ink pixels with a legible
glyph, `'真'` is a filled 12×10 rectangle, `'开'` is a filled 24×15 rectangle
plus an overflow bar.

So the overlay text is English and the terminal output is Chinese — a terminal
has CJK fonts, the viewer does not. `tests/test_example_overlay_text.py` scans
every `status_text()` call in `examples/` and fails on a non-ASCII literal.

### A window that ends in `Segmentation fault`

If a run that opened the viewer ends with `Segmentation fault (core dumped)`
**after** printing its closing line, the run itself succeeded — that is an
upstream GL teardown crash at process exit, not a simulation failure.
`--headless` / `--no-render` is unaffected. See the developer guide.
