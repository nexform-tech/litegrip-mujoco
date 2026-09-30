"""Trajectory record and replay for the simulated gripper.

Read this if you want to capture a motion in the simulation and play it back:
the approach path that seats a part, the squeeze profile you tuned, the wiggle
that shakes it loose. ``MujocoGripper.record_start()`` puts the motor into
zero gravity and samples what the jaws do; ``MujocoGripper.play_start()``
streams the result back as position commands.

The file format is the SDK's, byte for byte
-------------------------------------------

This module is a port of ``litegrip.trajectory``
(``nexform-tech/litegrip-python``), not a look-alike: ``_MAGIC``, ``_VERSION``,
``_HEADER`` and ``_SAMPLE`` are copied verbatim, and so is every validation rule
in :meth:`Trajectory.from_bytes`. A ``.lgt`` written here loads in the SDK and
the other way round, which is what lets a motion taught on real hardware be
replayed in the simulation. ``tests/test_sim_extras.py`` checks that mutual read
against the installed SDK whenever it is importable.

Why a recording stores ``openness`` and not radians
---------------------------------------------------

Each gripper has its own zero, direction and calibration (one unit opens toward
-1.42 rad, another toward +1.14 rad), so a raw angle is meaningless on a
different unit. A sample therefore carries the opening normalised by the
*recording* unit's travel -- dimensionless, direction-free -- and replay
converts it back with the *local* calibration. A trajectory taught on a
normal-mount gripper replays correctly on a reverse-mounted one. The recorded
angle, velocity and torque are kept as diagnostics only.

Where this port deviates, and why
---------------------------------

* **No bus, so no keep-alive frame.** The SDK's zero-gravity recorder streams a
  zero-torque frame every cycle because a DM motor self-locks a comm-loss fault
  about 100 ms after the frames stop. A simulation has no bus and no such fault;
  zero gravity here is a controller state that stays set until it is cleared, so
  :class:`TrajectoryRecorder` sets it once at :meth:`~TrajectoryRecorder.start`
  and clears it at :meth:`~TrajectoryRecorder.stop`.
* **No way to push the jaws by hand.** ``mujoco.viewer`` consumes the mouse for
  camera control and reports no drag events, so a human cannot back-drive the
  fingers the way they can on a slack real gripper. A recording taken here is
  therefore normally ``zero_gravity=False`` with a script driving the gripper
  from another thread -- the SDK documents the same mode for capturing a
  programmatic move.
* **No calibration gate.** The SDK refuses to record without a calibration
  because the normalised opening would be a guess. The simulated endpoints come
  from the URDF (or from the file passed to ``load_calibration()``), so they are
  known, and there is nothing to guess. :func:`~litegrip_mujoco.calibration.is_calibrated`
  treats a simulated device as calibrated for the same reason.
* **No timing seams off a config object.** The SDK reads ``sleep_fn`` and
  ``monotonic_fn`` from ``gripper.motion_config``. ``MujocoGripper`` has no such
  object, so both classes take them as explicit arguments and default to
  ``time.sleep`` / ``time.monotonic``.

What replay does not reproduce
------------------------------

Replay commands **position**, not force. The opening is clamped to the local
calibrated travel, and the recorded torque is never fed forward, so a squeeze
that was recorded against an object replays as a position path that presses with
whatever ``kp`` yields -- the grasp force you taught is *not* preserved. For a
repeatable grip force, replay the motion and then call
:meth:`~litegrip_mujoco.gripper.MujocoGripper.grasp` with an explicit
``force_n``.

Traps this module works around
------------------------------

* A file whose length does not match the sample count in its header is rejected,
  not parsed into half a trajectory.
* A sample whose timestamp did not advance is not stored, and a clock that will
  not advance aborts either loop -- rather than filling a "successful" recording
  with rows that all claim the same instant, or replaying forever.
* A blocking capture that did not fill raises and reports how many samples
  landed, rather than handing back a short recording as if it were whole.
"""

from __future__ import annotations

import logging
import os
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, List, Optional

from ._litegrip import LiteGripError

log = logging.getLogger("litegrip_mujoco.trajectory")

__all__ = [
    "DEFAULT_PLAY_RATE_HZ",
    "DEFAULT_RATE_HZ",
    "Trajectory",
    "TrajectoryBusyError",
    "TrajectoryEmptyError",
    "TrajectoryError",
    "TrajectoryFormatError",
    "TrajectoryNotActiveError",
    "TrajectoryPlayer",
    "TrajectoryRecorder",
    "TrajectoryRecordingError",
    "TrajectorySample",
    "resolve_path",
    "trajectory_dir",
]

#: Default sampling rate for a recording. Fast enough to keep the shape of a
#: motion, slow enough that a physics step and a state read always fit in the
#: 10 ms budget. Same number as the SDK's, so a recording made here and one made
#: on hardware are paced alike.
DEFAULT_RATE_HZ = 100.0

#: Default frame rate for replay. The SDK takes this from
#: ``MotionConfig.frame_interval`` (200 Hz by default); this is that number.
DEFAULT_PLAY_RATE_HZ = 200.0

#: Two clock readings closer together than this count as "the clock did not
#: advance" -- see :data:`_STALL_CYCLES`. One nanosecond, because the target
#: platform is Linux, where ``time.monotonic`` has nanosecond resolution: a
#: running loop always moves further than this between two cycles, and a stopped
#: clock never moves at all.
_CLOCK_EPS = 1e-9

#: How many consecutive non-advancing cycles abort a sampling or replay loop.
#: A real monotonic clock never does this -- five cycles of Python cannot land
#: inside one nanosecond -- so it only ever fires on a clock that has stopped,
#: where without the guard the loop would spin forever.
_STALL_CYCLES = 5


# -- file format ----------------------------------------------------------
#
# Copied from the SDK byte for byte; see the module docstring. Do not "tidy"
# these numbers: a change here is a change to a file format that hardware
# writes and reads.
#
#   header  magic 8s | version u16 | n_samples u32 | sample_hz f64 |
#           created f64 | pos_closed_rad f64 | pos_open_rad f64 |
#           rad_to_mm f64 | can_id u32 | mount 8s   = 66 B
#   sample  t f64 | openness f64 | position_rad f64 | velocity_rad_s f64 |
#           torque_nm f64                            = 40 B

_MAGIC = b"LGRTRJ01"
_VERSION = 1
_HEADER = struct.Struct("<8sHI5dI8s")
_SAMPLE = struct.Struct("<5d")
_MOUNT_FIELD = 8
#: Legal values of the ``mount`` header field. The SDK spells the same set out
#: inline, as ``("", "normal", "reverse")`` against an unused ``_MOUNTS`` tuple
#: that also carries ``None``; the empty string is what an unset mount serialises
#: to, so it belongs in the set that is actually compared.
_MOUNTS = ("", "normal", "reverse")


def trajectory_dir() -> str:
    """Directory a trajectory is saved to when only a name is given.

    ``~/.litegrip/trajectories`` -- next to the per-channel calibration files,
    because a trajectory belongs to the machine, not to the working directory
    the program happened to start in. ``LITEGRIP_TRAJ_DIR`` overrides it, for
    tests and for a controller with a read-only home.

    Same variable and same default as the SDK's ``litegrip.trajectory_dir()``,
    so both tools read one directory.
    """
    env = os.environ.get("LITEGRIP_TRAJ_DIR")
    if env:
        return env
    return os.path.join(os.path.expanduser("~"), ".litegrip", "trajectories")


def resolve_path(path: str) -> str:
    """A bare name lands in :func:`trajectory_dir`; anything else is a path.

    ``"pick"`` -> ``~/.litegrip/trajectories/pick.lgt``; ``"out/pick.lgt"`` and
    ``"/tmp/pick.lgt"`` are used as written.
    """
    if os.sep in path or (os.altsep and os.altsep in path):
        return path
    name = path if path.endswith(".lgt") else path + ".lgt"
    return os.path.join(trajectory_dir(), name)


# -- exceptions -----------------------------------------------------------


class TrajectoryError(LiteGripError):
    """Base class for trajectory record/replay errors."""


class TrajectoryBusyError(TrajectoryError):
    """Raised when a recording or replay is started while one is already running."""


class TrajectoryNotActiveError(TrajectoryError):
    """Raised when a stop is asked for but nothing is running."""


class TrajectoryEmptyError(TrajectoryError):
    """Raised when a capture or a replay would deal with zero samples."""


class TrajectoryRecordingError(TrajectoryError):
    """Raised when the recording loop died, or a timed capture did not fill."""


class TrajectoryFormatError(TrajectoryError):
    """Raised when a byte stream is not a well-formed trajectory file."""


# -- data -----------------------------------------------------------------


@dataclass(frozen=True)
class TrajectorySample:
    """One sample of a recorded motion.

    Attributes:
        t: Seconds since the recording started. A trimmed clock reading of the
            *recording* machine -- it means nothing after a reboot or on another
            host, and replay only ever uses the differences between samples.
        openness: Opening in ``[0, 1]`` (0 = closed, 1 = fully open). The channel
            replay actually follows; see the module docstring.
        position_rad: Raw motor angle at that instant -- diagnostic. Only
            meaningful against the calibration recorded alongside it.
        velocity_rad_s: Motor velocity -- diagnostic.
        torque_nm: Motor torque -- diagnostic. Also the closest thing to a force
            record, but replay does not feed it forward.
    """

    t: float
    openness: float
    position_rad: float
    velocity_rad_s: float = 0.0
    torque_nm: float = 0.0


@dataclass
class Trajectory:
    """A recorded motion: samples plus the calibration they were taken against.

    The geometry fields describe the gripper that *recorded* the trajectory and
    travel with it in the file, so a loaded trajectory still reports which unit
    it came from and how its ``openness`` values were derived. Replay ignores
    them in favour of the local gripper's own calibration.
    """

    samples: List[TrajectorySample] = field(default_factory=list)
    sample_hz: float = DEFAULT_RATE_HZ
    created: float = field(default_factory=time.time)
    can_id: int = 0x08
    pos_closed_rad: float = 0.0
    pos_open_rad: float = 0.0
    rad_to_mm: float = 0.0
    mount: Optional[str] = None

    def __len__(self) -> int:
        return len(self.samples)

    @property
    def duration(self) -> float:
        """Seconds from the first sample to the last (0.0 for a short one).

        The span, not the last sample's timestamp: a trajectory whose first
        sample does not sit at ``t = 0`` still lasts only as long as its own
        samples cover, and reporting the raw end stamp would make replay hold
        its opening for the whole offset before starting to move.
        """
        if not self.samples:
            return 0.0
        return float(self.samples[-1].t - self.samples[0].t)

    def openness_at(self, t: float) -> float:
        """Opening at time *t*, linearly interpolated between samples.

        Clamped at both ends: before the first sample and after the last one the
        nearest sample's opening is returned. Signals that move are sampled far
        faster than they move, so linear interpolation between neighbours is well
        below the mechanical resolution -- a smoother curve would be inventing
        detail the recording does not contain.
        """
        samples = self.samples
        if not samples:
            raise TrajectoryEmptyError("trajectory has no samples")
        if t <= samples[0].t:
            return samples[0].openness
        if t >= samples[-1].t:
            return samples[-1].openness
        lo, hi = 0, len(samples) - 1
        while hi - lo > 1:                      # bisect on t
            mid = (lo + hi) // 2
            if samples[mid].t <= t:
                lo = mid
            else:
                hi = mid
        a, b = samples[lo], samples[hi]
        span = b.t - a.t
        if span <= 0.0:
            return b.openness
        return a.openness + (t - a.t) / span * (b.openness - a.openness)

    # -- serialisation ----------------------------------------------------

    def to_bytes(self) -> bytes:
        """Serialise to the binary format described in this module's header.

        Raises:
            TrajectoryError: The mount name does not fit its field.
        """
        mount = (self.mount or "").encode("ascii")
        if len(mount) > _MOUNT_FIELD:
            raise TrajectoryError(
                f"mount name {self.mount!r} is longer than {_MOUNT_FIELD} bytes")
        parts = [_HEADER.pack(
            _MAGIC, _VERSION, len(self.samples), float(self.sample_hz),
            float(self.created), float(self.pos_closed_rad),
            float(self.pos_open_rad), float(self.rad_to_mm), int(self.can_id),
            mount.ljust(_MOUNT_FIELD, b"\x00"))]
        for s in self.samples:
            parts.append(_SAMPLE.pack(
                float(s.t), float(s.openness), float(s.position_rad),
                float(s.velocity_rad_s), float(s.torque_nm)))
        return b"".join(parts)

    @classmethod
    def from_bytes(cls, blob: bytes) -> "Trajectory":
        """Parse a trajectory file, or say precisely why it is not one.

        Every check here exists to stop a corrupt file from becoming a
        plausible-looking motion: the header's sample count has to match the
        payload exactly (a truncated download, a half-written file and a file
        with another file's tail appended are all caught by the same rule), and
        the samples have to be finite, in range and ordered in time.

        Raises:
            TrajectoryFormatError: Magic, version, length, header fields or
                sample values are not valid.
        """
        blob = bytes(blob)
        if len(blob) < _HEADER.size:
            raise TrajectoryFormatError(
                f"file is {len(blob)}B, too short for the {_HEADER.size}B header")
        (magic, version, n, hz, created, pos_closed, pos_open, rad_to_mm,
         can_id, mount_raw) = _HEADER.unpack_from(blob, 0)
        if magic != _MAGIC:
            raise TrajectoryFormatError(
                f"bad magic {magic!r} (expected {_MAGIC!r}) -- not a trajectory file")
        if version != _VERSION:
            raise TrajectoryFormatError(
                f"format version {version} is not the {_VERSION} this reads")
        want = _HEADER.size + n * _SAMPLE.size
        if len(blob) != want:
            raise TrajectoryFormatError(
                f"file is {len(blob)}B but its header declares {n} samples "
                f"({want}B, off by {len(blob) - want:+d}) -- truncated or with "
                f"trailing data; refusing to parse half a trajectory")
        if not (hz > 0.0):
            raise TrajectoryFormatError(f"illegal sample rate: {hz}")
        if not (rad_to_mm > 0.0):
            raise TrajectoryFormatError(f"illegal rad_to_mm: {rad_to_mm}")
        mount = mount_raw.split(b"\x00", 1)[0].decode("ascii", "replace")
        if mount not in _MOUNTS:
            raise TrajectoryFormatError(f"illegal mount name: {mount!r}")
        if not (abs(pos_open - pos_closed) * rad_to_mm > 0.0):
            raise TrajectoryFormatError(
                f"zero travel (closed={pos_closed}, open={pos_open})")

        samples: List[TrajectorySample] = []
        last_t = float("-inf")
        for i in range(n):
            values = _SAMPLE.unpack_from(blob, _HEADER.size + i * _SAMPLE.size)
            t, openness, position_rad, velocity, torque = values
            for name, value in zip(
                    ("t", "openness", "position_rad", "velocity_rad_s",
                     "torque_nm"), values):
                if value != value or value in (float("inf"), float("-inf")):
                    raise TrajectoryFormatError(
                        f"sample {i}: {name} is not finite: {value}")
            if not 0.0 <= openness <= 1.0:
                raise TrajectoryFormatError(
                    f"sample {i}: openness={openness} is outside [0, 1]")
            if t < last_t:
                raise TrajectoryFormatError(
                    f"sample {i}: t={t} is earlier than the previous {last_t} "
                    f"-- time is not monotonic")
            last_t = t
            samples.append(TrajectorySample(
                t=t, openness=openness, position_rad=position_rad,
                velocity_rad_s=velocity, torque_nm=torque))

        return cls(samples=samples, sample_hz=hz, created=created, can_id=can_id,
                   pos_closed_rad=pos_closed, pos_open_rad=pos_open,
                   rad_to_mm=rad_to_mm, mount=mount or None)

    def save(self, path: str) -> str:
        """Write to *path*, creating the parent directory. Returns the path.

        A bare name goes to :func:`trajectory_dir` (see :func:`resolve_path`).
        """
        target = resolve_path(path)
        parent = os.path.dirname(target)
        if parent:
            os.makedirs(parent, exist_ok=True)
        with open(target, "wb") as f:
            f.write(self.to_bytes())
        log.info("trajectory saved: %s (%d samples)", target, len(self.samples))
        return target

    @classmethod
    def load(cls, path: str) -> "Trajectory":
        """Read a trajectory written by :meth:`save`, or by the SDK."""
        with open(resolve_path(path), "rb") as f:
            return cls.from_bytes(f.read())


# -- openness <-> angle ---------------------------------------------------


def _clamp01(x: float) -> float:
    """Clamp to ``[0, 1]``.

    Same as the SDK's, including its warning: **this does not sanitise NaN**,
    because every comparison against NaN is false, so NaN passes through. Callers
    must reject non-finite values before clamping, not rely on this to bound them.
    """
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def _frac_from_theta(theta: float, config: Any) -> float:
    """Motor angle -> dimensionless opening in ``[0, 1]``, using *config*'s own
    endpoints.

    The device-local conversion, and the reason a trajectory is portable: the
    same angle on a differently calibrated unit is a different opening.

    The result is clamped, exactly as the SDK's ``rad_to_openness`` clamps. That
    is not cosmetic: a sample taken a hair past a limit would otherwise carry an
    openness of ``-3.7e-06``, and :meth:`Trajectory.from_bytes` rejects anything
    outside ``[0, 1]`` -- so the recorder would write files the loader refuses,
    including its own.
    """
    stroke = (abs(float(config.pos_open_rad) - float(config.pos_closed_rad))
              * float(config.rad_to_mm))
    if stroke <= 0.0:
        return 0.0
    closed = float(config.pos_closed_rad)
    opened = float(config.pos_open_rad)
    return _clamp01((float(theta) - closed) / (opened - closed))


def _theta_from_frac(frac_open: float, config: Any) -> float:
    """Dimensionless opening -> motor angle, using *config*'s own endpoints.

    Clamped to ``[0, 1]`` first, which bounds the target to the device's own
    travel -- the SDK's ``openness_to_rad`` does the same, so a trajectory whose
    recorded angles sit outside the local range still replays inside it.
    """
    closed = float(config.pos_closed_rad)
    opened = float(config.pos_open_rad)
    if float(config.rad_to_mm) <= 0.0:
        return closed
    return closed + _clamp01(frac_open) * (opened - closed)


def _config_mount(config: Any) -> Optional[str]:
    """``"normal"`` / ``"reverse"`` -- the name the endpoints spell out.

    The simulated endpoints come from the URDF rather than from a calibration
    file, so this is a reading and not a claim, which is why it is reported
    unconditionally. The SDK returns ``None`` here while its config still holds
    the placeholder defaults, because there the ordering is an assumption.
    """
    return ("normal"
            if float(config.pos_closed_rad) >= float(config.pos_open_rad)
            else "reverse")


# -- loop pacing ----------------------------------------------------------


class _Pacer:
    """Frame pacing and loop-rate measurement, shared by both loops.

    ``rest()`` sleeps whatever is left of a cycle's budget, so the loop holds its
    rate without drifting when a cycle overruns. ``loop_hz`` is the measured rate
    over the last second -- the number to look at when a recording sounds wrong,
    because a loop that cannot keep up drops samples rather than stretching time.
    """

    def __init__(self, dt: float, sleep_fn: Callable[[float], None],
                 monotonic_fn: Callable[[], float]) -> None:
        self._dt = dt
        self._sleep_fn = sleep_fn
        self._monotonic_fn = monotonic_fn
        self._loops = 0
        self._hz_t0 = 0.0
        self._hz_n0 = 0
        self.loop_hz = 0.0

    def rest(self, t0: float) -> None:
        self._loops += 1
        now = self._monotonic_fn()
        if self._hz_t0 == 0.0:
            self._hz_t0 = now
            self._hz_n0 = self._loops
        elif now - self._hz_t0 >= 1.0:
            self.loop_hz = (self._loops - self._hz_n0) / (now - self._hz_t0)
            self._hz_t0 = now
            self._hz_n0 = self._loops
        rest = self._dt - (self._monotonic_fn() - t0)
        if rest > 0.0:
            self._sleep_fn(rest)


# -- recording ------------------------------------------------------------


class TrajectoryRecorder:
    """Samples the gripper's state into a :class:`Trajectory`, on a thread.

    With ``zero_gravity=True`` the jaws are left back-drivable, which on real
    hardware is the hand-teaching mode. **In the simulation nothing can push
    them**: ``mujoco.viewer`` keeps the mouse for camera control and reports no
    drag events, so the usual simulation recording is ``zero_gravity=False``
    with a script driving the gripper from another thread -- the mode the SDK
    documents for capturing a programmatic move. The loop only *reads* state in
    that mode, so it does not fight the driver for control.

    Args:
        gripper: The :class:`~litegrip_mujoco.gripper.MujocoGripper` to sample.
        rate_hz: Samples per second. :data:`DEFAULT_RATE_HZ` (100) unless the
            motion is fast.
        zero_gravity: Leave the jaws torque-free so they can be pushed by hand.
        max_samples: Stop by itself after this many samples. ``None`` records
            until :meth:`stop`. Bounded recordings are what make a stuck clock
            visible as a timeout instead of a running process.
        sleep_fn, monotonic_fn: Timing seams for tests.
    """

    def __init__(
        self,
        gripper: Any,
        rate_hz: float = DEFAULT_RATE_HZ,
        zero_gravity: bool = True,
        max_samples: Optional[int] = None,
        sleep_fn: Optional[Callable[[float], None]] = None,
        monotonic_fn: Optional[Callable[[], float]] = None,
    ) -> None:
        if rate_hz <= 0.0:
            raise ValueError(f"rate_hz must be > 0, got {rate_hz!r}")
        self._g = gripper
        self._rate_hz = float(rate_hz)
        self._zero_gravity = bool(zero_gravity)
        self._max_samples = None if max_samples is None else int(max_samples)
        self._sleep_fn = sleep_fn or time.sleep
        self._monotonic_fn = monotonic_fn or time.monotonic
        self._pacer = _Pacer(1.0 / self._rate_hz, self._sleep_fn,
                             self._monotonic_fn)

        # Appended by the loop thread, read by the caller. No lock: list append
        # and len are atomic under the GIL, and a torn sample is impossible once
        # appended.
        self._samples: List[TrajectorySample] = []
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._error: Optional[BaseException] = None
        self._t0 = 0.0

    # -- lifecycle --------------------------------------------------------

    @property
    def is_recording(self) -> bool:
        return self._running

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    def start(self) -> None:
        """Start sampling. Raises :class:`TrajectoryBusyError` if already on."""
        if self._running:
            raise TrajectoryBusyError("trajectory recording is already running")
        if self._zero_gravity:
            self._g.enter_zero_gravity()
        self._samples = []
        self._error = None
        self._pacer.loop_hz = 0.0
        # Sample 0 is taken here, before the thread exists, with t = 0.0: the
        # recording then starts at a commanded instant instead of whenever the
        # scheduler happened to first run the loop.
        self._t0 = self._monotonic_fn()
        self._samples.append(self._sample_now(0.0))
        self._running = True
        self._thread = threading.Thread(
            target=self._run, name="litegrip-mujoco-trajectory-record",
            daemon=True)
        self._thread.start()
        log.info("recording started: %.0fHz zero_gravity=%s max_samples=%s",
                 self._rate_hz, self._zero_gravity, self._max_samples)

    def _run(self) -> None:
        try:
            self._loop()
        except BaseException as e:  # noqa: BLE001 -- surfaces through result()
            self._error = e
            log.warning("recording loop stopped: %s", e)
        finally:
            self._running = False

    def _loop(self) -> None:
        stall = 0
        last = self._t0
        while self._running:
            t0 = self._monotonic_fn()
            # A cycle whose clock did not move produced no sample: two rows with
            # the same timestamp are not two measurements, and appending one
            # anyway is how a stopped clock becomes a "successful" recording of
            # zero length. So the clock has to move before anything is stored --
            # and a clock that will not move is an error, not a slow capture.
            if t0 - last <= _CLOCK_EPS:
                stall += 1
                if stall >= _STALL_CYCLES:
                    raise TrajectoryRecordingError(
                        f"sampling clock did not advance for {stall} cycles "
                        f"(t={t0}) -- refusing to append samples forever on a "
                        f"clock that has stopped")
                last = t0
                self._pacer.rest(t0)
                continue
            stall = 0
            last = t0

            # Checked before storing, not after: start() already stored the
            # reference sample, so a check after the append would return
            # max_samples + 1 rows -- and for a cap the reference sample alone
            # already meets, it would depend on which thread got there first.
            if (self._max_samples is not None
                    and len(self._samples) >= self._max_samples):
                return
            self._samples.append(self._sample_now(t0 - self._t0))
            self._pacer.rest(t0)

    def _sample_now(self, t: float) -> TrajectorySample:
        state = self._g.get_state(wait=False)
        return TrajectorySample(
            t=float(t),
            openness=_frac_from_theta(state.position_rad, self._g.config),
            position_rad=float(state.position_rad),
            velocity_rad_s=float(state.velocity_rad_s),
            torque_nm=float(state.torque_nm),
        )

    def wait_for(self, n_samples: int, timeout: float,
                 poll: float = 0.005) -> int:
        """Block until *n_samples* have been captured; return the count.

        Polls the real clock, not the loop's ``monotonic_fn`` seam, so a test
        that stubs the loop's clock still gets a bounded wait. A loop that died
        raises its own error here immediately rather than after the timeout.

        Raises:
            TrajectoryRecordingError: The loop died, or *timeout* elapsed with
                fewer samples -- the message names the count, so a short capture
                is visible rather than silently accepted.
        """
        n = int(n_samples)
        deadline = time.monotonic() + float(timeout)
        while True:
            if self._error is not None:
                raise self._recording_error(len(self._samples))
            got = len(self._samples)
            if got >= n:
                return got
            if not self._running:
                raise TrajectoryRecordingError(
                    f"recording ended early: only {got}/{n} samples, loop stopped")
            if time.monotonic() >= deadline:
                raise TrajectoryRecordingError(
                    f"recording did not reach {n} samples within "
                    f"{float(timeout):.1f}s (got {got}) -- raise duration_s or "
                    f"lower rate_hz")
            time.sleep(poll)

    def stop(self, timeout: float = 2.0) -> None:
        """Stop the loop and leave the gripper holding its position.

        The hold happens even when the loop died: a failed recording must not
        leave the jaws slack, because slack jaws drop whatever they were holding.
        """
        self._running = False
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)
        self._thread = None
        if self._zero_gravity:
            try:
                self._g.exit_zero_gravity()
            except Exception as e:  # noqa: BLE001
                log.debug("exit_zero_gravity on stop failed: %s", e)
        log.info("recording stopped: %d samples", len(self._samples))

    # -- results ----------------------------------------------------------

    def _recording_error(self, got: int) -> TrajectoryRecordingError:
        """Wrap whatever killed the loop, without double-wrapping our own error."""
        if isinstance(self._error, TrajectoryRecordingError):
            return self._error
        return TrajectoryRecordingError(
            f"recording failed; the {got} samples taken are not returned: "
            f"{self._error}")

    def result(self, allow_empty: bool = False) -> Trajectory:
        """The captured trajectory.

        Refuses to hand back a capture that did not work: a dead loop raises
        :class:`TrajectoryRecordingError` (carrying the original error and the
        sample count) rather than returning the partial capture as if it were a
        whole recording.

        :class:`TrajectoryEmptyError` is for the one way to get nothing at all --
        :meth:`result` on a recorder that was never started. A started capture
        always holds at least the reference sample :meth:`start` takes.

        Raises:
            TrajectoryRecordingError: The sampling loop died.
            TrajectoryEmptyError: Nothing was ever captured.
        """
        if self._error is not None:
            raise self._recording_error(len(self._samples))
        if not self._samples and not allow_empty:
            raise TrajectoryEmptyError(
                "recording captured no samples at all (0) -- there was no time "
                "between start and stop")
        cfg = self._g.config
        return Trajectory(
            samples=list(self._samples),
            sample_hz=self._rate_hz,
            created=time.time(),
            can_id=int(self._g.can_id),
            pos_closed_rad=float(cfg.pos_closed_rad),
            pos_open_rad=float(cfg.pos_open_rad),
            rad_to_mm=float(cfg.rad_to_mm),
            mount=_config_mount(cfg),
        )

    def status(self) -> dict:
        """A snapshot of the recording session, for logging and diagnostics.

        The same keys, spelled the same way, as the SDK's recorder status -- so
        a log parser does not care which side produced it.
        """
        return {
            "active": self._running,
            "kind": "record",
            "samples": len(self._samples),
            "rate_hz": round(self._rate_hz, 1),
            "zero_gravity": self._zero_gravity,
            "loop_hz": round(self._pacer.loop_hz, 1),
            "error": None if self._error is None else str(self._error),
        }


# -- replay ---------------------------------------------------------------


class TrajectoryPlayer:
    """Streams a :class:`Trajectory` back to the gripper, on a thread.

    The target at each cycle is interpolated from the trajectory by **wall
    clock**: ``u = (now - t0) * speed``, then ``trajectory.openness_at(u)``.
    Advancing an index once per cycle instead would tie playback speed to the
    loop rate and accumulate drift, so a cycle that overruns would make every
    later sample late and a 2 s recording would take longer and longer to play.
    Here a slow cycle skips ahead, and the motion stays the length it was taught.

    The commanded angle is converted with the *local* gripper's endpoints, so the
    trajectory plays on a gripper with a different mount or calibration. Only
    position is commanded: the recorded velocity and torque are never sent as
    ``dq``/``tau`` feed-forward, because both are tied to the sign convention of
    the unit that recorded them.

    Args:
        gripper: The :class:`~litegrip_mujoco.gripper.MujocoGripper` to drive.
        trajectory: What to play. Rejected up front if it has no samples.
        speed: Multiplier on the recorded timing. ``0.5`` is half speed.
        kp, kd: Gains for the replay frames. ``None`` uses the gripper's
            configured gains.
        loop: Restart from the beginning instead of stopping at the end. A
            trajectory with a single sample is a pose with no length to restart,
            so looping it keeps holding that opening.
        align: Move to the trajectory's first opening (one blocking move) before
            following. Without it the first frame steps to sample 0 from wherever
            the jaws happen to be, which is a torque spike.
        rate_hz: Frame rate. ``None`` uses :data:`DEFAULT_PLAY_RATE_HZ`.
        sleep_fn, monotonic_fn: Timing seams for tests.
    """

    def __init__(
        self,
        gripper: Any,
        trajectory: Trajectory,
        speed: float = 1.0,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        loop: bool = False,
        align: bool = True,
        rate_hz: Optional[float] = None,
        sleep_fn: Optional[Callable[[float], None]] = None,
        monotonic_fn: Optional[Callable[[], float]] = None,
    ) -> None:
        if speed <= 0.0:
            raise ValueError(f"speed must be > 0, got {speed!r}")
        if len(trajectory) == 0:
            raise TrajectoryEmptyError("trajectory has no samples: nothing to play")
        self._g = gripper
        self._traj = trajectory
        self._speed = float(speed)
        self._kp = float(kp) if kp is not None else float(gripper.config.kp)
        self._kd = float(kd) if kd is not None else float(gripper.config.kd)
        self._loop = bool(loop)
        self._align = bool(align)
        # Where this trajectory's own clock starts. Zero for anything recorded
        # here; a hand-built or foreign one may not be, and openness_at() indexes
        # by the absolute stamp, so the phase is kept separately from the elapsed
        # time the pacing works in.
        self._origin = float(trajectory.samples[0].t)
        self._rate_hz = float(rate_hz if rate_hz is not None
                              else DEFAULT_PLAY_RATE_HZ)
        self._sleep_fn = sleep_fn or time.sleep
        self._monotonic_fn = monotonic_fn or time.monotonic
        self._pacer = _Pacer(1.0 / self._rate_hz, self._sleep_fn,
                             self._monotonic_fn)

        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._error: Optional[BaseException] = None
        self._frames = 0
        self._last_openness = trajectory.samples[0].openness
        self._completed = False

    # -- lifecycle --------------------------------------------------------

    @property
    def is_playing(self) -> bool:
        return self._running

    @property
    def is_finished(self) -> bool:
        """True once the trajectory has played through to its end."""
        return self._completed

    def start(self) -> None:
        """Start replaying. Raises :class:`TrajectoryBusyError` if already on."""
        if self._running:
            raise TrajectoryBusyError("trajectory replay is already running")
        self._running = True
        self._thread = threading.Thread(
            target=self._run, name="litegrip-mujoco-trajectory-play", daemon=True)
        self._thread.start()
        log.info("replay started: %d samples, %.2fs, speed=%.2f loop=%s",
                 len(self._traj), self._traj.duration, self._speed, self._loop)

    def _run(self) -> None:
        try:
            if self._align:
                self._align_to_start()
            self._loop_frames()
        except BaseException as e:  # noqa: BLE001 -- surfaces through status()
            self._error = e
            log.warning("replay loop stopped: %s", e)
        finally:
            self._running = False

    def _align_to_start(self) -> None:
        """One move to sample 0, so following starts from the right place."""
        target = _theta_from_frac(self._traj.samples[0].openness, self._g.config)
        log.info("replay aligning to first sample: openness=%.3f -> %.3f rad",
                 self._traj.samples[0].openness, target)
        try:
            self._g.goto_rad(target, kp=self._kp, kd=self._kd, duration=1.0)
        except Exception as e:  # noqa: BLE001
            # An abort (the window was closed) is the ordinary reason this fails;
            # the frames that follow still get their chance to report it.
            log.warning("replay align goto_rad failed: %s", e)

    def _loop_frames(self) -> None:
        duration = self._traj.duration
        t0 = self._monotonic_fn()
        last = t0
        stall = 0
        while self._running:
            cycle_start = self._monotonic_fn()
            # The recorder's rule, for the same reason: trajectory time is read
            # off this clock, so a clock that will not move means the replay can
            # never reach its end -- and a loop that keeps sending frames it
            # cannot advance past is flooding, not playing.
            if cycle_start - last <= _CLOCK_EPS:
                stall += 1
                if stall >= _STALL_CYCLES:
                    raise TrajectoryError(
                        f"replay clock did not advance for {stall} cycles "
                        f"(t={cycle_start}) -- the replay cannot move forward, "
                        f"refusing to spin emitting frames")
            else:
                stall = 0
            last = cycle_start
            t0 = self._emit_cycle(cycle_start, t0, duration)
            if t0 is None:
                return
            self._pacer.rest(cycle_start)

    def _emit_cycle(self, now: float, t0: float,
                    duration: float) -> Optional[float]:
        """Emit one frame; return the (possibly rewound) epoch, or None to stop."""
        if duration <= 0.0:
            # A one-sample trajectory is a pose, not a path: there is no time to
            # advance along. Looping it means holding that pose, which is the
            # only reading that keeps `loop=True` meaning "keeps going until
            # play_stop" -- a pose is exactly what a hold is for.
            if self._loop:
                self._emit(self._traj.samples[0].openness)
                return t0
            self._emit(self._traj.samples[-1].openness)
            self._completed = True
            log.info("replay finished: %d frames", self._frames)
            return None

        # Elapsed seconds along the trajectory, from the wall clock -- never an
        # index stepped once per cycle, which would tie the speed to the loop
        # rate and make a recorded 2 s path take longer every time.
        elapsed = (now - t0) * self._speed
        if elapsed >= duration:
            if self._loop:
                # Rewind by whole loops rather than resetting to `now`: a cycle
                # that overran keeps its phase instead of shifting the loop.
                elapsed %= duration
                t0 = now - elapsed / self._speed
            else:
                self._emit(self._traj.samples[-1].openness)
                self._completed = True
                log.info("replay finished: %d frames", self._frames)
                return None
        self._emit(self._traj.openness_at(self._origin + elapsed))
        return t0

    def _emit(self, openness: float) -> None:
        q = _theta_from_frac(openness, self._g.config)
        try:
            # MujocoGripper.send_mit_frame() returns None rather than a success
            # flag -- there is no bus to fail on -- so the failure signal here is
            # the exception (not connected), not a falsy return value.
            self._g.send_mit_frame(q=q, kp=self._kp, kd=self._kd, dq=0.0)
        except Exception as e:  # noqa: BLE001
            raise TrajectoryError(
                f"could not send the MIT frame -- replay aborted rather than "
                f"silently dropping frames: {e}") from e
        self._last_openness = openness
        self._frames += 1

    def stop(self, timeout: float = 2.0) -> None:
        """Stop replaying and leave the gripper holding its last target.

        The hold is one frame at the current position under the configured
        gains. On real hardware that holds the jaws only while frames keep
        arriving (the motor self-locks a comm-loss fault about 100 ms after they
        stop); in the simulation the controller keeps the target until something
        else changes it, so the hold lasts.
        """
        self._running = False
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)
        self._thread = None
        try:
            theta = float(self._g.get_position_rad())
            self._g.send_mit_frame(q=theta, kp=float(self._g.config.kp),
                                   kd=float(self._g.config.kd))
        except Exception as e:  # noqa: BLE001
            log.debug("hold on replay stop failed: %s", e)
        log.info("replay stopped: %d frames", self._frames)

    def wait(self, timeout: float) -> bool:
        """Block until the replay finishes. True if it did, False on timeout.

        A wall-clock playback of a stalling clock would otherwise spin forever,
        so the blocking wrapper gives it a deadline and this is how it asks.
        """
        thread = self._thread
        if thread is None:
            return True
        thread.join(timeout)
        return not thread.is_alive()

    def status(self) -> dict:
        """A snapshot of the replay session, for logging and diagnostics.

        The same keys, spelled the same way, as the SDK's player status.
        """
        return {
            "active": self._running,
            "kind": "play",
            "samples": len(self._traj),
            "frames": self._frames,
            "speed": round(self._speed, 3),
            "loop": self._loop,
            "completed": self._completed,
            "openness": round(self._last_openness, 4),
            "loop_hz": round(self._pacer.loop_hz, 1),
            "error": None if self._error is None else str(self._error),
        }
