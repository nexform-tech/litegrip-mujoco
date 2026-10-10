"""Calibration selection, validation, and provenance.

Every code path that moves real hardware ends up with a calibration file. The
default is the read-only factory calibration shipped inside the SDK package;
naming another one with ``--calibration`` is how a run is pointed at a specific
gripper instead. This module validates whichever file is used before the SDK
ever sees it, and stamps the device so the conversion helpers can tell a
calibrated device from a bare one.

The factory file is a usable set of defaults, not a measurement of the machine
in front of you: it is the test-bench fixture's geometry. That is why a run that
falls back to it says so out loud rather than letting an operator believe it is
their gripper's numbers.

Why a separate module
---------------------

``mirror.py`` is about exchanging ``frac_open`` between two devices, and its
inner loops run at 50 Hz. Selecting and validating a file is a different
concern, and it has to import **without** the ``litegrip`` SDK installed (CI
runs that way), so it lives here and depends only on the standard library plus
``._litegrip``.

The problem this exists for
---------------------------

The SDK's ``LiteGrip.load_calibration`` reads the path it is given, and on
``FileNotFoundError`` or ``JSONDecodeError`` silently falls through to the
read-only factory calibration shipped inside the package. It then returns
``True`` for either source, so the return value cannot tell you which file the
endpoints came from. A typo in a path, or a malformed file, therefore produces
a gripper that behaves as if calibrated while holding another machine's
numbers. On top of that, a file that parses but lacks a required key raises an
uncaught ``KeyError`` from inside the SDK.

This module closes all three: it validates the file *before* the SDK is called,
verifies the endpoints *after*, and keeps the factory file's provenance visible
instead of silent.

The invariant
-------------

The closed end must be numerically **larger** than the open end
(``pos_closed_rad > pos_open_rad``). The SDK documents this, and its own
``goto_rad`` clamp is only correct while it holds. The SDK's *default* values
violate it, which is how ``_endpoints()`` in :mod:`litegrip_mujoco.mirror`
tells "calibrated" from "not calibrated" today. Real calibration files satisfy
it; anything that does not is rejected here.

What this module cannot do
--------------------------

A well-formed calibration file for the *wrong machine* is indistinguishable
from a correct one — the format carries no machine identity. Nothing short of
the host GUI's positional sanity check (which needs a connected device) can
catch that. Choose the file deliberately.
"""
from __future__ import annotations

import json
import math
import os
import time
import warnings
import weakref
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from .gripper import _default_calib_path

#: Keys the SDK's ``LiteGrip.load_calibration`` dereferences unconditionally.
#: ``rad_to_mm`` is read outside its ``if key in data`` guard, so a file
#: without it raises ``KeyError`` from inside the SDK.
REQUIRED_KEYS: Tuple[str, ...] = (
    "zero_position_rad",
    "max_position_rad",
    "rad_to_mm",
)

#: Attribute stamped onto a device to record where its endpoints came from.
MARKER_ATTR = "_litegrip_mujoco_calibration"

#: Fallback for devices that refuse ``setattr``.
_MARKERS: "weakref.WeakKeyDictionary[Any, Provenance]" = weakref.WeakKeyDictionary()

#: Baseline width (rad). The endpoints must span at least this much; the real
#: mechanism spans 1.14 (simulation) to 1.84 (calibrated hardware). A
#: near-zero span means the file is nonsense and every conversion on it is
#: numerically meaningless.
MIN_SPAN_RAD = 1e-3


# ═════════════════════════════════════════════════════════════════════════
# Errors
# ═════════════════════════════════════════════════════════════════════════


class CalibrationError(RuntimeError):
    """Base class for every calibration problem in this package."""


class CalibrationRequiredError(CalibrationError):
    """A run that can move real hardware has no calibration file to use.

    Raised when no path was given and the SDK's factory calibration cannot be
    read either — there is nothing to fall back to, and guessing is not an
    option.
    """


class CalibrationFileError(CalibrationError):
    """A calibration file is missing, malformed, or violates the invariant."""


class CalibrationVerificationError(CalibrationError):
    """A device does not hold the calibration that was asked of it.

    Either the SDK silently loaded something else, or the endpoints were
    changed after the file was applied.
    """


class UncalibratedDeviceError(CalibrationError):
    """A device has no trustworthy ``(closed, open)`` endpoints.

    Replaces the ``RuntimeWarning`` plus millimetre fallback that earlier
    versions of :func:`litegrip_mujoco.mirror.read_frac_open` used.
    """


# ═════════════════════════════════════════════════════════════════════════
# Data
# ═════════════════════════════════════════════════════════════════════════


@dataclass(frozen=True)
class Calibration:
    """A validated calibration file.

    Attributes:
        path: Absolute path the endpoints were read from.
        pos_closed_rad: ``zero_position_rad`` from the file.
        pos_open_rad: ``max_position_rad`` from the file.
        rad_to_mm: Motor-radian to millimetre coefficient from the file.
        raw: The whole parsed JSON object, for optional fields (``can_id``,
            ``channel``, ``kp``, ``kd``, ``grasp_torque_threshold``, ...).
    """

    path: str
    pos_closed_rad: float
    pos_open_rad: float
    rad_to_mm: float
    raw: Mapping[str, Any] = field(default_factory=dict)

    @property
    def span_rad(self) -> float:
        """Travel from closed to open, in motor radians. Always positive."""
        return self.pos_closed_rad - self.pos_open_rad

    @property
    def travel_mm(self) -> float:
        """Travel from closed to open, in millimetres."""
        return self.span_rad * self.rad_to_mm

    @property
    def mtime(self) -> float:
        """Modification time, or ``0.0`` if the file has since vanished."""
        try:
            return os.path.getmtime(self.path)
        except OSError:
            return 0.0

    @property
    def is_sdk_default_path(self) -> bool:
        """True when this is the SDK's default user path.

        ``~/.litegrip/litegrip_calibration.json`` (or ``$LITEGRIP_CALIB``): what
        the SDK itself would load with no path, and what a bare
        ``LiteGrip`` uses.
        """
        return _same_file(self.path, default_calibration_path())

    @property
    def is_sdk_factory_path(self) -> bool:
        """True when this is the read-only calibration shipped in the SDK.

        This is the default a run uses when nothing else is named. It carries
        the bench fixture's measurements, so anything that reports which file
        was used has to say so.
        """
        factory = sdk_factory_calibration_path()
        return factory is not None and _same_file(self.path, factory)

    def describe(self) -> str:
        """One-line summary for console output and listings."""
        stamp = time.strftime("%Y-%m-%d %H:%M", time.localtime(self.mtime))
        return (
            f"closed={self.pos_closed_rad:+.6f} rad  "
            f"open={self.pos_open_rad:+.6f} rad  "
            f"travel≈{self.travel_mm:.1f} mm  ({stamp})"
        )


@dataclass(frozen=True)
class Provenance:
    """What is known about where a device's endpoints came from.

    Attributes:
        path: The file applied, or ``None`` for a deliberate opt-out.
        calibration: The validated file, cached so hot loops do not re-read it.
        reason: Why the marker was written; shown for opt-outs.
    """

    path: Optional[str]
    calibration: Optional[Calibration] = None
    reason: str = ""


# ═════════════════════════════════════════════════════════════════════════
# Paths
# ═════════════════════════════════════════════════════════════════════════


def default_calibration_path() -> str:
    """The SDK's default user calibration path.

    Honours ``LITEGRIP_CALIB`` first, exactly as the SDK does, so that the two
    never disagree about which file "the default" is. Computed on every call:
    caching it would break tests (and users) that repoint ``HOME``.
    """
    override = os.environ.get("LITEGRIP_CALIB")
    if override:
        return os.path.abspath(os.path.expanduser(override))
    return os.path.abspath(_default_calib_path())


def sdk_factory_calibration_path() -> Optional[str]:
    """Path of the SDK's read-only factory calibration, or ``None``.

    Derived defensively: if the SDK is absent, or its private constant moves,
    this returns ``None`` rather than letting a missing attribute disable the
    factory-file refusal in :func:`apply_calibration`.
    """
    try:
        from ._litegrip import load_litegrip

        module = load_litegrip()
    except Exception:  # noqa: BLE001 — 没装 SDK 就没有出厂文件
        return None

    gripper_module = getattr(module, "gripper", None)
    if gripper_module is None:
        try:  # pragma: no cover - 取决于 SDK 的导出方式
            import litegrip.gripper as gripper_module  # type: ignore[no-redef]
        except Exception:  # noqa: BLE001
            return None

    declared = getattr(gripper_module, "_FACTORY_CALIB", None)
    if declared:
        return os.path.abspath(str(declared))

    module_file = getattr(gripper_module, "__file__", None)
    if module_file:
        return os.path.abspath(
            os.path.join(os.path.dirname(module_file), "factory_calibration.json")
        )
    return None  # pragma: no cover - 只有打包方式极不寻常时才会走到


def _same_file(left: Optional[str], right: Optional[str]) -> bool:
    """True when both paths resolve to the same file on disk."""
    if not left or not right:
        return False
    try:
        return os.path.realpath(left) == os.path.realpath(right)
    except OSError:  # pragma: no cover - realpath 基本不抛
        return False


def _mtime(path: str) -> float:
    try:
        return os.path.getmtime(path)
    except OSError:
        return 0.0


# ═════════════════════════════════════════════════════════════════════════
# Loading and validation
# ═════════════════════════════════════════════════════════════════════════


def load_calibration_file(path: Any) -> Calibration:
    """Read and validate a calibration file.

    Validation happens *before* the SDK is handed the path, which is what
    removes its two silent-failure modes: a missing file and a malformed file
    both used to fall through to the factory calibration while reporting
    success, and a file without a required key used to raise ``KeyError`` from
    inside the SDK.

    Args:
        path: Path to a calibration JSON file.

    Returns:
        The validated :class:`Calibration`.

    Raises:
        CalibrationFileError: The path is missing, is a directory, is not
            valid JSON, lacks a required key, holds a non-numeric or
            non-finite value, or violates ``closed > open``.
    """
    if path is None or str(path).strip() == "":
        raise CalibrationFileError("没有给出标定文件路径")

    abspath = os.path.abspath(os.path.expanduser(str(path)))
    if os.path.isdir(abspath):
        raise CalibrationFileError(f"标定路径是目录，不是文件: {abspath}")
    if not os.path.isfile(abspath):
        raise CalibrationFileError(f"标定文件不存在: {abspath}")

    try:
        with open(abspath, "r", encoding="utf-8") as handle:
            data = json.load(handle)
    except json.JSONDecodeError as exc:
        raise CalibrationFileError(
            f"标定文件不是合法 JSON: {abspath}（{exc.msg}，第 {exc.lineno} 行）"
        ) from exc
    except OSError as exc:
        raise CalibrationFileError(f"标定文件读不出来: {abspath}（{exc}）") from exc

    if not isinstance(data, dict):
        raise CalibrationFileError(
            f"标定文件的顶层必须是 JSON 对象，实际是 {type(data).__name__}: {abspath}"
        )

    values: Dict[str, float] = {}
    for key in REQUIRED_KEYS:
        if key not in data:
            raise CalibrationFileError(
                f"标定文件缺少必填字段 {key!r}: {abspath}"
                "（SDK 读这个字段时会直接抛 KeyError，所以在这里拦下）"
            )
        try:
            value = float(data[key])
        except (TypeError, ValueError) as exc:
            raise CalibrationFileError(
                f"标定字段 {key!r} 不是数字: {abspath}（{data[key]!r}）"
            ) from exc
        if not math.isfinite(value):
            raise CalibrationFileError(
                f"标定字段 {key!r} 不是有限数: {abspath}（{data[key]!r}）"
            )
        values[key] = value

    closed = values["zero_position_rad"]
    open_ = values["max_position_rad"]
    rad_to_mm = values["rad_to_mm"]

    if rad_to_mm <= 0.0:
        raise CalibrationFileError(
            f"标定字段 'rad_to_mm' 必须为正: {abspath}（{rad_to_mm}）"
        )
    if closed <= open_:
        raise CalibrationFileError(
            f"标定端点违反不变量（闭合位必须数值更大）: {abspath}"
            f"（zero_position_rad={closed} <= max_position_rad={open_}）。"
            "这通常是 SDK 的默认值，不是标定文件。"
        )
    if closed - open_ < MIN_SPAN_RAD:
        raise CalibrationFileError(
            f"标定端点的量程太小（{closed - open_} rad < {MIN_SPAN_RAD}）: {abspath}"
        )

    return Calibration(
        path=abspath,
        pos_closed_rad=closed,
        pos_open_rad=open_,
        rad_to_mm=rad_to_mm,
        raw=data,
    )


def describe_candidate(path: Any) -> str:
    """One-line status of a candidate file, valid or not.

    Invalid files are described, never hidden: a typo in a key has to be
    visible in the listing, not silently filtered out.
    """
    try:
        cal = load_calibration_file(path)
    except CalibrationError as exc:
        return f"⚠ 不可用：{exc}"

    if cal.is_sdk_factory_path:
        return f"{cal.describe()}  ⚠ SDK 出厂标定（不给 --calibration 时的默认值）"
    if cal.is_sdk_default_path:
        return f"{cal.describe()}  ⚠ SDK 默认用户路径（不给 --calibration 时不会被自动选中）"
    return cal.describe()


# ═════════════════════════════════════════════════════════════════════════
# Discovery
# ═════════════════════════════════════════════════════════════════════════


def _scan_dirs(extra_dirs: Optional[Sequence[str]] = None) -> List[str]:
    """Directories searched for calibration files, in order, deduplicated."""
    ordered = [os.path.dirname(default_calibration_path()), os.getcwd()]
    ordered.extend(str(d) for d in (extra_dirs or ()) if d)

    result: List[str] = []
    for directory in ordered:
        if directory and directory not in result:
            result.append(directory)
    return result


def _json_files(directory: str) -> List[str]:
    """Regular ``*.json`` files in *directory*, newest first."""
    try:
        names = os.listdir(directory)
    except OSError:
        return []
    paths = [
        os.path.join(directory, name)
        for name in names
        if name.endswith(".json") and os.path.isfile(os.path.join(directory, name))
    ]
    paths.sort(key=_mtime, reverse=True)
    return paths


def discover_calibrations(
    explicit: Optional[str] = None,
    *,
    extra_dirs: Optional[Sequence[str]] = None,
) -> List[str]:
    """List candidate calibration files.

    The default user directory comes first, then the current working
    directory, then *extra_dirs*; within each, newest file first, because the
    GUI has just written the one you want. The SDK's factory calibration is
    excluded — it is offered nowhere, since it belongs to no machine.

    An explicit path is placed first and must exist.

    Args:
        explicit: A path supplied on the command line.
        extra_dirs: Additional directories to scan.

    Returns:
        Candidate paths, deduplicated by real path.

    Raises:
        CalibrationFileError: *explicit* was given but does not exist.
    """
    found: List[str] = []
    seen = set()

    def _add(candidate: str) -> None:
        abspath = os.path.abspath(os.path.expanduser(str(candidate)))
        if not os.path.isfile(abspath):
            return
        if _same_file(abspath, sdk_factory_calibration_path()):
            return
        key = os.path.realpath(abspath)
        if key in seen:
            return
        seen.add(key)
        found.append(abspath)

    if explicit:
        abspath = os.path.abspath(os.path.expanduser(str(explicit)))
        if not os.path.isfile(abspath):
            raise CalibrationFileError(
                f"--calibration 指定的文件不存在: {abspath}"
            )
        _add(abspath)

    _add(default_calibration_path())
    for directory in _scan_dirs(extra_dirs):
        for path in _json_files(directory):
            _add(path)

    # 可用的排前面，其余按新旧。扫描目录里躺着 `.releaserc.json` 之类的东西
    # 是常态，它们该沉到底下去，而不是跟真标定混在一起。
    head = found[:1] if explicit else []
    rest = found[len(head):]
    rest.sort(key=lambda p: (0 if _is_selectable(p) else 1, -_mtime(p)))
    return head + rest


def _is_selectable(path: str) -> bool:
    """True when the file would pass :func:`load_calibration_file`."""
    try:
        load_calibration_file(path)
    except CalibrationError:
        return False
    return True


# ═════════════════════════════════════════════════════════════════════════
# Selection
# ═════════════════════════════════════════════════════════════════════════


def _no_choice_message(reason: str) -> str:
    return "\n".join([
        f"[错误] 这次运行没有可用的标定文件：{reason}",
        "  默认用的是 SDK 包里那份出厂标定（台架夹具的实测参数）；"
        "要按这台夹爪",
        "  自己的尺寸驱动，请显式指定上位机保存的那份：",
        "     --calibration <路径>",
        "  标定文件由上位机标定后保存：litegrip-studio / litegrip-console，"
        "或 SDK",
        "  自带的 tools/gui/litegrip_gui.py。",
        "  要看看本机有哪些候选：--list-calibrations",
    ])


def resolve_calibration_path(explicit: Optional[str] = None) -> str:
    """Decide which calibration file this run will use.

    Two tiers, and the order is the priority:

    1. *explicit* — a path from ``--calibration`` — always wins and is
       validated immediately.
    2. Otherwise the SDK's packaged factory calibration, which is the default:
       it ships with the package, so a fresh machine runs without hunting for
       a file.

    There is no third tier. When neither works, this raises instead of scanning
    the machine for candidates: guessing which JSON belongs to which gripper is
    not something this package can do. Use :func:`discover_calibrations` to list
    them for a human, who then names one.

    Args:
        explicit: Path from ``--calibration``, or ``None``.

    Returns:
        The chosen, validated path.

    Raises:
        CalibrationRequiredError: No path was given and the SDK ships no
            readable factory calibration.
        CalibrationFileError: The explicit path does not exist or is invalid.
    """
    if explicit:
        load_calibration_file(explicit)  # 先验，出错时信息里带的是文件问题
        return os.path.abspath(os.path.expanduser(str(explicit)))

    factory = sdk_factory_calibration_path()
    if factory is None:
        raise CalibrationRequiredError(
            _no_choice_message("没有指定 --calibration，也没找到 SDK 自带的出厂标定")
        )
    try:
        load_calibration_file(factory)
    except CalibrationError as exc:
        raise CalibrationRequiredError(
            _no_choice_message(f"SDK 自带的出厂标定读不出来（{exc}）")
        ) from None
    return factory


# ═════════════════════════════════════════════════════════════════════════
# Provenance
# ═════════════════════════════════════════════════════════════════════════


def is_simulated_device(device: Any) -> bool:
    """True for a device whose endpoints are analytical truth, not calibrated.

    :class:`~litegrip_mujoco.MujocoGripper` and
    :class:`~litegrip_mujoco.DryRunGripper` set ``IS_SIMULATED = True``. A
    third-party simulated backend should do the same, or call
    :func:`mark_calibrated`.
    """
    return bool(getattr(device, "IS_SIMULATED", False))


def _store(device: Any, provenance: Provenance) -> None:
    try:
        setattr(device, MARKER_ATTR, provenance)
        return
    except Exception:  # noqa: BLE001 — __slots__ 之类的对象
        pass
    try:
        _MARKERS[device] = provenance
    except TypeError:
        warnings.warn(
            f"{type(device).__name__} 既不能 setattr 也不能弱引用，标定来源无法记录；"
            "read_frac_open / write_frac_open 会认为它没有标定。",
            RuntimeWarning,
            stacklevel=3,
        )


def _provenance(device: Any) -> Optional[Provenance]:
    marker = getattr(device, MARKER_ATTR, None)
    if isinstance(marker, Provenance):
        return marker
    try:
        return _MARKERS.get(device)
    except TypeError:  # pragma: no cover - 不可哈希的对象
        return None


def mark_calibrated(
    device: Any,
    path: Optional[str] = None,
    *,
    reason: str = "",
) -> Optional[Calibration]:
    """Record where a device's endpoints come from, without loading anything.

    For the rare cases :func:`apply_calibration` does not cover: a device the
    SDK's own ``load_calibration`` has already configured, or a deliberate
    opt-out (``path=None``).

    Args:
        device: The gripper.
        path: The file the endpoints came from, or ``None`` to opt out.
        reason: Why, shown in messages and reports.

    Returns:
        The validated file when *path* was given, else ``None``.
    """
    calibration = load_calibration_file(path) if path is not None else None
    _store(
        device,
        Provenance(
            path=calibration.path if calibration else None,
            calibration=calibration,
            reason=reason,
        ),
    )
    return calibration


def applied_calibration(device: Any) -> Optional[Calibration]:
    """The calibration file applied to *device*, or ``None``.

    ``None`` means either nothing was applied or the device was deliberately
    opted out; use :func:`is_calibrated` to tell those apart.
    """
    provenance = _provenance(device)
    return None if provenance is None else provenance.calibration


def is_calibrated(device: Any) -> bool:
    """True when the device's endpoints have a known provenance.

    Simulated devices count: their endpoints are analytical truth, so there is
    nothing to be unsure about.
    """
    return is_simulated_device(device) or _provenance(device) is not None


def apply_calibration(
    device: Any,
    path: Any,
    *,
    verify: bool = True,
) -> Calibration:
    """Load a calibration file into a device and verify it took effect.

    Two layers, because the SDK's silent fallback is not detectable by either
    one alone:

    1. The file is validated before the SDK sees it
       (:func:`load_calibration_file`), so the fallback's triggers — a missing
       file, a malformed one — cannot occur.
    2. The endpoints observed after loading must equal the file's exactly.
       This covers the residual race (the file replaced between our read and
       the SDK's) and any future change to the SDK's fallback.

    The SDK's packaged factory calibration is an ordinary file here: it is the
    default a run uses when nothing is named. It holds the *test-bench
    fixture's* measured endpoints, not this gripper's, so whatever reports
    which file was used has to say that out loud —
    ``examples/_common.open_real_gripper`` prints the path for exactly this
    reason.

    Args:
        device: The gripper to configure.
        path: The calibration file.
        verify: Whether to run layer 2. Leave this on.

    Returns:
        The validated file that was applied.

    Raises:
        CalibrationFileError: The file is invalid, or the device is simulated.
        CalibrationVerificationError: The device did not take the values.
    """
    calibration = load_calibration_file(path)

    if is_simulated_device(device):
        raise CalibrationFileError(
            f"{type(device).__name__} 是仿真设备（IS_SIMULATED=True），"
            "它的端点是解析真值，不套用真机标定文件。\n"
            "       真机路径请用 apply_calibration(real, path)；"
            "仿真侧不需要标定。"
        )

    loader = getattr(device, "load_calibration", None)
    if loader is None:
        raise CalibrationVerificationError(
            f"{type(device).__name__} 没有 load_calibration()，无法套用标定文件"
        )

    loaded = loader(calibration.path)
    if loaded is False:
        raise CalibrationVerificationError(
            f"{type(device).__name__}.load_calibration({calibration.path!r}) 返回 False —— "
            "标定没有载入"
        )

    if verify:
        config = getattr(device, "config", None)
        observed_closed = getattr(config, "pos_closed_rad", None)
        observed_open = getattr(config, "pos_open_rad", None)
        if observed_closed is None or observed_open is None:
            raise CalibrationVerificationError(
                f"{type(device).__name__} 载入标定后没有 config.pos_closed_rad / "
                "pos_open_rad，无法确认标定是否生效"
            )
        if (float(observed_closed) != calibration.pos_closed_rad
                or float(observed_open) != calibration.pos_open_rad):
            raise CalibrationVerificationError(
                f"{type(device).__name__} 载入后的端点与文件不符 —— 标定没有生效。\n"
                f"       要求 {calibration.path}: "
                f"closed={calibration.pos_closed_rad} open={calibration.pos_open_rad}\n"
                f"       实际 closed={observed_closed} open={observed_open}\n"
                "       常见原因：SDK 的 load_calibration() 静默回退到了它自带的"
                "出厂标定（文件缺失或损坏时它会这样做，并且仍然返回 True）。"
            )

    _store(
        device,
        Provenance(path=calibration.path, calibration=calibration, reason="applied"),
    )
    return calibration


def _check_device_still_holds(device: Any) -> None:
    """Raise when a marked device's endpoints drifted away from its file."""
    provenance = _provenance(device)
    if provenance is None or provenance.calibration is None:
        return
    config = getattr(device, "config", None)
    closed = getattr(config, "pos_closed_rad", None)
    open_ = getattr(config, "pos_open_rad", None)
    if closed is None or open_ is None:
        raise CalibrationVerificationError(
            f"{type(device).__name__} 的 config 丢了 θ 端点，"
            f"但它的标定是 {provenance.path}"
        )
    if (float(closed) != provenance.calibration.pos_closed_rad
            or float(open_) != provenance.calibration.pos_open_rad):
        raise CalibrationVerificationError(
            f"{type(device).__name__} 的端点在本进程里被改动过（例如又跑了一次 "
            f"calibrate()），已不再等于 {provenance.path} 里的值。\n"
            "       请重新选择并套用标定文件。"
        )


def select_calibration_for(
    device: Any,
    path: Optional[str] = None,
    *,
    allow_uncalibrated: bool = False,
) -> Optional[str]:
    """Decide which calibration *device* will use, without loading it yet.

    Split out from :func:`require_calibration` for callers that must obey the
    SDK's documented order — ``connect()``, then load, then ``enable()`` — and
    so must resolve the choice before the device is connected.

    Args:
        device: The gripper.
        path: Calibration file, or ``None`` for the SDK's factory calibration.
        allow_uncalibrated: Skip the requirement and record the opt-out.

    Returns:
        The path to apply later, or ``None`` when nothing needs applying
        (simulated device, already calibrated, or opted out).

    Raises:
        CalibrationRequiredError: Nothing was chosen and nothing could be.
        CalibrationFileError: The chosen file is invalid.
    """
    if is_simulated_device(device):
        return None

    if path is not None:
        load_calibration_file(path)  # 早失败：文件有问题就不该等到连上 CAN 才报
        return os.path.abspath(os.path.expanduser(str(path)))

    if _provenance(device) is not None:
        return None

    if allow_uncalibrated:
        print("[标定] ⚠ 本次运行不校验标定（allow_uncalibrated=True）—— "
              "真机会按它当前的 config 运动。")
        mark_calibrated(device, None, reason="allow_uncalibrated=True")
        return None

    return resolve_calibration_path(None)


def require_calibration(
    device: Any,
    path: Optional[str] = None,
    *,
    allow_uncalibrated: bool = False,
) -> Optional[Calibration]:
    """Make sure *device* is usable, choosing and applying a file if needed.

    This is the entry point library callers use. It is a no-op for simulated
    devices, applies *path* when given, accepts a device that already carries
    a provenance marker, and otherwise uses the SDK's packaged factory
    calibration.

    Args:
        device: The gripper.
        path: Calibration file, or ``None`` for the SDK's factory calibration.
        allow_uncalibrated: Skip the requirement and record the opt-out.

    Returns:
        The calibration applied, or ``None`` for a simulated or opted-out
        device.

    Raises:
        CalibrationRequiredError: Nothing was chosen and nothing could be.
        CalibrationError: The chosen file is invalid or did not take effect.
    """
    chosen = select_calibration_for(
        device, path, allow_uncalibrated=allow_uncalibrated
    )
    if chosen is None:
        return applied_calibration(device)
    return apply_calibration(device, chosen)


def require_usable_device(device: Any, *, action: str) -> None:
    """Enforce the two gates before a conversion touches a device.

    Gate 1 — provenance: the endpoints belong to a calibration someone chose,
    or the device is simulated, or the operator explicitly opted out.
    Gate 2 — validity: the endpoints satisfy the invariant (checked by the
    caller through ``_endpoints``).

    Raises:
        UncalibratedDeviceError: No provenance.
        CalibrationVerificationError: Provenance present but the endpoints
            have drifted away from it.
    """
    if is_simulated_device(device):
        return
    if _provenance(device) is None:
        raise UncalibratedDeviceError(
            f"{type(device).__name__} 没有标定来源，不能{action}。\n"
            "       真机必须先载入一份标定：apply_calibration(device, path)，"
            "或 DualGripper(calibration=path)；\n"
            "       缺省那份是 SDK 自带的出厂标定，"
            "用 resolve_calibration_path(None) 拿到它的路径。\n"
            "       确实要用当前 config 硬跑，可以显式 mark_calibrated(device, None, "
            "reason=...)。"
        )
    _check_device_still_holds(device)
