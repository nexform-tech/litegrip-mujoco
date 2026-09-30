"""litegrip-mujoco —— LiteGrip 夹爪的 MuJoCo 仿真环境。

    from litegrip_mujoco import MujocoGripper

    with MujocoGripper(render=True) as gripper:
        print(gripper.get_position())     # mm（真实毫米）
        gripper.open(duration=1.0)
        gripper.grasp(force_n=10.0)
        print(gripper.get_force())        # N

`MujocoGripper` 的公开 API 与真机 SDK 的 `litegrip.LiteGrip` 逐条对齐，因此
同一段控制代码改一行 import 就能从仿真切到真机::

    from litegrip_mujoco import MujocoGripper as LiteGrip   # 仿真
    from litegrip import LiteGrip                           # 真机

配套的三种联动见 :class:`DualGripper`（同一指令同时下发）与 :class:`MirrorMode`
（真机驱动仿真）；没有 CAN 硬件时用 ``--dry-run`` 配合 :class:`DryRunGripper`。

**毫米口径**：本包一律使用**真实毫米**（全行程 85.452 mm，来自 URDF 实测几何），
而不是 SDK 名义的 120 mm 刻度。详见 :mod:`litegrip_mujoco.constants`。
"""
from __future__ import annotations

from typing import Any, Optional

from . import constants, trajectory, world
from ._litegrip import HAS_SDK, sdk_unavailable_reason
from .calibration import (
    Calibration,
    CalibrationError,
    CalibrationFileError,
    CalibrationRequiredError,
    CalibrationVerificationError,
    UncalibratedDeviceError,
    applied_calibration,
    apply_calibration,
    default_calibration_path,
    describe_candidate,
    discover_calibrations,
    format_selection,
    is_calibrated,
    is_simulated_device,
    load_calibration_file,
    mark_calibrated,
    require_calibration,
    require_usable_device,
    resolve_calibration_path,
    sdk_factory_calibration_path,
    select_calibration_for,
)
from .dryrun import DryRunGripper
from .gripper import DEFAULT_MODEL, SCENE_MODEL, MujocoGripper
from .mirror import DualGripper, MirrorMode, read_frac_open, write_frac_open
from .trajectory import (
    Trajectory,
    TrajectoryBusyError,
    TrajectoryEmptyError,
    TrajectoryError,
    TrajectoryFormatError,
    TrajectoryNotActiveError,
    TrajectoryPlayer,
    TrajectoryRecorder,
    TrajectoryRecordingError,
    TrajectorySample,
    resolve_path,
    trajectory_dir,
)
from .world import MujocoContact
from .window import (
    CONFIRM_KEYS,
    QUIT_KEYS,
    TELEOP_KEYS,
    ZERO_GRAVITY_KEYS,
    KeyQueue,
    clicked,
    held,
    key_codes,
    key_label,
    pressed,
)

try:  # pragma: no cover - 取决于安装方式
    from importlib.metadata import PackageNotFoundError, version as _pkg_version

    try:
        __version__ = _pkg_version("litegrip-mujoco")
    except PackageNotFoundError:
        __version__ = "0.0.0-semantic-release"
except ImportError:  # pragma: no cover - Python < 3.8
    __version__ = "0.0.0-semantic-release"


def _load_sdk() -> Optional[Any]:
    """返回真的 `litegrip` 模块；未安装时返回 None（不抛异常）。"""
    try:
        from ._litegrip import load_litegrip

        return load_litegrip()
    except ImportError:
        return None


#: 真的 `litegrip` SDK 模块，未安装时为 None。
#:
#: 这里**不做** `import litegrip` —— 那个包的 ``__init__`` 会拖进 SocketCAN
#: 依赖。想要真的 SDK 用 :func:`require_sdk`。
litegrip = _load_sdk()


def require_sdk() -> Any:
    """返回真的 `litegrip` SDK 模块，未安装则抛出带安装提示的 ImportError。"""
    from ._litegrip import load_litegrip

    return load_litegrip()


__all__ = [
    # 核心
    "MujocoGripper",
    # 联动
    "DualGripper",
    "MirrorMode",
    "read_frac_open",
    "write_frac_open",
    # 标定：真机运动前必须先选定一份标定文件
    "Calibration",
    "CalibrationError",
    "CalibrationFileError",
    "CalibrationRequiredError",
    "CalibrationVerificationError",
    "UncalibratedDeviceError",
    "applied_calibration",
    "apply_calibration",
    "default_calibration_path",
    "describe_candidate",
    "discover_calibrations",
    "format_selection",
    "is_calibrated",
    "is_simulated_device",
    "load_calibration_file",
    "mark_calibrated",
    "require_calibration",
    "require_usable_device",
    "resolve_calibration_path",
    "sdk_factory_calibration_path",
    "select_calibration_for",
    # 无硬件
    "DryRunGripper",
    # 世界查询
    "MujocoContact",
    "world",
    # 轨迹录制与回放（.lgt 与 SDK 逐字节互通）
    "Trajectory",
    "TrajectorySample",
    "TrajectoryRecorder",
    "TrajectoryPlayer",
    "TrajectoryError",
    "TrajectoryBusyError",
    "TrajectoryNotActiveError",
    "TrajectoryEmptyError",
    "TrajectoryRecordingError",
    "TrajectoryFormatError",
    "resolve_path",
    "trajectory",
    "trajectory_dir",
    # 窗口与键盘输入（例程的循环骨架用）
    "KeyQueue",
    "CONFIRM_KEYS",
    "QUIT_KEYS",
    "TELEOP_KEYS",
    "ZERO_GRAVITY_KEYS",
    "clicked",
    "held",
    "key_codes",
    "key_label",
    "pressed",
    # 常量与工具
    "constants",
    "DEFAULT_MODEL",
    "SCENE_MODEL",
    "litegrip",
    "require_sdk",
    "HAS_SDK",
    "sdk_unavailable_reason",
    "__version__",
]
