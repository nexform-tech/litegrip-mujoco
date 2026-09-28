"""`litegrip` SDK 数据模型的本地兜底实现。

**这不是 SDK 的替代品，只是接口的逐字段镜像。** 存在的理由：`litegrip` 的
`__init__.py` 在模块级 `from . import can`，会把 SocketCAN 依赖拖进一个纯仿真包；
而 `MujocoGripper.get_state()` 又必须返回一个接口正确的状态对象（否则
`state.is_grasped` / `aperture_mm` 这些属性在没装 SDK 的机器上就没了）。

字段、默认值、属性语义逐条对齐 `litegrip/models.py` 与 `litegrip/constants.py`。
装了 SDK 时 `_litegrip/__init__.py` 会优先用真的那些，本文件不会被用到。

**唯一的有意偏离**：`GripperConfig` 的默认值采用仿真侧的自洽值
（`pos_closed_rad = 0.0` / `pos_open_rad = -1.14` / `max_stroke_mm = 85.452`），
而不是 SDK 里那组违反自身不变量的默认值。理由见 `constants.py` 的模块 docstring。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum, IntFlag
import time
from typing import Optional

# 仿真侧的自洽默认值。**不要**改成 SDK 的 0.0 / +1.14 / 120.0。
from ..constants import (
    DEFAULT_KD,
    DEFAULT_KP,
    GRASP_TORQUE_THRESHOLD,
    MM_SCALE,
    NM_TO_N,
    POS_CLOSED_RAD,
    POS_OPEN_RAD,
)

#: 仿真侧 rad_to_mm = MM_SCALE / 1.14 = 74.958。SDK 的名义值是 120/1.14 = 105.26。
FALLBACK_RAD_TO_MM = MM_SCALE / abs(POS_CLOSED_RAD - POS_OPEN_RAD)


class GripperMode(IntEnum):
    """Gripper control mode."""
    MIT = 0
    POSITION = 1
    VELOCITY = 2
    FORCE = 3


class GripperStatus(IntFlag):
    """Gripper status flags (bitmask)."""
    NONE = 0x00
    ENABLED = 0x01
    MOVING = 0x02
    AT_TARGET = 0x04
    GRASPED = 0x08
    ERROR = 0x10


@dataclass
class GripperState:
    """Live gripper state snapshot."""

    position_rad: float = 0.0
    velocity_rad_s: float = 0.0
    torque_nm: float = 0.0
    temperature_mos: int = 0
    temperature_coil: int = 0
    error_code: int = 0
    timestamp: float = field(default_factory=time.time)

    # Convenience — computed from raw values with unit conversion
    position_mm: float = 0.0
    force_n: float = 0.0

    @property
    def is_enabled(self) -> bool:
        """True if the motor is enabled (error_code == 1)."""
        return self.error_code == 1

    @property
    def is_error(self) -> bool:
        """True if a fault is active (error_code ∉ {0, 1})."""
        return self.error_code not in (0, 1)

    @property
    def is_moving(self) -> bool:
        """True if velocity exceeds a small threshold."""
        return abs(self.velocity_rad_s) > 0.01

    @property
    def aperture_mm(self) -> float:
        """Opening distance in mm (single-side displacement)."""
        return self.position_mm


@dataclass
class GripperConfig:
    """Gripper configuration — 仿真侧默认值，见模块 docstring。"""

    # CAN
    can_channel: str = "can0"
    can_id: int = 0x08
    mst_id: Optional[int] = None
    canfd_mode: bool = False

    # Control gains (MIT mode)
    kp: float = DEFAULT_KP
    kd: float = DEFAULT_KD

    # Position limits (rad) — 仿真侧满足 "闭合位数值更大" 的不变量
    pos_closed_rad: float = POS_CLOSED_RAD
    pos_open_rad: float = POS_OPEN_RAD

    # Mechanical stroke (mm) — 真实行程，不是 SDK 名义的 120
    max_stroke_mm: float = MM_SCALE

    # Unit conversion
    rad_to_mm: float = FALLBACK_RAD_TO_MM
    nm_to_n: float = NM_TO_N

    # Grasp detection
    grasp_torque_threshold: float = GRASP_TORQUE_THRESHOLD


@dataclass
class GripperInfo:
    """Static device information."""

    model: str = "LiteGrip"
    motor_type: str = "DM4310"
    can_id: int = 0x08
    mst_id: int = 0x18
    firmware_version: str = ""
    serial_number: str = ""


@dataclass
class CalibrationData:
    """Result of a gripper calibration run."""

    zero_position: float = 0.0
    max_position: float = -1.14
    travel_range: float = 1.14
    rad_to_mm: float = FALLBACK_RAD_TO_MM
    motor_type: str = "DM4310"
    can_id: int = 0x08
    mst_id: int = 0x18
    calibration_time: str = ""

    @property
    def travel_mm(self) -> float:
        """Full stroke in mm."""
        return self.travel_range * self.rad_to_mm


class GripperParams:
    """LiteGrip default parameters（仿真侧取值）。"""

    CAN_ID = 0x08
    MST_ID = 0x18
    MOTOR_TYPE = "DM4310"

    # Position limits (rad) — closed > open numerically
    POS_CLOSED_RAD = POS_CLOSED_RAD
    POS_OPEN_RAD = POS_OPEN_RAD

    # MIT quantization limits (DM4310)
    Q_MAX = 12.5      # rad
    DQ_MAX = 30.0     # rad/s
    TAU_MAX = 10.0    # Nm

    DEFAULT_KP = DEFAULT_KP
    DEFAULT_KD = DEFAULT_KD


class UnitConversion:
    """Unit conversion coefficients（仿真侧：真实毫米）。"""

    RAD_TO_MM = FALLBACK_RAD_TO_MM     # ≈ 74.96 mm/rad（SDK 名义值是 105.26）
    MM_TO_RAD = 1.0 / FALLBACK_RAD_TO_MM
    NM_TO_N = NM_TO_N
    N_TO_NM = 1.0 / NM_TO_N


class ErrorCode:
    """Damiao motor error codes."""
    DISABLED = 0
    ENABLED = 1
    UV_FAULT = 0x9
    OC_FAULT = 0xA
    MOS_OT = 0xB
    COIL_OT = 0xC


class LiteGripError(Exception):
    """Base error (fallback stub)."""


class NotInitializedError(LiteGripError):
    """Raised when the gripper is not connected or not enabled."""
