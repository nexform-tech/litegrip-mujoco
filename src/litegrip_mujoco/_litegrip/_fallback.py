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

#: 对齐 `litegrip.models.STALE_AFTER_S`。DM 电机使能时约 10 Hz 发状态帧，
#: 0.5 s 约等于漏掉 5 帧；电机自身的 CAN 超时故障在 ~0.9 s 触发。
STALE_AFTER_S = 0.5


class MotorType(IntEnum):
    """达妙电机型号（对齐 `litegrip.DM_Motor_Type`）。

    ⚠ 类名是 ``MotorType`` 而不是 ``DM_Motor_Type``：SDK 里真正的类就叫
    ``MotorType``，``DM_Motor_Type`` 只是包级的一个别名。名字得跟着类本身走，
    否则 ``repr()`` 会不一样（``<MotorType.DM4310: 1>`` vs
    ``<DM_Motor_Type.DM4310: 1>``），而 ``GripperParams.MOTOR_TYPE`` 这类
    值的 repr 是会进日志的。别名在下面补上。
    """
    DM3507 = 0
    DM4310 = 1
    DM4310_48V = 2
    DM4340 = 3
    DM4340_48V = 4
    DM6006 = 5
    DM6248P = 6
    DM8006 = 7
    DM8009 = 8
    DM10010L = 9
    DM10010 = 10
    DMH3510 = 11
    DMH6215 = 12
    DMS3519 = 13
    DMG6220 = 14


#: SDK 在包级暴露的名字（`litegrip.DM_Motor_Type`）。类名见上面的说明。
DM_Motor_Type = MotorType


class ControlMode(IntEnum):
    """电机控制模式（对齐 `litegrip.Control_Mode`；类名同 SDK，理由见上）。"""
    MIT_MODE = 0
    POS_VEL_MODE = 256
    VEL_MODE = 512
    POS_FORCE_MODE = 768


Control_Mode = ControlMode


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

    # 这批数值来自哪一帧状态；inf = 从未收到。失能的电机不主动发状态帧，
    # 所以首次使能前（或失能后）读到的其实是构造默认值，不是测量值。
    # ⚠ 字段顺序必须与 SDK 一致：data_age_s 在 timestamp 和 position_mm 之间。
    data_age_s: float = float("inf")

    # Convenience — computed from raw values with unit conversion
    position_mm: float = 0.0
    force_n: float = 0.0

    @property
    def has_data(self) -> bool:
        """True if at least one status frame has been decoded."""
        return self.data_age_s != float("inf")

    @property
    def is_stale(self) -> bool:
        """True if the snapshot is not backed by a recent status frame."""
        return self.data_age_s > STALE_AFTER_S

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
    MOTOR_TYPE = DM_Motor_Type.DM4310
    CONTROL_MODE = Control_Mode.MIT_MODE

    # Position limits (rad) — closed > open numerically
    POS_CLOSED_RAD = POS_CLOSED_RAD
    POS_OPEN_RAD = POS_OPEN_RAD

    # MIT quantization limits (DM4310)
    Q_MAX = 12.5      # rad
    DQ_MAX = 30.0     # rad/s
    TAU_MAX = 10.0    # Nm

    DEFAULT_KP = DEFAULT_KP
    DEFAULT_KD = DEFAULT_KD

    # Fault-recovery retry policy（仿真不做 CAN 重试，仅保持接口一致）
    FAULT_CLEAR_RETRIES = 5
    FAULT_CLEAR_DELAY_S = 0.02


class UnitConversion:
    """Unit conversion coefficients（仿真侧：真实毫米）。"""

    RAD_TO_MM = FALLBACK_RAD_TO_MM     # ≈ 74.96 mm/rad（SDK 名义值是 105.26）
    MM_TO_RAD = 1.0 / FALLBACK_RAD_TO_MM
    NM_TO_N = NM_TO_N
    N_TO_NM = 1.0 / NM_TO_N


class ErrorCode:
    """Damiao motor error codes (extracted from status frame data[0] >> 4)."""
    DISABLED = 0
    ENABLED = 1
    OV_FAULT = 0x8
    UV_FAULT = 0x9
    OC_FAULT = 0xA
    MOS_OT = 0xB
    COIL_OT = 0xC
    COMM_LOSS = 0xD
    OVERLOAD = 0xE


#: 对齐 `litegrip.constants.ERROR_DESCRIPTIONS`，含 0x8/0xD/0xE 三个新增码。
ERROR_DESCRIPTIONS = {
    0x0: "已失能",
    0x1: "已使能",
    0x8: "过压故障 (OV)",
    0x9: "欠压故障 (UV)",
    0xA: "过流故障 (OC)",
    0xB: "MOS 过温故障",
    0xC: "线圈过温故障",
    0xD: "通讯丢失 (CAN 超时)",
    0xE: "过载故障",
}


def describe_error(code: int) -> str:
    """Return a human-readable description for a motor error code."""
    return ERROR_DESCRIPTIONS.get(code, f"未知错误 (0x{code:X})")


class LiteGripError(Exception):
    """Base error (fallback stub)."""


class NotInitializedError(LiteGripError):
    """Raised when the gripper is not connected or not enabled."""


# SDK 里这五个都直接继承 LiteGripError。仿真不会抛出它们（没有 CAN 链路），
# 但它们必须在，否则共用代码里的 `except CommError` 在纯仿真环境会 ImportError。
class ConnectError(LiteGripError):
    """Raised when connecting to the CAN interface or motor fails."""


class CommError(LiteGripError):
    """Raised when a CAN exchange fails or the motor reports a fault."""


class CANTimeoutError(LiteGripError):
    """Raised when a CAN frame exchange times out."""


class HardwareError(LiteGripError):
    """Raised for motor-side hardware faults."""


class CommandError(LiteGripError):
    """Raised when a command is rejected by the motor."""
