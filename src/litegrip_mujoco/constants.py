"""LiteGrip 仿真侧常量与单位换算。

本模块刻意**不** import `litegrip` SDK 的任何默认值。原因见下。

════════════════════════════════════════════════════════════════════════
两套"毫米"口径 —— 这是本仓库最容易搞错的地方
════════════════════════════════════════════════════════════════════════

SDK 的 `UnitConversion.RAD_TO_MM = 120/1.14 ≈ 105.26`，`GripperConfig.max_stroke_mm
= 120.0`，即它认为全行程是 **120 mm 名义值**。但 URDF 实测机械行程是
**85.452 mm**（两指各 42.726 mm）。二者差 40%。

`litegrip-mujoco` 统一采用**真实毫米**：仿真里报出的 85.452 mm 就是卡尺量出来的
85.452 mm。要在真机侧也对齐，只需把真机的 `GripperConfig.max_stroke_mm` 设成
85.452 并重新标定，真机就同样直接报真实毫米——两侧零换算。

注意 SDK 的 `GripperState.aperture_mm` docstring 说 `position_mm` 是"单侧位移，
总开口要乘 2"。这与 `LiteGrip.goto()` 的实际算法矛盾：`goto()` 里
`position_rad = pos_closed_rad - position_mm / rad_to_mm`，`position_mm` 被当作
**从闭合位起算的全行程**使用。本模块以 `goto()` 的实际算法为准（那才是能被用户
观测到的行为），并把"绝对开口"另立一个量（见 `q_to_gap_mm`）。

════════════════════════════════════════════════════════════════════════
角度 θ 的零点与量程是**设备相关**的
════════════════════════════════════════════════════════════════════════

`position_rad`（θ）的零点和量程都随标定而变：

    仿真   θ_closed =  0.000000   θ_open = -1.140000   量程 1.1400 rad
    真机   θ_closed =  1.775959   θ_open = -0.064279   量程 1.8402 rad

（真机数据取自 `~/.litegrip/litegrip_calibration.json`。）

所以**θ 不能在仿真与真机之间直接互换**。跨设备唯一可移植的量是无量纲的
`frac_open ∈ [0, 1]`。镜像/双控一律走它，见 `mirror.py`。
"""
from __future__ import annotations

import math
from typing import Final

# ═════════════════════════════════════════════════════════════════════════
# 几何 —— 全部来自 litegrip-urdf，不是估计值
# ═════════════════════════════════════════════════════════════════════════

#: 单指行程 (m)。URDF xacro 的实测值（`stroke` arg 默认 0.042726）。
#: URDF 注释里另有一个 mesh 理论行程 0.0435（两指内侧面恰好贴合于 x=0）。
#: 这里取实测值，因为那才是真实机械硬限位；用 0.0435 会让 close() 越过物理
#: 限位 0.77 mm，并给每个上报的毫米数带上固定偏移。
STROKE: Final = 0.042726

#: 总开口行程的真实毫米数 = 2 × 42.726 mm。
#: 这就是 SDK 里 `max_stroke_mm` 应当被设成的值（默认是错的 120.0）。
MM_SCALE: Final = 85.452

#: 完全闭合时两指内侧面之间的物理间隙 (mm)。
#: 由网格几何决定：两指内侧面各在 x = ±0.000774 m。注意 URDF 注释里写的
#: 1.508 mm 是按"实测张开 86.960 mm"算的，而 87.000 - 85.452 给出 1.548 mm。
#: 本仓库以网格几何为准，即 1.548 mm（见 litegrip.xml 的关节 range 与测试）。
GAP_CLOSED_MM: Final = 1.548

#: 完全张开时两指内侧面之间的开口 (mm)。q=0 时精确成立。
GAP_OPEN_MM: Final = 87.000

#: 关节滑动范围 (m)。MJCF 的 joint range 即 [0, STROKE]。
Q_MIN: Final = 0.0
Q_MAX: Final = STROKE

# ═════════════════════════════════════════════════════════════════════════
# θ（电机 rad）—— 仿真侧的自洽取值
# ═════════════════════════════════════════════════════════════════════════

#: 仿真侧的 θ 端点。**刻意不 import SDK 的 POS_CLOSED_RAD / POS_OPEN_RAD**：
#: SDK 的默认值是 `POS_CLOSED_RAD = 0.0` / `POS_OPEN_RAD = +1.14`，而它自己在
#: models.py 的注释里写明不变量是"闭合位数值更大"——默认值违反了自己写的不变量，
#: 于是 `goto_rad()` 的 clamp
#:     max(pos_open_rad, min(pos_closed_rad, x))
#: 对任意 x 都返回 +1.14（实测 goto_rad(0.0/0.5/-1.14/3.0) 全部 → +1.1400）。
#: 仿真侧取 open = -1.14，满足不变量，clamp 行为正常。
POS_CLOSED_RAD: Final = 0.0
POS_OPEN_RAD: Final = -1.14

#: θ 对关节位移的比值 (rad/m) = 1.14 / 0.042726 ≈ 26.677。
#: 由"全行程对应 1.14 rad"这一 SDK 约定唯一确定。
R: Final = (POS_CLOSED_RAD - POS_OPEN_RAD) / STROKE

# ═════════════════════════════════════════════════════════════════════════
# 执行器与增益
# ═════════════════════════════════════════════════════════════════════════

#: 关节力 = GEAR × 电机力矩。取 10 是为了让 `data.ctrl` 的数值**就等于** SDK 的
#: `tau`(Nm)：MJCF 里 `<motor gear="10">` 使 `qfrc_actuator = 10 × ctrl`。
#:
#: ⚠ 这个 10 **不是实测的机械减速比**，而是 SDK 自己 `NM_TO_N = 10` 这一"近似
#: 换算"约定的编码（constants.py 原注释即写 "approximate N per Nm"）。也就是说
#: 仿真复刻的是 SDK 的**力语义**：`force_n` 牛顿 ↔ `force_n × 0.1` Nm。
#: 实测吻合：tau=1.0 Nm 时每指法向力 10.000 N。
GEAR: Final = 10.0

#: 电机力矩上限 (Nm)，取自 DM4310 / `GripperParams.TAU_MAX`。
TAU_MAX: Final = 10.0

#: 关节力上限 (N) = GEAR × TAU_MAX。
FORCE_MAX: Final = GEAR * TAU_MAX

#: 默认 MIT 增益。取自 `GripperParams.DEFAULT_KP/DEFAULT_KD`，不是 litearm 的
#: 260/5。这两个增益作用在 **θ 空间**，因此真机与仿真对同样的电机角误差产生
#: 同样的力矩，不需要移植增益。
DEFAULT_KP: Final = 100.0
DEFAULT_KD: Final = 2.0

#: 抓取判定阈值 (Nm)。取自 `GripperConfig.grasp_torque_threshold`，
#: 与 `LiteGrip.is_grasped()` 的判据一致（|torque| > 该值）。
GRASP_TORQUE_THRESHOLD: Final = 0.5

#: Nm ↔ N 换算，逐字沿用 SDK 的 `UnitConversion`。
NM_TO_N: Final = 10.0
N_TO_NM: Final = 0.1

#: 仿真积分步长 (s)。实测 dt=2ms 时 kd>=5 不收敛；dt=1ms 全稳。
TIMESTEP: Final = 0.001


# ═════════════════════════════════════════════════════════════════════════
# 换算函数
# ═════════════════════════════════════════════════════════════════════════
#
# 记号：
#   q         关节滑动量 (m)，两指同号，0=张开，STROKE=闭合
#   theta     电机角 (rad)，0=闭合，POS_OPEN_RAD=张开
#   mm        从闭合位起算的行程 (mm)，0=闭合，MM_SCALE=张开
#             （= SDK 的 `position_mm`，也是 `goto()` 接受的量）
#   gap       两指内侧面之间的**绝对开口** (mm)，GAP_CLOSED_MM=闭合
#   frac_open 无量纲开度，0=闭合，1=张开（跨设备唯一可移植的量）


def frac_open_from_q(q: float) -> float:
    """关节位移 → 无量纲开度 (0=闭合, 1=张开)。"""
    return 1.0 - q / STROKE


def q_from_frac_open(frac_open: float) -> float:
    """无量纲开度 → 关节位移 (m)。"""
    return STROKE * (1.0 - frac_open)


def mm_to_q(position_mm: float) -> float:
    """SDK 行程 (mm) → 关节位移 (m)。"""
    return STROKE * (1.0 - position_mm / MM_SCALE)


def q_to_mm(q: float) -> float:
    """关节位移 (m) → SDK 行程 (mm)。"""
    return MM_SCALE * (1.0 - q / STROKE)


def q_to_theta(q: float) -> float:
    """关节位移 (m) → 电机角 (rad)。"""
    return R * (q - STROKE)


def theta_to_q(theta: float) -> float:
    """电机角 (rad) → 关节位移 (m)。"""
    return STROKE + theta / R


def mm_to_theta(position_mm: float) -> float:
    """SDK 行程 (mm) → 电机角 (rad)。

    与 `LiteGrip.goto()` 的算式等价：
        position_rad = pos_closed_rad - position_mm / rad_to_mm
    其中 `rad_to_mm = MM_SCALE / 1.14 = 74.958`。仿真侧 `rad_to_mm` 就是这个值。
    """
    return POS_CLOSED_RAD + (position_mm / MM_SCALE) * (POS_OPEN_RAD - POS_CLOSED_RAD)


def theta_to_mm(theta: float) -> float:
    """电机角 (rad) → SDK 行程 (mm)。"""
    return MM_SCALE * (theta - POS_CLOSED_RAD) / (POS_OPEN_RAD - POS_CLOSED_RAD)


def theta_to_frac_open(theta: float) -> float:
    """电机角 (rad) → 无量纲开度。

    **这是镜像/双控跨设备交换状态时唯一该用的换算**，配合各设备自己的
    θ 端点。裸 θ 不可跨设备互换（见模块 docstring）。
    """
    return (theta - POS_CLOSED_RAD) / (POS_OPEN_RAD - POS_CLOSED_RAD)


def frac_open_to_theta(frac_open: float) -> float:
    """无量纲开度 → 电机角 (rad)。"""
    return POS_CLOSED_RAD + frac_open * (POS_OPEN_RAD - POS_CLOSED_RAD)


def q_to_gap_mm(q: float) -> float:
    """关节位移 (m) → 两指内侧面之间的绝对开口 (mm)。

    与 `q_to_mm()` 的区别：`q_to_mm()` 是从闭合位起算的**行程**（SDK 口径，
    闭合时为 0），本函数是**绝对开口**（闭合时为 GAP_CLOSED_MM）。
    """
    return GAP_CLOSED_MM + q_to_mm(q)


def gap_mm_to_q(gap_mm: float) -> float:
    """绝对开口 (mm) → 关节位移 (m)。

    例：要夹住 20 mm 见方的工件，取 ``gap_mm_to_q(20.0)``。
    """
    return mm_to_q(gap_mm - GAP_CLOSED_MM)


def nm_to_n(torque_nm: float) -> float:
    """电机力矩 (Nm) → 夹持力估计 (N)。

    逐字沿用 `LiteGrip.get_state()` 里的 ``force_n = torque_nm * NM_TO_N``。
    它是一个**由力矩推算的估计值**，不是接触力测量值——真机如此，仿真亦如此，
    这样 `get_force()` 在两侧才是同一个量（见 gripper.py 的说明）。
    """
    return torque_nm * NM_TO_N


def n_to_nm(force_n: float) -> float:
    """夹持力 (N) → 电机力矩前馈 (Nm)。等价于 SDK 的 ``N_TO_NM``。"""
    return force_n * N_TO_NM


def to_rad_s(velocity_m_s: float) -> float:
    """关节速度 (m/s) → 电机角速度 (rad/s)。"""
    return velocity_m_s * R


def to_m_s(velocity_rad_s: float) -> float:
    """电机角速度 (rad/s) → 关节速度 (m/s)。"""
    return velocity_rad_s / R


def clamp_theta(theta: float) -> float:
    """把 θ 限制在仿真侧的开合区间内。

    与 `LiteGrip.goto_rad()` 的 clamp 语义相同（这里因为满足
    ``POS_CLOSED_RAD > POS_OPEN_RAD`` 而真正起作用）。
    """
    return max(min(POS_CLOSED_RAD, POS_OPEN_RAD),
               min(max(POS_CLOSED_RAD, POS_OPEN_RAD), theta))


def clamp_q(q: float) -> float:
    """把关节位移限制在 [0, STROKE]。"""
    return max(Q_MIN, min(Q_MAX, q))


#: 由 θ 空间增益推出的等效关节空间增益系数：``kp_q = GEAR * kp * R``。
#: 推导：tau = kp·Δθ = kp·R·Δq，而 qfrc = GEAR·tau。故 kp_q = GEAR·kp·R。
GAIN_TO_JOINT: Final = GEAR * R


def damping_ratio(kp: float = DEFAULT_KP, kd: float = DEFAULT_KD,
                  mass: float = 0.85) -> float:
    """θ 空间增益对应的关节空间阻尼比 ζ（供文档与测试引用）。

    ζ = kd_q / (2·sqrt(kp_q·m))，其中 kp_q/kd_q 是上面的等效关节增益，
    m 是指的等效质量（DM4310 转子惯量折算后的 0.85 kg，见 litegrip.xml）。

    默认增益给出 ζ ≈ 1.7，是过阻尼——这是减速传动应有的，不要去"修正"它。
    """
    kp_q = GAIN_TO_JOINT * kp
    kd_q = GAIN_TO_JOINT * kd
    return kd_q / (2.0 * math.sqrt(kp_q * mass))


def natural_frequency(kp: float = DEFAULT_KP, mass: float = 0.85) -> float:
    """关节空间无阻尼自然频率 ω_n (rad/s)，供稳定性判断。"""
    return math.sqrt(GAIN_TO_JOINT * kp / mass)


#: 真机标定的两个 θ 端点（`~/.litegrip/litegrip_calibration.json`），仅作参考与
#: 文档用。仿真**不使用**它们——写在这里是为了让读者看到"θ 端点设备相关"这件事
#: 有具体数字支撑，也是 examples/04 与 05 里 `--dry-run` 虚拟夹爪的默认端点。
REAL_POS_CLOSED_RAD: Final = 1.775959
REAL_POS_OPEN_RAD: Final = -0.064279
