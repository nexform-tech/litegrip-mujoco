"""θ 空间 PD 控制器与目标斜坡，供仿真物理循环使用。

════════════════════════════════════════════════════════════════════════
为什么 PD 算在 θ 空间，而不是关节空间
════════════════════════════════════════════════════════════════════════

真机 SDK 的 MIT 帧是 ``f(q, dq, kp, kd, tau)``，其中 ``q`` 是**电机角** θ，
``kp``/``kd`` 也是电机侧的增益。DM4310 自己执行这条 PD 律。

要让人写好的 ``kp=150, kd=2`` 在仿真和真机上产生**同样的力矩**，仿真就必须用
同一组数、在同一个空间里算：

    tau = kp·(θ_target − θ) + kd·(θ̇_target − θ̇) + tau_ff

再经 ``gear`` 变成关节广义力：``qfrc = GEAR · tau``。等价的关节空间增益会自动
跟随：``kp_q = GEAR·kp·R``（R = 26.68 rad/m）。所以**不需要移植增益**。

默认 ``kp=100, kd=2`` 对应关节空间 ``kp_q ≈ 26682 N/m``、``kd_q ≈ 533.6 N·s/m``，
阻尼比 ζ ≈ 1.77、ω_n ≈ 177 rad/s —— 过阻尼，这是减速传动应有的，不要"修正"。
（力矩上限 TAU_MAX=10 Nm 对应约 3.75 mm 的位置误差，全行程移动必然饱和。）

════════════════════════════════════════════════════════════════════════
为什么用 <motor> + 外部 PD，而不是 <position>
════════════════════════════════════════════════════════════════════════

MJCF 的 ``<position>`` 内建伺服只接受固定的 ``kp``，无法逐次调用改增益，也无法
表达 ``tau_ff``；而且它的力语义是 MuJoCo 自己的，与 MIT 帧对不上。
用 ``<motor>`` 把力矩直接交给 ``data.ctrl``，MIT 帧的五个量就都有了对应位置。
"""
from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import numpy as np

from .constants import DEFAULT_KD, DEFAULT_KP, TAU_MAX

N_JOINTS = 2


class GripperPDController:
    """θ 空间 PD 控制器，输出直接写进 ``data.ctrl``。

    ``data.ctrl`` 的数值即 SDK 的 ``tau``(Nm) —— 因为 MJCF 的电机带 ``gear=10``。

    Usage::

        ctrl = GripperPDController(kp=100.0, kd=2.0)
        ctrl.set_target(theta_target)
        while ...:
            data.ctrl[:] = ctrl.compute(theta_actual, dtheta_actual)
    """

    def __init__(
        self,
        kp: float = DEFAULT_KP,
        kd: float = DEFAULT_KD,
        tau_max: float = TAU_MAX,
        n_joints: int = N_JOINTS,
    ) -> None:
        self.kp = float(kp)
        self.kd = float(kd)
        self.tau_max = float(tau_max)
        self.n_joints = n_joints

        self._theta_des = 0.0
        self._dtheta_des = 0.0
        self._tau_ff = 0.0

    # ── 设定 ───────────────────────────────────────────────────────────

    def set_target(self, theta: float, dtheta: float = 0.0) -> None:
        """设定目标电机角 (rad) 与目标角速度 (rad/s)。"""
        self._theta_des = float(theta)
        self._dtheta_des = float(dtheta)

    def set_feedforward(self, tau_nm: float) -> None:
        """设定力矩前馈 (Nm)。SDK 的 ``tau_feedforward``。

        夹持力就是靠这个量表达的：``tau_ff = force_n × N_TO_NM``。
        """
        self._tau_ff = float(tau_nm)

    def set_gains(self, kp: Optional[float] = None, kd: Optional[float] = None) -> None:
        """逐次调用改增益，对应 MIT 帧里每次都带 kp/kd。"""
        if kp is not None:
            self.kp = float(kp)
        if kd is not None:
            self.kd = float(kd)

    def reset(self) -> None:
        """目标归零、前馈清零。"""
        self._theta_des = 0.0
        self._dtheta_des = 0.0
        self._tau_ff = 0.0

    def hold_current(self, theta: float, tau_ff: Optional[float] = None) -> None:
        """把目标改写到**当前位置**，只留前馈力矩。

        这是力控的承重细节。位置误差归零后 ``kp·Δθ = 0``，关节力就只剩
        ``GEAR·tau_ff``，于是真正的力控才成立。``grasp()`` 检测到堵转后、
        以及 ``set_force()``，都走这条路。

        反过来说：只要目标还停在闭合位上，``close()`` 就**无法限力** —— 位置
        误差会压倒 τ_ff 并让执行器饱和。真机同理，不是仿真缺陷。
        """
        self._theta_des = float(theta)
        self._dtheta_des = 0.0
        if tau_ff is not None:
            self._tau_ff = float(tau_ff)

    # ── 计算 ───────────────────────────────────────────────────────────

    def compute(self, theta: float, dtheta: float) -> np.ndarray:
        """算出一条控制律的力矩，广播到两个指关节。

        两指的关节**轴向相反**，但取同一个值才是对称运动，因此这里两路输出相同。
        """
        tau = (
            self.kp * (self._theta_des - float(theta))
            + self.kd * (self._dtheta_des - float(dtheta))
            + self._tau_ff
        )
        tau = float(np.clip(tau, -self.tau_max, self.tau_max))
        return np.full(self.n_joints, tau, dtype=np.float64)

    @property
    def theta_des(self) -> float:
        return self._theta_des

    @property
    def tau_ff(self) -> float:
        return self._tau_ff

    def __repr__(self) -> str:
        return (
            f"GripperPDController(kp={self.kp:g}, kd={self.kd:g}, "
            f"tau_max={self.tau_max:g}, tau_ff={self._tau_ff:+.4f}, "
            f"theta_des={self._theta_des:+.4f})"
        )


# ═════════════════════════════════════════════════════════════════════════
# 目标斜坡
# ═════════════════════════════════════════════════════════════════════════
#
# 位置族的调用（open/close/goto/move_to）在 SDK 里都是"按 duration 走完一段"，
# 而且 duration 是**真实时钟**。仿真若把目标一步到位，会得到远快于真机的运动；
# 所以目标必须随时间铺开。
#
# 位置族用 min-jerk 斜坡（两端速度为零，实测 duration=1.0 时 0.825 s 到 95%
# 行程，与真机吻合）。grasp()/set_force() 用恒定目标 —— 斜坡会把堵转检测糊掉：
# 目标一直在动，位置就一直在变，永远不会被判为"停住了"。


class Ramp:
    """斜坡基类：按仿真时钟推进的目标。"""

    def __init__(self, start: float, goal: float, duration: float) -> None:
        self._start = float(start)
        self._goal = float(goal)
        self._duration = max(float(duration), 1e-9)
        self._t = 0.0
        self._done = abs(self._goal - self._start) < 1e-12

    @property
    def done(self) -> bool:
        return self._done

    @property
    def goal(self) -> float:
        return self._goal

    @property
    def elapsed(self) -> float:
        return self._t

    def _fraction(self) -> float:
        """归一化时间 u ∈ [0,1]。"""
        return min(1.0, self._t / self._duration)

    def advance(self, dt: float) -> Tuple[float, float]:
        """推进 dt，返回 (当前目标值, 当前目标速度)。"""
        raise NotImplementedError

    def finish(self) -> Tuple[float, float]:
        """直接把目标置到终点。"""
        self._t = self._duration
        self._done = True
        return self._goal, 0.0


class MinJerkRamp(Ramp):
    """最小 jerk 斜坡：s(u) = 10u³ − 15u⁴ + 6u⁵。

    两端位置、速度、加速度都连续，起停不"顿"。慢速指令下比梯形速度剖面更
    贴近真机的 MIT 流行为。
    """

    def advance(self, dt: float) -> Tuple[float, float]:
        if self._done:
            return self._goal, 0.0
        self._t += dt
        u = self._fraction()
        s = 10 * u**3 - 15 * u**4 + 6 * u**5
        ds_du = 30 * u**2 - 60 * u**3 + 30 * u**4
        delta = self._goal - self._start
        if self._t >= self._duration:
            self._done = True
            return self._goal, 0.0
        return self._start + delta * s, delta * ds_du / self._duration


class LinearRamp(Ramp):
    """匀速斜坡 —— 对应 SDK 的 ``move_at_speed``/``move_at_speed_rad``。

    SDK 的 ``_move_at_speed_rad()`` 是一条 ``q = start + (target-start)·frac``
    的线性插值，同时把 ``dq`` 前馈设成恒定速度。
    """

    def __init__(self, start: float, goal: float, speed: float) -> None:
        distance = abs(float(goal) - float(start))
        safe_speed = max(abs(float(speed)), 1e-9)
        super().__init__(start, goal, distance / safe_speed)
        self._speed = safe_speed if goal >= start else -safe_speed

    def advance(self, dt: float) -> Tuple[float, float]:
        if self._done:
            return self._goal, 0.0
        self._t += dt
        u = self._fraction()
        if self._t >= self._duration:
            self._done = True
            return self._goal, 0.0
        return self._start + (self._goal - self._start) * u, self._speed


class Hold:
    """恒定目标（不做斜坡）。用于 grasp() / set_force()。"""

    def __init__(self, value: float) -> None:
        self._value = float(value)

    @property
    def done(self) -> bool:
        return True

    @property
    def goal(self) -> float:
        return self._value

    def advance(self, dt: float) -> Tuple[float, float]:
        return self._value, 0.0


# ═════════════════════════════════════════════════════════════════════════
# 轨迹工具（供 examples/03_trajectory.py 录制与回放）
# ═════════════════════════════════════════════════════════════════════════


def minimum_jerk_trajectory(
    start: float, goal: float, duration: float, dt: float
) -> List[float]:
    """离线生成一条 min-jerk 轨迹的采样点（与 `MinJerkRamp` 同一条曲线）。

    Args:
        start: 起点。
        goal: 终点。
        duration: 总时长 (s)。
        dt: 采样间隔 (s)。

    Returns:
        采样点列表，首项为 start，末项精确为 goal。
    """
    n_steps = max(2, int(round(duration / dt)))
    out: List[float] = []
    for i in range(n_steps + 1):
        u = i / n_steps
        s = 10 * u**3 - 15 * u**4 + 6 * u**5
        out.append(start + (goal - start) * s)
    out[-1] = goal
    return out


def resample(points: Sequence[float], src_dt: float, dst_dt: float) -> List[float]:
    """把一条轨迹从 src_dt 重采样到 dst_dt（线性插值）。

    录制下来的轨迹常常是控制节拍（如 10 ms）采的，回放时可能要按物理步长
    （1 ms）喂进去。
    """
    if len(points) < 2 or src_dt <= 0 or dst_dt <= 0:
        return list(points)
    total = (len(points) - 1) * src_dt
    n_out = max(2, int(round(total / dst_dt)) + 1)
    xs = np.arange(len(points)) * src_dt
    xi = np.linspace(0.0, total, n_out)
    return list(np.interp(xi, xs, np.asarray(points, dtype=float)))
