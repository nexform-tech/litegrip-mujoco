"""DryRunGripper —— 无 CAN 硬件时顶替真机的虚拟夹爪。

供 ``examples/04_mirror_real.py --dry-run`` 与 ``examples/05_dual_control.py
--dry-run`` 使用，让"夹爪控制仿真""仿真控制夹爪"这两条流程在**没有硬件**的
机器上也能整条跑通。

════════════════════════════════════════════════════════════════════════
为什么不直接拿 MujocoGripper 冒充真机
════════════════════════════════════════════════════════════════════════

因为那样会把要验证的东西验证掉。`MujocoGripper` 的 θ 用仿真端点
（closed=0.0 / open=-1.14），如果"真机"也用它，那么镜像/双控里
``frac_open`` 的换算就退化成恒等变换 —— 恰恰把最容易出错的那一步（跨设备
换算）绕过去了，测试给不出任何保证。

所以本类内部跑一台独立的仿真，但对外**声称自己是真机**：θ 端点用真机标定值
（closed=1.775959 / open=-0.064279，量程 1.8402 rad，与仿真的 1.14 差 61%）。
于是 ``--dry-run`` 走的是与真机完全相同的换算路径，量程差、零点差都真实存在。

另加小幅传感器噪声，`get_state()` 的读数不会假到"一眼就是仿真"。
"""
from __future__ import annotations

import time
from typing import Any, Optional, Tuple

import numpy as np

from . import constants as C
from ._litegrip import (
    CalibrationData,
    GripperConfig,
    GripperInfo,
    GripperState,
)
from .gripper import MujocoGripper

#: 虚拟"真机"的 θ 端点 —— 取自真实标定文件，与仿真端点不同（这是刻意的）。
DRY_POS_CLOSED_RAD = C.REAL_POS_CLOSED_RAD   # 1.775959
DRY_POS_OPEN_RAD = C.REAL_POS_OPEN_RAD       # -0.064279


class DryRunGripper:
    """符合 `litegrip.LiteGrip` 接口的虚拟夹爪，内部是一台独立仿真。

    Args:
        model_path: 内部仿真用的模型。默认 ``assets/litegrip.xml``。
        realtime: 按真实时钟推进（这样 ``duration`` 的量纲与真机一致）。
        noise: 是否给状态读数加小幅传感器噪声。
        seed: 噪声随机种子。

    Usage::

        real = DryRunGripper()          # 顶替 litegrip.LiteGrip
        sim  = MujocoGripper(render=True)
        ...
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        realtime: bool = True,
        noise: bool = True,
        seed: int = 0,
        # 以下为与 LiteGrip 构造签名对齐而接受、仿真中不使用的参数
        config: Optional[GripperConfig] = None,
        channel: str = "can0",
        can_id: int = 0x08,
        mst_id: Optional[int] = None,
        canfd_mode: Optional[bool] = None,
        motor_type: Any = "DM4310",
    ) -> None:
        self._inner = MujocoGripper(
            model_path=model_path, render=False, realtime=realtime
        )
        self._rng = np.random.default_rng(seed)
        self._noise = bool(noise)
        self._channel = channel
        self._can_id = can_id
        self._mst_id = mst_id
        self._canfd_mode = bool(canfd_mode) if canfd_mode is not None else False
        self._motor_type = motor_type

        # 对外声称的配置：真机口径的 θ 端点 + 真实毫米行程。
        self._config = config if config is not None else GripperConfig(
            can_channel=channel,
            can_id=can_id,
            kp=C.DEFAULT_KP,
            kd=C.DEFAULT_KD,
            pos_closed_rad=DRY_POS_CLOSED_RAD,
            pos_open_rad=DRY_POS_OPEN_RAD,
            max_stroke_mm=C.MM_SCALE,
            rad_to_mm=C.MM_SCALE / abs(DRY_POS_CLOSED_RAD - DRY_POS_OPEN_RAD),
            nm_to_n=C.NM_TO_N,
            grasp_torque_threshold=C.GRASP_TORQUE_THRESHOLD,
        )

    # ══════════════════════════════════════════════════════════════════
    # 属性
    # ══════════════════════════════════════════════════════════════════

    @property
    def channel(self) -> str:
        return self._channel

    @property
    def can_id(self) -> int:
        return self._can_id

    @property
    def mst_id(self) -> Optional[int]:
        return self._mst_id

    @property
    def is_connected(self) -> bool:
        return self._inner.is_connected

    @property
    def is_enabled(self) -> bool:
        return self._inner.is_enabled

    @property
    def config(self) -> GripperConfig:
        return self._config

    # ══════════════════════════════════════════════════════════════════
    # 开度换算 —— 真机 θ 端点 ⇄ 内部仿真的 frac_open
    # ══════════════════════════════════════════════════════════════════

    def _frac(self) -> float:
        return self._inner.frac_open()

    def _to_device_theta(self, frac: float) -> float:
        return DRY_POS_CLOSED_RAD + frac * (DRY_POS_OPEN_RAD - DRY_POS_CLOSED_RAD)

    def _from_device_theta(self, theta: float) -> float:
        span = DRY_POS_OPEN_RAD - DRY_POS_CLOSED_RAD
        return (float(theta) - DRY_POS_CLOSED_RAD) / span

    # ══════════════════════════════════════════════════════════════════
    # 连接
    # ══════════════════════════════════════════════════════════════════

    def connect(self) -> bool:
        self._inner.connect()
        return True

    def disconnect(self) -> None:
        self._inner.disconnect()

    def enable(self) -> bool:
        return self._inner.enable()

    def disable(self) -> bool:
        return self._inner.disable()

    def clear_fault(self) -> bool:
        return self._inner.clear_fault()

    def stop(self) -> bool:
        return self._inner.stop()

    def send_mit_frame(self, q: float, kp: float, kd: float,
                       dq: float = 0.0, tau: float = 0.0) -> None:
        """``q`` 按**真机 θ 口径**解释，换算成开度再下发。"""
        frac = self._from_device_theta(q)
        self._inner.set_frac_open(frac)
        self._inner.send_mit_frame(
            C.q_to_theta(C.q_from_frac_open(frac)), kp, kd, dq=dq, tau=tau
        )

    def poll(self, timeout_s: float = 0.0) -> None:
        if timeout_s > 0:
            time.sleep(timeout_s)

    # ══════════════════════════════════════════════════════════════════
    # 运动
    # ══════════════════════════════════════════════════════════════════

    def home(self) -> bool:
        return self._inner.home()

    def open(self, kp: Optional[float] = None, kd: Optional[float] = None,
             duration: float = 1.0) -> bool:
        return self._inner.open(kp=kp, kd=kd, duration=duration)

    def close(self, kp: Optional[float] = None, kd: Optional[float] = None,
              force_n: Optional[float] = None, duration: float = 1.0) -> bool:
        return self._inner.close(kp=kp, kd=kd, force_n=force_n, duration=duration)

    def grasp(self, force_n: float = 10.0, kp: float = 150.0, kd: float = 2.0,
              duration: float = 3.0, stall_threshold: float = 0.001,
              stall_cycles: int = 5) -> bool:
        return self._inner.grasp(
            force_n=force_n, kp=kp, kd=kd, duration=duration,
            stall_threshold=stall_threshold, stall_cycles=stall_cycles,
        )

    def goto(self, position_mm: float, kp: Optional[float] = None,
             kd: Optional[float] = None, duration: float = 0.5) -> bool:
        return self._inner.goto(position_mm, kp=kp, kd=kd, duration=duration)

    def goto_rad(self, position_rad: float, kp: Optional[float] = None,
                 kd: Optional[float] = None, dq_target: float = 0.0,
                 tau_feedforward: float = 0.0, duration: float = 0.5) -> bool:
        """``position_rad`` 按**真机 θ 口径**解释。"""
        frac = self._from_device_theta(position_rad)
        return self._inner.goto_rad(
            C.q_to_theta(C.q_from_frac_open(frac)), kp=kp, kd=kd,
            dq_target=dq_target, tau_feedforward=tau_feedforward,
            duration=duration,
        )

    def move_to(self, target_rad: float, kp: Optional[float] = None,
                kd: Optional[float] = None, tau_feedforward: float = 0.0,
                duration: float = 1.0) -> bool:
        return self.goto_rad(target_rad, kp=kp, kd=kd,
                             tau_feedforward=tau_feedforward, duration=duration)

    def set_force(self, force_n: float, duration: float = 0.3) -> bool:
        return self._inner.set_force(force_n, duration=duration)

    def move_at_speed(self, target_mm: float, speed_mm_s: float = 30.0,
                      kp: Optional[float] = None, kd: Optional[float] = None) -> bool:
        return self._inner.move_at_speed(
            target_mm, speed_mm_s=speed_mm_s, kp=kp, kd=kd
        )

    def move_at_speed_rad(self, target_rad: float, speed_rad_s: float = 0.5,
                          kp: Optional[float] = None,
                          kd: Optional[float] = None) -> bool:
        frac = self._from_device_theta(target_rad)
        inner_speed = speed_rad_s * abs(C.POS_OPEN_RAD - C.POS_CLOSED_RAD) \
            / abs(DRY_POS_OPEN_RAD - DRY_POS_CLOSED_RAD)
        return self._inner.move_at_speed_rad(
            C.q_to_theta(C.q_from_frac_open(frac)), speed_rad_s=inner_speed,
            kp=kp, kd=kd,
        )

    def set_frac_open(self, frac_open: float) -> None:
        """镜像模式用 —— 直接按开度摆位，不经噪声也不经 θ 换算。"""
        self._inner.set_frac_open(frac_open)

    # ══════════════════════════════════════════════════════════════════
    # 零重力
    # ══════════════════════════════════════════════════════════════════

    def enter_zero_gravity(self, duration: float = 0.0) -> bool:
        return self._inner.enter_zero_gravity(duration=duration)

    def exit_zero_gravity(self) -> bool:
        return self._inner.exit_zero_gravity()

    # ══════════════════════════════════════════════════════════════════
    # 标定
    # ══════════════════════════════════════════════════════════════════

    def calibrate(self, **kwargs: Any) -> CalibrationData:
        return self._true_calibration()

    def calibrate_manual(self, **kwargs: Any) -> CalibrationData:
        return self._true_calibration()

    def calibrate_guided(self, **kwargs: Any) -> CalibrationData:
        return self._true_calibration()

    def _true_calibration(self) -> CalibrationData:
        return CalibrationData(
            zero_position=DRY_POS_CLOSED_RAD,
            max_position=DRY_POS_OPEN_RAD,
            travel_range=abs(DRY_POS_OPEN_RAD - DRY_POS_CLOSED_RAD),
            rad_to_mm=self._config.rad_to_mm,
            motor_type=str(self._motor_type),
            can_id=self._can_id,
            mst_id=self._mst_id or 0x18,
            calibration_time=time.strftime("%Y-%m-%dT%H:%M:%S"),
        )

    def save_calibration(self, path: Optional[str] = None) -> str:
        return self._inner.save_calibration(path)

    def load_calibration(self, path: Optional[str] = None) -> bool:
        return self._inner.load_calibration(path)

    # ══════════════════════════════════════════════════════════════════
    # 状态 —— 这一层才把真机 θ 口径与噪声加上去
    # ══════════════════════════════════════════════════════════════════

    def get_state(self, wait: bool = True) -> GripperState:
        inner = self._inner.get_state()
        frac = self._inner.frac_open()

        theta = self._to_device_theta(frac)
        theta_span = abs(DRY_POS_OPEN_RAD - DRY_POS_CLOSED_RAD)
        dtheta = inner.velocity_rad_s * theta_span / abs(C.POS_OPEN_RAD - C.POS_CLOSED_RAD)

        if self._noise:
            theta += float(self._rng.normal(0.0, 4e-4))
            dtheta += float(self._rng.normal(0.0, 2e-3))

        position_mm = frac * self._config.max_stroke_mm
        if self._noise:
            position_mm += float(self._rng.normal(0.0, 0.01))

        torque = inner.torque_nm + (
            float(self._rng.normal(0.0, 0.005)) if self._noise else 0.0
        )

        return GripperState(
            position_rad=theta,
            velocity_rad_s=dtheta,
            torque_nm=torque,
            temperature_mos=inner.temperature_mos,
            temperature_coil=inner.temperature_coil,
            error_code=inner.error_code,
            timestamp=inner.timestamp,
            position_mm=position_mm,
            force_n=C.nm_to_n(torque),
        )

    def get_position(self) -> float:
        return self.get_state().position_mm

    def get_position_rad(self) -> float:
        return self.get_state().position_rad

    def get_force(self) -> float:
        return self.get_state().force_n

    def get_torque(self) -> float:
        return self.get_state().torque_nm

    def get_error(self) -> int:
        return self._inner.get_error()

    def get_temperature(self) -> Tuple[int, int]:
        return self._inner.get_temperature()

    def get_info(self) -> GripperInfo:
        return GripperInfo(
            model="DryRunGripper",
            motor_type=str(self._motor_type),
            can_id=self._can_id,
            mst_id=self._mst_id or 0x18,
        )

    def is_moving(self) -> bool:
        return self.get_state().is_moving

    def is_grasped(self) -> bool:
        return abs(self.get_state().torque_nm) > self._config.grasp_torque_threshold

    def wait_for_ready(self, timeout: float = 5.0) -> bool:
        return self._inner.wait_for_ready(timeout=timeout)

    def read_param(self, rid: int, timeout_s: float = 0.5) -> float:
        raise NotImplementedError("虚拟夹爪没有电机寄存器")

    # ══════════════════════════════════════════════════════════════════
    # 生命周期
    # ══════════════════════════════════════════════════════════════════

    def __enter__(self) -> "DryRunGripper":
        self.connect()
        self.enable()
        return self

    def __exit__(self, *args: Any) -> None:
        self.disconnect()

    def __repr__(self) -> str:
        return (
            f"DryRunGripper(ch={self._channel}, CAN_ID=0x{self._can_id:02X}, "
            f"θ端点={DRY_POS_CLOSED_RAD:.4f}/{DRY_POS_OPEN_RAD:.4f}, "
            f"{'已连接' if self.is_connected else '未连接'})"
        )
