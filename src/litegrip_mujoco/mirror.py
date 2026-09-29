"""DualGripper / MirrorMode —— 真机与仿真联动。

两种模式::

    # 1. 双控：同一条指令同时下发给真机和仿真
    #    （用户需求里的"仿真控制夹爪"）
    dual = DualGripper(channel="can0", can_id=0x08, render=True)
    dual.start()
    dual.grasp(force_n=10.0)      # 两边一起动
    dual.disconnect()

    # 2. 镜像：真机的位置实时驱动仿真
    #    （用户需求里的"夹爪控制仿真"）
    from litegrip import LiteGrip
    real = LiteGrip(channel="can0", can_id=0x08)
    with MujocoGripper(render=True) as sim:
        mirror = MirrorMode(real, sim)
        mirror.start()

════════════════════════════════════════════════════════════════════════
跨设备交换的是 frac_open，**不是** θ
════════════════════════════════════════════════════════════════════════

`position_rad`（θ）的**零点和量程都是设备相关的**：

    仿真   θ_closed =  0.000000   θ_open = -1.140000   量程 1.1400 rad
    真机   θ_closed =  1.775959   θ_open = -0.064279   量程 1.8402 rad

把真机的 θ 直接写进仿真，夹爪会跑到完全错误的位置（量程差 61%，零点差更多）。
反过来同理。

唯一可移植的量是无量纲开度::

    frac_open = (θ - θ_closed) / (θ_open - θ_closed)      ∈ [0, 1]

每一侧用**自己的**标定端点算，于是两侧都不需要知道对方的标定口径。本模块所有
跨设备的数据流都只走这个量。

真机侧取 θ 端点用 ``config.pos_closed_rad`` / ``config.pos_open_rad``。这两个值
只有在设备**确实载入过一份人选定的标定文件**时才可信 —— 见
:mod:`litegrip_mujoco.calibration`。:func:`read_frac_open` 与
:func:`write_frac_open` 默认按此严格检查，端点不可信就直接报错，不再退回
``position_mm / max_stroke_mm``（那条老路径仍在 ``strict=False`` 下可达，
但库内没有任何调用点用它）。

**为什么不能"退回了事"**：真机上的 ``mm`` 口径取自 ``config.rad_to_mm``，而那个
系数是标定的产物。没标定时的 mm 数字与真实开口无关，用它去驱动真机等于照着
一个编出来的刻度运动。
"""
from __future__ import annotations

import threading
import time
from typing import Any, Callable, Optional, Tuple

from . import calibration, constants as C
from ._litegrip import load_litegrip
from .calibration import (
    Calibration,
    CalibrationError,
    UncalibratedDeviceError,
    applied_calibration,
    apply_calibration,
    is_simulated_device,
    require_usable_device,
    select_calibration_for,
)
from .gripper import MujocoGripper


# ═════════════════════════════════════════════════════════════════════════
# 跨设备开度换算
# ═════════════════════════════════════════════════════════════════════════


def _endpoints(device: Any) -> Optional[Tuple[float, float]]:
    """取设备的 ``(θ_closed, θ_open)``；不满足不变量（closed > open）则返回 None。"""
    cfg = getattr(device, "config", None)
    if cfg is None:
        return None
    closed = getattr(cfg, "pos_closed_rad", None)
    open_ = getattr(cfg, "pos_open_rad", None)
    if closed is None or open_ is None:
        return None
    closed, open_ = float(closed), float(open_)
    if closed <= open_:
        # 违反 SDK 自己写的不变量 —— 未标定，或撞上了它的默认值 bug。
        return None
    return closed, open_


def _strict_endpoints(device: Any, action: str) -> Tuple[float, float]:
    """Gate 1 (provenance) + gate 2 (validity), or raise."""
    calibration.require_usable_device(device, action=action)
    endpoints = _endpoints(device)
    if endpoints is None:
        cfg = getattr(device, "config", None)
        raise UncalibratedDeviceError(
            f"{type(device).__name__} 的 θ 端点不满足不变量（闭合位必须数值更大）: "
            f"pos_closed_rad={getattr(cfg, 'pos_closed_rad', None)}, "
            f"pos_open_rad={getattr(cfg, 'pos_open_rad', None)}。\n"
            "       这组值是 SDK 的默认值，说明标定没生效。"
            "请先 apply_calibration(device, path) 选一份真机标定文件。"
        )
    return endpoints


def read_frac_open(
    device: Any,
    warn: Optional[bool] = None,
    *,
    strict: bool = True,
) -> float:
    """从任意夹爪设备读出无量纲开度 ∈ [0, 1]。

    默认 ``strict=True``：先确认设备的端点来自一份人选定的标定文件，再用
    θ 与标定端点算开度（最准，与 mm 刻度无关）。端点不可信时抛
    :class:`~litegrip_mujoco.calibration.UncalibratedDeviceError`。

    Args:
        device: 任意夹爪（真机或仿真）。
        warn: **已废弃**，仅为兼容旧调用保留。传了它等价于 ``strict=False``。
        strict: 是否强制要求标定可信。``False`` 时退回旧行为
            （``position_mm / max_stroke_mm`` 并给 RuntimeWarning）。

    Raises:
        UncalibratedDeviceError: 设备没有可信端点。
        CalibrationVerificationError: 端点在本进程里被改动过。
    """
    if warn is not None:
        import warnings

        warnings.warn(
            "read_frac_open(warn=...) 已废弃：默认已改为 strict=True"
            "（未标定直接报错）。要旧行为请显式传 strict=False。",
            DeprecationWarning,
            stacklevel=2,
        )
        strict = False

    if strict:
        closed, open_ = _strict_endpoints(device, "读开度")
        theta = float(device.get_position_rad())
        return min(1.0, max(0.0, (theta - closed) / (open_ - closed)))

    endpoints = _endpoints(device)
    if endpoints is not None:
        closed, open_ = endpoints
        theta = float(device.get_position_rad())
        return min(1.0, max(0.0, (theta - closed) / (open_ - closed)))

    cfg = getattr(device, "config", None)
    stroke = float(getattr(cfg, "max_stroke_mm", 0.0) or 0.0)
    if stroke > 0.0:
        import warnings

        warnings.warn(
            f"{type(device).__name__} 的 θ 端点不满足不变量 "
            f"(pos_closed_rad={getattr(cfg, 'pos_closed_rad', None)}, "
            f"pos_open_rad={getattr(cfg, 'pos_open_rad', None)})，"
            "说明未标定；退回 position_mm/max_stroke_mm 口径。"
            "建议先跑标定。",
            RuntimeWarning,
            stacklevel=2,
        )
        return min(1.0, max(0.0, float(device.get_position()) / stroke))
    return 0.0


def write_frac_open(device: Any, frac_open: float, *, strict: bool = True) -> bool:
    """把无量纲开度写给任意夹爪设备。

    对真机走 ``goto_rad()`` 并**显式**算好 θ，从而绕开 ``config.rad_to_mm``
    （那个系数是标定的产物，不该参与跨设备换算）。这条路径要求标定端点可信：
    ``goto_rad()`` 的 clamp 只有在 ``closed > open`` 时才正确。

    这是**会动真机**的一侧，所以默认 ``strict=True``：没有可信端点就不写入。
    ``strict=False`` 时退回 ``goto(frac_open * max_stroke_mm)`` 的老行为 ——
    在真机上那等于照着一个没标定过的刻度运动，只给确实知道自己在做什么的人用。

    Raises:
        UncalibratedDeviceError: 设备没有可信端点（``strict=True``）。
        CalibrationVerificationError: 端点在本进程里被改动过。
        ValueError: ``strict=False`` 且既无端点也无 ``max_stroke_mm``。
    """
    frac_open = min(1.0, max(0.0, float(frac_open)))

    if strict:
        closed, open_ = _strict_endpoints(device, "写入开度（会驱动真机）")
        if hasattr(device, "set_frac_open"):
            device.set_frac_open(frac_open)
            return True
        return bool(device.goto_rad(closed + frac_open * (open_ - closed)))

    endpoints = _endpoints(device)
    if endpoints is None:
        cfg = getattr(device, "config", None)
        stroke = float(getattr(cfg, "max_stroke_mm", 0.0) or 0.0)
        if stroke <= 0.0:
            raise ValueError(
                f"{type(device).__name__} 既没有可用的 θ 端点，也没有 "
                "max_stroke_mm，无法写入开度"
            )
        return bool(device.goto(frac_open * stroke))

    closed, open_ = endpoints
    theta = closed + frac_open * (open_ - closed)
    if hasattr(device, "set_frac_open"):
        device.set_frac_open(frac_open)
        return True
    return bool(device.goto_rad(theta))


# ═════════════════════════════════════════════════════════════════════════
# 双控
# ═════════════════════════════════════════════════════════════════════════


class DualGripper:
    """同时控制一台真机和一台仿真夹爪。

    同一条指令下发给两边。仿真是真机的实时镜像算力来源，可用于可视化，也可在
    真机不在场时（``dry_run=True``）独立跑通全流程::

        dual = DualGripper(channel="can0", can_id=0x08, render=True)
        dual.start()
        dual.open()
        dual.grasp(force_n=10.0)
        dual.disconnect()

    ⚠ 命名说明：``close()`` 是**合拢夹爪**（与设备 API 一致），不是释放资源。
    释放资源用 ``disconnect()`` 或 ``shutdown()``。
    """

    def __init__(
        self,
        real: Any = None,
        channel: str = "can0",
        can_id: int = 0x08,
        mst_id: Optional[int] = None,
        sim_model_path: Optional[str] = None,
        render: bool = True,
        mirror_first: bool = True,
        dry_run: bool = False,
        realtime: bool = True,
        calibration: Optional[str] = None,
        allow_uncalibrated: bool = False,
        **sim_kwargs: Any,
    ) -> None:
        """
        Args:
            real: 已构造好的真机对象（鸭子类型）。缺省时按 ``dry_run`` 决定
                是造一个 `DryRunGripper` 还是 ``litegrip.LiteGrip``。
            channel / can_id / mst_id: 真机 CAN 参数。
            sim_model_path: 仿真模型路径，缺省 ``assets/litegrip.xml``。
            render: 是否打开仿真查看器。
            mirror_first: 启动时先让仿真对齐真机的当前开度。
            dry_run: 无 CAN 硬件时用虚拟夹爪代替真机。
            calibration: 本次真机运动使用的标定 JSON。缺省时**交互式**要求
                操作者选择；非交互终端直接抛 ``CalibrationRequiredError``。
                本参数缺失时绝不使用任何默认标定文件。
            allow_uncalibrated: 显式放弃标定校验（会在控制台留痕）。
                不传 ``calibration`` 又不开这个开关，就必须能弹出选择器。

        Raises:
            CalibrationRequiredError: 真机没有可用标定，且无法让操作者选择。
            CalibrationFileError: ``calibration`` 指向的文件不可用。
        """
        if real is None:
            if dry_run:
                from .dryrun import DryRunGripper

                real = DryRunGripper(realtime=realtime)
            else:
                sdk = load_litegrip()
                real = sdk.LiteGrip(channel=channel, can_id=can_id, mst_id=mst_id)

        # 一、先定标定，再碰硬件：文件有问题就不该等到连上 CAN 才报。
        #    真机对象此时已造好，但 LiteGrip() 本身不开 CAN、不使能。
        self._calibration_path = select_calibration_for(
            real, calibration, allow_uncalibrated=allow_uncalibrated
        )
        self._calibration: Optional[Calibration] = applied_calibration(real)

        self._real = real
        self._sim = MujocoGripper(
            model_path=sim_model_path, render=render, realtime=realtime, **sim_kwargs
        )
        self._sim.connect()
        self._sim.enable()

        self._mirror_first = mirror_first
        self._mirroring = False
        self._mirror_thread: Optional[threading.Thread] = None
        self._last_mirror_error: Optional[BaseException] = None
        self._lock = threading.RLock()

    # ── 生命周期 ───────────────────────────────────────────────────────

    def start(self) -> "DualGripper":
        """连接真机、套用标定并校验、使能，按需先做一次镜像对齐。

        顺序是刻意的：``connect()``（不动）→ 载入标定并**逐字段校验** →
        ``enable()``（第一个会给电机上电的调用）。校验没过就绝不使能，
        断开后原样抛出。
        """
        try:
            if not getattr(self._real, "is_connected", False):
                self._real.connect()
            if self._calibration_path is not None:
                self._calibration = apply_calibration(
                    self._real, self._calibration_path
                )
            require_usable_device(self._real, action="使能并运动")
            self._real.enable()
        except Exception:
            self._sim.disconnect()
            try:
                self._real.disconnect()  # 不留半配置/已使能状态
            except Exception:  # noqa: BLE001 — 断开失败不该盖住原始异常
                pass
            raise
        if self._mirror_first:
            self._sync_once()
            self._start_mirroring()
        return self

    def _sync_once(self) -> None:
        """把仿真摆到真机当前开度。"""
        try:
            frac = read_frac_open(self._real)
        except Exception:
            return
        with self._lock:
            self._sim.set_frac_open(frac)

    def _start_mirroring(self) -> None:
        self._mirroring = True
        self._mirror_thread = threading.Thread(
            target=self._mirror_loop, daemon=True, name="dual_mirror"
        )
        self._mirror_thread.start()

    def _mirror_loop(self) -> None:
        while self._mirroring:
            try:
                frac = read_frac_open(self._real)
                with self._lock:
                    self._sim.set_frac_open(frac)
            except CalibrationError as exc:
                # 标定没了就不该继续转：记下来、停表，让 __repr__ 和调用方
                # 都能看见，而不是 50 Hz 静默空转。
                self._last_mirror_error = exc
                self._mirroring = False
                break
            except Exception:
                pass
            time.sleep(0.02)  # ~50 Hz

    # ── 运动指令：两边一起 ─────────────────────────────────────────────

    def _both(self, sim_call: Callable[[], Any],
              real_call: Callable[[], Any]) -> Tuple[Any, Any]:
        """并行下发同一条指令，返回 ``(真机结果, 仿真结果)``。"""
        self._mirroring = False
        result: list = [None]

        def _run_sim() -> None:
            try:
                result[0] = sim_call()
            except Exception as exc:  # noqa: BLE001 — 要如实带回给调用方
                result[0] = exc

        thread = threading.Thread(target=_run_sim, daemon=True)
        thread.start()
        try:
            real_result = real_call()
        finally:
            thread.join()
        if self._mirror_first:
            self._mirroring = True
        if isinstance(result[0], Exception):
            raise result[0]
        return real_result, result[0]

    def open(self, duration: float = 1.0) -> Tuple[Any, Any]:
        """完全张开两边。"""
        return self._both(
            lambda: self._sim.open(duration=duration),
            lambda: self._real.open(duration=duration),
        )

    def close(self, force_n: Optional[float] = None,
              duration: float = 1.0) -> Tuple[Any, Any]:
        """合拢两边（``close()`` = 合拢夹爪，不是释放资源）。"""
        return self._both(
            lambda: self._sim.close(force_n=force_n, duration=duration),
            lambda: self._real.close(force_n=force_n, duration=duration),
        )

    def grasp(self, force_n: float = 10.0, duration: float = 3.0,
              **kwargs: Any) -> Tuple[Any, Any]:
        """抓取两边。真机走它自己的堵转检测。"""
        return self._both(
            lambda: self._sim.grasp(force_n=force_n, duration=duration, **kwargs),
            lambda: self._real.grasp(force_n=force_n, duration=duration, **kwargs),
        )

    def home(self) -> Tuple[Any, Any]:
        """回闭合位。"""
        return self._both(lambda: self._sim.home(), lambda: self._real.home())

    def set_force(self, force_n: float, duration: float = 0.3) -> Tuple[Any, Any]:
        """在当前位置施加夹持力。"""
        return self._both(
            lambda: self._sim.set_force(force_n, duration=duration),
            lambda: self._real.set_force(force_n, duration=duration),
        )

    def move_to_frac(self, frac_open: float, duration: float = 1.0) -> Tuple[Any, Any]:
        """按**无量纲开度**移动两边 —— 跨设备唯一安全的定位口径。

        想让真机和仿真停在"同一个位置"，就该用这个，而不是 ``goto(mm)``：
        mm 只有在两侧 ``max_stroke_mm`` 一致时才是同一个量。
        """
        return self._both(
            lambda: write_frac_open(self._sim, frac_open),
            lambda: write_frac_open(self._real, frac_open),
        )

    def goto(self, position_mm: float, duration: float = 0.5) -> Tuple[Any, Any]:
        """按毫米移动两边。

        ⚠ 只有真机的 ``config.max_stroke_mm`` 已改成真实行程（85.452）时，
        两侧的"毫米"才是同一个量。否则请改用 :meth:`move_to_frac`。
        """
        return self._both(
            lambda: self._sim.goto(position_mm, duration=duration),
            lambda: self._real.goto(position_mm, duration=duration),
        )

    # ── 状态 ───────────────────────────────────────────────────────────

    def get_real_state(self) -> Any:
        """真机状态快照。"""
        return self._real.get_state()

    def get_sim_state(self) -> Any:
        """仿真状态快照。"""
        return self._sim.get_state()

    def compare(self) -> dict:
        """比对两侧开度，便于确认联动是否真的对上了。

        Returns:
            ``{"real_frac":…, "sim_frac":…, "delta":…}``，delta 为无量纲开度差。
        """
        real_frac = read_frac_open(self._real)
        sim_frac = self._sim.frac_open()
        return {"real_frac": real_frac, "sim_frac": sim_frac,
                "delta": real_frac - sim_frac}

    # ── 安全与生命周期 ─────────────────────────────────────────────────

    def request_stop(self) -> None:
        """两边同时急停。"""
        try:
            self._real.stop()
        finally:
            self._sim.stop()

    def disconnect(self) -> None:
        """停止镜像、断开两边。"""
        self._mirroring = False
        thread, self._mirror_thread = self._mirror_thread, None
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        for device in (self._sim, self._real):
            try:
                device.disconnect()
            except Exception:
                pass

    #: ``shutdown`` 是 ``disconnect`` 的别名 —— 别用 ``close()``，那个是合拢夹爪。
    shutdown = disconnect

    @property
    def real(self) -> Any:
        return self._real

    @property
    def sim(self) -> MujocoGripper:
        return self._sim

    @property
    def calibration(self) -> Optional[Calibration]:
        """本次运行套用的标定文件；仿真设备或显式放行时为 None。"""
        return self._calibration

    @property
    def last_mirror_error(self) -> Optional[BaseException]:
        """镜像线程因标定失效而停表时记下的异常，否则为 None。"""
        return self._last_mirror_error

    def __enter__(self) -> "DualGripper":
        return self.start()

    def __exit__(self, *args: Any) -> None:
        self.disconnect()

    def __repr__(self) -> str:
        return (
            f"DualGripper(real={type(self._real).__name__}, "
            f"sim={self._sim!r})"
        )


# ═════════════════════════════════════════════════════════════════════════
# 镜像
# ═════════════════════════════════════════════════════════════════════════


class MirrorMode:
    """让仿真跟随任意真机夹爪的开度::

        from litegrip import LiteGrip
        from litegrip_mujoco import MujocoGripper, MirrorMode

        real = LiteGrip(channel="can0", can_id=0x08)
        real.connect(); real.enable()

        with MujocoGripper(render=True) as sim:
            mirror = MirrorMode(real, sim, rate_hz=50.0)
            mirror.start()
            ...
            mirror.stop()

    仿真是**纯跟随**的：镜像期间不要另发运动指令给 ``sim``，否则两者会互相打架。

    ``real_gripper`` 是真机时必须带一份人选定的标定（``calibration=`` 或交互式
    选择）—— 镜像读的是 ``frac_open``，而它由 ``config`` 的 θ 端点算出，端点没
    标定过就只是个编出来的数。仿真设备自动豁免。
    """

    def __init__(self, real_gripper: Any, sim_gripper: MujocoGripper,
                 rate_hz: float = 50.0,
                 calibration: Optional[str] = None,
                 allow_uncalibrated: bool = False) -> None:
        """
        Args:
            real_gripper: 被跟随的真机。
            sim_gripper: 跟随的仿真。
            rate_hz: 镜像频率。
            calibration: 本次真机运动使用的标定 JSON。缺省时**交互式**要求
                选择；非交互终端直接抛 ``CalibrationRequiredError``。
            allow_uncalibrated: 显式放弃标定校验（会在控制台留痕）。

        Raises:
            CalibrationRequiredError: 真机没有可用标定，且无法让操作者选择。
            CalibrationFileError: ``calibration`` 指向的文件不可用。
        """
        self._real = real_gripper
        self._sim = sim_gripper
        self._rate_hz = float(rate_hz)
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._last_frac: Optional[float] = None
        self._samples = 0
        self._error: Optional[BaseException] = None

        # 镜像只读真机，从不给它发指令，所以这里不做 connect/enable 那套顺序，
        # 只把标定选好并套上（load_calibration 只改 config，不需要 CAN）。
        self._calibration_path = select_calibration_for(
            real_gripper, calibration, allow_uncalibrated=allow_uncalibrated
        )
        self._calibration: Optional[Calibration] = applied_calibration(real_gripper)
        if self._calibration_path is not None:
            self._calibration = apply_calibration(real_gripper, self._calibration_path)

    def start(self) -> None:
        """开始镜像。"""
        if self._running:
            return
        # 在**调用方线程**里先把标定问题暴露出来 —— 子线程里抛异常没人接得住。
        require_usable_device(self._real, action="开始镜像")
        self._running = True
        self._error = None
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="mirror_mode"
        )
        self._thread.start()

    def stop(self) -> None:
        """停止镜像。"""
        self._running = False
        thread, self._thread = self._thread, None
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)

    def _loop(self) -> None:
        dt = 1.0 / self._rate_hz if self._rate_hz > 0 else 0.02
        while self._running:
            try:
                frac = read_frac_open(self._real)
                self._sim.set_frac_open(frac)
                self._last_frac = frac
                self._samples += 1
            except CalibrationError as exc:
                # 标定没了就停表并记下原因；继续 50 Hz 空转等于假装还在镜像。
                self._error = exc
                self._running = False
                break
            except Exception:
                pass
            time.sleep(dt)

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def samples(self) -> int:
        """已镜像的样本数（可用于确认真的在跑）。"""
        return self._samples

    @property
    def last_frac_open(self) -> Optional[float]:
        """最近一次镜像到的开度。"""
        return self._last_frac

    @property
    def calibration(self) -> Optional[Calibration]:
        """本次运行套用的标定文件；仿真设备或显式放行时为 None。"""
        return self._calibration

    @property
    def last_error(self) -> Optional[BaseException]:
        """镜像线程因标定失效而停表时记下的异常，否则为 None。"""
        return self._error

    def __enter__(self) -> "MirrorMode":
        self.start()
        return self

    def __exit__(self, *args: Any) -> None:
        self.stop()
