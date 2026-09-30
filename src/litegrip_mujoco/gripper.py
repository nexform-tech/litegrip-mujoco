"""MujocoGripper —— `litegrip.LiteGrip` 的 MuJoCo 仿真替身。

目标是**原地替换**：同一段控制代码，改一行 import 就能从仿真切到真机::

    from litegrip_mujoco import MujocoGripper as LiteGrip   # 仿真
    from litegrip import LiteGrip                           # 真机

    with LiteGrip() as g:
        g.open(duration=1.0)
        g.grasp(force_n=10.0)
        print(g.get_state().force_n)

════════════════════════════════════════════════════════════════════════
API 对等 —— 以及它带来的一个命名陷阱
════════════════════════════════════════════════════════════════════════

`litegrip.LiteGrip` 里 ``close()`` 是"**合拢夹爪**"。所以本类的资源释放**不能**
叫 ``close()``（litearm-mujoco 的 `MujocoArm.close()` 是释放资源，那个名字在这里
是错的）。释放走 ``disconnect()``，与 SDK 一致。

════════════════════════════════════════════════════════════════════════
仿真时钟与 duration
════════════════════════════════════════════════════════════════════════

后台线程按 ``timestep`` 推进物理。``realtime=True``（默认）时用**绝对截止时刻**
把仿真时钟锁到真实时钟上，于是 ``duration`` 既是仿真秒也是墙上秒，与真机"按
duration 走完一段"的语义一致。``realtime=False`` 时全速跑，``duration`` 仍按
仿真秒算（测试用得上，快几百倍）。

════════════════════════════════════════════════════════════════════════
torque_nm / force_n 在仿真里到底是什么
════════════════════════════════════════════════════════════════════════

``torque_nm`` 报的是**电机当前出力矩**，即 ``data.ctrl``（= ``qfrc_actuator/GEAR``）。
DM 电机上报的是它自己的力矩估计，堵转/受力时与指令一致，这是同一个量的仿真对应物。

``force_n`` 逐字沿用 SDK 的 ``torque_nm × NM_TO_N``。**它是一个由力矩推算的估计
值，不是接触力测量。** 这一点必须说清楚，因为它会以两种方式骗人：

1. **空夹也会报力。** 指爪顶到闭合限位时电机一样出力矩，``get_force()`` 照样
   返回 ``force_n``，而两指之间什么都没有。
2. **``qfrc_constraint`` 不能拿来当"夹持力"。** 那个量在指爪顶死限位、夹爪里空无
   一物时读数**正好等于**执行器力，无法区分"夹住工件 10 N"和"空夹顶死 10 N"。
   实测：``ctrl=1.0`` 且工件已掉到地板上时，``qfrc_constraint`` 仍是 −5.000 N。

仿真之所以照抄这个口径，是因为**真机就是这个口径**——只有这样 ``get_force()``
在两侧才是同一个量。想判断"真的夹住了东西"，用 ``is_grasped()``（也是 SDK 的
力矩阈值判据）或自己看接触。
"""
from __future__ import annotations

import inspect
import os
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import constants as C
from ._litegrip import (
    HAS_SDK,
    CalibrationData,
    GripperConfig,
    GripperInfo,
    GripperMode,
    GripperState,
    GripperStatus,
    NotInitializedError,
)
from .controller import (
    GripperPDController,
    Hold,
    LinearRamp,
    MinJerkRamp,
    Ramp,
    TrajectoryRamp,
)
from .window import (
    CONFIRM_KEYS,
    QUIT_KEYS,
    ZERO_GRAVITY_KEYS,
    KeyQueue,
    clicked,
    held,
    key_codes,
    key_label,
    pressed,
)

#: 默认模型：纯夹爪（2 自由度、无外部物体），适合做单位与动力学基准。
DEFAULT_MODEL = "litegrip.xml"

#: 演示场景：夹爪 + 地板 + 一个由夹具托住的工件。
SCENE_MODEL = "scene.xml"

#: `grasp()` 的控制节拍 (s)。与 SDK 的 ``interval = 0.01`` 一致。
#: 这个值很关键：堵转判据必须在这个节拍上量 Δθ，而不是每个物理步。
_GRASP_INTERVAL = 0.01

#: `grasp()` 判定成功后保持力矩的时长 (s)，与 SDK 一致。
_GRASP_HOLD_S = 0.3

#: 轮询斜坡是否走完的间隔 (s)。
_POLL_S = 0.002


def _assets_dir() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")


def _resolve_model_path(model_path: Optional[str]) -> str:
    """把 ``model_path`` 解析成一个真实存在的文件。

    接受：显式路径、assets 目录里的文件名、None（默认 litegrip.xml）。
    """
    assets = _assets_dir()
    if model_path is None:
        candidates = [os.path.join(assets, DEFAULT_MODEL)]
    else:
        candidates = [
            model_path,
            os.path.join(assets, model_path),
            os.path.join(assets, os.path.basename(model_path)),
        ]
    for path in candidates:
        if os.path.isfile(path):
            return os.path.abspath(path)
    raise FileNotFoundError(
        f"找不到 MuJoCo 模型 {model_path!r}；试过：{candidates}。"
        f"assets 目录：{assets}"
    )


def _accepts_key_callback(launch: Any) -> bool:
    """``launch_passive`` 收不收 ``key_callback``（MuJoCo ≥ 3.1 才收）。

    用签名判断，而不是"调用失败就退回两参数版"。后者看着更简单，但
    ``TypeError`` 也可能来自 ``launch_passive`` **内部** —— 那时窗口已经建好
    了，重试会在屏幕上多留一个没人 sync 的孤儿窗口。查签名不会产生这个后果。
    """
    try:
        params = inspect.signature(launch).parameters
    except (TypeError, ValueError):
        return False
    return "key_callback" in params


class MujocoGripper:
    """LiteGrip 夹爪的 MuJoCo 仿真实现，公开 API 与 `litegrip.LiteGrip` 对齐。

    Args:
        model_path: MJCF 路径。默认 ``assets/litegrip.xml``；
            传 ``"scene.xml"`` 可得到带地板与工件的演示场景。
        render: 是否打开 MuJoCo 被动查看器。
        config: `GripperConfig`。缺省时用仿真侧的自洽默认值
            （``max_stroke_mm = 85.452``、``pos_open_rad = -1.14``）。
        channel / can_id / mst_id / canfd_mode / motor_type: 为 API 对等而接受，
            仿真中不使用（保留在属性里，便于同一段代码打印/记录）。
        realtime: True 时把仿真时钟锁到真实时钟，``duration`` 才是墙上秒。

    Usage::

        with MujocoGripper() as g:
            g.open()
            print(g.get_position())        # mm，真实毫米
            g.close(force_n=10.0)
            print(g.get_force())           # N
    """

    #: 仿真设备的端点来自模型本身（解析真值），不是标定的产物。
    #: `litegrip_mujoco.calibration` 靠这个标记豁免真机那套"必须先选标定文件"
    #: 的强制；第三方仿真后端也应当设上它，或调用 `mark_calibrated()`。
    IS_SIMULATED = True

    def __init__(
        self,
        model_path: Optional[str] = None,
        render: bool = False,
        config: Optional[GripperConfig] = None,
        channel: str = "can0",
        can_id: int = 0x08,
        mst_id: Optional[int] = None,
        canfd_mode: Optional[bool] = None,
        motor_type: Any = "DM4310",
        realtime: bool = True,
        keyframe: Optional[str] = None,
    ) -> None:
        import mujoco  # 延迟 import：让 `--help` 之类的纯文本路径不必加载 MuJoCo

        self._mujoco = mujoco
        self._model_path = _resolve_model_path(model_path)

        self._channel = channel
        self._can_id = can_id
        self._mst_id = mst_id
        self._canfd_mode = bool(canfd_mode) if canfd_mode is not None else False
        self._motor_type = motor_type
        self._config = config if config is not None else _sim_config()
        self._realtime = bool(realtime)

        self._model = mujoco.MjModel.from_xml_path(self._model_path)
        self._data = mujoco.MjData(self._model)

        # 受驱动的关节数 = 执行器数。**不能用 nv**：scene.xml 里工件带 freejoint，
        # nv=8 而实际只有 2 个指关节，按 nv 建控制器会把 8 个力矩往 2 个执行器里塞。
        self._n_joints = int(self._model.nu)
        if self._n_joints < 1:
            raise ValueError(f"模型 {self._model_path} 没有执行器，不是夹爪模型")

        # 由执行器反查关节在 qpos/qvel 里的地址。不要假设指关节排在数组最前面
        # ——scene.xml 的 freejoint 恰好排在后面，但那是巧合，不是契约。
        self._qpos_idx = [
            int(self._model.jnt_qposadr[self._model.actuator_trnid[a, 0]])
            for a in range(self._n_joints)
        ]
        self._dof_idx = [
            int(self._model.jnt_dofadr[self._model.actuator_trnid[a, 0]])
            for a in range(self._n_joints)
        ]
        if len(set(self._qpos_idx)) != self._n_joints:
            raise ValueError(
                f"模型 {self._model_path} 的多个执行器挂在同一个关节上，无法独立控制"
            )

        self._ctrl = GripperPDController(
            kp=self._config.kp, kd=self._config.kd, n_joints=self._n_joints
        )

        # 按**名字**解析等式。绝不能按索引：scene.xml 用 <include> 引入
        # litegrip.xml，两个文件的 <equality> 会合并，索引顺序随文件结构变化
        # （实测合并后是 ['finger_coupling', 'fixture']，索引 0 是耦合不是夹具）。
        self._eq_ids = {
            mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_EQUALITY, i): i
            for i in range(self._model.neq)
        }

        self._lock = threading.RLock()
        self._ramp: Optional[Ramp] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._viewer: Any = None
        self._abort = threading.Event()

        # 按键台账。必须在 _open_viewer() 之前建好：launch_passive() 之后
        # key_callback 随时可能被查看器自己的线程调用。
        self._keys = KeyQueue()

        # 仿真时钟 (s)，在 _tick 里按 dt 累加。与墙钟无关 —— realtime=False 时
        # 两者会差很远，而 settle()/录制要的是**仿真**时间。
        self._sim_time = 0.0

        self._connected = False
        self._enabled = False
        self._status_flags = GripperStatus.NONE
        self._error_code = 0
        self._zero_gravity = False

        # 轨迹录制/回放。惰性建，因为大多数会话用不到。
        self._recorder: Any = None
        self._player: Any = None
        self._play_ramp: Optional[TrajectoryRamp] = None

        # 温度模型：不是标定值，只是让 get_temperature() 有意义的示意模型。
        self._temp_mos = float(_AMBIENT_C)
        self._temp_coil = float(_AMBIENT_C)

        if keyframe is not None:
            self._apply_keyframe(keyframe)
        else:
            # 默认停在完全张开位，与 LiteGrip 上电后的行为一致。
            self._apply_keyframe("open")

        if render:
            self._open_viewer()

    # ══════════════════════════════════════════════════════════════════
    # 属性（与 LiteGrip 同名，都是 property）
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
        return self._connected

    @property
    def is_enabled(self) -> bool:
        return self._enabled

    @property
    def config(self) -> GripperConfig:
        return self._config

    # ── 仿真专有属性 ────────────────────────────────────────────────

    @property
    def model(self) -> Any:
        """底层 ``mjtModel``。仿真专有。"""
        return self._model

    @property
    def data(self) -> Any:
        """底层 ``mjData``。仿真专有 —— 想直接读接触、力、位姿时用。"""
        return self._data

    @property
    def model_path(self) -> str:
        """已解析的模型绝对路径。仿真专有。"""
        return self._model_path

    # ══════════════════════════════════════════════════════════════════
    # 连接与使能
    # ══════════════════════════════════════════════════════════════════

    def connect(self) -> bool:
        """启动后台仿真线程。对应真机的建立 CAN 连接。"""
        if self._connected:
            return True
        self._running = True
        self._thread = threading.Thread(
            target=self._sim_loop, daemon=True, name="litegrip_sim"
        )
        self._thread.start()
        self._connected = True
        return True

    def disconnect(self) -> None:
        """停止仿真线程并关闭查看器。

        **注**：这不是 ``close()`` —— ``close()`` 是合拢夹爪。
        """
        self._running = False
        self._abort.set()
        # 先把查看器摘下来：仿真线程拿不到它，下一轮就不会再 sync()。
        viewer, self._viewer = self._viewer, None
        thread, self._thread = self._thread, None
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        # join 超时就说明线程可能正卡在 sync() 里。此时关查看器会和它抢
        # mjData —— 宁可把窗口留给进程退出时回收，也不要制造一个
        # "退出瞬间段错误"。实测这个崩溃在 Wayland + 远程桌面下很常见。
        if viewer is not None and (thread is None or not thread.is_alive()):
            try:
                viewer.close()
            except Exception:
                pass
        self._connected = False
        self._enabled = False
        self._status_flags = GripperStatus.NONE

    def enable(self) -> bool:
        """使能电机（仿真里即允许输出力矩）。"""
        self._check_connected()
        self._enabled = True
        self._error_code = 1
        self._status_flags |= GripperStatus.ENABLED
        return True

    def disable(self) -> bool:
        """失能电机。指爪变为自由（只剩阻尼）。"""
        self._check_connected()
        self._enabled = False
        self._error_code = 0
        self._status_flags &= ~GripperStatus.ENABLED
        with self._lock:
            self._ramp = None
            self._ctrl.reset()
            self._data.ctrl[:] = 0.0
        return True

    def clear_fault(self) -> bool:
        """清除故障。"""
        self._check_connected()
        self._error_code = 1 if self._enabled else 0
        self._status_flags &= ~GripperStatus.ERROR
        return True

    def stop(self) -> bool:
        """急停：中止当前运动并把力矩归零，但不失能。"""
        self._abort.set()
        with self._lock:
            self._ramp = None
            self._ctrl.reset()
            # 顺带解掉 command_fraction 可能压上去的限幅。急停之后再运动应当是
            # 正常出力，而不是继承上一次的力上限。
            self._ctrl.set_torque_limit(None)
            self._data.ctrl[:] = 0.0
        self._status_flags &= ~GripperStatus.MOVING
        return True

    # ══════════════════════════════════════════════════════════════════
    # 运动 —— 高层
    # ══════════════════════════════════════════════════════════════════

    def home(self) -> bool:
        """回闭合位。

        与 SDK 一致，目标取**常量** ``POS_CLOSED_RAD`` 而非 ``config.pos_closed_rad``。
        仿真侧 ``POS_CLOSED_RAD = 0.0`` 恰好就是闭合位，所以这里行为正确。

        ⚠ 真机上这个函数是有问题的：SDK 的 ``POS_CLOSED_RAD = 0.0`` 并非标定出来的
        闭合角（实测标定值是 1.775959），再叠加 ``goto_rad()`` 的 clamp bug，最终会
        驱动夹爪**张开**，与它 "Move to the closed (zero) position" 的 docstring 相反。
        仿真侧不复刻这个 bug —— 那会让例程给出错误示范。详见 README 的"已知问题"。
        """
        self._check_connected()
        self._check_enabled()
        return self.move_to(C.POS_CLOSED_RAD, duration=1.0)

    def open(
        self,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        duration: float = 1.0,
    ) -> bool:
        """完全张开。"""
        self._check_connected()
        self._check_enabled()
        return self.move_to(self._config.pos_open_rad, kp=kp, kd=kd,
                            duration=duration)

    def close(
        self,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        force_n: Optional[float] = None,
        duration: float = 1.0,
    ) -> bool:
        """合拢夹爪。

        ⚠ **``close()`` 无法限力。** 目标一路指向闭合位，位置误差
        ``kp·Δθ`` 会压倒 ``force_n`` 带来的前馈力矩并让执行器饱和。实测
        ``force_n=0 / 10 / 20`` 得到的接触力是一样的。真机同理。

        真正的力控要用 :meth:`grasp`（检测到堵转后把目标改写到当前位置，位置
        误差归零，只剩前馈力矩）或 :meth:`set_force`。
        """
        self._check_connected()
        self._check_enabled()
        tau_ff = C.n_to_nm(force_n) if force_n is not None else 0.0
        return self.move_to(self._config.pos_closed_rad, kp=kp, kd=kd,
                            tau_feedforward=tau_ff, duration=duration)

    def grasp(
        self,
        force_n: float = 10.0,
        kp: float = 150.0,
        kd: float = 2.0,
        duration: float = 3.0,
        stall_threshold: float = 0.001,
        stall_cycles: int = 5,
    ) -> bool:
        """自适应抓取：先合拢到堵转，再改用前馈力矩保持夹持力。

        流程（与 SDK 的 ``grasp()`` 一致）：

        1. 以 ``kp``/``kd`` 合拢，目标为闭合位；
        2. 按 10 ms 节拍采样 θ，``|Δθ| < stall_threshold`` 连续 ``stall_cycles``
           次即判定堵转；
        3. 判定成功后把目标**改写到当前位置**、只留 ``tau_ff``，保持 0.3 s 后返回 True。

        第 3 步是力控成立的关键：目标改写到当前位置后位置误差归零，关节力就只剩
        ``GEAR · tau_ff``，夹持力才真的等于 ``force_n``。

        ⚠ **空夹时也会返回 True。** 指爪顶到闭合限位同样是"停住了"，这与 SDK
        行为一致，是既有语义而非仿真缺陷。

        Args:
            force_n: 目标夹持力 (N)。
            kp / kd: 接近段的增益。
            duration: 最长合拢时间 (s)，超时即返回 False。
            stall_threshold: 堵转判据 (rad)，**在 θ 空间**，与 SDK 同口径。
            stall_cycles: 连续多少次判定为堵转。

        Returns:
            检测到堵转返回 True；超时返回 False。
        """
        self._check_connected()
        self._check_enabled()

        tau_ff = C.n_to_nm(force_n)
        deadline = time.monotonic() + duration
        self._abort.clear()

        # 接近段：恒定目标（**不能**用斜坡 —— 目标一直在动就永远判不出堵转）。
        with self._lock:
            self._ctrl.set_gains(kp, kd)
            self._ctrl.set_feedforward(tau_ff)
            self._ramp = Hold(self._config.pos_closed_rad)
            self._status_flags |= GripperStatus.MOVING

        last_theta = self._theta()
        stall_count = 0
        try:
            while time.monotonic() < deadline:
                if self._abort.is_set():
                    return False
                self._check_alive()
                time.sleep(_GRASP_INTERVAL)
                current = self._theta()
                delta = abs(current - last_theta)
                if delta < stall_threshold:
                    stall_count += 1
                    if stall_count >= stall_cycles:
                        # 堵转确认 —— 保持当前位置 + 前馈力矩
                        with self._lock:
                            self._ctrl.hold_current(current, tau_ff)
                            self._ramp = Hold(current)
                            self._status_flags |= GripperStatus.GRASPED
                        time.sleep(_GRASP_HOLD_S)
                        return True
                else:
                    stall_count = 0
                last_theta = current
        finally:
            with self._lock:
                self._status_flags &= ~GripperStatus.MOVING

        # 超时：既没堵转也没走完
        self._status_flags |= GripperStatus.AT_TARGET
        return False

    # ══════════════════════════════════════════════════════════════════
    # 运动 —— 中层
    # ══════════════════════════════════════════════════════════════════

    def goto(
        self,
        position_mm: float,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        duration: float = 0.5,
    ) -> bool:
        """移动到绝对位置（mm）。0 = 闭合，``MM_SCALE`` = 全开。

        与 SDK 的算式等价：``position_rad = pos_closed_rad - position_mm/rad_to_mm``。
        """
        self._check_connected()
        self._check_enabled()
        return self.goto_rad(C.mm_to_theta(position_mm), kp=kp, kd=kd,
                             duration=duration)

    def goto_rad(
        self,
        position_rad: float,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        dq_target: float = 0.0,
        tau_feedforward: float = 0.0,
        duration: float = 0.5,
    ) -> bool:
        """移动到绝对角度（rad），带爬升目标。"""
        self._check_connected()
        self._check_enabled()
        target = C.clamp_theta(position_rad)
        return self._run_ramp(
            MinJerkRamp(self._theta(), target, duration),
            kp=kp, kd=kd, tau_ff=tau_feedforward, dq_target=dq_target,
        )

    def move_to(
        self,
        target_rad: float,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        tau_feedforward: float = 0.0,
        duration: float = 1.0,
    ) -> bool:
        """同 :meth:`goto_rad`，只是默认 duration 更长。"""
        return self.goto_rad(target_rad, kp=kp, kd=kd,
                             tau_feedforward=tau_feedforward, duration=duration)

    def set_force(self, force_n: float, duration: float = 0.3) -> bool:
        """在当前位置施加夹持力。

        与 SDK 一致：``kp=150, kd=2``，目标 = 当前 θ，前馈 = ``force_n × 0.1``。
        因为目标就在当前位置，位置误差为零，关节力只剩前馈 —— 这才是真正的力控。
        """
        self._check_connected()
        self._check_enabled()
        current = self._theta()
        tau_nm = C.n_to_nm(force_n)
        with self._lock:
            self._ctrl.set_gains(150.0, 2.0)
            self._ctrl.set_feedforward(tau_nm)
            self._ctrl.hold_current(current, tau_nm)
            self._ramp = Hold(current)
        self._sleep(duration)
        return True

    # ══════════════════════════════════════════════════════════════════
    # 速度控制
    # ══════════════════════════════════════════════════════════════════

    def move_at_speed(
        self,
        target_mm: float,
        speed_mm_s: float = 30.0,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
    ) -> bool:
        """以恒定线速度（mm/s）移动到目标位置。"""
        self._check_connected()
        self._check_enabled()
        current_mm = C.q_to_mm(self._q())
        distance = abs(target_mm - current_mm)
        if distance < 0.01 or speed_mm_s <= 0:
            return True
        # mm/s → rad/s：先换算到 θ，再除以 MM_SCALE/1.14
        rad_to_mm = C.MM_SCALE / abs(C.POS_CLOSED_RAD - C.POS_OPEN_RAD)
        return self.move_at_speed_rad(
            C.mm_to_theta(target_mm), speed_mm_s / rad_to_mm, kp=kp, kd=kd
        )

    def move_at_speed_rad(
        self,
        target_rad: float,
        speed_rad_s: float = 0.5,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
    ) -> bool:
        """以恒定角速度（rad/s）移动到目标角度。"""
        self._check_connected()
        self._check_enabled()
        current = self._theta()
        if abs(target_rad - current) < 1e-4 or speed_rad_s <= 0:
            return True
        target = C.clamp_theta(target_rad)
        return self._run_ramp(
            LinearRamp(current, target, speed_rad_s),
            kp=kp, kd=kd, tau_ff=0.0, dq_target=None,
        )

    # ══════════════════════════════════════════════════════════════════
    # 低层帧接口
    # ══════════════════════════════════════════════════════════════════

    def send_mit_frame(
        self, q: float, kp: float, kd: float, dq: float = 0.0, tau: float = 0.0
    ) -> None:
        """直接下发一帧 MIT 控制量：``f(q, dq, kp, kd, tau)``。

        在仿真里的含义是"把控制律设成这样，让物理线程继续走"。真机上这是一帧
        CAN 报文，需要持续发。想单步推进用 :meth:`step`。
        """
        self._check_connected()
        with self._lock:
            self._ramp = None
            self._ctrl.set_gains(kp, kd)
            self._ctrl.set_target(q, dq)
            self._ctrl.set_feedforward(tau)

    def poll(self, timeout_s: float = 0.0) -> None:
        """真机上用于排空 CAN 接收缓冲。仿真里只做等价的时间等待。"""
        if timeout_s > 0:
            time.sleep(timeout_s)

    def read_param(self, rid: int, timeout_s: float = 0.5) -> float:
        """仿真中不可用 —— 没有电机寄存器。

        Raises:
            NotImplementedError: 总是抛出。真机上它读的是 DM 驱动器寄存器。
        """
        raise NotImplementedError(
            "仿真里没有电机寄存器；read_param() 只在真机上可用。"
        )

    # ══════════════════════════════════════════════════════════════════
    # 零重力
    # ══════════════════════════════════════════════════════════════════

    def enter_zero_gravity(self, duration: float = 0.0) -> bool:
        """进入零重力（拖动示教）模式：增益清零，指爪可被外力推动。

        ⚠ 与真机有关键差异：本模型的指爪沿**水平** X 轴滑动，重力不产生任何
        关节力矩（实测 ``qfrc_bias[:2]`` 恒为 ``[0, 0]``）。所以这里不需要、也
        没有做重力补偿——"零重力"在仿真里就等于"增益为零"。若把夹爪改成立装
        （挂到机械臂法兰上）就必须补上 ``qfrc_bias``，否则它会自己滑走。
        """
        self._check_connected()
        with self._lock:
            self._zero_gravity = True
            self._ramp = None
            self._ctrl.set_gains(0.0, 0.0)
            self._ctrl.set_feedforward(0.0)
        if duration > 0:
            self._sleep(duration)
            self.exit_zero_gravity()
        return True

    def exit_zero_gravity(self) -> bool:
        """退出零重力模式，恢复配置里的增益。"""
        with self._lock:
            self._zero_gravity = False
            self._ctrl.set_gains(self._config.kp, self._config.kd)
        return True

    # ══════════════════════════════════════════════════════════════════
    # 状态读取
    # ══════════════════════════════════════════════════════════════════

    def get_state(self, wait: bool = True) -> GripperState:
        """返回一帧状态快照。``wait`` 在仿真里无意义（保留以对齐签名）。

        与真机的一处**刻意差异**：``data_age_s`` 恒为 ``0.0``，于是 ``is_stale``
        恒为 ``False``、``has_data`` 恒为 ``True``。真机上这个字段是有意义的 ——
        失能的电机不主动发状态帧，``get_state()`` 返回的是缓存或构造默认值，
        所以 SDK 给它 ``inf`` 默认值让调用方能识别"这不是当前测量"。
        仿真里状态是从 ``mjData`` 直接算出来的，**定义上瞬时且精确**，没有
        陈旧可言。照抄 ``inf`` 会让 `if state.is_stale: 丢弃读数` 在仿真里
        永远为真，把一次好读数当成坏读数丢掉。
        """
        self._check_connected()
        with self._lock:
            theta = self._theta()
            dtheta = self._dtheta()
            torque = self._applied_torque()
            q = self._q_reported()
        return GripperState(
            position_rad=theta,
            velocity_rad_s=dtheta,
            torque_nm=torque,
            temperature_mos=int(round(self._temp_mos)),
            temperature_coil=int(round(self._temp_coil)),
            error_code=self._error_code,
            timestamp=time.time(),
            data_age_s=0.0,
            position_mm=C.q_to_mm(q),
            force_n=C.nm_to_n(torque),
        )

    def refresh_status(self, timeout_s: float = 0.5) -> bool:
        """请求一帧状态。仿真里恒返回 ``True``。

        真机上这个方法是为了**失能状态下也能读到位置**：失能的电机不主动发
        状态帧，``get_state()`` 只会返回缓存，所以 SDK 发 0xCC 刷新命令主动
        要一帧（电机不管使能与否都会应答）。仿真里状态随时可读，没有这个
        问题，所以直接返回 True；``timeout_s`` 保留以对齐签名。

        保留这个方法本身是必要的：真机代码里 ``real.refresh_status()`` 换成
        ``MujocoGripper`` 之后不能变成 ``AttributeError``。
        """
        self._check_connected()
        return True

    def get_position(self) -> float:
        """当前行程 (mm)。0 = 闭合，``MM_SCALE`` = 全开。"""
        return self.get_state().position_mm

    def get_position_rad(self) -> float:
        """当前电机角 (rad)。"""
        self._check_connected()
        return self.get_state().position_rad

    def get_force(self) -> float:
        """夹持力估计 (N)。

        ⚠ 由力矩推算，不是接触力测量 —— 空夹顶到限位时同样会报出数值。见模块 docstring。
        """
        return self.get_state().force_n

    def get_torque(self) -> float:
        """电机力矩 (Nm)。"""
        self._check_connected()
        return self.get_state().torque_nm

    def get_error(self) -> int:
        """电机错误码（0=失能，1=使能，0x9=欠压…）。"""
        self._check_connected()
        return self._error_code

    def get_temperature(self) -> Tuple[int, int]:
        """返回 ``(MOS温度, 线圈温度)``，摄氏度。

        仿真中是示意模型（环境温度 + 与 |τ|² 成正比的焦耳热 + 一阶散热），
        URDF 里没有任何热参数，不要拿这些数字当热设计依据。
        """
        self._check_connected()
        return int(round(self._temp_mos)), int(round(self._temp_coil))

    def get_info(self) -> GripperInfo:
        """返回设备元信息。"""
        return GripperInfo(
            model=os.path.basename(self._model_path),
            motor_type=str(self._motor_type),
            can_id=self._can_id,
            mst_id=self._mst_id or 0,
        )

    def is_moving(self) -> bool:
        """是否正在运动（``|角速度| > 0.01 rad/s``，与 SDK 同判据）。"""
        return self.get_state().is_moving

    def is_grasped(self) -> bool:
        """``|力矩| > grasp_torque_threshold``（默认 0.5 Nm），与 SDK 同判据。"""
        return abs(self.get_state().torque_nm) > self._config.grasp_torque_threshold

    def wait_for_ready(self, timeout: float = 5.0) -> bool:
        """阻塞直到使能且静止。"""
        start = time.time()
        while time.time() - start < timeout:
            if self._error_code == 1 and not self.is_moving():
                return True
            time.sleep(0.05)
        return False

    # ══════════════════════════════════════════════════════════════════
    # 标定
    # ══════════════════════════════════════════════════════════════════
    #
    # 仿真里"标定"是**解析真值**：模型自己就知道行程端点，不需要去撞限位。
    # 三个 calibrate* 接口都保留以实现 API 对等，但它们不产生运动。

    def calibrate(
        self,
        kp: float = 60.0,
        kd: float = 2.0,
        step_rad: float = 0.1,
        stall_delta: float = 0.0003,
        stall_cycles: int = 8,
        max_iter: int = 30,
    ) -> CalibrationData:
        """返回仿真模型的开合端点（解析真值，不运动）。"""
        return self._true_calibration()

    def calibrate_manual(
        self, duration: float = 30.0, settle_time: float = 2.0,
        sample_interval: float = 0.01,
    ) -> CalibrationData:
        """返回仿真模型的开合端点（解析真值，不运动）。"""
        return self._true_calibration()

    def calibrate_guided(
        self, kp: float = 60.0, kd: float = 2.0, step_rad: float = 0.08,
        stall_delta: float = 0.0004, stall_cycles: int = 6, max_iter: int = 40,
    ) -> CalibrationData:
        """返回仿真模型的开合端点（解析真值，不运动）。

        ⚠ 真机上这个函数写死了 120 刻度（``gripper.py`` 里
        ``rad_to_mm = 120.0 / travel``，忽略 ``config.max_stroke_mm``），而
        ``calibrate()`` 与 ``calibrate_manual()`` 都正确使用了它。走引导式标定
        必然产出 120 名义刻度，抵消掉本仓库统一的真实毫米口径。详见 README。
        """
        return self._true_calibration()

    def _true_calibration(self) -> CalibrationData:
        return CalibrationData(
            zero_position=C.POS_CLOSED_RAD,
            max_position=C.POS_OPEN_RAD,
            travel_range=abs(C.POS_CLOSED_RAD - C.POS_OPEN_RAD),
            rad_to_mm=C.MM_SCALE / abs(C.POS_CLOSED_RAD - C.POS_OPEN_RAD),
            motor_type=str(self._motor_type),
            can_id=self._can_id,
            mst_id=self._mst_id or 0x18,
            calibration_time=time.strftime("%Y-%m-%dT%H:%M:%S"),
        )

    def save_calibration(self, path: Optional[str] = None) -> str:
        """把当前 ``config`` 写成标定 JSON（格式与 SDK 相同）。"""
        import json

        if path is None:
            path = _default_calib_path()
        data = {
            "zero_position_rad": self._config.pos_closed_rad,
            "max_position_rad": self._config.pos_open_rad,
            "rad_to_mm": self._config.rad_to_mm,
            "can_id": self._config.can_id,
            "channel": self._config.can_channel,
            "kp": self._config.kp,
            "kd": self._config.kd,
            "grasp_torque_threshold": self._config.grasp_torque_threshold,
            "calibration_time": time.strftime("%Y-%m-%dT%H:%M:%S"),
        }
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w") as handle:
            json.dump(data, handle, indent=2)
        return path

    def load_calibration(self, path: Optional[str] = None) -> bool:
        """从标定 JSON 载入端点。格式与 SDK 相同，可直接用真机的标定文件。"""
        import json

        if path is None:
            path = _default_calib_path()
        try:
            with open(path, "r") as handle:
                data = json.load(handle)
        except (FileNotFoundError, ValueError):
            return False

        self._config.pos_closed_rad = float(data["zero_position_rad"])
        self._config.pos_open_rad = float(data["max_position_rad"])
        if "rad_to_mm" in data:
            self._config.rad_to_mm = float(data["rad_to_mm"])
        for key, attr in (
            ("can_id", "can_id"),
            ("channel", "can_channel"),
            ("kp", "kp"),
            ("kd", "kd"),
            ("grasp_torque_threshold", "grasp_torque_threshold"),
        ):
            if key in data:
                setattr(self._config, attr, data[key])
        if "can_id" in data:
            self._can_id = int(data["can_id"])
        return True

    # ══════════════════════════════════════════════════════════════════
    # 仿真专有
    # ══════════════════════════════════════════════════════════════════

    def connected(self) -> bool:
        """后台仿真线程是否在跑。

        与 :attr:`is_connected` 的区别：那个是 SDK 对等面上的"逻辑连接"标志，
        ``connect()`` 一置到位就为 True；这个还额外要求线程真的活着。窗口被关
        掉之后线程会自己退出，于是 ``connected()`` 变 False，而 ``is_connected``
        仍是 True 直到 :meth:`disconnect`。

        循环该不该继续，看这个。
        """
        return bool(self._connected and self._running)

    @property
    def gui(self) -> bool:
        """当前有没有窗口（构造时 ``render=True`` 或调过 :meth:`launch_viewer`）。"""
        return self._viewer is not None

    @property
    def sim_time(self) -> float:
        """累计仿真时间 (s)，从构造起算。

        与 :meth:`data` 里的 ``time`` 不同：那个会被 :meth:`reset` 用关键帧
        清零，这个只增不减。录制轨迹、量"跑了多久"都用它 —— 一个会被复位打断
        的时钟做不了这两件事。

        ``realtime=False`` 时它会跑在墙钟前面，所以别拿它当秒表。
        """
        return float(self._sim_time)

    def pump(self) -> bool:
        """推进一拍并刷新画面，返回**循环该不该继续**。放进 ``while`` 条件里。

            while gripper.pump():
                ...            # 读按键、打印状态

        ``False`` 的两种情形：窗口被关掉了，或者有人调了 :meth:`stop`（中止标志
        置位）。两种都意味着"该收尾了"，所以调用方不需要分辨。

        **不会重复推进物理**：仿真线程在跑时它只等，没跑时才自己走一步。所以
        有窗口、有线程的常规跑法下，物理由线程按 ``realtime`` 的节拍推，主循环
        只管刷新和响应输入。
        """
        if self._abort.is_set():
            return False
        viewer = self._viewer
        if viewer is not None:
            try:
                if not viewer.is_running():
                    return False
            except Exception:
                return False

        if self._connected:
            # 物理线程在推。这里只等它一拍 —— 自己再 mj_step 一次就是两个线程
            # 同时碰 mjData（见 _open_viewer 的 docstring）。
            time.sleep(_POLL_S)
            return True

        with self._lock:
            self._tick(float(self._model.opt.timestep))
            if viewer is not None:
                # 没有线程就没人 sync，画面会冻在最后一帧。补上。
                try:
                    viewer.sync()
                except Exception:
                    return False
        if self._realtime:
            time.sleep(float(self._model.opt.timestep))
        return True

    def step(self, n: int = 1) -> None:
        """手动推 n 个物理步（仅在未 connect()、即线程没跑时用）。"""
        if self._connected:
            raise RuntimeError("后台线程已在跑；直接让仿真自己推进即可")
        with self._lock:
            for _ in range(n):
                self._tick(self._model.opt.timestep)

    def settle(
        self,
        seconds: Optional[float] = None,
        *,
        tolerance_rad: float = C.DEFAULT_SETTLE_TOLERANCE_RAD,
        stall_rad_s: float = C.DEFAULT_SETTLE_STALL_RAD_S,
        stall_s: float = C.DEFAULT_SETTLE_STALL_S,
    ) -> Tuple[float, bool]:
        """放仿真自己跑。不改变控制目标 —— 要它走向哪儿先设好目标。

        两种用法，差别只在**什么时候停**：

        ``settle(0.5)``
            跑满 0.5 秒。哪怕指爪第 1 ms 就已经到位也照跑不误 —— 这段时间
            是留给**别的东西**的：工件下落、夹具松开后靠摩擦挂住、温度模型
            爬升。旧版就是这个语义（那时只是单纯睡一觉），保持不变。

        ``settle()``
            不带参数：跑到指爪**停下来**为止，最长
            :data:`constants.DEFAULT_SETTLE_TIMEOUT_S`。这里"停下来"有两种
            结局，返回值把它们分开。

        Args:
            seconds: 跑多久（**墙钟**秒）。不给则用"等到停住"模式。
            tolerance_rad: 目标角误差小于它就算"到位"。
            stall_rad_s: 角速度小于它就算"停住了"。
            stall_s: 连续停住这么久（**仿真**秒）才认定是停住，避免把
                起步瞬间的零速度误判成堵转。

        Returns:
            ``(elapsed_s, reached)``：实际走过的**仿真**秒数，以及结束时目标角
            误差是否在 ``tolerance_rad`` 以内。

            不带参数时 ``reached`` 才是重点：``False`` 且 ``elapsed_s`` 远小于
            超时 = **撞上工件停在半路**（夹住了东西）；``False`` 且 ``elapsed_s``
            贴着超时 = 一直没停到位（任务没完成）。带参数时它只是"跑完之后到没
            到位"的一次快照。

        Note:
            返回值是**新增**的 —— 旧版返回 ``None``。仓内所有调用点都忽略返回值，
            所以这处改动向后兼容；但它是公开签名的一部分，写进 PR 说明。
        """
        dt = float(self._model.opt.timestep)
        t0 = float(self._sim_time)

        if seconds is None:
            deadline = time.monotonic() + C.DEFAULT_SETTLE_TIMEOUT_S
            prev_t = t0
            stalled = 0.0
            while True:
                self._advance_settle(dt)
                if self._at_target(tolerance_rad):
                    return (float(self._sim_time) - t0, True)

                now_t = float(self._sim_time)
                if abs(self._dtheta()) <= stall_rad_s:
                    stalled += now_t - prev_t
                    if stalled >= stall_s:
                        break
                else:
                    stalled = 0.0
                prev_t = now_t

                if self._abort.is_set() or time.monotonic() >= deadline:
                    break
            return (float(self._sim_time) - t0, False)

        end = time.monotonic() + max(0.0, float(seconds))
        while time.monotonic() < end:
            self._advance_settle(dt)
            if self._abort.is_set():
                break
        return (float(self._sim_time) - t0, self._at_target(tolerance_rad))

    def _at_target(self, tolerance_rad: float) -> bool:
        """到位判定 = **斜坡走完** 且 目标角误差在容差内。

        "误差在容差内"这一条单独用是不够的：沿着斜坡匀速走的时候，目标每步只
        挪一丁点，PD 跟得又紧，位置误差会一直贴着零 —— 于是半路就会误判成
        "到位了"。必须等斜坡自己走完。
        """
        if self._ramp is not None and not self._ramp.done:
            return False
        return abs(self._theta() - self._ctrl.theta_des) <= tolerance_rad

    def _advance_settle(self, dt: float) -> None:
        """让仿真前进一小拍：线程在跑就等，没跑就自己走一步。"""
        self._check_alive()
        if self._connected:
            # 物理线程在推。这里只等，绝不自己 mj_step —— 两个线程同时碰
            # mjData 就是 _open_viewer 的 docstring 里记着的那个段错误。
            time.sleep(_POLL_S)
        else:
            with self._lock:
                self._tick(dt)

    def command_fraction(
        self,
        fraction: float,
        *,
        force_n: Optional[float] = None,
        velocity_m_s: float = C.DEFAULT_VELOCITY_M_S,
    ) -> bool:
        """发一条"走向某个开度"的指令，**立刻返回**。

        与 :meth:`goto` 那一族的区别只有一处：不等。指令装上斜坡就走，剩下的
        交给物理循环。这是遥操作/镜像这类"每拍发一条新目标"的场合要的形状 ——
        用阻塞版会把控制回路自己卡住。

        Args:
            fraction: 归一化开度 ∈ [0, 1]。0 = 全闭，1 = 全开。
            force_n: 电机出力上限 (N)。给定时把它换算成力矩限幅压在电机上，
                所以撞到东西顶多出这么大劲。**这不是力控** —— 位置环还在，
                误差大时输出会贴着上限饱和；想"夹住就不动了"要用
                :meth:`grasp`/:meth:`set_force`。不给则用结构上限
                :data:`constants.TAU_MAX`。
            velocity_m_s: 单指线速度上限 (m/s)。默认
                :data:`constants.DEFAULT_VELOCITY_M_S`，即全行程 1 秒。

        Note:
            斜坡起点取**当前**位置，不是上一条指令的目标。否则连发几条指令时，
            限速算的是"从上一个目标算起"的距离，实际速度会超。
        """
        self._check_connected()
        self._check_enabled()

        with self._lock:
            theta_now = self._theta()
        theta_goal = C.q_to_theta(C.q_from_frac_open(float(np.clip(fraction, 0.0, 1.0))))
        speed_rad_s = max(C.to_rad_s(abs(float(velocity_m_s))), 1e-9)
        self._install_ramp(
            LinearRamp(theta_now, theta_goal, speed_rad_s),
            None,
            None,
            0.0,
            None,
            tau_max=None if force_n is None else C.n_to_nm(float(force_n)),
        )
        return True

    # ── 窗口与输入 ─────────────────────────────────────────────────────

    def keyboard_events(self) -> Dict[int, int]:
        """取走自上次调用以来按下的键，返回 ``{键码: 事件}``。

        无窗口时返回空字典 —— 调用方不需要先判断有没有窗口。

        事件表由查看器的回调线程填、在这里被取空，所以**一次按键只会有一次
        事件**；循环里每次 :meth:`pump` 之后取一次即可，别在两处都取。
        键码用 :func:`litegrip_mujoco.key_label` 转成可读名字。
        """
        return self._keys.drain()

    def mouse_events(self) -> List[Any]:
        """永远是空列表。**MuJoCo 的被动查看器不报鼠标事件。**

        ``launch_passive`` 只收一个 ``key_callback``；鼠标被查看器内部拿去转
        视角和拖动扰动球了。这个方法是为了让例程的源码形状与 pybullet 那套一致
        而存在的哑元：``clicked(events) or pressed(events, CONFIRM_KEYS)`` 这种
        写法里，真值永远来自按键那一支。别指望它哪天会返回东西。
        """
        return []

    def status_text(self, lines: Sequence[str]) -> bool:
        """在查看器左上角叠加几行字。返回是否真的显示出来了。

        仿真读数（开度、力、状态）打在窗口里比刷终端方便：终端一行行滚走，
        窗口里的字贴着画面。

        没有窗口、查看器太老没有 ``set_texts``、或底层抛异常时返回 ``False``，
        **不抛异常** —— 叠字是锦上添花，不该让仿真因为显示不出来而中断。所以
        返回值只用来决定"要不要退回终端打印"。
        """
        viewer = self._viewer
        if viewer is None:
            return False
        setter = getattr(viewer, "set_texts", None)
        if setter is None:
            return False
        try:
            font = self._mujoco.mjtFontScale.mjFONTSCALE_150
            grid = self._mujoco.mjtGridPos.mjGRID_TOPLEFT
        except AttributeError:
            return False
        try:
            with self._lock:
                setter([(font, grid, str(line), "") for line in lines])
        except Exception:
            return False
        return True

    def focus_camera(
        self,
        *,
        lookat: Optional[Sequence[float]] = None,
        distance: Optional[float] = None,
        azimuth: Optional[float] = None,
        elevation: Optional[float] = None,
    ) -> bool:
        """摆一下查看器的相机。只改给了的参数，返回是否改成功。

        自动取景就是这个用法：算好夹爪中心，:meth:`link_aabb` 量出尺度，再把
        ``lookat``/``distance`` 设过来。没有窗口时返回 ``False``。
        """
        viewer = self._viewer
        cam = getattr(viewer, "cam", None) if viewer is not None else None
        if cam is None:
            return False
        try:
            with self._lock:
                if lookat is not None:
                    cam.lookat[:] = np.asarray(lookat, dtype=np.float64)[:3]
                if distance is not None:
                    cam.distance = float(distance)
                if azimuth is not None:
                    cam.azimuth = float(azimuth)
                if elevation is not None:
                    cam.elevation = float(elevation)
        except Exception:
            return False
        return True

    def reset(self, keyframe: Optional[str] = None) -> None:
        """复位到关键帧（默认为模型里的 ``home``，没有则回全开）。

        复位后控制器**保持在该关键帧**：目标被改写成关键帧位置的 θ，
        ``ctrl`` 归零。若只清目标（``_ctrl.reset()`` 把目标设成 0），
        位置环会立刻把指爪从关键帧拽走 —— 复位到 ``open`` 之后指爪马上
        自己合拢，复位就等于没做。
        """
        with self._lock:
            if keyframe is not None:
                self._apply_keyframe(keyframe)
            else:
                try:
                    self._apply_keyframe("home")
                except KeyError:
                    self._apply_keyframe("open")
            self._ramp = None
            self._ctrl.reset()
            self._ctrl.set_gains(self._config.kp, self._config.kd)
            self._ctrl.set_target(self._theta(), 0.0)
            self._abort.clear()
            self._status_flags &= ~(GripperStatus.MOVING | GripperStatus.GRASPED)

    def release_fixture(self, name: str = "fixture") -> bool:
        """解除场景夹具，让工件只靠夹持摩擦留在指间。

        仅对带 ``<weld>`` 夹具的场景模型有意义（见 ``assets/scene.xml``）。
        找不到该等式时抛 ``KeyError``。
        """
        return self._set_equality(name, False)

    def hold_fixture(self, name: str = "fixture") -> bool:
        """重新启用场景夹具。"""
        return self._set_equality(name, True)

    def _set_equality(self, name: str, active: bool) -> bool:
        if name not in self._eq_ids:
            raise KeyError(
                f"模型里没有名为 {name!r} 的等式；现有：{sorted(self._eq_ids)}"
            )
        with self._lock:
            self._data.eq_active[self._eq_ids[name]] = 1 if active else 0
        return True

    def gap_mm(self) -> float:
        """两指内侧面之间的**绝对开口** (mm)。闭合时 1.548 mm。

        与 :meth:`get_position` 的区别：后者是从闭合位起算的行程（闭合时为 0），
        本方法给的是绝对开口。见 ``constants.py``。
        """
        return C.q_to_gap_mm(self._q_reported())

    def frac_open(self) -> float:
        """无量纲开度 ∈ [0, 1]。0 = 闭合，1 = 全开。

        **跨设备（仿真 ↔ 真机）唯一可移植的开合量。** θ 的零点和量程都随标定
        而变，不能直接互换；见 ``constants.py`` 与 ``mirror.py``。
        """
        return C.frac_open_from_q(self._q_reported())

    def set_frac_open(self, frac_open: float) -> None:
        """按无量纲开度直接摆位（**不做斜坡**，用于镜像）。

        只写指关节那两项，绝不能写成 ``qpos[:n]`` —— scene.xml 里后面还跟着
        工件的 freejoint，整片赋值会把工件的位姿一起清掉。
        """
        with self._lock:
            q = C.q_from_frac_open(float(np.clip(frac_open, 0.0, 1.0)))
            self._ramp = None
            for i in self._qpos_idx:
                self._data.qpos[i] = q
            for i in self._dof_idx:
                self._data.qvel[i] = 0.0
            self._ctrl.set_target(C.q_to_theta(q), 0.0)

    def launch_viewer(self) -> None:
        """打开被动查看器（等价于构造时 ``render=True``）。"""
        self._open_viewer()

    def sync_viewer(self) -> None:
        """手工同步一次查看器画面。"""
        if self._viewer is not None:
            self._viewer.sync()

    # ══════════════════════════════════════════════════════════════════
    # 上下文管理器
    # ══════════════════════════════════════════════════════════════════

    def __enter__(self) -> "MujocoGripper":
        self.connect()
        self.enable()
        return self

    def __exit__(self, *args: Any) -> None:
        self.disconnect()

    def __repr__(self) -> str:
        status = "已连接" if self._connected else "未连接"
        enabled = "已使能" if self._enabled else "未使能"
        return (
            f"MujocoGripper(model={os.path.basename(self._model_path)}, "
            f"{status}, {enabled}, gap={self.gap_mm():.3f}mm)"
        )

    # ══════════════════════════════════════════════════════════════════
    # 内部
    # ══════════════════════════════════════════════════════════════════

    def _sim_loop(self) -> None:
        """后台物理循环。绝对截止时刻调度，避免睡偏累积。"""
        dt = float(self._model.opt.timestep)
        deadline = time.monotonic()
        while self._running:
            with self._lock:
                self._tick(dt)
                # sync() 必须和 mj_step() 在同一把锁里：sync() 会把 mjData
                # 拷给查看器内部的副本，而主线程的 close()/goto() 斜坡循环
                # 同样在锁内 mj_step。放在锁外就是两个线程一起碰 mjData，
                # 与 _open_viewer 里那个顺序 bug 是同一种病。
                viewer = self._viewer
                if viewer is not None:
                    try:
                        if not viewer.is_running():
                            # 窗口被关掉 = 操作者要退出。置中止并结束循环，
                            # 好让阻塞在 open()/close() 里的主线程醒过来。
                            #
                            # **这里不碰 self._viewer**：关窗和 close() 是两件
                            # 事。把引用清在这里，disconnect() 就再也关不到那个
                            # 已经关掉的句柄（无害），但更要紧的是清掉之后
                            # disconnect() 的"线程卡住就别关窗"判定会失去依据，
                            # 反而可能去关一个正在被 sync 的窗口。收尾统一交给
                            # disconnect()。
                            self._abort.set()
                            self._running = False
                            break
                        viewer.sync()
                    except Exception:
                        self._viewer = None
            if self._realtime:
                deadline += dt
                slack = deadline - time.monotonic()
                if slack > 0:
                    time.sleep(slack)
                else:
                    # 落后太多就重新对齐，不要试图追赶积压
                    deadline = time.monotonic()

    def _tick(self, dt: float) -> None:
        """推进一个物理步（调用者必须持锁）。"""
        if self._ramp is not None:
            value, rate = self._ramp.advance(dt)
            self._ctrl.set_target(value, rate)
            if self._ramp.done:
                self._status_flags |= GripperStatus.AT_TARGET

        if self._enabled and not self._zero_gravity:
            self._data.ctrl[:] = self._ctrl.compute(self._theta(), self._dtheta())
        else:
            self._data.ctrl[:] = 0.0

        self._mujoco.mj_step(self._model, self._data)
        self._sim_time += dt
        self._update_thermal(dt)

    def _install_ramp(
        self,
        ramp: Ramp,
        kp: Optional[float],
        kd: Optional[float],
        tau_ff: float,
        dq_target: Optional[float],
        tau_max: Optional[float] = None,
    ) -> None:
        """清中止标志、装增益/前馈/斜坡，并置 MOVING。**非阻塞**。

        这一步和"轮询到走完"是两件事，``_run_ramp`` 把两者串起来是为了给
        ``open()``/``close()`` 那种"发一条指令、等它到位"的语义；而
        :meth:`command_fraction` 只要前半段 —— 装上就走，由物理循环推。

        ``tau_max=None`` 表示恢复结构上限 :data:`constants.TAU_MAX`。**每次
        调用都会重设**，所以一次限力运动不会把限幅漏给下一条指令。

        ⚠ 顺序是这个方法的一部分，别重排：先清 ``_abort``，再在锁内一次性把
        增益、前馈、目标、斜坡全部就位。``_run_ramp`` 的力语义用例依赖
        "``_abort`` 已清"这一前置条件。
        """
        self._abort.clear()
        with self._lock:
            self._ctrl.set_gains(
                kp if kp is not None else self._config.kp,
                kd if kd is not None else self._config.kd,
            )
            self._ctrl.set_feedforward(tau_ff)
            self._ctrl.set_torque_limit(tau_max)
            if dq_target is not None:
                self._ctrl.set_target(self._theta(), dq_target)
            self._ramp = ramp
            self._status_flags |= GripperStatus.MOVING

    def _run_ramp(
        self,
        ramp: Ramp,
        kp: Optional[float],
        kd: Optional[float],
        tau_ff: float,
        dq_target: Optional[float],
    ) -> bool:
        """装上一段斜坡并阻塞到它走完。"""
        self._install_ramp(ramp, kp, kd, tau_ff, dq_target)

        try:
            while not ramp.done:
                if self._abort.is_set():
                    return False
                self._check_alive()
                time.sleep(_POLL_S)
        finally:
            with self._lock:
                self._status_flags &= ~GripperStatus.MOVING
                self._status_flags |= GripperStatus.AT_TARGET
        return True

    def _sleep(self, seconds: float) -> None:
        """按墙钟睡 ``seconds``，可被 stop() 打断。分段睡以免长时间持锁。"""
        end = time.monotonic() + max(0.0, seconds)
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0 or self._abort.is_set():
                return
            time.sleep(min(remaining, 0.01))

    def _theta(self) -> float:
        """当前电机角。两指刚性耦合，取均值以抑制数值抖动。"""
        return C.q_to_theta(self._q())

    def _dtheta(self) -> float:
        qvel = self._data.qvel
        v = float(np.mean([qvel[i] for i in self._dof_idx]))
        return C.to_rad_s(v)

    def _q(self) -> float:
        qpos = self._data.qpos
        return float(np.mean([qpos[i] for i in self._qpos_idx]))

    def _q_reported(self) -> float:
        """对外汇报用的关节位移，已夹到 [0, STROKE]。

        软关节限位在撞到闭合位时会过冲约 2 µm，不夹的话 ``get_position()`` 会
        报出 ``-0.002 mm`` 这种真机不可能出现的负行程。
        """
        return C.clamp_q(self._q())

    def _applied_torque(self) -> float:
        """电机当前出力矩 (Nm) = ``data.ctrl``（= ``qfrc_actuator / GEAR``）。"""
        if not self._enabled or self._zero_gravity:
            return 0.0
        return float(self._data.ctrl[0]) if self._data.ctrl.size else 0.0

    def _apply_keyframe(self, name: str) -> None:
        key_id = self._mujoco.mj_name2id(
            self._model, self._mujoco.mjtObj.mjOBJ_KEY, name
        )
        if key_id < 0:
            raise KeyError(
                f"模型 {os.path.basename(self._model_path)} 里没有关键帧 {name!r}"
            )
        self._mujoco.mj_resetDataKeyframe(self._model, self._data, key_id)
        self._mujoco.mj_forward(self._model, self._data)

    def _update_thermal(self, dt: float) -> None:
        """一阶热模型：焦耳热 ∝ τ²，向环境温度散热。示意用，非标定值。"""
        tau = abs(float(self._data.ctrl[0])) if self._data.ctrl.size else 0.0
        heat = _HEAT_GAIN * tau * tau
        for attr, tau_thermal in (("_temp_coil", _TAU_COIL), ("_temp_mos", _TAU_MOS)):
            current = getattr(self, attr)
            target = _AMBIENT_C + heat * tau_thermal
            setattr(self, attr, current + (target - current) * dt / tau_thermal)

    def _open_viewer(self) -> None:
        """打开被动查看器。**不会**顺手启动仿真线程。

        ⚠ 顺序是这个方法的全部要点：查看器必须在**仿真线程启动之前**建好。

        ``mujoco.viewer.launch_passive()`` 内部会对同一个 ``mjData`` 调
        ``mj_forward()``（见 mujoco/viewer.py 的 launch_passive），而
        ``_sim_loop`` 同时在 ``mj_step()`` 同一个 mjData。两个线程同时进
        mjData 的 arena，MuJoCo 就没法给它扩容，于是
        ``mj_makeConstraint: nefc under-allocation`` —— 运气差的时候不是报错
        而是直接段错误。

        这个 bug 曾经藏在这里：原先先 ``connect()`` 起线程、再
        ``launch_passive()``，两者必然重叠。表现是**间歇性**的（碰不上就没事），
        实测三次里崩一次、崩的位置每次不同，所以尤其容易被当成"环境问题"。
        查看器只建窗口，连不连由调用方决定。
        """
        with self._lock:
            if self._viewer is not None:
                return
            try:
                import mujoco.viewer

                launch = mujoco.viewer.launch_passive
                if _accepts_key_callback(launch):
                    self._viewer = launch(
                        self._model, self._data, key_callback=self._on_key
                    )
                else:
                    # MuJoCo 太老，launch_passive 不收 key_callback。窗口照开，
                    # 只是收不到按键 —— 例程会退化成"只能看，不能遥控"。
                    self._viewer = launch(self._model, self._data)
            except Exception as exc:
                raise RuntimeError(
                    f"打不开 MuJoCo 查看器（需要图形环境）：{exc}"
                ) from exc

    def _on_key(self, keycode: int) -> None:
        """查看器按键回调。**在查看器自己的线程里跑**。

        这个函数只能做一件事：把一个整数塞进 :class:`KeyQueue`。绝不能在这里
        碰 ``mjData``（物理线程正拿着它）、不能开关窗口、不能打印 —— 回调里
        抛出的异常会堆在查看器线程上，而它离 MuJoCo 的内部状态太近。
        """
        self._keys.feed(int(keycode))

    def _check_connected(self) -> None:
        if not self._connected:
            raise NotInitializedError("未连接 — 请先调用 connect() 或使用 with 上下文")

    def _check_enabled(self) -> None:
        if not self._enabled:
            raise NotInitializedError("未使能 — 请先调用 enable()")

    def _check_alive(self) -> None:
        """确认后台物理线程还活着。

        物理线程一旦因异常退出，斜坡就再也不会被推进，阻塞式调用会**永远**等下去。
        宁可在这里响亮地失败，也不要让调用方挂死。
        """
        if self._thread is not None and not self._thread.is_alive():
            raise RuntimeError(
                "仿真线程已退出（多半是物理步里抛了异常）。"
                "重跑并查看 stderr 里 'Exception in thread litegrip_sim' 的堆栈。"
            )


# ═════════════════════════════════════════════════════════════════════════
# 模块级辅助
# ═════════════════════════════════════════════════════════════════════════

#: 热模型参数（示意，非标定值）。
_AMBIENT_C = 25.0
_HEAT_GAIN = 180.0        # τ=10 Nm 时的稳态温升上限系数
_TAU_COIL = 25.0          # 线圈热时间常数 (s)
_TAU_MOS = 40.0           # MOS 热时间常数 (s)


def _sim_config() -> GripperConfig:
    """仿真侧的默认配置：真实毫米口径 + 满足不变量的 θ 端点。"""
    return GripperConfig(
        can_channel="can0",
        can_id=0x08,
        kp=C.DEFAULT_KP,
        kd=C.DEFAULT_KD,
        pos_closed_rad=C.POS_CLOSED_RAD,
        pos_open_rad=C.POS_OPEN_RAD,
        max_stroke_mm=C.MM_SCALE,
        rad_to_mm=C.MM_SCALE / abs(C.POS_CLOSED_RAD - C.POS_OPEN_RAD),
        nm_to_n=C.NM_TO_N,
        grasp_torque_threshold=C.GRASP_TORQUE_THRESHOLD,
    )


def _default_calib_path() -> str:
    """默认标定路径，与 SDK 的 ``DEFAULT_CALIB`` 位置保持一致。"""
    return os.path.join(os.path.expanduser("~"), ".litegrip", "litegrip_calibration.json")
