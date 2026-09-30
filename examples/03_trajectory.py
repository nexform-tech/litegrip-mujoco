#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 03 · 轨迹录制与回放 — 手拖一遍，仿真和夹爪一起重现

录的时候真机进**零重力**：两个手指的力被撤掉，可以用手推着走，SDK 在后台按 100 Hz
采样，记下每一拍的开度。录完存成一段 ``.lgt``，回放时同一段轨迹**同时**喂给真机和
MuJoCo——真机的电机按轨迹走，窗口里的仿真跟着显示，所以「仿真里看到的」就是
「夹爪正在做的」。

录的是**归一化开度**（0 闭合 … 1 张开），不是角度：换一台夹爪、换一份标定，同一段
轨迹照样能放，因为开度会按**本机**的标定换算成角度。

前提:
  1. 真机接在 CAN 总线上（默认 can0，用 --channel 换），录制时得能够到两个手指
  2. 装了带轨迹接口的 litegrip SDK（三个真机样例用的是同一份，
     nexform-tech/litegrip-python）：
       pip install -e /path/to/litegrip-python
     或 export LITEGRIP_SDK_DIR=/path/to/litegrip-python/src
     或把 litegrip-python 仓库克隆到本仓库的同级目录
  3. 一份可用的标定：录制要求 SDK 处于**已标定**状态（轨迹按行程归一，没有标定
     就算不出开度），回放也要按本机行程把开度换算回角度。不给 --calib 就用 SDK
     包里那份出厂标定；出厂文件也读不出来才会在终端里列候选让你选
  4. 录制需要手指能被推动，所以录制期间**不要**让别的程序同时驱动这台夹爪

两种用法:

  * **录一段再放**（默认）：连接 → 进零重力 → 手拖 → **按 Enter / 空格结束录制**
    → 存盘 → **按 Enter / 空格** → 真机和仿真一起回放。
  * **放一段已有的**（``--play``）：默认**只灌仿真**，不连真机、一帧都不发，用来
    回看之前录的东西。要在真机上也放，加 ``--real``。

录制结束**不会**自己接着回放，要一个明确的动作才开始，所以这里有两个动作：录制期间
Enter / 空格 是「录完了」，Esc / Q 是「这次不算，退出」；录完之后 Enter / 空格 是
「开始回放」，Esc / Q 是「先不放，退出」。回放会把真机动起来，这几秒手要离开行程，
所以**不**让它自己开始。

注意：**使能态**的电机静默就锁进通信丢失故障（0xD）——红灯闪、位置照读、指令一律
不执行。多久算静默，两个数不一致：本仓在这台机器上实测约 0.9 s，而 SDK 自己的文档
写的是约 100 ms，取短的为准——本样例按 200 Hz 连续喂，两个数都够不着。录制结束时
SDK 只补一帧 ``exit_zero_gravity()`` 就撒手，所以本样例在「录制结束 → 开始回放」这
段空档里自己发「锁在实测位置」的保持帧——等你确认的那段也算在里面，回放结束时也
一样。发的不是运动指令：目标就是它当时的位置，零前馈。

注意：回放结束后手指停在轨迹终点，**不会**自己回去。按 Esc / Q 退出时本样例会失能
（0xFD），手指随之变松、可能因自重滑动。

没有 CAN 硬件时用 ``--dry-run``：这时**仿真自己当那台夹爪**——主循环按一条脚本化
的开合曲线摆位，后台录制线程照常按 100 Hz 采样，存盘、读回、回放整条链路都跑到，
只有 CAN 总线这一层没有。仿真里手指推不动（查看器把鼠标留着控制相机），所以这条
路径是脚本驱动而不是手拖；真机那一段**未经验证**。

运行:
  python3 examples/03_trajectory.py --record 6 --calib ~/.litegrip/litegrip_calibration.json
  python3 examples/03_trajectory.py --calib ~/.litegrip/litegrip_calibration.json
  python3 examples/03_trajectory.py --play 03_hand_taught-20260929-120000
  python3 examples/03_trajectory.py --play 03_hand_taught-20260929-120000 --real
  python3 examples/03_trajectory.py --play 03_hand_taught-20260929-120000 --speed 0.5 --headless
  python3 examples/03_trajectory.py --dry-run --record 3 --headless
"""
import argparse
import math
import sys
import time

from _common import (  # noqa: I001  (必须先于 litegrip_mujoco)
    SAFETY_BANNER,
    add_common_args,
    add_hardware_args,
    check_sdk_api,
    fraction_to_gap_mm,
    import_litegrip,
    make_sim,
    open_real_gripper,
    rad_to_fraction,
    status_line,
)

from litegrip_mujoco import CONFIRM_KEYS, QUIT_KEYS, pressed

#: 录制/回放的采样率 [Hz]（SDK 的默认值）。录得比手动拖动快得多，所以样本之间
#: 线性插值不会漏掉手上的动作。
RATE_HZ = 100.0

#: 空档期保持帧的频率 [Hz]，和 04 里那个保活循环同一个理由、同一个数。
#: 只有真机路径用得上：仿真里没有总线，也就没有通信丢失故障。
FRAME_HZ = 200.0
FRAME_DT = 1.0 / FRAME_HZ

#: 实测的通信超时闩锁时间 [s]：使能态的电机静默这么久就报 0xD。
#:
#: 只用来把话说具体，逻辑上不依赖它——保持帧是按 200 Hz 发的，比它短一个数量级。
#: 而且这个数本身只是本仓的实测：电机的 ``TIMEOUT`` 寄存器读到过 8000 ms、也读到过
#: 0（SDK 自己标了「待查」），SDK 的轨迹模块又写「约 100 ms」。按最短的那个喂。
MEASURED_COMM_LOSS_S = 0.9

#: 终端读数的最小刷新间隔 [s]。
PRINT_DT = 0.5

#: 读一帧状态帧最多等多久 [s]（SDK 的 ``get_state(wait=True)`` 内部也是等 50 ms）。
POLL_WAIT_S = 0.05

#: 存盘用的名字前缀；真正的文件名还会带上录制时刻，免得覆盖上一次录的。
DEFAULT_NAME = "03_hand_taught"

#: ``--dry-run`` 里那条「手拖」曲线的周期 [s]：从全开滑到 30%，再滑回来。
#: 选 3 s 是为了让 100 Hz 采样在一段里采到 300 个点——比真手拖慢，看得更清楚。
DRY_RUN_PERIOD_S = 3.0

#: ``--dry-run`` 曲线的开度范围（最低 → 最高）。留一点余量不打到 0，因为
#: ``set_frac_open`` 会夹在 [0, 1] 上，贴着端点看不出插值。
DRY_RUN_LOW, DRY_RUN_HIGH = 0.3, 1.0


def parse_args():
    ap = argparse.ArgumentParser(
        description="样例 03 · 轨迹录制与回放：手拖一遍真机，仿真和夹爪一起重现",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_common_args(ap)
    add_hardware_args(ap)
    ap.add_argument("--record", type=float, default=0.0,
                    help="录制多少秒后自动停（默认 0 = 一直录到 Enter/空格）")
    ap.add_argument("--play", default=None,
                    help="回放一段已有的轨迹（名字或路径，不带 .lgt 也行）。默认"
                         "**只灌仿真**：不连真机、一帧都不发；要在真机上也放，"
                         "加 --real")
    ap.add_argument("--real", action="store_true",
                    help="配合 --play：这段轨迹也下发给真机（两个手指会真实运动）")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="回放倍速（默认 1.0；0.5 = 半速，2 = 两倍速）")
    ap.add_argument("--dry-run", action="store_true",
                    help="不碰 CAN：**仿真自己当那台夹爪**，主循环按一条脚本化的"
                         "开合曲线摆位，后台录制线程照常按 100 Hz 采样。存盘、"
                         "读回、回放整条链路都跑到。真机那一段未经验证")
    args = ap.parse_args()
    if args.play and args.record:
        ap.error("--record 和 --play 不能同时给：--play 放的是已有的轨迹，"
                 "不会现场录一段")
    if args.real and not args.play:
        ap.error("--real 只在 --play 时有意义：录制本来就是在真机上录的")
    if args.real and args.dry_run:
        ap.error("--real 要连真机，--dry-run 正好相反：两个只能给一个")
    if args.speed <= 0:
        ap.error(f"--speed 要大于 0（给的是 {args.speed:g}）")
    return args


def read_state(gripper, timeout_s=POLL_WAIT_S):
    """读一帧状态；这一帧没等到就返回 ``None``。

    这是 :func:`_common.fresh_state` 的同一条判据，只是多吞一层传输异常（真机在
    CAN 适配器掉线时会抛）。``poll`` 为真就意味着这次调用里解出了一帧本电机的状态
    帧，所以紧随其后的快照是实测值。等不到就返回 ``None``，调用方**不许**拿缓存里
    的数当位置——缓存里可能是 ``MotorState._position`` 的初值 ``0.0``，那不是
    「夹爪在 0 弧度」，是「从没读到过」。
    """
    try:
        if not gripper.poll(timeout_s=timeout_s):
            return None
        return gripper.get_state(wait=False)
    except Exception:            # 传输层掉了等等：没有帧就是没有帧
        return None


def mirror(gripper, sim, state):
    """把仿真手指瞬移到真机现在的位置，返回归一化开度（读不到就返回 ``None``）。

    用 ``set_frac_open``（运动学瞬移）而不是 ``command_fraction``（跑动力学追过去）：
    镜像要的是「真机现在在哪」，不是「仿真打算去哪」。读不到状态帧时**不动**画面，
    让它停在最后一次读数上——照缓存的伪值渲染等于撒谎。
    """
    if state is None:
        return None
    fraction = rad_to_fraction(gripper, state.position_rad)
    sim.set_frac_open(fraction)
    return fraction


class HoldKeeper:
    """空档期给电机喂「锁在实测位置」的保持帧。

    为什么必须有它：录制结束时 SDK 只补一帧 ``exit_zero_gravity()`` 就撒手，而
    **使能态**的电机静默约 :data:`MEASURED_COMM_LOSS_S` 就锁进通信丢失故障（0xD：
    红灯闪、位置照读、指令一律不执行）。存盘、打印摘要、准备回放这几步都落在这个
    空档里，不喂帧的话，等回放开始时电机已经哑了。回放结束之后同理。

    目标只在一个**相位边界**上重新定（``hold_at``），不在每拍读一次——每拍拿当次
    读数现造目标的话，一次读数冻结就会变成一条指向伪值的新指令，那是阶跃。没读到
    过位置就**不发**：拿缓存里的 0.0 当目标是发一条指向别处的指令，比少发一帧危险
    得多。
    """

    def __init__(self, gripper):
        self._gripper = gripper
        self._last = 0.0
        self.target_rad = None
        self.frames = 0

    def hold_at(self, state):
        """把目标重新定到这次实测的位置上；读不到就不动（返回 ``False``）。"""
        if state is None:
            return False
        self.target_rad = float(state.position_rad)
        return True

    def tick(self, now):
        """到点了就发一帧保持帧；返回这一拍发没发。"""
        if self.target_rad is None or now - self._last < FRAME_DT:
            return False
        self._last = now
        cfg = self._gripper.config
        self._gripper.send_mit_frame(q=self.target_rad, kp=cfg.kp, kd=cfg.kd)
        self.frames += 1
        return True


def feed(keeper):
    """喂一帧保持帧（没有 keeper、或还没读到过位置就什么都不做）。

    **只在没有录制/回放会话的时候调用**：那两种会话跑起来时 CAN 的收发归 SDK 的
    后台线程，本样例再往总线上插一脚就是两条流抢同一根总线——回放的轨迹会被打散。
    会话之间的空档才轮到它，见 :data:`MEASURED_COMM_LOSS_S`。
    """
    if keeper is not None:
        keeper.tick(time.monotonic())


def dry_run_openness(elapsed_s: float) -> float:
    """``--dry-run`` 那条「手拖」曲线在 ``elapsed_s`` 时刻的开度。

    一条余弦：从 :data:`DRY_RUN_HIGH` 滑到 :data:`DRY_RUN_LOW` 再滑回来，周期
    :data:`DRY_RUN_PERIOD_S`。选余弦而不是三角波，是因为它的导数连续——三角波的
    折点会被 100 Hz 采样记成一段看着像抖动的折线，而这条曲线里每一处弯都是真的。
    """
    middle = (DRY_RUN_HIGH + DRY_RUN_LOW) / 2.0
    half = (DRY_RUN_HIGH - DRY_RUN_LOW) / 2.0
    return middle + half * math.cos(2.0 * math.pi * elapsed_s / DRY_RUN_PERIOD_S)


def record_phase(gripper, sim, args):
    """``[3]`` 录制：进零重力、手拖、Enter/空格（或 ``--record N`` 秒）结束。

    结束录制的键**只**是 Enter / 空格：Esc / Q 在这里是「这段不要了，退出」，不会
    留下轨迹。录完也不回放——回放要另一次确认，见 :func:`wait_for_start`。

    ``--dry-run`` 时 ``gripper`` 就是 ``sim`` 本身（仿真当那台夹爪），录制器的
    ``zero_gravity`` 给 ``False``：仿真里推不动手指，这条路径是主循环脚本驱动。

    Returns:
        录到的轨迹；这次录制是空的（一个样本都没有）、或者中途按了退出键时返回
        ``None``。
    """
    dry_run = gripper is sim
    print("\n[3] 录制：手拖一遍")
    if dry_run:
        print("   [dry-run] 仿真自己当那台夹爪：主循环按脚本曲线摆位，"
              "后台录制线程照常采样")
        print(f"   [dry-run] 曲线是 {DRY_RUN_PERIOD_S:g} s 一条余弦，"
              f"开度 {DRY_RUN_HIGH * 100:.0f}% → {DRY_RUN_LOW * 100:.0f}% → "
              f"{DRY_RUN_HIGH * 100:.0f}%")
    else:
        print("   [真机] 进零重力——两个手指的力被撤掉，可以直接用手推动")
    print(f"   [后台] 按 {RATE_HZ:g} Hz 采样"
          + (f"；{args.record:g} s 后自动停" if args.record > 0 else ""))
    print("   按 Enter / 空格 结束录制（录完还要确认一次才回放）；"
          "按 Esc / Q 放弃这次录制并退出")
    if dry_run:
        # zero_gravity=False：仿真里没有「松开的电机」，手指也不会被别人推动，
        # 唯一的驱动方是下面这个循环。录制器因此只读状态，不抢控制权。
        gripper.record_start(rate_hz=RATE_HZ, zero_gravity=False)
    else:
        # zero_gravity=True 时**录制器自己**在流零力矩帧，本样例这一路一个运动指令都
        # 不能发（SDK 原话：Do not drive the gripper from the caller while that runs）。
        gripper.record_start(rate_hz=RATE_HZ, zero_gravity=True)
    state = None
    fraction = None
    started = time.monotonic()
    last_print = 0.0
    abandoned = False
    while sim.connected():
        keys = sim.keyboard_events()
        if pressed(keys, CONFIRM_KEYS):
            print("\n   收到确认键：录制结束")
            break
        if pressed(keys, QUIT_KEYS):
            print("\n   收到退出键：这次录制作废")
            abandoned = True
            break
        now = time.monotonic()
        elapsed = now - started
        if args.record > 0 and elapsed >= args.record:
            print(f"\n   录满 {args.record:g} s")
            break
        if dry_run:
            # 摆到这条曲线上，然后让物理走一步：录制线程每一拍读到的 frac_open()
            # 就是这里写进去的值。
            fraction = dry_run_openness(elapsed)
            sim.set_frac_open(fraction)
        else:
            state = read_state(gripper)
            fraction = mirror(gripper, sim, state)
        samples = int(gripper.trajectory_status().get("samples") or 0)
        sim.status_text([
            f"录制中 · 已采 {samples} 个样本   "
            + ("未读到状态帧（窗口停在最后读数）" if fraction is None else
               f"开度 {fraction * 100:5.1f}%   "
               f"开口 {fraction_to_gap_mm(fraction):5.2f} mm")
        ])
        if now - last_print >= PRINT_DT:
            last_print = now
            if fraction is None:
                print("  [真机] 读不到状态帧：窗口停在最后一次读数上。"
                      "录制还在继续——它在 SDK 的后台线程里。")
            else:
                print("  " + status_line(
                    "仿真" if dry_run else "真机", fraction=fraction,
                    aperture_mm=fraction_to_gap_mm(fraction),
                    force_n=None if state is None else state.force_n,
                    moving=None if state is None else bool(state.is_moving),
                ) + f" · 已采 {samples} 个样本")
        if not sim.pump():
            break

    # record_stop() 内部会补一帧 exit_zero_gravity() 把电机恢复成正常闭环，然后就
    # 没人喂帧了——所以它一返回，调用方就得马上把保持帧接上（见 main 里的 feed 和
    # HoldKeeper：这是那个 0.9 s 预算里唯一要防的空档）。
    try:
        trajectory = gripper.record_stop()
    except Exception as exc:     # TrajectoryEmptyError / TrajectoryRecordingError
        print(f"\n   这次没录到东西：{exc}")
        return None
    if abandoned:
        print("   按了退出键，这段不保存、也不回放")
        return None
    print(f"\n   录到 {len(trajectory)} 个样本 · {trajectory.duration:.2f} s")
    return trajectory


def save_phase(trajectory):
    """``[4]`` 存盘：写进 SDK 的轨迹目录（``~/.litegrip/trajectories``）。

    文件名带上录制时刻，免得覆盖上一次录的；也**不写进本仓库**——``.lgt`` 是从这台
    机器上量出来的数据，不是源码。

    Returns:
        存下来的名字（不带目录），下次 ``--play`` 就用它。
    """
    name = f"{DEFAULT_NAME}-{time.strftime('%Y%m%d-%H%M%S')}"
    path = trajectory.save(name)
    print(f"\n[4] 保存轨迹\n   {path}")
    print(f"   下次回放：--play {name}")
    return name


def wait_for_start(sim, keeper):
    """录制与回放之间的闸门：等到一个明确的「开始回放」才开始，返回等没等到。

    录完直接接着放，是**真机在这一跑里第一次自己动**，而这时候操作员的手多半还在
    手指上、眼睛还在夹爪那边。所以这里要一个明确动作：按 Enter / 空格。Esc / Q
    表示「先不放，退出」——轨迹已经存下来了，随时可以用 ``--play`` 再放。

    等待期间照旧喂保持帧：这时候电机还使能着，静默约 :data:`MEASURED_COMM_LOSS_S`
    就锁 0xD，而「等你确认」正好是一段没人喂帧的时间。

    ``--headless`` 时没有窗口、也没人按得了，直接放行——否则会一直等下去。
    """
    if not sim.gui:
        print("\n   录制结束。--headless 没有窗口，直接回放")
        return True
    print("\n   录制结束。回放会把真机动起来，所以不自己开始：")
    print("   按 Enter / 空格 → 开始回放；按 Esc / Q → 先不放，退出")
    sim.status_text(["录完了 · 按 Enter / 空格开始回放"])
    # 先丢掉结束录制那一拍的按键：同一个 Enter / 空格不该一次算两回——按一下结束
    # 录制，紧接着又被当成「开始回放」。
    sim.keyboard_events()
    while sim.connected():
        keys = sim.keyboard_events()
        if pressed(keys, CONFIRM_KEYS):
            print("\n   收到开始信号：回放")
            return True
        if pressed(keys, QUIT_KEYS):
            print("\n   收到退出键：先不放")
            # 窗口里那行字要跟着改，不然画面还停在「等你确认」。
            sim.status_text(["没有确认，这次不回放 · 按 Esc / Q 退出"])
            return False
        feed(keeper)
        if not sim.pump():
            break
    return False


def load_trajectory(name):
    """读一段轨迹（纯文件 I/O，不碰 CAN）；读不出来就带着原因退出。

    用本仓的 :class:`~litegrip_mujoco.Trajectory`，格式与 SDK 逐字节相同——所以
    ``--play`` 这一步**不需要** SDK（也不需要 CAN），一段 ``.lgt`` 加本地时钟就够。
    """
    from litegrip_mujoco import Trajectory, trajectory_dir

    try:
        return Trajectory.load(name)
    except FileNotFoundError as exc:
        raise SystemExit(
            f"找不到这段轨迹：{name}\n"
            f"   不带目录的名字会在 {trajectory_dir()} 下找，"
            "并自动补 .lgt 后缀。\n"
            f"   （原始错误：{exc}）"
        ) from exc
    except Exception as exc:     # TrajectoryFormatError 等
        raise SystemExit(f"这段轨迹读不出来：{name}\n   （{exc}）") from exc


def describe(trajectory, source):
    """一行说明这段轨迹是什么、有多长。"""
    print(f"   {source} · {len(trajectory)} 个样本 · {trajectory.duration:.2f} s"
          + (f" · 开度 {trajectory.samples[0].openness:.3f} → "
             f"{trajectory.samples[-1].openness:.3f}" if len(trajectory) else ""))
    print(f"   记录时的夹爪：can_id={trajectory.can_id:#04x}"
          + (f" · mount={trajectory.mount}" if trajectory.mount else "")
          + "（回放按本机标定换算，不用它）")


def play_online(gripper, sim, trajectory, args, keeper):
    """``[5]`` 回放：同一段轨迹同时喂真机和仿真。

    用 ``play_start``（后台线程）而不是 ``play``（阻塞到放完）：阻塞版把主线程占满，
    仿真窗口就没人刷了，看不到「同时回放」。主循环因此只做两件事——读真机位置镜像
    到仿真、看回放放完没有；回放期间**不发**自己的保持帧，总线归回放线程。
    """
    print("\n[5] 回放：仿真与夹爪同时")
    print(f"   [真机] 先对齐到第一个样本，再按 {args.speed:g} 倍速回放")
    gripper.play_start(trajectory, speed=args.speed, align=True)
    last_print = 0.0
    try:
        while sim.connected():
            if pressed(sim.keyboard_events(), QUIT_KEYS):
                print("\n   收到退出键")
                break
            now = time.monotonic()
            state = read_state(gripper)
            fraction = mirror(gripper, sim, state)
            status = gripper.trajectory_status()
            done = not status.get("active")
            sim.status_text([
                f"回放中 · {status.get('frames', 0)} 帧 · "
                f"{status.get('loop_hz', 0):.0f} Hz   "
                + ("未读到状态帧（窗口停在最后读数）" if fraction is None else
                   f"开度 {fraction * 100:5.1f}%   "
                   f"开口 {fraction_to_gap_mm(fraction):5.2f} mm")
            ])
            if now - last_print >= PRINT_DT:
                last_print = now
                if fraction is None:
                    print("  [真机] 读不到状态帧：窗口停在最后一次读数上。"
                          "回放还在继续——它在 SDK 的后台线程里。")
                else:
                    print("  " + status_line(
                        "真机", fraction=fraction,
                        aperture_mm=fraction_to_gap_mm(fraction),
                        force_n=state.force_n, moving=bool(state.is_moving),
                    ) + f" · {status.get('frames', 0)} 帧")
            if done:
                print(f"\n   放完了（{status.get('frames', 0)} 帧 · 实测循环 "
                      f"{status.get('loop_hz', 0):.1f} Hz）")
                break
            if not sim.pump():
                break
    finally:
        # play_stop() 会把夹爪留在最后一个目标位上——那一帧之后又没人喂了，所以
        # 紧接着就得把保持帧接上，而且目标要重新取**现在**的位置（还用回放开始前
        # 那个目标的话，本样例就是在下一条没人要求的运动指令：把手指从轨迹终点
        # 拽回起点）。
        gripper.play_stop()
        keeper.hold_at(read_state(gripper))
        feed(keeper)
    if keeper.target_rad is not None:
        print(f"   [真机] 锁在当前位置 {keeper.target_rad:+.4f} rad"
              "（零前馈，不命令运动；手指不会自己回起点）")


def play_offline(sim, trajectory, args):
    """``[5]`` 回放：只灌仿真（``--play`` 的默认路径，也是 ``--dry-run`` 的收尾）。

    不连真机、不发送任何帧，所以这条路随时可跑：一段 ``.lgt`` 加上本地时钟就够了，
    开度直接问 ``Trajectory.openness_at(t)``（样本之间线性插值，两端夹住）。
    """
    print("\n[5] 回放：只灌仿真")
    print("   （--play 默认不碰真机：不连、不使能、一帧都不发。"
          "要在真机上也放，加 --real）")
    duration = trajectory.duration / args.speed
    started = time.monotonic()
    last_print = 0.0
    while sim.connected():
        if pressed(sim.keyboard_events(), QUIT_KEYS):
            print("\n   收到退出键")
            break
        now = time.monotonic()
        t = min((now - started) * args.speed, trajectory.duration)
        fraction = trajectory.openness_at(t)
        sim.set_frac_open(fraction)
        sim.status_text([
            f"回放中 · {t:5.2f} / {trajectory.duration:.2f} s   "
            f"开度 {fraction * 100:5.1f}%   "
            f"开口 {fraction_to_gap_mm(fraction):5.2f} mm"
        ])
        if now - last_print >= PRINT_DT:
            last_print = now
            print("  " + status_line(
                "仿真", fraction=fraction,
                aperture_mm=fraction_to_gap_mm(fraction),
            ) + f" · {t:5.2f} / {trajectory.duration:.2f} s")
        if now - started >= duration:
            break
        if not sim.pump():
            break
    print(f"\n   放完了（{duration:.2f} s，{args.speed:g} 倍速）")


def main():
    args = parse_args()
    # --dry-run 的默认录制时长：不给 --record 就录满一条余弦周期多一点。
    if args.dry_run and args.record <= 0:
        args.record = DRY_RUN_PERIOD_S + 0.5

    print("样例 03 · 轨迹录制与回放")
    if args.play:
        print(f"   --play {args.play}：只灌仿真"
              + ("，另外下发给真机（两个手指会真实运动）" if args.real else
                 "，不连真机"))
    elif args.dry_run:
        print("   --dry-run：不碰 CAN，仿真自己当那台夹爪（真机那一段未经验证）")
    else:
        print(SAFETY_BANNER)

    litegrip = None
    if not args.dry_run:
        # 三个真机样例用的是同一份 SDK（nexform-tech/litegrip-python）。缺接口就
        # 别连——宁可现在停，也别在循环里才发现。--dry-run 与 --play 的纯仿真路径
        # 都不需要它；--real 需要，所以下面还要再判一次。
        litegrip = import_litegrip()
        check_sdk_api(litegrip)

    print("\n[1] 载入仿真模型")
    sim = make_sim(args.headless)
    sim.enable()      # --dry-run 里录制器要读一个已使能的夹爪
    sim.focus_camera()
    print(f"   {sim.model_path}")

    need_hardware = ((not args.play) or args.real) and not args.dry_run
    gripper = None
    keeper = None
    saved = None
    trajectory = None
    try:
        print("\n[2] 连接真机")
        if need_hardware:
            gripper = open_real_gripper(args, enable=True)
            keeper = HoldKeeper(gripper)
            # 使能之后就得有人喂帧：先读一帧实测位置把保持帧的目标定下来，读不到
            # 就干脆不发（见 HoldKeeper）。
            keeper.hold_at(read_state(gripper))
            feed(keeper)
        elif args.dry_run:
            print("   跳过：--dry-run 不碰 CAN，仿真自己当那台夹爪")
        else:
            print("   跳过：--play 默认不碰真机（要真机也一起放就加 --real）")

        replay_now = True
        if args.play:
            print("\n[3] 录制")
            print("   跳过：--play 放的是已有的轨迹")
            print("\n[4] 读取轨迹")
            trajectory = load_trajectory(args.play)
            describe(trajectory, args.play)
            if not len(trajectory):
                print("   这段轨迹一个样本都没有，没有东西可放（录制那次是空的？）")
                return 1
        else:
            trajectory = record_phase(sim if args.dry_run else gripper, sim, args)
            if trajectory is None:
                return 1
            # record_stop() 之后到 play_start() 之前是这次运行里唯一的空档：中间
            # 只做「读一帧位置 → 写文件 → 打印」，几十毫秒，远在 0.9 s 的预算之内。
            # 两头顶上保持帧，是让它不依赖「这几步一定不慢」这个假设——等确认
            # 的那段时间也算在里面，所以 wait_for_start 自己也喂。
            if keeper is not None:
                keeper.hold_at(read_state(gripper))
                feed(keeper)
            saved = save_phase(trajectory)
            replay_now = wait_for_start(sim, keeper)

        if gripper is None:
            play_offline(sim, trajectory, args)
        elif not replay_now:
            print("\n[5] 回放")
            print(f"   跳过：没有确认开始。轨迹存下来了，随时可以 "
                  f"--play {saved} 放")
        else:
            feed(keeper)
            play_online(gripper, sim, trajectory, args, keeper)

        if keeper is not None:
            print(f"   空档期发了 {keeper.frames} 帧保持帧"
                  "（使能态静默就锁 0xD：本仓实测约 "
                  f"{MEASURED_COMM_LOSS_S:g} s，SDK 文档写约 0.1 s）")
            print("   按 Esc / Q 退出（退出会失能：手指变松、可能因自重滑动）")
            while sim.connected():
                if pressed(sim.keyboard_events(), QUIT_KEYS):
                    print("\n   收到退出键")
                    break
                feed(keeper)
                if not sim.pump():
                    break
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        if gripper is not None:
            # 退出前**失能**（0xFD），而不是最后补一帧就不管：使能态的电机静默约
            # 0.9 s 就锁 0xD，失能则不需要任何帧。
            try:
                gripper.disable()
            finally:
                gripper.disconnect()
            print("\n[真机] 已失能并关闭（手指变松、可能因自重滑动）")
        elif need_hardware:
            print("\n[真机] 未连接")
        sim.disconnect()
        print("[仿真] 已关闭")

    if trajectory is None:       # 中途 Ctrl-C，还没读到/录到轨迹
        print("完成（没有轨迹）。反向的（仿真控真机）见 "
              "examples/05_dual_control.py")
        return 1
    print(f"完成（{len(trajectory)} 个样本 · {trajectory.duration:.2f} s"
          + (f" · 存为 {saved}" if saved else "") + "）。"
          "反向的（仿真控真机）见 examples/05_dual_control.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
