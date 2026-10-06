#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 05 · 遥操作模式 — 按键盘，真机跟着走

方向是 **仿真 → 真机**：键盘是手里的操纵杆，真机跟着它动。

流程（全在 MuJoCo 窗口里操作）：

  1. 启动后真机**不动**：本样例按 200 Hz 发「锁在实测位置」的保持帧，同时把真机
     的实测位置实时镜像进窗口——窗口显示的是真机**现在在哪**，不是你想让它去哪。
  2. 按 **←/→** 改「目标开度」——真机才开始走。每按一下走一档（默认 5%），按住
     不放就是连着按（GLFW 的自动重复会一档一档地来）。真机的目标是这个值，但位置
     目标每帧最多前进「速度」允许的距离，收尾还会按 RAMP_DOWN_S 减速（见
     KeyboardDrive），所以连按到 100% 也是按速度限定的速度走完全程，不会跟着按键
     跳，也不会在到达时被速度前馈的台阶顶回来。
  3. 按 **↓/↑** 改「速度」——位置目标的最大速率，100% 就是手指额定
     RATED_SPEED_MM_S。随时可调：按快键真机也不会跟着快。
  4. 按 **-/+** 改「夹持力」——手指到位后用这个力矩顶住（夹住工件时的推力）。
  5. 按 **Space** 停下（目标留在原地，真机锁在当前位置）；按 **H** 回到全开。
  6. **Esc / Q** 退出，退出前会**明确失能**（0xFD）：电机不再出力，手指会松、
     夹着的工件会掉，但不会在电机上留下通信超时故障（原因见下）。

为什么是键盘而不是滑条：MuJoCo 的被动查看器只给一个键盘回调
（``launch_passive(key_callback=...)``），没有鼠标回调，也没有可加的控件——
pybullet 那版的三个 ``addUserDebugParameter`` 滑条在这里建不出来。所以目标是
**离散步进**（一次按键 = 一档），不是连续拖动。手感不同，但限速、收尾减速、
只在收拢方向加力这些语义与 pybullet 那版一致。

前提:
  1. 真机接在 CAN 总线上（默认 can0，用 --channel 换）
  2. 装了本仓库要的 litegrip SDK（没有发布到 PyPI，从源码装；三个真机样例用的是
     同一份，nexform-tech/litegrip-python）:
       pip install -e /path/to/litegrip-python
     或 export LITEGRIP_SDK_DIR=/path/to/litegrip-python/src
     或把 litegrip-python 仓库克隆到本仓库的同级目录
  3. 一份可用的标定。标定文件由上位机标定后保存得到：
       litegrip-studio / litegrip-console，或 SDK 自带的 tools/gui/litegrip_gui.py
     标定的角度和毫米刻度是一台机器一个值，拿别人的算目标角，轻则夹不住、重则一条
     指令撞限位。所以优先用 ``--calib`` 指**这台夹爪**自己那份；不给就用 SDK 包里
     那份出厂标定（台架夹具的实测参数），出厂文件也读不出来才会在终端里列出候选让
     你选；选不出来（非交互、没有候选）直接退出。
     这条路径**要求有窗口**：目标由键盘给，没有窗口就没法操作。

注意：会驱动真机！**没有「确认」这一步**：按一下方向键就发指令，所以别在真机动
的时候乱按。第一次跑务必先 dry-run：

    python3 examples/05_dual_control.py --dry-run    # 只开窗口，绝不碰 CAN

没有显示（或想跑 CI）的时候：

    python3 examples/05_dual_control.py --dry-run --headless --duration 12

没有窗口就没有键盘，这一条路上目标改由一段脚本曲线给——它顶替的是「那只手」，
只在 --dry-run 下存在。

真机「能读不能控」怎么办（位置读得到、发指令不动、驱动板红灯闪）：

    python3 examples/05_dual_control.py --status             # 只连、只读，不发运动指令
    python3 examples/05_dual_control.py --status --clear-fault   # 清掉锁死的故障

红灯闪 + 位置照读 + 指令无效，是电机进了**锁死**的故障态，而 --status 打的那个
错误码就是它的名字（0xD = 通信丢失、0x9 = 欠压、0xA = 过流、0xB/0xC = 过温……）。
两类原因最常见：

  * **没人喂帧**：使能态的电机静默约 MEASURED_COMM_LOSS_S 就报 0xD。所以空闲也得
    持续发帧（见 IdleKeeper）——本样例空闲时发的是锁位帧。
  * **一条接不住的指令**：MIT 的 kp 是位置刚度，一整段行程的阶跃会让电机在第一帧
    就被要求输出 ``kp × 1.845 rad`` 那么大的力矩。SDK 默认 kp=100 Nm/rad，那是
    185 Nm，而额定只有 ~10 Nm；本机标定现在是 5.0，同样的阶跃约 9 Nm。但 kp 是
    标定文件里的一项、随时可能被改回去，所以本样例限制的是**位置目标每帧走多远**
    （见 KeyboardDrive），和 kp 取多少无关，也和按键按得多快无关。

为什么要自己发帧：SDK 的 move_to()/goto_rad() 内部是 control_mit_stream()，
它自己 sleep 5 ms 循环、不让出控制权，MuJoCo 窗口会卡住、也读不到按键。所以这里
用 SDK 公开的 send_mit_frame() + poll() 自己组循环——这正是它们被公开出来的用途
（自定义控制循环，自己管时序）。

运行:
  python3 examples/05_dual_control.py --calib /path/to/这台夹爪的标定.json
  python3 examples/05_dual_control.py                # 不给就用 SDK 出厂标定
  python3 examples/05_dual_control.py --force 20 --speed 40
  python3 examples/05_dual_control.py --channel can1        # 换 CAN 口
  python3 examples/05_dual_control.py --status              # 只连、只读，不发运动指令
  python3 examples/05_dual_control.py --dry-run --headless --duration 12
"""
import argparse
import math
import sys
import time

from _common import (  # noqa: I001  (必须先于 litegrip_mujoco)
    FRESH_WAIT_S,
    MAX_GRIP_FORCE_N,
    RATED_SPEED_MM_S,
    SAFETY_BANNER,
    STATUS_WAIT_S,
    add_common_args,
    add_hardware_args,
    fraction_to_gap_mm,
    fraction_to_target_rad,
    fresh_state,
    import_litegrip,
    make_sim,
    open_real_gripper,
    rad_to_fraction,
    status_line,
)

from litegrip_mujoco import QUIT_KEYS, TELEOP_KEYS, pressed

# 发 MIT 帧的频率 [Hz]。达妙电机要持续收帧才保持力矩；SDK 自己用的也是 200 Hz
# （5 ms 一帧），这里对齐。
FRAME_HZ = 200.0
FRAME_DT = 1.0 / FRAME_HZ

# 终端状态刷新的最小间隔 [s]（窗口里是每帧刷，终端刷太快没法看）。
PRINT_DT = 0.5

# 一次按键走多远。开度是归一化百分比（一档 5%），速度是百分比（一档 10%），
# 夹持力是牛顿（一档 5 N）。
APERTURE_STEP = 5.0
SPEED_STEP = 10.0
FORCE_STEP_N = 5.0

# 「目标开度变了吗」的阈值 [%]。目标只由按键改，每档 5%，所以这个阈值只是防浮点
# 噪声；也正因为按键就是唯一的输入，**按一下就是一档**，不会有滑条那样的连续量。
STEP_EPS = 1e-9

DEFAULT_FORCE_N = 10.0
DEFAULT_SPEED_PCT = 100.0

# 收尾减速的时间尺度 [s]：速度从满速降到 0 大约用这么久。
#
# 为什么必须有这一段（实测现象：「先到位，然后回弹一下」，张开时尤其明显）。
#
# MIT 帧里的 ``dq`` 是**目标速度**，电机自己算
# ``tau = kp×(q目标 − q实际) + kd×(dq目标 − dq实际) + 前馈``。不减速的话，位置
# 目标到达的那一帧 ``dq`` 会从满速直接变成 0，阻尼项 ``kd×(dq目标 − dq实际)``
# 于是整个反向——等于给电机一个 ``kd × v`` 的**力矩台阶**，方向与刚才的运动相反。
# 本机标定 ``kp=5.0 kd=2.0``，满速 ``85 mm/s ÷ 61.01 = 1.39 rad/s``：
#
#     kd × v = 2.0 × 1.39 ≈ 2.8 Nm
#
# 而位置项要拿误差补出同样的力矩，需要 ``2.8 ÷ 5.0 = 0.56 rad``——那是整个行程
# （1.41 rad）的 40%。也就是说位置环根本接不住这个台阶，电机只能靠机构自己往回
# 让一点、把误差重新建立起来：看到的就是「先到位、再回弹」。它是**速度**的函数，
# 不是距离的函数，所以行程越短越不明显、跑得越快越明显。
#
# 按 ``v = √(2·a·剩余距离)`` 收尾之后，``dq`` 在最后几帧连续降到 0，那个台阶就
# 没有了。代价是到位晚那么几帧（默认 0.15 s 的量级）。
#
# 注意这不是 05 独有的毛病：SDK 自己的 ``_move_at_speed_rad`` 是同样的一刀切
# （``dq = direction*speed if i < steps else 0.0``），直接调
# ``gripper.move_at_speed()`` 收尾也会这样。
RAMP_DOWN_S = 0.15

# 保持段的默认时长 [s]：到位后加力顶住的时间。
DEFAULT_HOLD_S = 0.5

# 实测：**使能态**的电机静默约这么久就闩锁 0xD 通信丢失故障（SDK 在真机上量到
# 的）。
#
# 别拿 ``TIMEOUT`` 寄存器（RID 9）当依据：它读到过 8000 ms，也读到过 0＝当前不
# 生效，和实测的 ~0.9 s 都对不上，SDK 自己把这条标成「待查」。行为按实测走——空闲
# 也持续发帧。之前照寄存器那个 8000 ms 推出来的结论是错的。
MEASURED_COMM_LOSS_S = 0.9

# 空闲时也必须持续发帧 [Hz]，和运动时同频。
#
# 这不是可选的优化：使能态的电机静默约 MEASURED_COMM_LOSS_S 就锁进通信丢失故障
# ——红灯闪、位置照读、指令一律不执行。空闲不发帧 = 在窗口里多看一眼就把真机看哑
# 了。SDK 的 ``control_mit_stream`` 和 LiteGrip 控制台都是持续发帧的，正是为此。
IDLE_HZ = FRAME_HZ

# 等一帧新状态帧时，每次「喂一帧 + 收一拍」的时间片 [s]。
#
# 取一帧的时长（5 ms）：这样每一片里都恰有一次收帧机会，等一帧的正常代价就是
# 一到两片，而读不到时最多浪费 FRESH_WAIT_S 而不是更多。见 wait_fresh_while_feeding。
FEED_SLICE_S = FRAME_DT

# ``--dry-run --headless`` 那条路上顶替「那只手」的脚本：``(目标开度, 到位后停留 s)``。
#
# 每段先花 SCRIPT_MOVE_S 线性走到目标，再原地停 dwell 秒。停留段不是装饰：到位之后
# 才看得到「[#N] 停住 → 交回保活」那一步，而保活帧恰恰是真机上最容易出事的地方
# （停发就锁 0xD）。没有停留，驱动永远在跟随中，这条路径就白跑了。
#
# 有窗口时不用这个脚本：那时键盘就是那只手。
SCRIPT_STEPS = ((1.00, 1.0), (0.25, 1.0), (0.75, 1.0), (0.00, 1.5))
SCRIPT_MOVE_S = 1.5
_SCRIPT_SPANS = tuple(SCRIPT_MOVE_S + dwell for _target, dwell in SCRIPT_STEPS)
_SCRIPT_PERIOD_S = sum(_SCRIPT_SPANS)


def parse_args():
    ap = argparse.ArgumentParser(
        description="样例 05 · 遥操作模式：按键盘，真机跟着走",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_common_args(ap)
    add_hardware_args(ap)
    ap.add_argument("--dry-run", action="store_true",
                    help="不连真机、不下发任何指令（只开窗口看流程）")
    ap.add_argument("--force", type=float, default=DEFAULT_FORCE_N,
                    help=f"夹持力前馈 [N]（默认 {DEFAULT_FORCE_N:g}，"
                         f"上限 {MAX_GRIP_FORCE_N:g}）")
    ap.add_argument("--speed", type=float, default=DEFAULT_SPEED_PCT,
                    help=f"位置目标的最大速率 [%%]，100%% = 手指额定 "
                         f"{RATED_SPEED_MM_S:g} mm/s（默认 "
                         f"{DEFAULT_SPEED_PCT:g}，超过 100 按 100 处理）")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="跑多少秒后自动退出（默认 0 = 一直跑到 Esc/Q 或关窗口）")
    ap.add_argument("--noise", action="store_true",
                    help="配合 --dry-run：给替身的读数加传感器噪声")
    ap.add_argument("--status", action="store_true",
                    help="只连接、只读状态并诊断故障，**不打开窗口、不下发任何运动"
                         "指令**；真机「能读不能控」时先用它看看")
    ap.add_argument("--clear-fault", action="store_true",
                    help="配合 --status：清掉锁死的故障并重新使能（只发零力矩帧，"
                         "但清除流程含 disable→enable，电机会短暂失力）")
    args = ap.parse_args()
    if args.clear_fault and not args.status:
        ap.error("--clear-fault 要配合 --status 用（它只清故障，不驱动）")
    if args.dry_run and args.status:
        ap.error("--status 和 --dry-run 是两件事：前者要连真机看状态，后者不碰真机")
    return args


class KeyboardDrive:
    """跟着按键走的**限速**位置跟随器：一帧最多走 ``速度 × FRAME_DT``。

    与 pybullet 那版跟着滑条走的驱动是同一套语义，只是目标由按键改（
    :meth:`retarget` 的调用方从滑条读数换成了键盘事件）。

    为什么必须限速，不能把目标直接发：MIT 的位置增益是标定文件里的 ``kp``：一帧
    要的力矩就是 ``kp × 目标与实测的差``。SDK 默认 kp=100 Nm/rad，一整段行程
    （1.845 rad）的阶跃会在第一帧要求 ``100 × 1.845 ≈ 185 Nm``，而 DM4310 额定
    只有 ~10 Nm——电流瞬间拉满，电机进欠压/过流保护并**锁死**，红灯闪烁，之后就不
    再执行任何指令（位置照常回报，所以看起来是「能读、不能控」）。本机标定现在把
    kp 调到 5.0，同样的阶跃约 9 Nm、落在额定之内，但 kp 是标定里的一项、随时可能
    被改回去，所以不押它：限速限制的是**位置目标每帧的增量**，与 kp 取多少无关。

    这也正是「按一下就动」还能安全的原因。``q_want`` 是按键指的位置，``q`` 是真正
    被命令的位置，两者之间隔着这道限速：连按到 100% 也只是让它按速度限定的速度走
    完全程，而不是跟着按键跳。

    一帧的 ``(q, dq, tau)``：

      ``q``    受限地朝 ``q_want`` 挪一步（挪到了就正好停在 ``q_want``）
      ``dq``   这一步的速度，作为前馈帮电机跟上（不额外使劲）。**收尾会减速**：
               快到目标时按 ``√(2·a·剩余距离)`` 走，见 RAMP_DOWN_S
      ``tau``  只有**到位之后**才是夹持力前馈，运动中恒为 0——一边走一边顶，会在
               工件还没夹住之前就把力顶上去；这和 SDK ``set_force()`` 只在停住
               之后加力是同一个道理。

    ``q`` 只在**帧真的发出去之后**才推进（见 ``send()``）：发丢的帧不能算数，
    否则下一帧就得一次走两步，把这道限速自己绕过去。
    """

    def __init__(self, start_rad, speed_rad_s, kp, kd):
        self.q = float(start_rad)         # 已经命令到的位置
        self.q_want = float(start_rad)    # 按键现在指的位置
        self.speed_rad_s = max(0.0, float(speed_rad_s))
        self.kp = kp
        self.kd = kd
        # 最后一次改目标是不是**收拢**方向（见 ``retarget()``）。张开时加力会顶住
        # 电机不让它张开，所以只有收拢才允许加力。
        self.closing = False
        self.frames = 0
        self.dropped = 0                  # send_mit_frame 返回 False 的帧数
        self.last_move = time.monotonic()  # 最后一次真的挪动的时间

    @property
    def max_step_rad(self):
        """一帧允许走的最大角度。**这是整个样例的安全边界。**"""
        return self.speed_rad_s * FRAME_DT

    def arrived(self):
        """已经到目标了吗。"""
        return abs(self.q_want - self.q) <= 1e-12

    def retarget(self, q_want):
        """目标又变了：换一个新目标。

        下一次 ``send()`` 仍然只走一个 ``max_step_rad``——目标换得再远，一帧的
        增量不变，所以「连着按」永远不会变成阶跃。
        """
        self.q_want = float(q_want)
        self.closing = self.q_want > self.q

    def frame(self, tau_nm=0.0):
        """这一帧会发出去的 ``(q, dq, tau)``。**纯函数，不改状态。**

        速度上限是 ``max_step_rad``，但快到位时还会再低一档：剩余距离不够保持当前
        速度时按 ``v = √(2·a·剩余距离)`` 走（``a = 满速 / RAMP_DOWN_S``），于是
        ``dq`` 在到达目标的过程中连续降到 0，而不是在最后一帧被一步切成 0。为什么
        必须这样，见 RAMP_DOWN_S。

        ``v`` 恒 ≤ 满速，所以 ``max_step_rad`` 仍然是硬边界；减速只让收尾多花几帧，
        不改变任何一帧能走多远。
        """
        delta = self.q_want - self.q
        speed = self.speed_rad_s
        if delta != 0.0:
            decel = self.speed_rad_s / RAMP_DOWN_S
            speed = min(speed, math.sqrt(2.0 * decel * abs(delta)))
        step = speed * FRAME_DT
        moved = delta if abs(delta) <= step else math.copysign(step, delta)
        q = self.q + moved
        # 到位判据用**这一帧的 q**，不是提交后的：最后一步收在 q_want 上，那一帧
        # 就该带上夹持力，而不是等到下一帧才开始顶。
        return q, moved / FRAME_DT, (tau_nm if abs(self.q_want - q) <= 1e-12 else 0.0)

    def advance(self, now, tau_nm=0.0):
        """推进一帧并提交（**不发帧**）。dry-run 用它——那里没有真机可发。

        帧数照计：dry-run 报的是「本来会发出去多少帧」，和真机跑同一套节奏才有得比。
        """
        q, dq, tau = self.frame(tau_nm)
        self._commit(q, now)
        self.frames += 1
        return q, dq, tau

    def due(self, now, last_sent):
        """该发下一帧了吗（按 FRAME_HZ 限速）。"""
        return now - last_sent >= FRAME_DT

    def done(self, now, hold_s):
        """到位之后又 ``hold_s`` 没动过——这条驱动可以交回保活了。

        判的是「真的没动」，不是「目标没变」：还在跟随的途中，不管目标有没有在动，
        都不算结束。
        """
        return self.arrived() and (now - self.last_move) >= hold_s

    def send(self, gripper, now, tau_nm=0.0):
        """发一帧，返回是否发出去了。

        ``send_mit_frame`` 在「没连接」或「没使能」时返回 ``False``。这个返回值
        以前被丢掉了，于是帧全都没发出去也照样打印「发完 N 帧」——排查「能读不能
        控」时这会把人带偏，所以它会自己数着。
        """
        q, dq, tau = self.frame(tau_nm)
        if not gripper.send_mit_frame(q=q, kp=self.kp, kd=self.kd, dq=dq, tau=tau):
            self.dropped += 1
            return False
        self._commit(q, now)
        self.frames += 1
        return True

    def _commit(self, q, now):
        if q != self.q:
            self.q = q
            self.last_move = now


def plan_speed(gripper, percent):
    """「速度 %」→ 位置目标的最大角速度 [rad/s]。

    100% 就是额定手指速度 RATED_SPEED_MM_S。超过 100% 不照做——按 100% 处理并
    说明；那是会拉保护的用法，而且窗口里也没有更快的东西可比。
    """
    pct = float(percent)
    if pct > 100.0:
        print(f"   速度 {pct:.0f}% 超过额定，已按 100%"
              f"（{RATED_SPEED_MM_S:g} mm/s）处理")
        pct = 100.0
    rad_to_mm = gripper.config.rad_to_mm or 61.012
    return RATED_SPEED_MM_S * max(0.0, pct) / 100.0 / rad_to_mm


def scripted_fraction(elapsed_s):
    """``--dry-run --headless`` 下顶替「那只手」的开度。

    第一段的起点就是第一个目标（1.0），所以开头那一段只是「原地停 1 s」，然后才
    开始动——启动瞬间不会先朝一个没人要的方向走一趟。之后按 :data:`SCRIPT_STEPS`
    循环：走一段、停一段。
    """
    t = elapsed_s % _SCRIPT_PERIOD_S
    previous = SCRIPT_STEPS[0][0]
    for (target, _dwell), span in zip(SCRIPT_STEPS, _SCRIPT_SPANS):
        if t < SCRIPT_MOVE_S:
            return previous + (target - previous) * (t / SCRIPT_MOVE_S)
        t -= SCRIPT_MOVE_S
        if t < span - SCRIPT_MOVE_S:
            return target
        t -= span - SCRIPT_MOVE_S
        previous = target
    return SCRIPT_STEPS[-1][0]     # pragma: no cover — 上面的取模保证走不到


# 没装 SDK（或者 SDK 比这颗错误码还老）时的兜底表，内容抄自
# ``litegrip.constants.ERROR_DESCRIPTIONS`` 的当前版本。0x8/0xD/0xE 是 SDK 后来
# 补上的；旧版会把它们报成「未知错误」，而本机最容易闩上的恰恰是 0xD，所以这里
# 必须自己认识它。
FALLBACK_ERRORS = {
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


def describe_code(error_code):
    """错误码 → 可读文本。**没装 SDK 也必须能说人话。**

    故障路径是最不该抛异常的地方：负责报故障的代码自己崩了，操作员就只剩一个回溯，
    看不到电机报的到底是哪一条。所以 ``describe_error`` 是懒导入且**带兜底**的。

    认得这个码就用 SDK 的说法（0xD 已经在它的 ``ERROR_DESCRIPTIONS`` 里了）；
    SDK 回「未知错误」＝它的表里没有这个码（那是 ``describe_error`` 唯一的兜底
    话术），这时才退回本表。反过来先查本表是不行的：两处描述会慢慢走偏。
    """
    try:
        from litegrip.constants import describe_error
    except ImportError:
        return FALLBACK_ERRORS.get(error_code, f"未知错误 (0x{error_code:X})")
    text = describe_error(error_code)
    if text.startswith("未知错误"):
        return FALLBACK_ERRORS.get(error_code, text)
    return text


def fault_of(state):
    """状态里有故障就返回可读描述，否则 ``None``。

    电机的故障（0xD 通信丢失、0x9 欠压、0xA 过流、0xB/0xC 过温）是**锁死**的：
    不报错也不动，位置照常回报。必须在循环里盯着，否则会一直对着一个已经不听话的
    电机发帧。
    """
    if not state.is_error:
        return None
    return f"{describe_code(state.error_code)} (0x{state.error_code:X})"


def hold_frame(gripper):
    """「锁在当前位置」的一帧：``(q, kp, kd, dq, tau)``；读不到就返回 ``None``。

    目标就是电机**现在**的位置、零速度、零前馈——命令出来的一瞬间误差为零，所以
    不会产生任何运动，只是让手指有刚度、并且把电机的通信超时计数器喂上。

    这是空闲时的保活帧，也是 ``LiteGrip.exit_zero_gravity`` 说的「锁在当前位」。
    用它而不是 ``kp=0``：``kp=0`` 会让手指变软，可能在自重下自己出溜。

    「现在的位置」必须真的**是现在**，所以走 ``fresh_state``（等到一帧新的状态帧）
    而不是 ``get_state(wait=False)`` 的缓存。缓存里没读到过位置时是 SDK 的初值
    ``0.0``——拿它当目标发出去，就是一条指向 0 rad 的**阶跃**指令，电机按标定里的
    ``kp``（SDK 默认 100 Nm/rad）去追那个根本不存在的误差。少发一帧不会让电机乱
    动，发错目标会，所以读不到就返回 ``None``，调用方负责不发。

    本函数**不会**去催电机开口。所以调用它的地方必须是**已使能**的电机——使能态的
    DM 电机自己会持续发状态帧，等一下就有；未使能的电机不发帧，这里只会一直返回
    ``None``。
    """
    state = fresh_state(gripper)
    if state is None:
        return None
    cfg = gripper.config
    return (state.position_rad, cfg.kp, cfg.kd, 0.0, 0.0)


class IdleKeeper:
    """空闲保活：没有指令在走的时候，照样按 IDLE_HZ 接着发帧。

    这不是可选的优化，是必须的。**使能态**的电机静默约 MEASURED_COMM_LOSS_S 就锁
    进通信丢失故障——位置照读、指令不执行、红灯闪；而且在窗口里多看一眼就够触发，
    **发再多帧也解不开**（得显式清故障）。别拿 ``TIMEOUT`` 寄存器推算这个时长，它
    和实测对不上，见 MEASURED_COMM_LOSS_S。

    发什么：优先「接着上一条指令的最后一帧发」——这样夹持力不会因为空闲而松掉；
    还没下发过指令时发一条锁位帧（目标 = 实测位置、零前馈），不命令任何运动。

    一条已经定下来的帧可以一直重发，**没读过位置就不该造新帧**。这两件事的区别就是
    安全与危险的分界：重发同一帧，电机的目标不动，最坏也只是它没跟上；而拿一个旧
    读数（或 ``0.0``）现造一帧，就是把「读数坏了」变成一条指向别处的阶跃指令。所以
    本类只在初始化时定一次目标，之后一直重发；那次读不到就不发，下一拍再试，读到了
    才开始。
    """

    def __init__(self, gripper, hz=IDLE_HZ):
        self.gripper = gripper
        self.frame = hold_frame(gripper)   # None = 还没读到可信位置
        # 记住的这一帧是不是收拢指令——只有收拢才允许带夹持力。锁位帧不含运动，
        # 所以是 False。
        self.closing = False
        self.interval = 1.0 / hz
        self.last_sent = float("-inf")
        self.frames = 0
        self.dropped = 0
        self.starved = 0                      # 因为读不到位置而没发帧的次数

    def remember(self, drive):
        """记住这条驱动停住的位置，之后接着按它发。

        前馈力矩不写进这一帧：它跟着「夹持力」走（见 ``set_force``），所以夹住工件
        之后再按 +/- 改力，力道会跟着变，而不是停在按键那一刻的值。
        """
        self.frame = (drive.q, drive.kp, drive.kd, 0.0, 0.0)
        self.closing = drive.closing

    def set_force(self, tau_nm):
        """更新保活帧的前馈力矩 [Nm]。0 = 这一帧只是锁位。"""
        if self.frame is None:
            return
        q, kp, kd, dq, _ = self.frame
        self.frame = (q, kp, kd, dq, max(0.0, float(tau_nm)))

    def maybe_send(self, now):
        """到点就发一帧。

        Returns:
            True/False = 发出去了/没发出去（读不到位置、未使能、已断开）；
            None = 还没到点。
        """
        if now - self.last_sent < self.interval:
            return None
        if self.frame is None:
            # 还没有一个可信的目标——宁可这一拍不发，也不拿缓存的伪值现造一帧。
            # 电机是使能态，自己会持续发状态帧，下一拍读到了就开始发锁位帧。
            self.frame = hold_frame(self.gripper)
            if self.frame is None:
                self.starved += 1
                return False
        self.last_sent = now
        q, kp, kd, dq, tau = self.frame
        if self.gripper.send_mit_frame(q=q, kp=kp, kd=kd, dq=dq, tau=tau):
            self.frames += 1
            return True
        self.dropped += 1
        return False


def wait_fresh_while_feeding(gripper, keeper, timeout_s=FRESH_WAIT_S):
    """等一帧新状态帧，**等待期间照常把帧喂出去**；等不到返回 ``None``。

    为什么不能只调一次 ``fresh_state``：达妙电机是**收到一帧回一帧**，自己不会凭空
    发状态帧——SDK 的 ``_enable_and_hold`` 也是「一边发零增益帧、一边等回帧」，同一
    个原因。而本样例的收发全在主循环这一个线程里，主循环每拍开头那次非阻塞
    ``poll``（见 ``read_real``）已经把上一帧的回帧收走了。于是单线程里干等 50 ms
    时：没有帧在飞、也没人发帧 → 回帧永远不来，干等到超时。

    实测形态（真机，按住 ←/→）：每按一下都走这条路，终端刷出一屏「读不到真机的
    状态帧」，真机一步都不动，退出时还报「一次目标都没改过」。04 没有这个毛病——
    它的循环里没有那次提前的 poll，等的时候上一帧的回帧还在飞。

    等待期间调 ``keeper.maybe_send`` 而不是新造一帧：发出去的还是它本来就在发的
    那一帧（目标 = 上次锁住的位置、不命令任何运动），只是别让总线空着。

    Args:
        gripper: 夹爪。
        keeper: 空闲保活（可以为 ``None``，那就只能干等）。
        timeout_s: 最多等多久 [s]。

    Returns:
        ``GripperState``；``timeout_s`` 内没有新的状态帧则 ``None``。
    """
    deadline = time.monotonic() + timeout_s
    while True:
        left = deadline - time.monotonic()
        if left <= 0.0:
            return None
        if keeper is not None:
            keeper.maybe_send(time.monotonic())
        state = fresh_state(gripper, timeout_s=min(FEED_SLICE_S, left))
        if state is not None:
            return state


def read_real(gripper):
    """读一帧真机状态，返回 ``(开度, 夹持力 N, 是否在动, 故障)``。

    ``get_state(wait=False)`` 只做一次非阻塞 poll 再读缓存，不阻塞主循环。本样例的
    CAN 收发全在主循环这一个线程里，没有第二个线程来抢帧。

    故障描述一起带出来，是因为**锁死的故障在状态帧上是看不出来的**：位置照常更新、
    ``is_moving`` 照常为假，只有错误码变了。不盯着它就会一直发帧。
    """
    state = gripper.get_state(wait=False)
    return (
        rad_to_fraction(gripper, state.position_rad),
        state.force_n,
        bool(state.is_moving),
        fault_of(state),
    )


def real_line(fraction, force_n, moving):
    """真机的一行状态（本仓的开口是**真实毫米**，见 _common.fraction_to_gap_mm）。"""
    return status_line(
        "真机", fraction=fraction, aperture_mm=fraction_to_gap_mm(fraction),
        force_n=force_n, moving=moving,
    )


def read_registers(gripper):
    """读几个 DM 寄存器，读不到的跳过（**只发读请求，不是运动指令**）。

    ``TIMEOUT`` 是电机对看门狗**声明**的时长；实测的闩锁时间
    （MEASURED_COMM_LOSS_S）和它对不上，见 ``--status`` 的说明。这里读它只是为了
    把真值打出来，逻辑上不依赖它。
    """
    from litegrip.can.protocol import DM_REG

    wanted = ("TIMEOUT", "CTRL_MODE", "UV_Value", "OC_Value", "OT_Value")
    out = {}
    for name in wanted:
        rid = getattr(DM_REG, name, None)
        if rid is None:
            continue
        try:
            out[name] = float(gripper.read_param(int(rid), timeout_s=0.5))
        except Exception:
            pass          # 读不到就算了，不能因为一个寄存器把诊断搞挂
    return out


def run_status(args):
    """``--status``：只连接、只读，诊断真机为什么「能读不能控」。

    这条路径**不使能、不发运动指令、不开窗口**：``open_real_gripper(enable=False)``
    只做 connect + load_calibration，之后发出去的只有 DM 的读请求（0x33），不带
    位置/力矩目标。所以电机不会产生任何运动，可以在夹着工件、或手指在别人手里的
    时候安全地跑。

    **未使能的电机不主动发状态帧**，所以这条路径上读不到实时位置是**正常结果**，
    不是错误：位置行和错误码行会被跳过（SDK 的 ``get_state()`` 这时返回的 position
    只是它构造时的初值 ``0.0``，打出来看着像「夹爪在 5.5%」，实际含义是「从没读到
    过」），寄存器照读、退出码照常按那里的故障判定给。想让电机开口就先使能，也就是
    跑不带 ``--status`` 的本样例。

    标定照样要先选：读回来的位置要换成开度，靠的就是标定的角度和 ``rad_to_mm``
    ——用别台机器的刻度换算，打出来的百分比是错的，而这条路径存在的意义就是让这个
    百分比可信。

    加 ``--clear-fault`` 才会写：发的也只是 SDK 的故障清除序列——全程
    ``kp=0/kd=0/tau=0`` 的零力矩帧，**不命令任何运动**。但要说清楚：
    ``clear_fault()`` 内部是 disable → clear → enable，中间那一瞬间电机是失力的，
    手指可能因自重轻微滑动。夹着东西或需要保持位置时先托住再清。

    Returns:
        0 = 健康、无故障，或未使能导致读不到状态帧（判不了故障，不是故障）；
        1 = 有故障但没清或清除失败。
    """
    print("样例 05 · 遥操作模式（--status：只连接、只读，不动电机）")
    gripper = open_real_gripper(args, enable=False)
    cleared = 0
    try:
        # ── 1. 读一帧状态 ──
        # 未使能的电机不会自己发帧，所以这里等不到是**预期**结果，不是错误。绝不能
        # 退回 get_state() 的缓存：那时它是 MotorState 的初值 0.0——打印出来就是
        # 「5.5% / 6.63 mm」这种**伪造**读数，拿来判断故障只会把人带偏。
        print("\n[1] 读一帧状态")
        state = fresh_state(gripper, timeout_s=STATUS_WAIT_S)
        if state is None:
            print(f"   未使能：电机不主动发状态帧，等了 {STATUS_WAIT_S:g} s 没有新帧。")
            print("      这是 --status 这条只读路径的正常结果，不是错误（本样例不使能，"
                  "也没有打开电机的公开接口）。")
            print("      因此下面没有位置行、也没有错误码行：get_state() 这时返回的"
                  "position 只是它构造时的初值 0.0，「从没读到过」才是它的真意。")
            print("      想看实时位置就跑不带 --status 的本样例（会先使能，使能后"
                  "电机自己持续发帧）。")
        else:
            fraction = rad_to_fraction(gripper, state.position_rad)
            print("  " + status_line(
                "真机", fraction=fraction,
                aperture_mm=fraction_to_gap_mm(fraction),
                force_n=state.force_n, moving=bool(state.is_moving),
            ))
            print(f"   错误码 0x{state.error_code:X} · "
                  f"{describe_code(state.error_code)}"
                  f"（刚等到的一帧实测值，不是缓存）")

        # ── 2. 读寄存器 ──
        print("\n[2] 读寄存器（只发读请求）")
        registers = read_registers(gripper)
        if registers:
            shown = " · ".join(f"{k}={v:g}" for k, v in registers.items())
            print(f"   {shown}")
        timeout_ms = registers.get("TIMEOUT")
        if timeout_ms:
            print(f"   通信超时保护（TIMEOUT, RID 9）= {timeout_ms:g} ms")
        elif "TIMEOUT" in registers:
            print("   通信超时保护（TIMEOUT, RID 9）= 0（这个寄存器当前不生效）")
        if "TIMEOUT" in registers:
            print(f"   别拿这个寄存器当依据：实测**使能态**的电机静默约 "
                  f"{MEASURED_COMM_LOSS_S:g} s 就锁 0xD 通信丢失故障，与寄存器读数"
                  "对不上（这台机器读到过 8000，也读到过 0），SDK 自己把这条标成"
                  "「待查」。")
            print("      所以 04/05 空闲时照 200 Hz 持续发帧，不赌这个数字；只读"
                  "不喂帧（或跑了别的只读脚本）同样会把它看哑。")

        if state is None:
            # 判不了故障码：位置和错误码都只在状态帧里。说「没有故障」是撒谎，但读
            # 不到帧本身也不是故障——未使能的电机就是不开口。照实说明，退出 0。
            print("\n故障：判不了。错误码只在状态帧里，而这一路（未使能）读不到"
                  "状态帧。\n"
                  "      想判故障就跑不带 --status 的本样例：它会先使能，"
                  "使能后电机自己发帧。")
            return 0

        if not state.is_error:
            print("\n没有故障。真机能正常接受指令——想动它就直接跑本样例"
                  "（不带 --status）。")
            return 0

        # ── 3. 故障与清除 ──
        print(f"\n[3] 故障与清除\n真机处在锁死的故障态：{fault_of(state)}")
        print("      位置照样能读、但任何指令都不会被执行（这就是「能读不能控」）。")
        if not args.clear_fault:
            print("\n   要清掉它，加 --clear-fault 再跑一次：\n"
                  "       python3 examples/05_dual_control.py --status --clear-fault\n"
                  "   （只发零力矩帧，不命令运动；但清除流程会 disable→enable，\n"
                  "    那一瞬间电机失力，手指可能因自重滑动——先托住夹爪。）")
            return 1

        print("\n   正在清除故障（只发零力矩帧）...")
        if not gripper.clear_fault():
            print("   清除失败，电机可能还在故障态。查供电（欠压常见于电源"
                  "带不动）后重试。")
            return 1
        cleared = 1
        after = gripper.get_state(wait=True)
        if after.is_error:
            print(f"   清完还是故障态：{fault_of(after)}")
            return 1
        print(f"   故障已清除，电机已重新使能（错误码 0x{after.error_code:X}）。"
              "现在可以跑本样例正常下发了。")
        return 0
    finally:
        # disconnect() 自己会先失能（0xFD）再关总线，而 clear_fault() 结束时电机
        # 是使能态的——所以清完故障退出，电机**不会**保持在原来的位置：手指会松。
        gripper.disconnect()
        print("[真机] 已关闭"
              + ("（已清故障并失能：手指会松、夹着的工件会掉）" if cleared else
                 "（未使能，未发送任何运动指令）"))


def main():
    args = parse_args()
    if args.status:
        return run_status(args)
    if args.headless and not args.dry_run:
        raise SystemExit(
            "样例 05 靠键盘来设目标（MuJoCo 的查看器没有可加的滑条控件），"
            "没有窗口就没法操作。\n"
            "   想看无窗口的纯仿真请用 examples/01_hello_sim.py（只读）"
            " 或 examples/02_move_sim.py；\n"
            "   想看真机 → 仿真（不需要操作）请用 examples/04_mirror_real.py；\n"
            "   想无窗口看本样例的流程加 --dry-run：那条路上目标由脚本曲线给。"
        )

    force_n = max(0.0, min(MAX_GRIP_FORCE_N, args.force))
    speed_pct = min(100.0, max(0.0, args.speed))
    scripted = args.dry_run and args.headless

    if args.dry_run:
        print("样例 05 · 遥操作模式（--dry-run：不碰真机，只走流程）")
        if scripted:
            print("   --headless：没有窗口就没有键盘，目标改由一段脚本给"
                  f"（{len(SCRIPT_STEPS)} 段，每段走 {SCRIPT_MOVE_S:g} s 停一下，"
                  f"周期 {_SCRIPT_PERIOD_S:g} s）——它顶替的是「那只手」")
        # dry-run 也要先选标定：目标角和毫米刻度都由它决定，真机跑的就是这一套。
        # 不碰 CAN。
        gripper = open_real_gripper(args, dry_run=True, noise=args.noise)
        live = False
        n_to_nm = 0.1
    else:
        print(SAFETY_BANNER)
        print("\n样例 05 · 遥操作模式")
        gripper = open_real_gripper(args)
        live = True
        n_to_nm = import_litegrip().UnitConversion.N_TO_NM

    # 从这里起全部在 try 里：真机一旦使能（上面 open_real_gripper 干的事），任何
    # 异常都必须走到 finally 去 disable/disconnect。否则程序带着一个「已使能、但
    # 再没人喂帧」的电机退出——静默约 0.9 s 就锁通信超时故障（红灯闪），而且因为
    # 进程已经死了，连是哪一步炸的都看不到。窗口建得慢也算在这里面。
    sim = None
    keeper = None
    drive = None
    last_sent = 0.0
    last_print = 0.0
    last_target_pct = None   # None = 还没对齐过基准，第一帧只对齐
    drags = 0                # 改过几次目标 = 建过几条驱动
    rejected = 0             # 因为读不到状态帧而拒发的次数（按键没白按，但真机没动）
    faulted = False          # 真机报故障：停发、不再对着不听话的电机发帧
    shown_rad = None         # 窗口里的「命令位置」（dry-run 与没读过真机时用它）
    started = time.monotonic()

    try:
        sim = make_sim(args.headless)
        sim.enable()
        sim.focus_camera()
        # 窗口里的起点用真机（dry-run 下是替身）**现在**的读数：这样第一帧不会因为
        # 「窗口摆在一个地方、真机在另一个地方」而先跳一下。
        shown_rad = gripper.get_state(wait=False).position_rad
        max_speed_rad_s = plan_speed(gripper, speed_pct)

        print(f"   [仿真] {sim.model_path}")
        print("   ←/→ 目标开度（一档 5%）· ↓/↑ 速度（一档 10%）· "
              "-/+ 夹持力（一档 5 N）")
        print("   Space 停下 · H 回全开 · Esc/Q 退出")
        print(f"   位置目标每帧最多走 {max_speed_rad_s * FRAME_DT:.6f} rad"
              f"（= 速度 × {FRAME_DT * 1000:g} ms），{FRAME_HZ:g} Hz 发帧——"
              "按得再快也不会变成一条阶跃指令")
        if not live:
            print("   （dry-run：不连真机、不发任何帧，窗口里走的是命令值）")
        else:
            # 使能之后**立刻**喂一帧锁在当前位置，不等主循环第一圈。SDK 的
            # `enable()` 现在会自己抱在实测位置（`_enable_and_hold`：使能帧 →
            # 0.05 s 零力矩流 → 等一帧新状态 → 锁位），但那条流只有 50 ms，之后
            # 就没人喂了，而建窗口要几百毫秒——使能态静默约 MEASURED_COMM_LOSS_S
            # 就锁 0xD，等不起。所以先喂上，计数器归零。
            keeper = IdleKeeper(gripper)
            keeper.maybe_send(time.monotonic())
            print(f"   [真机] 已锁在当前位置（保活 {IDLE_HZ:g} Hz 已开始，"
                  f"目标 = 实测位置、零前馈，不命令运动）")
            if keeper.frame is None:
                print(f"      读不到状态帧（等了 {FRESH_WAIT_S * 1000:.0f} ms）："
                      "还没有可信的锁位目标，先不发帧。使能态的电机自己会持续发"
                      "状态帧，读到就开始发。")

        while sim.connected():
            events = sim.keyboard_events()
            if pressed(events, QUIT_KEYS):
                print("\n   收到退出键")
                break

            now = time.monotonic()
            if args.duration > 0 and now - started >= args.duration:
                print(f"\n跑满 {args.duration:g} s，退出")
                break

            if live:
                real_fraction, real_force_n, real_moving, fault = read_real(gripper)
                if fault and not faulted:
                    # 进了故障态：立刻停发，别再对着不听话的电机发帧了。窗口留着
                    # 不关，好让人把上面这些字读完。
                    print(f"\n真机报故障：{fault}")
                    print("   已停止发帧。清故障（不动电机）："
                          "examples/05_dual_control.py --status --clear-fault")
                    drive = None
                    faulted = True
            else:
                real_fraction, real_force_n, real_moving, fault = 0.0, 0.0, False, None

            # ── 键盘 ──
            # 每次按键走一档。GLFW 的自动重复会把「按住不放」变成一串按下事件，
            # 所以按住就是连着按——但一帧最多只走 max_step_rad，按得再快也一样。
            #
            # ``target_pct`` 的起点是**当前目标**（有驱动就是它指的地方，没有就是
            # 窗口里显示的那个位置），按一下在这个基础上加减一档。
            target_pct = rad_to_fraction(
                gripper, drive.q_want if drive is not None else shown_rad) * 100.0
            if scripted:
                target_pct = scripted_fraction(now - started) * 100.0
            else:
                if pressed(events, TELEOP_KEYS["aperture_down"]):
                    target_pct -= APERTURE_STEP
                if pressed(events, TELEOP_KEYS["aperture_up"]):
                    target_pct += APERTURE_STEP
                if pressed(events, TELEOP_KEYS["slower"]):
                    speed_pct = max(0.0, speed_pct - SPEED_STEP)
                if pressed(events, TELEOP_KEYS["faster"]):
                    speed_pct = min(100.0, speed_pct + SPEED_STEP)
                if pressed(events, TELEOP_KEYS["force_down"]):
                    force_n = max(0.0, force_n - FORCE_STEP_N)
                if pressed(events, TELEOP_KEYS["force_up"]):
                    force_n = min(MAX_GRIP_FORCE_N, force_n + FORCE_STEP_N)
                if pressed(events, TELEOP_KEYS["home"]):
                    target_pct = 100.0
                if pressed(events, TELEOP_KEYS["stop"]):
                    # 停下：目标留在原处。已经在跟着的目标不变，所以真机照常走完
                    # 当前这一步然后停住，而不是急停或回弹。
                    print(f"   停下：目标留在当前值（{target_pct:.1f}%）")
            target_pct = min(100.0, max(0.0, target_pct))

            # ── 目标变了就（重新）建驱动 ──
            # 第一帧只对齐基准：目标没变过就不算「改过」。
            if last_target_pct is None:
                last_target_pct = target_pct
            moved_target = abs(target_pct - last_target_pct) > STEP_EPS
            if moved_target:
                last_target_pct = target_pct
            if moved_target and not faulted:
                target_rad = fraction_to_target_rad(gripper, target_pct / 100.0)
                if drive is None:
                    # 每条驱动都要有实测的起点：目标改之前发的保活帧只是「锁在电机
                    # 说的位置」，还没有一条指令。拿旧读数当起点，限速本身就没有意义
                    # 了——一步就是从错的地方走到目标。所以拿不到就拒绝，别猜。
                    if live:
                        state = wait_fresh_while_feeding(gripper, keeper)
                        if state is None:
                            rejected += 1
                            print(f"\n读不到真机的状态帧（等了 "
                                  f"{FRESH_WAIT_S * 1000:.0f} ms，期间保活帧照发），"
                                  "**不下发**：")
                            print("   限速要按「现在」的位置算，拿旧读数算出来的"
                                  "是一条阶跃指令，电机接不住。")
                            print("   先看真机怎么了："
                                  "python3 examples/05_dual_control.py --status")
                            continue
                        fault = fault_of(state)
                        if fault:
                            # 锁死的故障下，发什么都白搭，还会掩盖真正的原因
                            drive = None
                            faulted = True
                            print(f"\n真机报故障：{fault}")
                            print("   故障是锁死的：位置照读，但电机不执行任何指令。"
                                  "请先清故障再下发：")
                            print("   python3 examples/05_dual_control.py --status "
                                  "--clear-fault")
                            continue
                        start_rad = state.position_rad
                        travel_mm = abs(target_rad - start_rad) \
                            * gripper.config.rad_to_mm
                        print(f"\n[目标 #{drags + 1}] 开度 {target_pct:.1f}% · "
                              f"从实测 {start_rad:+.4f} rad 起步 · "
                              f"路程 {travel_mm:.1f} mm · "
                              f"限速 {speed_pct:.0f}% "
                              f"（{speed_pct / 100.0 * RATED_SPEED_MM_S:.0f} mm/s）")
                    else:
                        # dry-run：没有真机可读，接着上一条驱动停住的位置走。
                        start_rad = shown_rad
                        print(f"\n[目标 #{drags + 1}] 开度 {target_pct:.1f}% · "
                              f"（dry-run：起点用命令值 {start_rad:+.4f} rad）")
                    drive = KeyboardDrive(
                        start_rad=start_rad, speed_rad_s=plan_speed(gripper, speed_pct),
                        kp=gripper.config.kp, kd=gripper.config.kd)
                    drags += 1
                    last_sent = 0.0
                drive.retarget(target_rad)
                # 速度随时可调：它改的是**位置目标每帧的增量**，不是这条驱动开跑时
                # 的速度。
                drive.speed_rad_s = plan_speed(gripper, speed_pct)

            # ── 发帧 ──
            if faulted:
                pass                       # 故障态一帧都不发：发了也不执行
            elif drive is not None:
                # 前馈力矩是恒定推的，不分方向：张开时加力会顶住电机不让它张开，
                # 所以只在**收拢**方向加——收拢时加力才是「夹紧」。而且只有到位之后
                # 才加（KeyboardDrive.frame 里判的）。
                tau_nm = force_n * n_to_nm if drive.closing else 0.0
                if drive.due(now, last_sent):
                    last_sent = now
                    if live:
                        sent = drive.send(gripper, now, tau_nm)
                    else:
                        drive.advance(now, tau_nm)     # dry-run：推进，不发帧
                        sent = True
                    if not sent and drive.dropped == 1:
                        # send_mit_frame 返回 False = 没使能 / 没连接。旧版这里直接
                        # 吞掉了，于是「一条帧都没发出去」也报「发完 N 帧」。
                        print("   帧没发出去（send_mit_frame 返回 False）："
                              "真机可能未使能或已断开")
                if drive.done(now, DEFAULT_HOLD_S):
                    print(f"   [#{drags}] 停住 {DEFAULT_HOLD_S:g} s，"
                          f"{'发出' if live else '（dry-run）模拟'} "
                          f"{drive.frames} 帧"
                          + (f"（丢 {drive.dropped} 帧）" if drive.dropped else "")
                          + "，接着按这个位置保活")
                    # 交回保活：夹持力不会因为「停住」而松掉，电机也不会因为收不到
                    # 帧而锁超时故障。下一次改目标会重新读一次实测位置。
                    if keeper is not None:
                        keeper.remember(drive)
                    shown_rad = drive.q
                    drive = None
            elif keeper is not None:
                # 空闲保活。见 IdleKeeper：停发 = 等电机的通信超时保护把真机锁成
                # 故障态。
                keeper.set_force(force_n * n_to_nm if keeper.closing else 0.0)
                sent = keeper.maybe_send(now)
                if sent is False and keeper.dropped == 1:
                    print("   空闲保活帧没发出去（send_mit_frame 返回 "
                          "False）：真机可能未使能或已断开")
                elif sent is False and keeper.starved and keeper.starved % 200 == 1:
                    # 不是「发失败」，是「不敢发」：保活帧的目标必须是实测位置，读不到
                    # 就不造这一帧（见 IdleKeeper）。每 200 次报一次免得刷屏。
                    print(f"   读不到真机状态帧，保活帧发不出去（第 "
                          f"{keeper.starved} 次）：目标得按实测位置算，拿不到就不发。"
                          f"\n      查 CAN 连接和供电，或先跑 --status 看真机状态。")

            # 窗口显示的是**真机在哪**（dry-run 下是命令值）。用 set_frac_open 而不是
            # command_fraction：前者是运动学瞬移，只把模型摆到那个位置，不驱动任何
            # 东西——这不是「仿真控制真机」的那条路径，是镜像。
            if drive is not None:
                shown_rad = drive.q
            sim.set_frac_open(
                real_fraction if live else rad_to_fraction(gripper, shown_rad))

            # 阶段名要两份：窗口里那份必须是 ASCII——查看器的内置字体没有中文字形，
            # 中文进去就是一片实心方块（见 MujocoGripper.status_text 的说明）。
            # 终端那份照旧用中文。
            if faulted:
                phase, phase_win = "故障", "FAULT"
            elif drive is not None and not drive.arrived():
                phase = f"跟随中 → {target_pct:.0f}%"
                phase_win = f"TRACK -> {target_pct:.0f}%"
            elif drive is not None and drive.closing and force_n > 0.0:
                phase = f"到位·加力 {force_n:g} N"
                phase_win = f"HOLD {force_n:g} N"
            elif drive is not None:
                phase, phase_win = "到位", "HOLD"
            elif not live:
                phase, phase_win = "空闲", "IDLE"
            elif real_moving:
                phase, phase_win = "锁位·真机在动", "LOCK (hardware moving)"
            else:
                phase, phase_win = "锁位", "LOCK"
            commanded = rad_to_fraction(gripper, shown_rad) * 100
            if live:
                sim.status_text([
                    f"REAL {real_fraction * 100:5.1f}%   "
                    f"CMD {commanded:5.1f}%   "
                    f"force {real_force_n:5.2f} N   {phase_win}"
                ])
            else:
                sim.status_text([
                    f"CMD {commanded:5.1f}%   TARGET {target_pct:5.1f}%   "
                    f"speed {speed_pct:3.0f}%   {phase_win}"
                    "   (dry-run: no hardware)"
                ])
            if now - last_print >= PRINT_DT:
                last_print = now
                if live:
                    print("  " + real_line(real_fraction, real_force_n, real_moving)
                          + f" · 命令 {commanded:5.1f}% · {phase}")
                else:
                    print(f"  [命令] {commanded:5.1f}% · 目标 {target_pct:5.1f}% · "
                          f"速度 {speed_pct:3.0f}% · 力 {force_n:4.1f} N · {phase}"
                          " · （dry-run：没有真机）")

            if not sim.pump():
                break
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        if live:
            # 退出时**明确失能**（0xFD），而不是只停发帧：
            #   · 只发一帧 kp=0 的话，电机还是「使能 + 没人喂帧」——实测这种状态
            #     静默约 0.9 s 就锁 0xD 通信丢失故障，而进程一退就没人能清它；
            #   · disable() 把电机放到「已失能」：它不再需要帧，也不会闩故障。
            # 代价说清楚：失能后手指是软的，夹着的工件会掉、手指可能因自重滑动。
            # 「退出把电机留成故障态」比松手严重得多，所以选失能。
            gripper.disable()
            gripper.disconnect()
            print("[真机] 已失能（0xFD）并断开：手指会松、夹着的工件会掉；"
                  "这样退出不会在电机上留下通信超时故障")
        else:
            gripper.disconnect()
            print("[真机] 已关闭（dry-run：未连接 CAN）")
        if sim is not None:
            sim.disconnect()
        if keeper is not None and live:
            print(f"   [真机] 保活帧 {keeper.frames} 条"
                  + (f"，丢 {keeper.dropped} 条" if keeper.dropped else ""))

    if faulted:
        return 1
    if drags == 0 and rejected == 0:
        print("\n一次目标都没改过：按 ←/→ 真机才会动，启动之后它一直锁在当前位置。")
    elif drags == 0:
        # 按了键、但一条指令都没发出去。这和「一次都没按」不是一回事，别报成一样：
        # 上一版不管按没按都报「一次目标都没改过」，把真正的原因埋掉了。
        print(f"\n按键收到了 {rejected} 次，但一条指令都没发出去：每次都读不到真机的"
              "状态帧，真机一步都没动。")
        print("   查 CAN 连接和供电，或先跑 "
              "python3 examples/05_dual_control.py --status 看真机状态。")
    print("完成。反向的（真机 → 仿真）见 examples/04_mirror_real.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
