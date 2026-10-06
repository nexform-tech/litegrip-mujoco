#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 04 · 镜像模式 — 仿真跟随真机的开度同步显示

方向是 **真机 → 仿真**：真机是「主」，仿真只是显示器。每一帧读一次真机的位置，就把
仿真手指瞬移过去，所以画面显示的就是真机现在的样子，没有跟随延迟、也不会自己漂。

用法:
  不给选项        默认模式：持续发「锁在实测位置」的保持帧，手指有刚度、推它会顶回来
  --zero-gravity  真机的电机失力，可以用手推着走（推荐，最能看出镜像效果）——这时
                  手指也会因重力或外力自己滑动，托住夹爪再看
  --passive       只连接、只读，不使能也不发帧，由别的程序去驱动真机
  --dry-run       没有硬件时用：拿一台替身顶替真机，另起线程按脚本动作驱动它
  Z               运行中随时在失力 / 使能之间切换（--passive 与 --dry-run 下无效）
  Esc / Q         退出（退出前会失能：手指会松、夹着的东西会掉）

前提: 真机接在 CAN 总线（默认 can0，用 --channel 换）· 装好 litegrip SDK · 有一份
      这台夹爪的标定（不给 --calib 就用 SDK 出厂那份）。装 SDK、选标定、为什么
      「只是看」也得发帧见 examples/README.zh-CN.md。
"""
import argparse
import math
import sys
import threading
import time

from _common import (  # noqa: I001  (必须先于 litegrip_mujoco)
    FRESH_WAIT_S,
    add_common_args,
    add_hardware_args,
    fraction_to_gap_mm,
    fresh_state,
    make_sim,
    open_real_gripper,
    rad_to_fraction,
    status_line,
)

from litegrip_mujoco import QUIT_KEYS, pressed

# 发 MIT 帧 / 刷新镜像的频率 [Hz]，和 SDK 自己的流式循环一致。
#
# 这个频率不只是「运动时才用」：**使能态**的电机静默约 MEASURED_COMM_LOSS_S 就锁
# 进通信丢失故障——红灯闪、位置照读、指令一律不执行。本样例即使只是「看」，也必须
# 按这个频率持续发帧；只 poll 不喂帧，看一秒就把真机看哑了。
FRAME_HZ = 200.0
FRAME_DT = 1.0 / FRAME_HZ

# 实测的通信超时闩锁时间 [s]：使能态的电机静默这么久就报 0xD（SDK 在真机上量到
# 的）。只用来把话说具体，逻辑上不依赖它——电机的 ``TIMEOUT`` 寄存器读到过 8000、
# 也读到过 0，和这个实测值都对不上（SDK 标注「待查」）。
MEASURED_COMM_LOSS_S = 0.9

# 终端读数的最小刷新间隔 [s]。
PRINT_DT = 0.5

# 切换失力/使能的按键。
ZERO_GRAVITY_KEY = ord("z")

#: ``--dry-run`` 里那只「手」的脚本 [s]：交替全开 → 全闭 → 半开，无限循环。
#:
#: 半开那一档是为了让画面停在行程中间——只看两端点的话，θ → 开度的**线性**关系
#: 是对是错看不出来，而那正是这个样例要展示的换算。
DRY_RUN_SCRIPT = (
    ("open()", lambda real: real.open(duration=1.0), 1.2),
    ("close()", lambda real: real.close(duration=1.0), 1.2),
    ("goto(42.7 mm)", lambda real: real.goto(42.726, duration=0.6), 0.8),
    ("grasp(10 N)", lambda real: real.grasp(force_n=10.0, duration=3.0), 3.4),
)


def parse_args():
    ap = argparse.ArgumentParser(
        description="样例 04 · 镜像模式：把真机的位置实时镜像到 MuJoCo 里",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_common_args(ap)
    add_hardware_args(ap)
    ap.add_argument("--zero-gravity", action="store_true",
                    help="启动就让真机失力（可用手推动手指，仿真跟着走）")
    ap.add_argument("--passive", action="store_true",
                    help="只连接、只读：**不使能**、一帧都不发——已经有别的程序在"
                         "驱动真机时用；单跑的话别加（没人喂帧，真机会锁通信超时"
                         "故障）")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="跑多少秒后自动退出（默认 0 = 一直跑到 Esc/Q 或关窗口）")
    ap.add_argument("--dry-run", action="store_true",
                    help="不碰 CAN：用 DryRunGripper 顶替真机，另起一个线程按脚本"
                         "动作驱动它。θ → 开度的换算与接真机时是同一条路")
    ap.add_argument("--noise", action="store_true",
                    help="配合 --dry-run：给替身的读数加传感器噪声")
    args = ap.parse_args()
    if args.dry_run and args.passive:
        ap.error("--dry-run 里没有别的程序能驱动替身：那只手是本样例自己起的线程。"
                 "两个一起给就没人动它了")
    if args.dry_run and args.zero_gravity:
        ap.error("--dry-run 下失力没有意义：替身的每一帧都是直接摆位，"
                 "没有「电机出力」可撤")
    return args


def mirror(sim, fraction):
    """把仿真手指瞬移到真机的开度上。

    用 ``set_frac_open``（运动学瞬移）而不是 ``command_fraction``（跑动力学追
    过去）：镜像要的是「真机现在在哪」，不是「仿真打算去哪」。瞬移不会有跟随滞后，
    也不会因为仿真里的接触、惯性而偏离真机。

    顺带一提，瞬移在 MuJoCo 这里**不需要**关重力：``set_frac_open`` 除了写 qpos
    还会把控制器的目标设到同一个位置，所以使能的夹爪会自己顶住自重（实测把开度
    摆到 0.5，0.3 s 后读数仍是 0.5000）。pybullet 那版要 ``gravity=(0,0,0)``，
    是因为它的 ``reset_fraction`` 只写关节值。
    """
    sim.set_frac_open(fraction)


def set_zero_gravity(gripper, on, was_on):
    """开/关真机失力模式，返回新的状态（没变就原样返回）。"""
    if on == was_on:
        return was_on
    if on:
        gripper.enter_zero_gravity()  # 发一帧 kp=0/kd=0 打底
        print("   [真机] 失力——可以推动手指了（再按 Z 恢复）")
    else:
        gripper.exit_zero_gravity()   # 锁在当前位，回到正常闭环
        print("   [真机] 恢复使能（锁在当前位置）")
    return on


def drive_stand_in(real, stop: threading.Event) -> None:
    """``--dry-run`` 里那只「手」：按一段脚本动作反复驱动替身，直到 ``stop`` 置位。

    用**阻塞**的 ``open`` / ``close`` / ``goto`` / ``grasp``，不是 ``set_frac_open``：
    这些是驱动一台真机会用的调用，让替身按真实的时间常数走完，镜像那边才有东西可
    看（``set_frac_open`` 是瞬移，画面会跳）。它们在后台线程里跑，主循环照常读。

    读不到就跳过这一步并说清楚：替身故障时**不该**中断样例，但也不该假装它动了。
    """
    while not stop.is_set():
        for label, action, wait_s in DRY_RUN_SCRIPT:
            if stop.is_set():
                return
            try:
                action(real)
            except Exception as exc:   # noqa: BLE001 — 替身故障要如实显示
                print(f"   [dry-run] {label} 失败："
                      f"{type(exc).__name__}: {exc}")
                continue
            if stop.wait(wait_s):
                return


def main():
    args = parse_args()

    print("样例 04 · 镜像模式（真机 → 仿真）")
    if args.dry_run:
        print("   --dry-run：不碰 CAN，用 DryRunGripper 顶替真机"
              "（真机那一段未经验证）")
    # --passive 是「只看别人的」：**不使能**、一帧都不发。以前这里也是使能了再一帧
    # 不发，于是名不副实——使能态的电机静默约 0.9 s 就自己锁 0xD 通信丢失故障。
    # 不使能的电机不需要帧，也就没有这个故障可闩，而且不用给手指任何刚度，
    # 「只读」才是真的只读。
    gripper = open_real_gripper(args, enable=not args.passive,
                               dry_run=args.dry_run, noise=args.noise)

    # 看一眼就使能：镜像只写 qpos，但 set_frac_open 同时把控制器的目标设过去，
    # 使能之后手指才会顶住自重。不使能的话瞬移完就开始往下出溜。
    sim = make_sim(args.headless)
    sim.enable()
    sim.focus_camera()

    print(f"   [仿真] {sim.model_path}")
    print("   Z = 真机失力/恢复 · Esc/Q = 退出"
          + (f" · {args.duration:g} s 后自动退出" if args.duration > 0 else ""))

    # --dry-run 下「总线」不存在，喂帧也就无从谈起：替身的每一帧都是直接摆位，
    # 拿保持帧去喂它反而会把画面钉在 hold_rad 上，和那只手对着拉。所以这条路径按
    # --passive 的规矩来（一帧不发），只是驱动方换成了本样例自己的线程。
    passive = args.passive or args.dry_run
    zero_gravity = False
    if args.passive:
        print("   （--passive：不使能、一帧都不发，只读。真机得由别的程序喂帧；"
              "这里不抢总线）")
        if args.zero_gravity:
            print("   （--zero-gravity 在 --passive 下无效：失力也要发帧）")
    elif args.dry_run:
        print("   （--dry-run：一帧不发——替身没有总线，也就没有静默锁死的故障；"
              "它由本样例另起的线程驱动）")
        print("   （Z 在这里无效：失力要发一帧 kp=0，而替身的每一帧都是直接摆位，"
              "发出去只会把它钉住）")
    elif args.zero_gravity:
        zero_gravity = set_zero_gravity(gripper, True, zero_gravity)
    else:
        print(f"   （真机保持使能，本样例每 {FRAME_DT * 1000:.0f} ms 发一条"
              "「锁在实测位置」的保持帧——不命令运动，只是防止电机"
              "因收不到帧而锁通信超时故障。想用手推着看镜像，"
              "加 --zero-gravity 或运行中按 Z）")

    stop = threading.Event()
    driver = None
    if args.dry_run:
        print("   [dry-run] 另起线程驱动替身："
              + " → ".join(label for label, _a, _w in DRY_RUN_SCRIPT) + " → 循环")
        driver = threading.Thread(target=drive_stand_in, args=(gripper, stop),
                                  name="dry-run-driver", daemon=True)
        driver.start()

    started = time.monotonic()
    last_frame = 0.0
    last_print = 0.0
    frames = 0
    hold_rad = None                 # 锁位帧的目标：读到实测位置的那一刻定一次
    starved = 0                     # 读不到状态帧、于是没发成锁位帧的次数

    try:
        while sim.connected():
            events = sim.keyboard_events()
            if pressed(events, QUIT_KEYS):
                print("\n   收到退出键")
                break
            if pressed(events, (ZERO_GRAVITY_KEY,)) and not args.passive \
                    and not args.dry_run:
                was_zero_gravity = zero_gravity
                zero_gravity = set_zero_gravity(gripper, not zero_gravity,
                                                zero_gravity)
                if was_zero_gravity and not zero_gravity:
                    # 刚从失力恢复：手指可能已经被推到别处了，锁位目标必须重新取
                    # **现在**的位置。还用失力之前那个目标的话，本样例就是在命令
                    # 电机走回原处——一次没人要求的运动（SDK 的 exit_zero_gravity
                    # 自己也是锁在当前位置）。
                    state_now = fresh_state(gripper)
                    hold_rad = state_now.position_rad if state_now else None

            now = time.monotonic()
            if args.duration > 0 and now - started >= args.duration:
                print(f"\n跑满 {args.duration:g} s，退出")
                break

            # 读真机位置：**必须**是新鲜读数，所以走 fresh_state（等到一帧新的状态
            # 帧）而不是 get_state(wait=False) 的缓存。本样例展示的就是「真机现在
            # 在哪」，而缓存里可能是冻结的旧值——读不到时更是 MotorState 的初值
            # 0.0，照它渲染画面等于撒谎。读不到就把画面停在最后一次读数上，并在
            # 窗口和终端里都说明。
            state = fresh_state(gripper)
            stale = state is None
            if stale:
                state = gripper.get_state(wait=False)     # 只为把画面停住

            if not stale and hold_rad is None:
                # 锁位帧的目标只定这一次，之后一直重发同一个值。每拍都拿当次读数
                # 现造目标的话，一次读数冻结就会变成一条指向伪值的新指令——那是
                # 阶跃，见 examples/05 里 hold_frame 的说明。
                hold_rad = state.position_rad
                if not passive:
                    print(f"   [真机] 锁在实测位置 {hold_rad:+.4f} rad"
                          "（零前馈，不命令运动）")

            # 两种模式都必须**持续发帧**，理由见 FRAME_HZ：使能态的电机静默约
            # MEASURED_COMM_LOSS_S 就锁通信丢失故障。只在这一个线程里收发，不会
            # 有第二个线程抢 CAN 帧。
            if not passive and now - last_frame >= FRAME_DT:
                last_frame = now
                if zero_gravity:
                    # 失力：kp=0/kd=0，手指可以被手推动（kp=0 时 q 给什么都不出力）
                    gripper.send_mit_frame(q=0.0, kp=0.0, kd=0.0)
                    frames += 1
                elif hold_rad is not None:
                    # 正常模式：锁在**实测位置**（零前馈、目标就是它现在的位置）
                    # ——不命令任何运动，只是让手指有刚度、把超时计数器喂上。
                    gripper.send_mit_frame(q=hold_rad, kp=gripper.config.kp,
                                           kd=gripper.config.kd)
                    frames += 1
                else:
                    # 还没读到过位置：不造锁位帧——目标只能是实测位置，拿缓存的伪值
                    # 当目标是发一条阶跃指令出去，比少发一帧危险得多。
                    # 这里电机是**使能态**，自己会持续发状态帧，所以只是还在等
                    # （enable 之后总要先收到第一帧）；不额外发任何请求帧去催它。
                    starved += 1
                    if starved % 200 == 1:
                        print(f"   [真机] 还在等状态帧（第 {starved} 次）："
                              "读到实测位置才开始发锁位帧——"
                              "目标必须是实测位置，不能拿缓存的伪值造。")

            real_fraction = rad_to_fraction(gripper, state.position_rad)
            mirror(sim, real_fraction)

            # 窗口里的字只能写 ASCII：查看器的内置字体没有中文字形，中文会画成实心
            # 方块（见 MujocoGripper.status_text 的说明）。终端打印照旧用中文。
            sim.status_text([
                f"REAL {real_fraction * 100:5.1f}%   "
                f"gap {fraction_to_gap_mm(real_fraction):5.2f} mm   "
                f"force {state.force_n:5.2f} N   "
                + ("no status frame (frozen)" if stale else
                   ("ZERO-G (nudge by hand)" if zero_gravity else
                    ("PASSIVE (sends no frames)" if passive else "LOCKED")))
            ])

            if now - last_print >= PRINT_DT:
                last_print = now
                if stale:
                    print(f"  [真机] 读不到状态帧（等了 {FRESH_WAIT_S * 1000:.0f}"
                          " ms）：画面停在最后一次读数上，位置和力都不可信。")
                else:
                    print("  " + status_line(
                        "真机", fraction=real_fraction,
                        aperture_mm=fraction_to_gap_mm(real_fraction),
                        force_n=state.force_n,
                        moving=bool(state.is_moving),
                    ) + f" · 真机 θ {state.position_rad:+.4f} rad")

            if not sim.pump():
                break
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        stop.set()
        if driver is not None:
            driver.join(timeout=3.0)
        if not passive:
            # 退出前**失能**（0xFD），而不是只发一帧 exit_zero_gravity() 就走：
            # 那一帧之后没人再喂，使能态的电机静默约 0.9 s 就锁 0xD。失能则不需要
            # 任何帧，也就没有故障可闩。
            # 代价和 --zero-gravity 退出时一样：手指变软、可能因自重滑动。
            gripper.disable()
            print("[真机] 已失能（0xFD）：手指会松、可能因自重滑动；"
                  "不会在电机上留下通信超时故障")
        # disconnect() 只在使能过的时候才会补一帧 0xFD，所以 --passive 这条路到
        # 这里为止确实一帧都没发出去。
        gripper.disconnect()
        sim.disconnect()
        if args.passive:
            print("[真机] 已关闭（未使能、未发送任何帧）")
        elif args.dry_run:
            print("[真机] 已关闭（替身，从未连过 CAN）")
        else:
            print("[真机] 已关闭")

    what = "一帧都没发" if passive else f"{frames} 帧保活/零重力指令"
    print(f"完成（{what}）。"
          f"反向的（仿真 → 真机）见 examples/05_dual_control.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
