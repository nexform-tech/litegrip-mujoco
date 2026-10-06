#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 02 · 仿真运动 — 速度受限的全行程开合，以及按归一化开度定位

不用真机、不碰 CAN：命令 → 手指按额定速度走 → settle() 等它到位，全在 MuJoCo 里。
想让夹爪自己走一段你自己拖出来的动作，继续看 examples/03_trajectory.py。

演示:
  sim.command_fraction(fraction, force_n=, velocity_m_s=)  命令一个开度（立即返回）
  sim.settle()                    等手指真的到目标，返回 (用时, 是否到位)
  sim.frac_open() / sim.gap_mm()  读回实测开度
  sim.status_text() / sim.pump()  窗口里刷状态、推进仿真

运行:
  python3 examples/02_move_sim.py                       # 开窗口跑两段演示
  python3 examples/02_move_sim.py --headless            # 无窗口（跑得快，exit 0）
  python3 examples/02_move_sim.py --speed 0.02          # 慢速收爪（约 2 s 全行程）
  python3 examples/02_move_sim.py --force 20            # 20 N 夹持力上限
  python3 examples/02_move_sim.py --scene               # 用场景模型，多加一段夹工件
"""
import argparse
import sys

from _common import (  # noqa: I001  (必须先于 litegrip_mujoco)
    MAX_GRIP_FORCE_N,
    add_common_args,
    make_sim,
    status_line,
)

from litegrip_mujoco import QUIT_KEYS, constants as C, pressed


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="样例 02 · 仿真运动：速度受限的开合与按开度定位（不接真机）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_common_args(ap)
    ap.add_argument("--force", type=float, default=10.0,
                    help=f"夹持力上限 [N]（默认 10，上限 {MAX_GRIP_FORCE_N:g}）")
    ap.add_argument("--speed", type=float, default=C.DEFAULT_VELOCITY_M_S,
                    help=f"手指速度 [m/s]（默认 {C.DEFAULT_VELOCITY_M_S:.5f}，"
                         f"即真机的 85 mm/s）")
    ap.add_argument("--scene", action="store_true",
                    help="用 scene.xml（带台面、夹具和工件）跑，追加一段夹工件的演示")
    return ap.parse_args()


def show(sim, label: str = "仿真", *, force: bool = True) -> None:
    print(status_line(
        label,
        fraction=sim.frac_open(),
        aperture_mm=sim.gap_mm(),
        force_n=sim.get_force() if force else None,
    ))


def demo_travel(sim, args: argparse.Namespace) -> None:
    """全行程开合——顺便量一下速度受不受限。"""
    print("\n[1] 全行程开合（速度受限）")
    for target, name in ((0.0, "闭合"), (1.0, "张开")):
        sim.command_fraction(target, force_n=args.force, velocity_m_s=args.speed)
        spent, reached = sim.settle()
        flag = "到位" if reached else "没到位（被顶住了）"
        print(f"   → {name}：用掉 {spent:.3f} s 仿真时间 · {flag}")
        show(sim)
    print(f"   单指行程 {C.STROKE * 1000:.2f} mm · 单指速度 {args.speed * 1000:.2f} mm/s"
          f" → 期望单程 ≈ {C.STROKE / args.speed:.2f} s")
    print(f"   两个手指对冲，所以开口变化的速度是这个的两倍："
          f"{args.speed * 2000:.1f} mm/s（真机规格 85 mm/s）")


def demo_midpoint(sim, args: argparse.Namespace) -> None:
    """按归一化开度走到中间位——这是仿真和真机共用的「同一种语言」。"""
    print("\n[2] 走到中间位（归一化开度）")
    for fraction in (0.5, 0.25, 0.75):
        sim.command_fraction(fraction, force_n=args.force, velocity_m_s=args.speed)
        spent, reached = sim.settle()
        got = sim.frac_open()
        print(f"   命令 {fraction * 100:5.1f}% → 实测 {got * 100:5.1f}% · "
              f"开口 {sim.gap_mm():5.2f} mm · {spent:.3f} s · "
              f"{'到位' if reached else '没到位'}")
    sim.command_fraction(1.0)
    sim.settle()


def demo_grasp(sim, args: argparse.Namespace) -> None:
    """场景模型里夹住一个工件——这段讲清楚「力上限」和「力控」不是一回事。

    现场顺序是有讲究的，两步都不能省：

    * ``release_fixture()`` 让场景自带的那个工件先落到地板上。夹爪是**固定安装**
      的，两指之间的东西只会自由落体——不腾空这块地方，新方块一放进去就被旧工件
      挤住。
    * 合拢必须**快**。方块放进指间就开始掉，慢吞吞地合拢会看着它滑出去。所以这
      一段用 :meth:`~litegrip_mujoco.MujocoGripper.grasp`（恒定目标 + 堵转检测，
      没有限速斜坡），而不是 ``command_fraction``。

    要讲的两件事：

    * ``command_fraction(force_n=)`` 的 ``force_n`` 是**出力上限**，不是力控。
      位置环还在，目标一路指向闭合位时 ``kp·Δθ`` 会压倒前馈力矩并让输出饱和——
      实测 ``force_n=0 / 10 / 20`` 拿到的接触力是一样的。``grasp()`` 才是力控：
      检测到堵转后把目标改写到当前位置，位置误差归零，只剩前馈力矩。
    * ``get_force()`` 是**电机侧**的力（``GEAR × tau``），接触力是**指面侧**的，
      两者口径不同，夹住时数值也不相等。
    """
    print("\n[3] 夹住一个工件（仅 --scene 有）")
    sim.reset("fixture")
    sim.settle(0.05)
    sim.release_fixture()
    sim.settle(1.0)
    name = sim.add_box(0, size=(0.010, 0.010, 0.015))
    print(f"   已把 {name} 搬到指间（20 × 20 × 30 mm）")

    reached = sim.grasp(force_n=args.force, duration=1.0)
    print(f"   grasp(force_n={args.force:.1f}) → "
          f"{'检测到堵转，已转为力保持' if reached else '超时，没夹住'}")
    show(sim)
    print(f"   开口停在 {sim.gap_mm():.2f} mm，而不是闭合的 "
          f"{C.GAP_CLOSED_MM:.2f} mm → 指面压在了工件上")
    print(f"   方块 20 mm 宽，指面间距 {sim.gap_mm():.2f} mm → 压进去 "
          f"{20.0 - sim.gap_mm():.3f} mm（接触刚度有限，不是穿模）")

    # 指面侧与电机侧应当对得上：静止夹持时两指的接触力就是电机推出来的力。
    # 左右分开算，因为摩擦会让两侧不等。
    for side in ("left", "right"):
        points = [c for c in sim.contacts(only=("spawn_box_0",))
                  if side in (c.geom1 + c.geom2)]
        total = sum(c.force_n for c in points)
        print(f"   {side:5s} 指面侧 {total:5.2f} N（{len(points)} 个接触点，"
              f"接触是面不是点）")
    print(f"   get_force() 报 {sim.get_force():5.2f} N（由电机力矩推算，与指面侧"
          f"对得上）")
    print(f"   但它不是请求的 {args.force:.1f} N：grasp() 只把前馈力矩设成 "
          f"{args.force:.1f} N，")
    print(f"   位置环仍然接着，工件回弹让目标位与实测位差出一点角度，kp·Δθ 就加到"
          f"了前馈上，")
    print(f"   实测超了 {sim.get_force() - args.force:.1f} N。force_n=0 时更明显："
          f"本场景下实测仍报 6.2 N。")
    print("   所以 grasp() 的 force_n 是前馈基准，不是夹持力的闭环设定值。")
    print("   真机上是否同样超调，本机没有 CAN 硬件，未经验证。")

    sim.open()
    sim.settle(1.0)
    print("   张开后工件留在原处（本仓不做抓取规划，只演示力与接触）")


def interactive(sim) -> None:
    """有窗口时：实时刷状态，Esc/Q 或关窗退出。"""
    if not sim.gui:
        return
    print("\n[4] 实时状态（Esc / Q 退出）")
    while sim.connected():
        if pressed(sim.keyboard_events(), QUIT_KEYS):
            print("\n   收到退出键")
            break
        line = status_line(
            "仿真",
            fraction=sim.frac_open(),
            aperture_mm=sim.gap_mm(),
            force_n=sim.get_force(),
        )
        print(line, end="\r")
        sim.status_text([
            f"开度 {sim.frac_open() * 100:5.1f}%  "
            f"开口 {sim.gap_mm():5.2f} mm  "
            f"力 {sim.get_force():5.2f} N"
        ])
        if not sim.pump():
            break
    print()


def main() -> int:
    args = parse_args()
    args.force = max(0.0, min(MAX_GRIP_FORCE_N, args.force))
    if args.speed <= 0.0:
        # 限速是斜坡的时间尺度，0 或负数会让斜坡永远走不完（除零）。
        print(f"--speed 必须为正，收到 {args.speed}；改用默认 "
              f"{C.DEFAULT_VELOCITY_M_S:.5f} m/s")
        args.speed = C.DEFAULT_VELOCITY_M_S

    print("样例 02 · 仿真运动（不接真机，不会动真机）")
    sim = make_sim(args.headless, model_path="scene.xml" if args.scene else None)
    try:
        print(f"   MJCF      {sim.model_path}")
        print(f"   指关节    {sim.model.nu} 个受驱动器驱动的移动副")
        print(f"   初始状态  {sim.frac_open() * 100:.1f}% · "
              f"开口 {sim.gap_mm():.2f} mm")
        # 和真机一样，不使能就一条运动指令都发不出去。仿真里没有硬件的风险，
        # 但这个门是刻意保留的——这样同一段控制代码在仿真和真机上的
        # 前置条件完全一致（真机上忘了 enable() 是最常见的一次「怎么不动」）。
        sim.enable()
        sim.focus_camera()

        demo_travel(sim, args)
        demo_midpoint(sim, args)
        if args.scene:
            demo_grasp(sim, args)
        show(sim)

        interactive(sim)
    finally:
        sim.disconnect()
    print("\n完成。想把一段手拖的动作录下来重放，继续看 examples/03_trajectory.py"
          "；真机版本见 examples/05_dual_control.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
