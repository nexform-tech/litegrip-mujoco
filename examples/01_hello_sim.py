#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 01 · 独立仿真 — 创建仿真夹爪并读取状态（只读，不运动）

只加载模型、读它现在在哪，一个运动指令都不发。适合第一次跑、或者只想确认
MuJoCo 和自带 MJCF 能起来的时候。

演示:
  MujocoGripper(render=True)            加载自带 MJCF，打开可视化窗口
  sim.model_path / sim.model.nu         模型身份：用的哪份描述、几个受驱动关节
  sim.frac_open()                       归一化开度（0 闭合 … 1 张开）
  sim.gap_mm()                          钳口间隙 [mm]（由指面几何算出）
  sim.get_position()                    行程位置 [mm]（本仓的真实毫米）
  sim.get_position_rad()                仿真关节角 [rad]（与真机刻度不同，见下）
  sim.status_text() / sim.pump()        窗口里刷状态、推进仿真（读按键要靠它）
  sim.keyboard_events() + pressed()     Esc / Q 退出
  sim.disconnect()                      关闭仿真

运行:
  python3 examples/01_hello_sim.py                  # 开窗口看实时状态
  python3 examples/01_hello_sim.py --headless       # 无窗口，打印完直接退出
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
        description="样例 01 · 独立仿真：创建仿真夹爪并读取状态"
                    "（只读，不运动，不接真机）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_common_args(ap)
    ap.add_argument(
        "--scene", action="store_true",
        help="用 scene.xml（带台面、夹具和工件）而不是纯夹爪模型",
    )
    return ap.parse_args()


def show_model(sim) -> None:
    """模型常量：全都是只读的，一个字都没命令给电机。"""
    print("\n[1] 模型常量")
    print(f"   MJCF      {sim.model_path}")
    print(f"   指关节    {sim.model.nu} 个受驱动器驱动的移动副")
    print(f"   单指行程  {C.STROKE * 1000:.2f} mm（0 = 张开，走满行程 = 闭合）")
    print(f"   钳口间隙  {C.GAP_CLOSED_MM:.2f} mm（闭合）… "
          f"{C.GAP_OPEN_MM:.2f} mm（张开）")
    print(f"   整程开口  {C.MM_SCALE:.2f} mm —— 本仓一律用这个真实毫米口径，")
    print(f"             而不是 SDK 名义的 120 mm 刻度")
    print(f"   力上限    {C.FORCE_MAX:g} N（= 齿轮比 {C.GEAR:g} × 力矩上限 "
          f"{C.TAU_MAX:g} N·m；例程的力上限是 {MAX_GRIP_FORCE_N:g} N）")
    print(f"   默认速度  {C.DEFAULT_VELOCITY_M_S * 1000:.2f} mm/s（单指速度；"
          f"两指对冲，开口变化是它的两倍）")
    print(f"   物理步长  {C.TIMESTEP * 1000:g} ms")
    print(f"   仿真时刻  {sim.sim_time:.3f} s")


def show_opening(sim) -> None:
    """同一个开度有三种写法——这是本仓库最容易混起来的地方。"""
    print("\n[2] 当前开度（三种写法）")
    print(f"   归一化开度  frac_open()        = {sim.frac_open() * 100:5.1f}%"
          f"     ← 两边唯一共用的量")
    print(f"   行程位置    get_position()     = {sim.get_position():6.2f} mm"
          f"（本仓的真实毫米，全行程 {C.MM_SCALE:.2f} mm）")
    print(f"   钳口间隙    gap_mm()           = {sim.gap_mm():6.2f} mm"
          f"（两个指面的实际距离）")
    print(f"   夹持力      get_force()        = {sim.get_force():6.2f} N"
          f"（现在没夹东西，应当是 0）")
    print(f"   仿真关节角  get_position_rad() = {sim.get_position_rad():+.4f} rad"
          f"（{C.POS_CLOSED_RAD:+.2f} = 闭合 … {C.POS_OPEN_RAD:+.2f} = 张开）")
    print("   注意这个角度不是真机的电机角：本仓仿真用 0 → -1.14 rad，"
          "而真机\n             的标定角形如 +1.776 → -0.064 rad。两者刻度不同，"
          "所以跨设备\n             比较只能看归一化开度——见 examples/03_trajectory.py。")


def interactive(sim) -> None:
    """有窗口时：实时刷状态，Esc/Q 或关窗退出。只读——不发任何指令。"""
    if not sim.gui:
        return
    print("\n[3] 实时状态（Esc / Q 退出）")
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
        # 窗口里的字只能写 ASCII：查看器的内置字体没有中文字形，中文会画成实心
        # 方块（见 MujocoGripper.status_text 的说明）。终端打印照旧用中文。
        sim.status_text([
            f"open {sim.frac_open() * 100:5.1f}%  "
            f"gap {sim.gap_mm():5.2f} mm  "
            f"force {sim.get_force():5.2f} N"
        ])
        if not sim.pump():
            break
    print()


def main() -> int:
    args = parse_args()

    print("样例 01 · 独立仿真（只读，不运动；不接真机，不会动真机）")
    sim = make_sim(args.headless, model_path="scene.xml" if args.scene else None)
    try:
        sim.focus_camera()
        show_model(sim)
        show_opening(sim)
        interactive(sim)
    finally:
        sim.disconnect()
    print("\n完成。下一步看 examples/02_move_sim.py，让手指动起来")
    return 0


if __name__ == "__main__":
    sys.exit(main())
