#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 02 · 仿真内运动 — 位置 / 速度 / 力的各种控制方式（不需要任何硬件）

分两段跑，因为**力的数字只在夹住东西时才有意义**：

  第一段  纯夹爪模型（litegrip.xml），没有工件 → 位置、速度、以及
          ``close(force_n=…)`` 为什么限不住力
  第二段  演示场景（scene.xml），工件由夹具托在两指之间 → 抓取、力控，
          并用"松开夹具后工件掉不掉"来验证抓取是否真的成立

演示:
  open() / close()              位置控制（min-jerk 目标斜坡）
  goto(mm)                      绝对位置
  move_at_speed(mm, mm/s)       恒速运动
  grasp(N)                      自适应抓取：先合拢到堵转，再保持夹持力
  set_force(N)                  在当前位置施加夹持力（真·力控）
  release_fixture() / hold_fixture()

⚠ 关于力控的三条已知语义（仿真与真机一致，不是仿真缺陷）：
  1. ``close(force_n=…)`` **无法限力**。目标一路指向闭合位，位置误差 kp·Δθ 会
     压倒前馈力矩并让执行器饱和。实测空夹时 force_n=0/10/20 给出同一个结果。
     要力控就用 ``grasp()`` 或 ``set_force()``。
  2. ``grasp(force_n)`` 的实际夹持力会**略大于**请求值。堵转确认窗口（5×10 ms）
     里指爪还在往前走，目标被改写到窗口末尾的位置，于是留下一段固定位置误差。
     实测 10 N 请求 → 15.51 N（20 mm 方块）。力对 force_n 仍单调，可当带偏置的
     开环力控用。详见 assets/litegrip.xml 头部的标定表。
  3. 同理，``set_force()`` / ``get_force()`` 在**没有夹住任何东西**时没有意义 ——
     那是指爪顶着硬限位，位置环在跟一个推不动的目标较劲。只有夹稳之后，
     目标=当前位置、位置误差归零，力矩才真的等于请求值。

前置（只做一次 · 在仓库根目录）:
  pip install -e ".[dev]"     # 本包是 src 布局，不装就 import 不到

运行:
  python3 examples/02_move_sim.py
  python3 examples/02_move_sim.py --no-render
"""
import argparse
import time

from litegrip_mujoco import MujocoGripper


def hdr(text):
    print(f"\n{'─' * 62}\n{text}\n{'─' * 62}")


def open_sim(model, no_render):
    """连一台仿真；开不出窗口就退回无窗口。"""
    gripper = MujocoGripper(model_path=model, render=not no_render)
    try:
        gripper.connect()
    except RuntimeError as exc:
        print(f"  [警告] {exc} → 改为无窗口运行")
        gripper = MujocoGripper(model_path=model, render=False)
        gripper.connect()
    gripper.enable()
    return gripper


# ══════════════════════════════════════════════════════════════════════════
# 第一段：纯夹爪模型，没有工件
# ══════════════════════════════════════════════════════════════════════════


def kinematics(g):
    hdr("[1] 位置控制 open() / close()")
    for label, fn in (("open", g.open), ("close", g.close)):
        t0 = time.monotonic()
        fn(duration=1.0)
        print(f"  {label:6s} 用时 {time.monotonic() - t0:.2f}s"
              f"  行程 {g.get_position():7.3f} mm"
              f"  开口 {g.gap_mm():7.3f} mm")

    hdr("[2] 绝对位置 goto(mm) —— 0 = 闭合，85.452 = 全开")
    for mm in (0.0, 20.0, 42.726, 85.452):
        g.goto(mm, duration=0.6)
        print(f"  goto({mm:7.3f}) → 行程 {g.get_position():7.3f} mm"
              f"  开口 {g.gap_mm():7.3f} mm")

    hdr("[3] 速度控制 move_at_speed(mm, mm/s)")
    for speed in (80.0, 20.0):
        g.goto(85.452, duration=0.8)
        g.settle(0.2)
        start = g.get_position()
        t0 = time.monotonic()
        g.move_at_speed(0.0, speed_mm_s=speed)
        dt = time.monotonic() - t0
        travelled = start - g.get_position()
        print(f"  目标 {speed:5.1f} mm/s：走完 {travelled:7.3f} mm"
              f" 用时 {dt:.2f}s  → 实测均速 {travelled / dt:6.1f} mm/s")

    print("\n  空夹（两指之间什么都没有）时 get_force() 的读数是没意义的 ——")
    print("  指爪顶在硬限位上，位置误差被限位吃掉，只剩前馈分量。")
    print("  力的数字只有在夹住东西之后才作数，见第二段 [4]/[6]。")


# ══════════════════════════════════════════════════════════════════════════
# 第二段：演示场景，工件由夹具托在两指之间
# ══════════════════════════════════════════════════════════════════════════


def grasping(g):
    obj = g.model.body("object").id

    hdr("[4] close(force_n=…) 为什么限不住力 —— 拿工件当靶子")
    print("  目标一路指向闭合位，指爪停在工件表面时留下约 9 mm 的位置误差，")
    print("  kp·Δθ 远大于前馈 n_to_nm(force_n)，执行器直接饱和到 ±TAU_MAX：\n")
    for n in (0.0, 5.0, 10.0, 20.0):
        g.reset("fixture")
        g.settle(0.2)
        g.close(force_n=n, duration=1.0)
        print(f"  close(force_n={n:5.1f}) → {g.get_force():7.3f} N"
              f"   （力矩 {g.get_torque():+7.4f} Nm，开口 {g.gap_mm():7.3f} mm）")
    print("\n  请求 0 N 却给了 100 N。对照 grasp()：")
    for n in (5.0, 10.0, 20.0):
        g.reset("fixture")
        g.settle(0.2)
        g.grasp(force_n=n, duration=3.0)
        print(f"  grasp(force_n={n:5.1f}) → {g.get_force():7.3f} N"
              f"   （力矩 {g.get_torque():+7.4f} Nm，开口 {g.gap_mm():7.3f} mm）")
    print("\n  → 差别在于 grasp() 检测到堵转后把目标**改写到当前位置**，位置误差")
    print("    归零，关节力只剩前馈 GEAR·tau_ff = force_n。这是整个力模型的")
    print("    承重细节：力控 = 位置误差归零 + 力矩前馈，缺一不可。")

    hdr("[5] 抓取 grasp(N) —— 用焊死的工件验证抓取是否真的成立")
    print("  工件由 <weld> 夹具托在两指之间。先夹紧、再解除夹具，之后工件")
    print("  **只靠摩擦**留在指间 —— 这才是对夹持力的真实验证。\n")

    g.reset("fixture")
    g.settle(0.3)
    z0 = float(g.data.xpos[obj][2])
    print(f"  [初始] 工件 z = {z0 * 1000:.3f} mm（夹具托住，指爪张开）")

    for n in (5.0, 10.0, 20.0):
        g.reset("fixture")
        g.settle(0.2)

        t0 = time.monotonic()
        ok = g.grasp(force_n=n, duration=3.0)
        dt = time.monotonic() - t0
        f_meas, gap = g.get_force(), g.gap_mm()

        g.release_fixture()
        g.settle(2.0)
        dz = (float(g.data.xpos[obj][2]) - z0) * 1000.0
        held = abs(dz) < 5.0

        print(f"  grasp({n:5.1f} N) → {str(ok):5s} 用时 {dt:.2f}s"
              f"  开口 {gap:7.3f} mm  夹持力 {f_meas:6.3f} N"
              f"   松开工件后 z {dz:+.3f} mm → "
              f"{'✅ 夹住了' if held else '❌ 掉了'}")

    hdr("[6] 力控 set_force(N) —— 必须已经夹稳才成立")
    g.reset("fixture")
    g.settle(0.2)
    g.grasp(force_n=10.0, duration=3.0)
    # 此刻工件被夹具焊住（很硬），指爪是压在一个刚体上
    print("  （工件仍被夹具焊住，指爪压在刚体上，接触极硬，读数偏大）")
    for n in (5.0, 10.0):
        g.set_force(n, duration=0.6)
        print(f"  set_force({n:5.1f}) → {g.get_force():7.3f} N")

    g.release_fixture()
    g.settle(1.0)
    print("\n  解除夹具后工件只靠摩擦支撑，接触变软，读数才跟着请求值走：")
    for n in (2.0, 5.0, 10.0, 20.0):
        g.set_force(n, duration=0.6)
        print(f"  set_force({n:5.1f}) → {g.get_force():7.3f} N"
              f"   （力矩 {g.get_torque():+.4f} Nm）")

    hdr("[7] open() → 工件落到地板")
    g.open(duration=1.0)
    g.settle(2.0)
    z = float(g.data.xpos[obj][2]) * 1000.0
    print(f"  工件 z = {z:.3f} mm"
          f"   （地板 z=-80 mm + 半高 15 mm = -65 mm 即落稳）")
    print(f"  is_grasped() = {g.is_grasped()}")


def main():
    ap = argparse.ArgumentParser(description="LiteGrip 夹爪 · 仿真内运动")
    ap.add_argument("--no-render", action="store_true", help="不开可视化窗口")
    args = ap.parse_args()

    plain = scene = None
    try:
        print("\n【第一段】纯夹爪模型（无工件）")
        plain = open_sim(None, args.no_render)
        kinematics(plain)
        plain.disconnect()
        plain = None

        print("\n\n【第二段】演示场景（工件 + 夹具）")
        scene = open_sim("scene.xml", args.no_render)
        grasping(scene)

        print("\n✅ 完成。全部在仿真内，不需要任何硬件。")
        print("   下一步：examples/03_trajectory.py 录制并回放轨迹")

    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        for g in (plain, scene):
            if g is not None:
                g.disconnect()


if __name__ == "__main__":
    main()
