#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 01 · 独立仿真 — 建立夹爪仿真、读状态、开合一次（不需要任何硬件）

演示:
  MujocoGripper(render=True)   创建仿真并打开可视化窗口
  get_state() / get_position() 读取状态（θ / mm / 力矩 / 夹持力）
  gap_mm()                     两指之间的**绝对开口**（与 SDK 的行程口径不同）
  open() / close()             开合一次
  disconnect()                 释放资源

⚠ 注意这里没有 `close()` 释放资源这回事 —— `close()` 是**合拢夹爪**，
  与真机 SDK 同名同义。释放资源用 `disconnect()`。

运行:
  python3 examples/01_hello_sim.py
  python3 examples/01_hello_sim.py --no-render     # 无图形环境
"""
import argparse
import time

from litegrip_mujoco import MujocoGripper


def main():
    ap = argparse.ArgumentParser(description="LiteGrip 夹爪 · 独立仿真最小样例")
    ap.add_argument("--no-render", action="store_true", help="不开可视化窗口")
    ap.add_argument("--scene", action="store_true",
                    help="用演示场景（带地板与工件）而不是纯夹爪模型")
    args = ap.parse_args()

    model = "scene.xml" if args.scene else None
    gripper = MujocoGripper(model_path=model, render=not args.no_render)
    try:
        gripper.connect()
    except RuntimeError as exc:
        print(f"[警告] {exc}\n[警告] 改为无窗口运行")
        gripper = MujocoGripper(model_path=model, render=False)
        gripper.connect()
    gripper.enable()

    try:
        print(f"\n[模型] {gripper.model_path}")
        print(f"[夹爪] {gripper!r}")

        state = gripper.get_state()
        print("\n[初始状态]")
        print(f"  position_rad  (电机角)     = {state.position_rad:+.4f} rad")
        print(f"  velocity_rad_s(电机角速度) = {state.velocity_rad_s:+.4f} rad/s")
        print(f"  torque_nm     (力矩)       = {state.torque_nm:+.4f} Nm")
        print(f"  position_mm   (行程)       = {state.position_mm:.3f} mm")
        print(f"  force_n       (夹持力估计) = {state.force_n:.3f} N")
        print(f"  is_enabled={state.is_enabled}  is_moving={state.is_moving}")
        print(f"\n[开口] 两指内侧面间距 = {gripper.gap_mm():.3f} mm"
              f"   （无量纲开度 {gripper.frac_open():.4f}）")
        print("       行程口径 = %.3f mm（0 = 闭合）"
              % gripper.get_position())

        # ── 开合一次 ──
        print("\n[1] close(duration=1.0) → 合拢")
        t0 = time.monotonic()
        gripper.close(duration=1.0)
        print(f"    用时 {time.monotonic() - t0:.2f}s  开口 = {gripper.gap_mm():.3f} mm")

        time.sleep(0.3)
        print("\n[2] open(duration=1.0) → 张开")
        t0 = time.monotonic()
        gripper.open(duration=1.0)
        print(f"    用时 {time.monotonic() - t0:.2f}s  开口 = {gripper.gap_mm():.3f} mm")

        print("\n✅ 完成。本样例全程不需要任何硬件。")
        print("   下一步：examples/02_move_sim.py 演示完整的运动与力控")

    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        gripper.disconnect()


if __name__ == "__main__":
    main()
