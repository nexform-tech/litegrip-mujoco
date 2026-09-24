#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 05 · 仿真控制夹爪 — 同一条指令同时下发仿真与真机（双控）

   一条指令 ──┬──► 真机 LiteGrip  (CAN)
              └──► MuJoCo 仿真      （可视化 / 影子验证）

这是"先仿真后上机"的落地形态：把真机当成仿真的执行器，仿真当成真机的显示器。
两条链路并行下发，跑完立刻比对开度 —— 偏差大就说明模型与真机对不上，
该去查标定或查机械，而不是继续往下跑。

演示:
  DualGripper(channel, can_id, dry_run=…)  建立双控
  dual.start()                             连接 + 使能 + 首帧对齐
  dual.open/close/goto/move_to_frac/grasp  两边同时下发
  dual.compare()                           比对两侧开度
  dual.request_stop()                      两边同时急停
  dual.disconnect()                        释放资源（**不是** close()）

⚠ 定位口径：``dual.goto(mm)`` 只有在真机的 ``config.max_stroke_mm`` 已改成
  真实行程 85.452 时，两侧的"毫米"才是同一个量。否则请用
  ``dual.move_to_frac(frac)`` —— 无量纲开度是与标定口径无关的唯一安全口径。

⚠ ``close()`` 是**合拢夹爪**，不是释放资源。释放资源用 ``disconnect()``。

前置（只做一次 · 在仓库根目录）:
  pip install -e ".[dev]"     # 本包是 src 布局，不装就 import 不到

运行（无硬件，用虚拟夹爪顶替真机）:
  python3 examples/05_dual_control.py --dry-run

运行（真机 · 需要 can0 已配置、24V 上电、已完成标定）:
  python3 examples/05_dual_control.py --channel can0 --can-id 0x08

⚠ 真机模式会让夹爪**真实运动**。先确认行程内没有手、线缆和障碍物。
"""
import argparse
import sys
import time

from litegrip_mujoco import HAS_SDK, DualGripper, sdk_unavailable_reason


def hdr(text):
    print(f"\n{'─' * 62}\n{text}\n{'─' * 62}")


def check_args(args):
    if args.dry_run:
        return
    if not HAS_SDK:
        print(f"[错误] 未安装 litegrip SDK：{sdk_unavailable_reason()}")
        print("       先安装：pip install -e /home/qaz/lite-grip")
        print("       或改用 --dry-run 在无硬件下跑通全流程。")
        sys.exit(2)


def report(dual, label):
    """跑完一条指令后比对两侧。"""
    time.sleep(0.3)
    cmp = dual.compare()
    flag = "✅" if abs(cmp["delta"]) < 0.05 else "⚠️ "
    print(f"  {label:24s} 真机 {cmp['real_frac']:6.3f}"
          f"   仿真 {cmp['sim_frac']:6.3f}"
          f"   Δ {cmp['delta']:+.4f} {flag}")
    return cmp


def run(dual):
    hdr("[1] 位置控制 —— 两边同时")
    dual.open(duration=1.0)
    report(dual, "open()")
    dual.close(duration=1.0)
    report(dual, "close()")

    hdr("[2] 绝对位置 —— move_to_frac（跨设备安全口径）")
    for frac in (0.25, 0.5, 0.75, 1.0):
        dual.move_to_frac(frac, duration=0.6)
        report(dual, f"move_to_frac({frac:.2f})")

    hdr("[3] 绝对位置 —— goto(mm)，需要真机行程已标定为 85.452")
    for mm in (0.0, 20.0, 42.726, 85.452):
        dual.goto(mm, duration=0.6)
        report(dual, f"goto({mm:.3f} mm)")

    hdr("[4] 抓取 —— 真机走它自己的堵转检测")
    dual.move_to_frac(1.0, duration=0.6)
    t0 = time.monotonic()
    real_ok, sim_ok = dual.grasp(force_n=10.0, duration=3.0)
    dt = time.monotonic() - t0
    print(f"  grasp(force_n=10.0)  用时 {dt:.2f}s"
          f"   真机 {real_ok}   仿真 {sim_ok}")
    print("  注意：空夹时两边都返回 True —— 指爪顶到闭合限位同样是"
          "\"停住了\"，这是 SDK 的既有语义。")
    rs, ss = dual.get_real_state(), dual.get_sim_state()
    print(f"  真机 force_n={rs.force_n:7.3f} N   "
          f"仿真 force_n={ss.force_n:7.3f} N")

    hdr("[5] 力控 —— 在当前位置施加夹持力")
    dual.set_force(8.0, duration=0.3)
    print(f"  真机 {dual.get_real_state().force_n:7.3f} N"
          f"   仿真 {dual.get_sim_state().force_n:7.3f} N")

    hdr("[6] 张开 —— 收尾")
    dual.open(duration=1.0)
    report(dual, "open()")


def main():
    ap = argparse.ArgumentParser(
        description="LiteGrip 夹爪 · 仿真与真机双控")
    ap.add_argument("--dry-run", action="store_true",
                    help="无 CAN 硬件时用虚拟夹爪顶替真机")
    ap.add_argument("--channel", default="can0", help="CAN 接口名")
    ap.add_argument("--can-id", type=lambda s: int(s, 0), default=0x08,
                    help="夹爪 CAN ID（可用 0x 前缀）")
    ap.add_argument("--mst-id", type=lambda s: int(s, 0), default=None,
                    help="主机 CAN ID（缺省用 SDK 默认值）")
    ap.add_argument("--no-render", action="store_true", help="不开可视化窗口")
    ap.add_argument("--no-mirror-first", action="store_true",
                    help="启动时不先让仿真对齐真机开度")
    args = ap.parse_args()
    check_args(args)

    hdr("[0] 建立双控")
    if args.dry_run:
        print("  真机 = DryRunGripper（虚拟夹爪）—— 全流程与接真机一致，")
        print("         只是 CAN 那一层被换掉了。")
    else:
        print(f"  真机 = litegrip.LiteGrip(channel={args.channel!r},"
              f" can_id={args.can_id:#04x})")

    dual = DualGripper(
        channel=args.channel,
        can_id=args.can_id,
        mst_id=args.mst_id,
        render=not args.no_render,
        mirror_first=not args.no_mirror_first,
        dry_run=args.dry_run,
    )
    try:
        dual.start()
        print(f"  {dual!r}")
        print("  已使能，并已把仿真对齐到真机当前开度。")
        run(dual)

        hdr("[7] 本次运行的开度偏差统计")
        cmp = dual.compare()
        print(f"  真机 {cmp['real_frac']:.4f}   仿真 {cmp['sim_frac']:.4f}"
              f"   Δ {cmp['delta']:+.4f}")
        print("  Δ 长期偏大 → 查真机标定（pos_open_rad / pos_closed_rad）")
        print("             或查模型行程口径（应为 85.452 mm）")

        print("\n✅ 完成。同一段代码，仿真和真机走的是同一条路径。")
        print("   把 --dry-run 去掉、补上 --channel/--can-id 就是真机运行。")
        print("   切换单机目标只需换 import：")
        print("       from litegrip_mujoco import MujocoGripper as LiteGrip  # 仿真")
        print("       from litegrip import LiteGrip                          # 真机")

    except KeyboardInterrupt:
        print("\n\n用户中断 —— 正在急停两边")
        try:
            dual.request_stop()
        except Exception as exc:  # noqa: BLE001
            print(f"[警告] 急停失败: {exc}")
    finally:
        dual.disconnect()


if __name__ == "__main__":
    main()
