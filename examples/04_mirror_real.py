#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 04 · 夹爪控制仿真 — 真机的实际开度实时驱动仿真（数字孪生）

   真机 LiteGrip ──(CAN, θ)──► MirrorMode ──(frac_open)──► MuJoCo 仿真

仿真是**纯跟随**的：它不产生任何指令，只是把真机的开度按 50 Hz 画出来。
用途是给真机配一双眼睛 —— 看到指爪在什么位置、有没有真的夹住、有没有干涉。

演示:
  MirrorMode(real, sim, rate_hz=50)   建立单向镜像
  mirror.start() / stop()             启停
  mirror.samples / last_frac_open     确认真的在跑
  read_frac_open(device)              从任意夹爪读无量纲开度

⚠ 为什么交换的是 `frac_open` 而不是电机角 θ：
  θ 的**零点和量程都是设备相关的**。仿真的 θ ∈ [-1.14, 0]，真机标定后
  θ ∈ [-0.064279, 1.775959]（量程 1.8402）。同一个 `position_rad` 在两侧
  指的是完全不同的开度，直接对拷必然错位。`frac_open ∈ [0,1]` 是唯一与
  标定口径无关的量，所以跨设备只交换它。

前置（只做一次 · 在仓库根目录）:
  pip install -e ".[dev]"     # 本包是 src 布局，不装就 import 不到

运行（无硬件，用虚拟夹爪顶替真机）:
  python3 examples/04_mirror_real.py --dry-run
  python3 examples/04_mirror_real.py --dry-run --noise      # 带传感器噪声

运行（真机 · 需要 can0 已配置、24V 上电、已完成标定）:
  python3 examples/04_mirror_real.py --channel can0 --can-id 0x08

⚠ 真机模式会让夹爪**真实运动**。先确认行程内没有手、线缆和障碍物。
"""
import argparse
import sys
import time

from litegrip_mujoco import (
    HAS_SDK,
    DryRunGripper,
    MujocoGripper,
    MirrorMode,
    read_frac_open,
    require_sdk,
    sdk_unavailable_reason,
)

RATE_HZ = 50.0


def hdr(text):
    print(f"\n{'─' * 62}\n{text}\n{'─' * 62}")


def make_real(args):
    """按参数造一台"真机"：--dry-run 用虚拟夹爪，否则用 SDK。"""
    if args.dry_run:
        print("  [真机] DryRunGripper（虚拟夹爪，内部是一台独立仿真）")
        print("         它对外声称的是**真机口径**的 θ 端点，因此下面的")
        print("         frac_open 换算路径与接真机时完全一致。")
        return DryRunGripper(realtime=True, noise=args.noise)

    if not HAS_SDK:
        print(f"[错误] 未安装 litegrip SDK：{sdk_unavailable_reason()}")
        print("       先安装：pip install -e /home/qaz/lite-grip")
        print("       或改用 --dry-run 在无硬件下跑通全流程。")
        sys.exit(2)

    sdk = require_sdk()
    print(f"  [真机] {sdk.__name__}.LiteGrip(channel={args.channel!r},"
          f" can_id={args.can_id:#04x})")
    return sdk.LiteGrip(channel=args.channel, can_id=args.can_id,
                        mst_id=args.mst_id)


def drive_script(real, mirror):
    """在真机上跑一段脚本动作，每步打印两侧开度。"""
    def show(label):
        frac = read_frac_open(real, warn=False)
        sim_frac = mirror.last_frac_open
        delta = "" if sim_frac is None else f"  Δ={frac - sim_frac:+.4f}"
        print(f"  {label:22s} 真机 {frac:6.3f}"
              f"   仿真 {('  --  ' if sim_frac is None else f'{sim_frac:6.3f}')}"
              f"{delta}")

    steps = [
        ("open()", lambda: real.open(duration=1.0), 1.2),
        ("goto(20 mm)", lambda: real.goto(20.0, duration=0.6), 0.8),
        ("goto(60 mm)", lambda: real.goto(60.0, duration=0.8), 1.0),
        ("goto(42.7 mm)", lambda: real.goto(42.726, duration=0.6), 0.8),
        ("close()", lambda: real.close(duration=1.0), 1.2),
        ("grasp(10 N)", lambda: real.grasp(force_n=10.0, duration=3.0), 3.4),
        ("open()", lambda: real.open(duration=1.0), 1.2),
    ]
    for label, action, wait in steps:
        try:
            action()
        except Exception as exc:  # noqa: BLE001 — 真机故障要如实显示，别中断示例
            print(f"  {label:22s} ❌ {type(exc).__name__}: {exc}")
            continue
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            time.sleep(0.1)
        show(label)

    print("\n  等待镜像收敛 …")
    time.sleep(0.5)
    show("最终")


def main():
    ap = argparse.ArgumentParser(
        description="LiteGrip 夹爪 · 真机驱动仿真（数字孪生）")
    ap.add_argument("--dry-run", action="store_true",
                    help="无 CAN 硬件时用虚拟夹爪顶替真机")
    ap.add_argument("--channel", default="can0", help="CAN 接口名")
    ap.add_argument("--can-id", type=lambda s: int(s, 0), default=0x08,
                    help="夹爪 CAN ID（可用 0x 前缀）")
    ap.add_argument("--mst-id", type=lambda s: int(s, 0), default=None,
                    help="主机 CAN ID（缺省用 SDK 默认值）")
    ap.add_argument("--rate", type=float, default=RATE_HZ, help="镜像频率 Hz")
    ap.add_argument("--noise", action="store_true",
                    help="--dry-run 时给读数加传感器噪声")
    ap.add_argument("--no-render", action="store_true", help="不开可视化窗口")
    args = ap.parse_args()

    hdr("[1] 建立设备")
    real = make_real(args)
    # 构造也要在 try 里面：render=True 时查看器就是在构造函数里开的。
    try:
        sim = MujocoGripper(render=not args.no_render)
        sim.connect()
    except RuntimeError as exc:
        print(f"  [警告] {exc} → 改为无窗口运行")
        sim = MujocoGripper(render=False)
        sim.connect()
    print(f"  [仿真] {sim!r}")
    print("         刻意**不**使能：镜像只是按开度摆位，使能后位置环会跟")
    print("         镜像写入的 qpos 打架。要自己发指令就换 05 的双控模式。")

    mirror = None
    try:
        real.connect()
        real.enable()
        print("  两台设备就绪。")

        # ── 先让仿真对齐真机的当前开度 ──
        frac0 = read_frac_open(real, warn=True)
        sim.set_frac_open(frac0)
        sim.settle(0.2)
        print(f"  仿真已对齐到真机当前开度 {frac0:.4f}"
              f"（开口 {sim.gap_mm():.3f} mm）")

        # ── 启动镜像 ──
        hdr(f"[2] 启动镜像（{args.rate:.0f} Hz，真机 → 仿真）")
        mirror = MirrorMode(real, sim, rate_hz=args.rate)
        t_mirror = time.monotonic()
        mirror.start()
        print("  仿真是纯跟随的：镜像期间不要再给 sim 发运动指令，否则互相打架。")

        # ── 驱动真机，观察仿真跟随 ──
        hdr("[3] 在**真机**上执行动作，仿真应当实时跟随")
        drive_script(real, mirror)

        # ── 统计 ──
        hdr("[4] 镜像统计")
        elapsed = time.monotonic() - t_mirror
        print(f"  已镜像样本 {mirror.samples} 个 / {elapsed:.2f} s"
              f"  → 实测 {mirror.samples / elapsed:.1f} Hz"
              f"（目标 {args.rate:.0f} Hz）")
        last = mirror.last_frac_open
        print(f"  最近开度   {'(尚未采样)' if last is None else f'{last:.4f}'}")
        real_frac = read_frac_open(real, warn=False)
        sim_frac = sim.frac_open()
        print(f"  真机 {real_frac:.4f}  仿真 {sim_frac:.4f}"
              f"  Δ = {real_frac - sim_frac:+.4f}")
        print(f"  仿真开口   {sim.gap_mm():.3f} mm")
        print("\n✅ 完成。真机的每一个动作都在仿真里看得见。")
        print("   下一步：examples/05_dual_control.py 反过来——"
              "一条指令同时下发两边")

    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        if mirror is not None:
            mirror.stop()
        for device, name in ((real, "真机"), (sim, "仿真")):
            try:
                device.disconnect()
            except Exception as exc:  # noqa: BLE001
                print(f"[警告] 断开{name}失败: {exc}")


if __name__ == "__main__":
    main()
