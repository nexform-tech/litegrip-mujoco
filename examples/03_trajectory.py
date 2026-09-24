#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 03 · 轨迹录制与回放 — 采一条位置轨迹、存盘、再放一遍（不需要硬件）

演示:
  录制   以固定频率采样 (t, position_rad, position_mm, torque_nm, force_n)
  存盘   JSON（可读、可 diff、可手工编辑）
  回放   按**记录时的时间戳**逐点重发目标，因此回放的时间轴与录制一致
  校验   对比录制与回放的逐点误差

轨迹文件格式::

    {
      "version": 1,
      "rate_hz": 50.0,
      "model": "…/litegrip.xml",
      "samples": [{"t": 0.0, "position_rad": 0.0, "position_mm": 85.452,
                   "torque_nm": 0.0, "force_n": 0.0}, …]
    }

⚠ 回放写的是 **position_mm（行程口径）**，不是电机角。同一个 mm 只有在两侧
  ``max_stroke_mm`` 一致时才是同一个物理位置；要跨设备（真机 ↔ 仿真）复现，
  请改用无量纲开度 ``frac_open``（见 examples/04_mirror_real.py）。

前置（只做一次 · 在仓库根目录）:
  pip install -e ".[dev]"     # 本包是 src 布局，不装就 import 不到

运行:
  python3 examples/03_trajectory.py                      # 录制 → 存盘 → 回放
  python3 examples/03_trajectory.py --no-render
  python3 examples/03_trajectory.py --load traj.json     # 只回放已有文件
  python3 examples/03_trajectory.py --loop 3             # 回放 3 遍
"""
import argparse
import json
import os
import threading
import time

from litegrip_mujoco import MujocoGripper
from litegrip_mujoco.constants import DEFAULT_KD, DEFAULT_KP

DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "trajectory.json")
STROKE_MM = 85.452


def hdr(text):
    print(f"\n{'─' * 62}\n{text}\n{'─' * 62}")


def _sample(gripper, samples, t_start):
    samples.append({
        "t": round(time.monotonic() - t_start, 6),
        "position_rad": round(gripper.get_position_rad(), 6),
        "position_mm": round(gripper.get_position(), 6),
        "torque_nm": round(gripper.get_torque(), 6),
        "force_n": round(gripper.get_force(), 6),
    })


def record(gripper, rate_hz=50.0, path=DEFAULT_PATH):
    """执行一段脚本动作，同时按**固定频率**采样。

    运动指令（``goto``）是阻塞的，所以它跑在一个后台线程里；采样在前台按
    绝对时间基准走。直接在主线程里"先 goto 再 sample"会让采样率被 goto 的
    时长绑架 —— 那样记下来的 ``t`` 是假的，回放的时间轴也是假的。
    """
    dt = 1.0 / rate_hz
    samples = []
    t_start = time.monotonic()

    def run(label, target_mm, duration):
        """后台发一条 goto，前台按 dt 采点，直到它跑完。"""
        print(f"  录制 {label} …")
        worker = threading.Thread(
            target=gripper.goto, args=(target_mm,),
            kwargs={"duration": duration}, daemon=True,
        )
        worker.start()
        next_t = time.monotonic()
        while worker.is_alive():
            _sample(gripper, samples, t_start)
            next_t += dt
            time.sleep(max(0.0, next_t - time.monotonic()))
        worker.join()
        _sample(gripper, samples, t_start)

    # ── 脚本动作：合 → 开 → 半开 → 合（夹爪上电时是张开的）──
    run("close  85.452 → 0 mm", 0.0, 1.0)
    run("open   0 → 85.452 mm", STROKE_MM, 1.0)
    run("goto   85.452 → 42.726 mm", 42.726, 0.6)
    run("close  42.726 → 0 mm", 0.0, 0.6)

    data = {
        "version": 1,
        "rate_hz": rate_hz,
        "model": gripper.model_path,
        "samples": samples,
    }
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2, ensure_ascii=False)
    print(f"\n  已录制 {len(samples)} 个采样点 → {path}")
    print(f"  时长 {samples[-1]['t']:.3f} s，频率 {rate_hz:.0f} Hz")
    return data


def load(path):
    with open(path, encoding="utf-8") as fh:
        data = json.load(fh)
    if data.get("version") != 1:
        raise ValueError(f"不支持的轨迹文件版本: {data.get('version')!r}")
    return data


def replay(gripper, data, loops=1, kp=DEFAULT_KP, kd=DEFAULT_KD):
    """把记录的 θ 流按原时间戳重新灌进去，返回逐点跟随误差。

    回放**用 ``send_mit_frame()`` 而不是 ``goto()``**。这不是风格问题：

    · ``goto()`` 是"一条指令 + 一段 min-jerk 斜坡"，本身就在生成轨迹。
      用它按 50 Hz 重发，等于每 20 ms 把斜坡重置一次，指爪永远追不上 ——
      实测 RMS 误差 14 mm，最大 85 mm，纯粹是自欺。
    · ``send_mit_frame()`` 才是"**流式设定值**"：它只设控制律，由物理线程
      自己积分。这正是真机上回放轨迹的形态（一串 CAN 报文），也是 MIT 协议
      的本意。

    回放的是**电机角 θ**（``position_rad``）而不是毫米 —— θ 是设备真正接收
    的量，中间不经过任何行程口径的换算。
    """
    samples = data["samples"]
    errors = []

    for lap in range(loops):
        if loops > 1:
            print(f"  第 {lap + 1}/{loops} 遍 …")
        t0 = time.monotonic()
        for s in samples:
            time.sleep(max(0.0, t0 + s["t"] - time.monotonic()))
            gripper.send_mit_frame(s["position_rad"], kp=kp, kd=kd,
                                   dq=0.0, tau=0.0)
            if lap == 0:
                errors.append(gripper.get_position() - s["position_mm"])

    return errors


def main():
    ap = argparse.ArgumentParser(description="LiteGrip 夹爪 · 轨迹录制与回放")
    ap.add_argument("--no-render", action="store_true", help="不开可视化窗口")
    ap.add_argument("--path", default=DEFAULT_PATH, help="轨迹文件路径")
    ap.add_argument("--rate", type=float, default=50.0, help="采样频率 Hz")
    ap.add_argument("--load", action="store_true", help="跳过录制，只回放已有文件")
    ap.add_argument("--loop", type=int, default=1, help="回放遍数")
    args = ap.parse_args()

    # 构造也要在 try 里面：render=True 时查看器就是在构造函数里开的。
    try:
        gripper = MujocoGripper(render=not args.no_render)
        gripper.connect()
    except RuntimeError as exc:
        print(f"[警告] {exc} → 改为无窗口运行")
        gripper = MujocoGripper(render=False)
        gripper.connect()
    gripper.enable()

    try:
        if args.load:
            hdr(f"[1] 加载 {args.path}")
            data = load(args.path)
        else:
            hdr("[1] 录制轨迹")
            data = record(gripper, rate_hz=args.rate, path=args.path)

        hdr("[2] 录制内容")
        samples = data["samples"]
        print(f"  采样点 {len(samples)}  频率 {data['rate_hz']:.0f} Hz"
              f"  时长 {samples[-1]['t']:.3f} s")
        print(f"  {'t (s)':>8} {'行程 (mm)':>12} {'电机角 (rad)':>14}"
              f" {'力矩 (Nm)':>11}")
        step = max(1, len(samples) // 10)
        for s in samples[::step]:
            print(f"  {s['t']:8.3f} {s['position_mm']:12.3f}"
                  f" {s['position_rad']:14.4f} {s['torque_nm']:11.4f}")

        hdr(f"[3] 回放 × {args.loop}"
            f"（流式 θ 设定值 @ {data['rate_hz']:.0f} Hz，"
            f"kp={DEFAULT_KP:g} kd={DEFAULT_KD:g}）")
        # 先摆到轨迹起点，否则第一个点的误差只是"起点不对"，与回放质量无关。
        gripper.goto(samples[0]["position_mm"], duration=0.5)
        gripper.settle(0.2)
        t0 = time.monotonic()
        errors = replay(gripper, data, loops=args.loop)
        wall = time.monotonic() - t0
        expected = samples[-1]["t"] * args.loop

        worst = max(abs(e) for e in errors)
        rms = (sum(e * e for e in errors) / len(errors)) ** 0.5
        print(f"\n  回放用时 {wall:.2f} s，轨迹时长 × 遍数 = {expected:.2f} s"
              f"   → 时间轴{'一致' if abs(wall - expected) < 0.5 else '不一致'}")
        print(f"  跟随误差（实际行程 − 录制行程）  RMS {rms:.4f} mm  最大 {worst:.4f} mm")
        print("  误差不为零是正常的：这是位置环跟随一条 50 Hz 设定值流的滞后，")
        print("  不是时间轴错位。加大 kp 可以减小它，代价是更容易振荡。")

        print("\n✅ 完成。轨迹文件可读可 diff，也可以手工编辑后再回放。")
        print("   下一步：examples/04_mirror_real.py 用真机驱动仿真")

    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        gripper.disconnect()


if __name__ == "__main__":
    main()
