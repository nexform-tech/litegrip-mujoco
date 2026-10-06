#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""05 遥操作样例的等待逻辑：等状态帧时不能让 CAN 总线空着。

这不是一次「多读几次而已」的调参，是一条会**卡死真机操作**的回归。达妙电机收到
一帧才回一帧状态（SDK 的 ``_enable_and_hold`` 也是「一边发零增益帧一边等」，同
一个原因），而 05 的主循环每拍开头先做一次非阻塞 ``poll``（显示用，见
``read_real``），把上一帧的回帧收走了。于是单线程里干等 50 ms 时没有帧在飞、也
没人发帧，回帧永远不来——每按一次方向键就白等一次，真机一步不动。

这里用一台「只对收到的帧回一帧」的假夹爪把那个竞态固化下来：``fresh_state``
单独调必然超时，``wait_fresh_while_feeding`` 必须成功，并且必须真的把帧发出去。

真机上的最终确认只能靠手上那台夹爪——本文件证明的是等待期间确实在喂帧。
"""
from __future__ import annotations

import importlib.util
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"
if str(EXAMPLES) not in sys.path:
    sys.path.insert(0, str(EXAMPLES))

_SPEC = importlib.util.spec_from_file_location(
    "example_05_dual_control", EXAMPLES / "05_dual_control.py")
assert _SPEC is not None and _SPEC.loader is not None
teleop = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(teleop)


class ReplyPerCommandGripper:
    """收到一帧才回一帧状态的假夹爪（达妙电机的行为）。"""

    def __init__(self):
        self.pending = 0          # 已经发出、还没被收走的回帧
        self.sent = 0
        self.last_frame = None

    def send_mit_frame(self, q, kp, kd, dq=0.0, tau=0.0):
        self.sent += 1
        self.last_frame = (q, kp, kd, dq, tau)
        self.pending += 1
        return True

    def poll(self, timeout_s=0.0):
        if self.pending:
            self.pending -= 1
            return True
        return False

    def get_state(self, wait=False):
        return SimpleNamespace(position_rad=0.37, force_n=0.0, is_moving=False,
                               error_code=1)


class AlwaysDueKeeper:
    """一直该发帧的保活（真实那只是 200 Hz，这里不必等）。"""

    def __init__(self, gripper):
        self.gripper = gripper
        self.sends = 0

    def maybe_send(self, now):
        self.sends += 1
        self.gripper.send_mit_frame(q=0.37, kp=5.0, kd=2.0)
        return True


def _consume_pending(gripper):
    """模拟主循环每拍开头那次显示用的非阻塞 poll。"""
    gripper.poll(timeout_s=0.0)


def test_fresh_state_times_out_when_nobody_feeds_the_bus():
    """回归现场：干等必然超时——这就是「按了键真机不动」的原因。"""
    gripper = ReplyPerCommandGripper()
    gripper.send_mit_frame(q=0.37, kp=5.0, kd=2.0)   # 保活刚发出去的那一帧
    _consume_pending(gripper)                        # 主循环的显示读把它收走了

    assert teleop.fresh_state(gripper, timeout_s=0.005) is None


def test_waiting_while_feeding_gets_the_frame():
    """等待期间继续喂帧，回帧就会来。"""
    gripper = ReplyPerCommandGripper()
    gripper.send_mit_frame(q=0.37, kp=5.0, kd=2.0)
    _consume_pending(gripper)

    keeper = AlwaysDueKeeper(gripper)
    state = teleop.wait_fresh_while_feeding(gripper, keeper, timeout_s=0.1)

    assert state is not None
    assert state.position_rad == pytest.approx(0.37)
    assert keeper.sends >= 1, "等待期间一帧都没发——总线还是空着"


def test_feeding_wait_still_times_out_on_a_silent_bus():
    """真的没帧时照样拒绝下发，不能为了「等到」而编一个目标出来。"""
    gripper = ReplyPerCommandGripper()      # 谁都不发帧
    started = time.monotonic()
    state = teleop.wait_fresh_while_feeding(gripper, None, timeout_s=0.02)
    elapsed = time.monotonic() - started

    assert state is None
    assert elapsed < 1.0, "超时后必须立刻返回，不能把主循环卡住"
