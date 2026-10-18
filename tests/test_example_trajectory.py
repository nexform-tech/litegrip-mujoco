# -*- coding: utf-8 -*-
"""03 轨迹样例里两处「等按键」的闸门：无头时不能变成死等。

03 有两段停下来等操作员的地方：录完等确认回放（``wait_for_start``），和跑完等
Esc / Q 退出（``wait_for_exit``）。两段都靠窗口取按键，而 ``--headless`` 没有窗口：
等下去既没有按键来、也没人关得了窗口，进程就停在那里不退出。带真机跑时这段还在
使能态，动不了也退不出，只能 Ctrl-C —— 而 Ctrl-C 走的是 ``finally`` 里的失能。

``wait_for_start`` 一开始就有「无头直接放行」这条；``wait_for_exit`` 原先没有，
它内联在 ``main()`` 里，所以既没人测得到、红起来也只表现为「这条命令要 Ctrl-C」。
本文件把两段都钉住，用替身而不是真机（真机那半边只能在台上用眼睛确认）。
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

from litegrip_mujoco import CONFIRM_KEYS, QUIT_KEYS
from litegrip_mujoco.window import KEY_PRESSED

#: 一拍「什么都没按」。``keyboard_events()`` 返回的是「键码 → 事件位」的映射。
NO_KEYS: dict = {}

#: 一拍「按了某个键」。
def press(*keys: int) -> dict:
    return {int(code): KEY_PRESSED for code in keys}

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"
if str(EXAMPLES) not in sys.path:
    sys.path.insert(0, str(EXAMPLES))

_SPEC = importlib.util.spec_from_file_location(
    "example_03_trajectory", EXAMPLES / "03_trajectory.py")
assert _SPEC is not None and _SPEC.loader is not None
traj = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(traj)


class FakeSim:
    """够 ``wait_for_*`` 用的仿真替身。

    ``connected()`` 永远为真，``pump()`` 默认永远为真 —— 也就是**窗口一直开着、
    而且没有按键进来**，正是原先会死在里面的那种状态。所以 ``limit`` 是一道保险：
    循环转过去 ``limit`` 拍就直接抛，测试红掉而不是把整个套件挂住（挂住的测试在
    CI 里只表现为超时，看不出是哪条闸门漏了）。
    """

    def __init__(self, *, gui=True, events=(), pumps=None, limit=20):
        self.gui = gui
        self._events = list(events)
        self._pumps = pumps          # 允许成功几次；None = 一直成功
        self._limit = limit
        self.keyboard_calls = 0
        self.pump_calls = 0
        self.status = None

    def keyboard_events(self):
        self.keyboard_calls += 1
        if self.keyboard_calls > self._limit:
            raise AssertionError(
                f"闸门没有在 {self._limit} 拍内返回 —— 这条用例等的是死等")
        return self._events.pop(0) if self._events else dict(NO_KEYS)

    def pump(self):
        self.pump_calls += 1
        return self._pumps is None or self.pump_calls <= self._pumps

    def connected(self):
        return True

    def status_text(self, lines):
        self.status = lines


class CountingKeeper:
    """只数 ``tick()`` —— 闸门等着的这段时间里喂帧有没有断。"""

    def __init__(self):
        self.ticks = 0

    def tick(self, now):
        self.ticks += 1

    @property
    def frames(self):
        return self.ticks


# ══════════════════════════════════════════════════════════════════════════
# wait_for_exit
# ══════════════════════════════════════════════════════════════════════════


class TestWaitForExit:
    def test_headless_does_not_wait_for_a_key(self, capsys):
        """无头时不等按键：没有窗口就没有按键来源，等下去是死等。

        替身故意做成「窗口没关、也没有按键」——正是老代码会一直转的那种状态。
        """
        sim = FakeSim(gui=False)
        keeper = CountingKeeper()

        assert traj.wait_for_exit(sim, keeper) is False
        assert sim.keyboard_calls == 0, "无头时不该去取按键"
        assert sim.pump_calls == 0, "无头时不该推进循环（推进了就说明进了死等）"
        assert keeper.ticks == 0, "没进循环就不该发帧"
        assert "headless" in capsys.readouterr().out

    def test_it_keeps_feeding_until_the_quit_key(self):
        """窗口在时：喂着保持帧等到按键，收到 Esc / Q 才返回。

        这段等待不能只转圈不喂帧 —— 电机还使能着，静默约 0.9 s 就锁 0xD。
        """
        sim = FakeSim(events=[NO_KEYS, NO_KEYS, press(*QUIT_KEYS)])
        keeper = CountingKeeper()

        assert traj.wait_for_exit(sim, keeper) is True
        assert keeper.ticks == 2, "按到键之前每一拍都要喂帧"
        assert sim.pump_calls == 2

    def test_closing_the_window_ends_the_wait(self):
        """关窗口 = 结束等待（真机由 finally 失能）。"""
        sim = FakeSim(pumps=2)
        keeper = CountingKeeper()

        assert traj.wait_for_exit(sim, keeper) is False
        assert sim.pump_calls == 3, "第 3 次 pump 返回 False 才停"


# ══════════════════════════════════════════════════════════════════════════
# wait_for_start
# ══════════════════════════════════════════════════════════════════════════


class TestWaitForStart:
    def test_headless_replays_without_confirmation(self):
        """无头时直接回放：这条闸门存在的理由是「别在操作员手还扶着时自己动」，
        而无头没人扶着。"""
        sim = FakeSim(gui=False)
        keeper = CountingKeeper()

        assert traj.wait_for_start(sim, keeper) is True
        assert sim.keyboard_calls == 0
        assert sim.pump_calls == 0

    def test_confirm_key_starts_the_replay(self):
        # 第一拍是「结束录制」那一拍，`wait_for_start` 开头就把它丢掉；确认键在
        # 第二拍，也就是循环看到的第一拍。
        sim = FakeSim(events=[NO_KEYS, press(*CONFIRM_KEYS)])
        keeper = CountingKeeper()

        assert traj.wait_for_start(sim, keeper) is True
        assert sim.pump_calls == 0, "收到确认就该直接返回，不再推进"

    def test_quit_key_means_do_not_replay(self):
        sim = FakeSim(events=[NO_KEYS, press(*QUIT_KEYS)])
        keeper = CountingKeeper()

        assert traj.wait_for_start(sim, keeper) is False

    def test_the_confirm_keystroke_is_not_counted_twice(self):
        """结束录制那一拍的 Enter 不该又被当成「开始回放」。

        按一下 Enter 结束录制，紧接着这一拍还在队列里；`wait_for_start` 开头先丢掉
        一拍，否则「按一下 = 结束 + 开始」——真机在手还扶着的时候就动了。
        """
        sim = FakeSim(events=[press(*CONFIRM_KEYS), NO_KEYS], pumps=1)
        keeper = CountingKeeper()

        assert traj.wait_for_start(sim, keeper) is False, (
            "第一拍被丢弃后，队列里剩下的 Enter 不该再算一次确认")
