# -*- coding: utf-8 -*-
"""``examples/_common.py`` 里跟主机侧有关的那些部分——主要是 CAN 接口。

这一段是样例里唯一会碰**系统配置**的地方（要用 sudo 改接口），所以钉子要密：解析
的样本是 ``ip -details link show`` 的**真输出**，跑的命令逐条比对 argv，不动系统。
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"
if str(EXAMPLES) not in sys.path:
    sys.path.insert(0, str(EXAMPLES))

#: ``_common`` 缺依赖时会 execv 进仓库 .venv 重跑。在 pytest 里那会替掉测试进程，
#: 所以先把它按下去。
os.environ.setdefault("LITEGRIP_MUJOCO_REEXEC", "1")

import _common  # noqa: E402


# ── CAN 接口：探测与拉起 ────────────────────────────────────────────────
#
# 样本是 ``ip -details link show`` 的**真输出**：前四份照抄上位机
# litegrip-studio 的 selftest.py（从内核抓的原文，连 tab 缩进和行尾空格都是 ip
# 自己的），后面几份是 2026-10-06 在本机抓的。
IP_RAISED_CLASSIC = (
    "2: can0: <NOARP,UP,LOWER_UP> mtu 16 qdisc pfifo_fast state UP "
    "mode DEFAULT group default qlen 10\n"
    "    link/can  promiscuity 0 minmtu 0 maxmtu 0 \n"
    "\t  bitrate 1000000 sample-point 0.750 \n"
)
IP_RAISED_FD = (
    "2: can0: <NOARP,UP,LOWER_UP> mtu 72 qdisc pfifo_fast state UP "
    "mode DEFAULT group default qlen 10\n"
    "    link/can  promiscuity 0 minmtu 0 maxmtu 0 \n"
    "\t  bitrate 1000000 sample-point 0.750 \n"
    "\t  dbitrate 2000000 dsample-point 0.800 \n"
    "\t  fd on fd-non-iso off\n"
)
IP_UNPLUGGED = 'Device "can0" does not exist.\n'
#: ``lo``：管理上 up，operstate 却是 UNKNOWN。所以只能看 ``<>`` 里的 UP，
#: 不能看后面那个 ``state UNKNOWN``。
IP_NOT_CAN = (
    "1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN "
    "mode DEFAULT group default qlen 1000\n"
    "    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00\n"
)
#: 本机 ``can0``，2026-10-06 抓的：健康，但 ``restart-ms 0``（bus-off 不自愈）。
IP_HERE_HEALTHY = (
    "66: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP "
    "mode DEFAULT group default qlen 10\n"
    "    link/can  promiscuity 0 minmtu 0 maxmtu 0 \n"
    "    can state ERROR-ACTIVE restart-ms 0 \n"
    "\t  bitrate 1000000 sample-point 0.750 \n"
    "\t  tq 50 prop-seg 7 phase-seg1 7 phase-seg2 5 sjw 2\n"
)
#: 没配过就被 down 掉的接口。注意 ``<BERR-REPORTING>`` 夹在 ``can`` 和 ``state``
#: 中间——上位机那条 ``\bcan state (\S+)`` 在这里一个状态都读不到。
IP_DOWN = (
    "2: can0: <NOARP> mtu 16 qdisc noop state DOWN "
    "mode DEFAULT group default qlen 10\n"
    "    link/can  promiscuity 0 minmtu 0 maxmtu 0 \n"
    "    can <BERR-REPORTING> state STOPPED (berr-counter tx 0 rx 0) "
    "restart-ms 0 \n"
    "\t  bitrate 1000000 sample-point 0.750 \n"
)
#: BUS-OFF：**标志位和比特率全都正常**，但一帧都发不出去。真机上撞到的就是这个
#: 形状——「Network is down」听起来像接口没起来，其实接口看着好好的。
IP_BUS_OFF = (
    "66: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP "
    "mode DEFAULT group default qlen 10\n"
    "    link/can  promiscuity 0 minmtu 0 maxmtu 0 \n"
    "    can state BUS-OFF restart-ms 0 \n"
    "\t  bitrate 1000000 sample-point 0.750 \n"
)


def _ip(argv, rc: int, text: str = ""):
    """一条给替身 ``subprocess.run`` 的规定答案。"""
    return tuple(argv), (rc, text)


def _ip_runner(*replies):
    """``subprocess.run`` 的替身：只回答准备好的命令。

    没准备到的命令直接报错——「多跑了一条命令」正是这些测试要拦住的事，让它变成
    一次失败而不是一次静默通过。调用记录留在 ``run.calls`` 里，是
    ``(argv, kwargs)``。

    同一条命令写两遍（拉起前后各探测一次）按**顺序**依次回答，最后一个一直用下去。
    写成 ``dict`` 就会只剩最后一条——拉起前的探测直接读到「已经好了」，于是一条
    命令都不跑，测试还「通过」了。
    """
    answers: dict = {}
    for key, value in replies:
        answers.setdefault(key, []).append(value)
    calls = []

    def run(argv, **kwargs):
        calls.append((list(argv), kwargs))
        key = tuple(argv[1:] if argv and argv[0] == "sudo" else argv)
        if key not in answers:
            raise AssertionError(f"没准备这条命令：{argv}")
        queue = answers[key]
        rc, text = queue.pop(0) if len(queue) > 1 else queue[0]
        return SimpleNamespace(returncode=rc, stdout=text, stderr="")

    run.calls = calls
    return run


def _probe(channel="can0", text=IP_HERE_HEALTHY, rc=0):
    return _ip(["ip", "-details", "link", "show", channel], rc, text)


def _sudo(calls):
    """记录里的特权命令（argv 列表）。"""
    return [argv for argv, _ in calls if argv and argv[0] == "sudo"]


def _link_probe(text, channel="can0"):
    return _ip_runner(_probe(channel, text))


def _repair_ok(from_text=IP_DOWN):
    """从 ``from_text`` 这个状态开始、三条命令都成功的替身。"""
    return _ip_runner(
        _probe(text=from_text),
        _ip(["ip", "link", "set", "can0", "down"], 0),
        _ip(["ip", "link", "set", "can0", "type", "can", "bitrate",
             "1000000", "restart-ms", "100", "fd off"], 0),
        _ip(["ip", "link", "set", "can0", "up"], 0),
        _probe(text=IP_HERE_HEALTHY),
    )


def _interactive(monkeypatch, on: bool) -> None:
    """让 ``_common`` 看到一个（或看不到）交互式 stdin。

    换的是 ``_common`` 自己那份 ``sys``，不是真模块的，pytest 自己的 stdin 不受影响。
    """
    monkeypatch.setattr(_common, "sys", SimpleNamespace(
        stdin=SimpleNamespace(isatty=lambda: on)))


class TestReadingTheCanLink:
    """``ip -details link show`` 的输出长什么样，只有 ``parse_can_link`` 知道——
    所以每一份真输出都在这里钉一遍。"""

    def test_a_healthy_classic_interface_is_ready(self):
        state = _common.parse_can_link(IP_HERE_HEALTHY)
        assert (state.exists, state.up, state.is_can) == (True, True, True)
        assert state.bitrate == 1_000_000
        assert (state.fd, state.can_state) == (False, "ERROR-ACTIVE")
        assert state.ready(1_000_000)

    def test_a_bitrate_mismatch_is_not_ready(self):
        assert not _common.parse_can_link(IP_HERE_HEALTHY).ready(500_000)

    def test_a_missing_interface_is_not_an_interface(self):
        state = _common.parse_can_link(IP_UNPLUGGED, 1)
        assert not state.exists
        assert not state.ready(1_000_000)
        assert state.describe() == "不存在"

    def test_a_nonzero_exit_is_not_an_interface_either(self):
        assert not _common.parse_can_link("", 2).exists

    def test_loopback_is_not_a_can_interface(self):
        """``lo`` 是唯一会把两个坑一次踩全的那种接口：``state UNKNOWN`` 不能读成
        「没 up」（那就成了子串判断），而它没有 ``link/can``，也就不是 CAN。"""
        state = _common.parse_can_link(IP_NOT_CAN)
        assert state.up, "把 operstate 的 UNKNOWN 当成「没起来」了"
        assert not state.is_can
        assert state.describe() == "不是 CAN 接口"
        assert not state.ready(1_000_000)

    def test_can_fd_is_read_as_the_nominal_bitrate(self):
        """FD 的 ``dbitrate 2000000`` 不是标称比特率，``bitrate 1000000`` 才是。"""
        state = _common.parse_can_link(IP_RAISED_FD)
        assert state.bitrate == 1_000_000
        assert state.fd
        assert not state.ready(1_000_000), "FD 不该被判成「已经对了」"

    def test_a_down_interface_reads_its_controller_state(self):
        state = _common.parse_can_link(IP_DOWN)
        assert not state.up
        assert state.is_can
        # ``can <BERR-REPORTING> state STOPPED``：上位机那条 ``\bcan state`` 正则
        # 在这里读不到任何状态，读不到就分不出 STOPPED 和 BUS-OFF。
        assert state.can_state == "STOPPED"
        assert not state.ready(1_000_000)

    def test_a_down_interface_that_still_has_a_carrier_is_not_up(self):
        """``<NOARP,LOWER_UP>``：管理上没起来，载波却在——``LOWER_UP`` 里也有
        "UP"，所以这里不能做子串判断。判错了就会认为接口「已经好了」、什么都不做，
        然后第一帧发不出去。"""
        state = _common.parse_can_link(IP_DOWN.replace("<NOARP>", "<NOARP,LOWER_UP>"))
        assert not state.up
        assert not state.ready(1_000_000)

    def test_bus_off_is_the_state_that_lies(self):
        """**这次修复的关键一条。** BUS-OFF 的接口标志位是 ``UP,LOWER_UP``、比特率
        也是对的，光看标志位会判成「已经对了」、一条命令都不跑——然后第一帧发不出
        去，报出来的还是「夹爪可能未上电」。"""
        state = _common.parse_can_link(IP_BUS_OFF)
        assert state.up and state.bitrate == 1_000_000, "样本没构造对"
        assert state.deaf
        assert not state.ready(1_000_000)
        assert "BUS-OFF" in state.describe()

    def test_error_passive_is_reported_but_not_repaired(self):
        """总线边际时控制器会掉进 ERROR-PASSIVE，它自己会恢复：报告，但不改。"""
        text = IP_HERE_HEALTHY.replace("ERROR-ACTIVE", "ERROR-PASSIVE")
        state = _common.parse_can_link(text)
        assert state.ready(1_000_000)
        assert "ERROR-PASSIVE" in state.describe()

    def test_a_healthy_state_says_nothing_extra(self):
        assert _common.parse_can_link(IP_HERE_HEALTHY).describe() == (
            "已 up，经典 CAN，比特率 1000000")


class TestProbingTheCanLink:
    def test_it_asks_ip_and_nothing_else(self):
        probe = _link_probe(IP_HERE_HEALTHY)
        assert _common.probe_can_link("can0", run=probe).ready(1_000_000)
        assert [argv for argv, _ in probe.calls] == [
            ["ip", "-details", "link", "show", "can0"]]

    def test_it_pins_the_locale(self):
        """后面要按 ``does not exist`` 这种英文串分支，别让 locale 改掉它。"""
        probe = _link_probe(IP_HERE_HEALTHY)
        _common.probe_can_link("can0", run=probe)
        assert probe.calls[0][1]["env"]["LC_ALL"] == "C"

    def test_a_name_that_is_not_a_device_name_is_never_run(self):
        for bad in ("can0; rm -rf /", "", "$(whoami)", "can 0", "x" * 20):
            probe = _link_probe(IP_HERE_HEALTHY)
            assert _common.probe_can_link(bad, run=probe) is None
            assert probe.calls == [], f"{bad!r} 竟然进了命令"

    def test_a_probe_that_cannot_run_is_not_a_verdict(self):
        """没有 ``ip``、命令超时：返回 ``None``（没结论），不是「接口有病」。"""
        def missing(argv, **kwargs):
            raise FileNotFoundError("ip")
        assert _common.probe_can_link("can0", run=missing) is None


class TestBringingTheCanLinkUp:
    """照上位机的三条规则：便利不是闸门、只改真正不对的状态、失败时说清楚。"""

    def test_a_ready_interface_runs_no_command_at_all(self, monkeypatch):
        """「少改」的钉子：接口已经对了，就一条特权命令都不跑，**也不弹密码**。"""
        _interactive(monkeypatch, True)
        probe = _link_probe(IP_HERE_HEALTHY)
        assert _common.ensure_can_link("can0", run=probe) is True
        assert _sudo(probe.calls) == []
        assert len(probe.calls) == 1, "只该探测一次"

    def test_a_down_interface_goes_down_configure_up(self, monkeypatch):
        """三条命令、按这个顺序，多一条都没有。"""
        _interactive(monkeypatch, True)
        runner = _repair_ok()
        assert _common.ensure_can_link("can0", run=runner) is True
        assert _sudo(runner.calls) == [
            ["sudo", "ip", "link", "set", "can0", "down"],
            ["sudo", "ip", "link", "set", "can0", "type", "can", "bitrate",
             "1000000", "restart-ms", "100", "fd off"],
            ["sudo", "ip", "link", "set", "can0", "up"],
        ]

    def test_it_never_goes_through_a_shell(self, monkeypatch):
        """命令是 list argv，接口名是**其中一个参数**——拼成一行再交给 shell，名字里
        有任何东西都会被解释。（``fd off`` 那个空格是 ``ip`` 自己的选项写法，必须连着
        写成一个参数。）"""
        _interactive(monkeypatch, True)
        runner = _repair_ok()
        _common.ensure_can_link("can0", run=runner)
        for argv, kwargs in runner.calls:
            assert isinstance(argv, list), f"命令不是 list argv：{argv}"
            assert "shell" not in kwargs, f"{argv} 走了 shell"
            assert not any(" " in a for a in argv if a != "fd off"), argv

    def test_an_fd_interface_is_reported_and_left_alone(self, monkeypatch):
        _interactive(monkeypatch, True)
        lines = []
        probe = _link_probe(IP_RAISED_FD)
        assert _common.ensure_can_link("can0", run=probe, out=lines.append) is False
        assert _sudo(probe.calls) == []
        assert "CAN FD" in "\n".join(lines)

    def test_a_non_can_interface_is_left_alone(self, monkeypatch):
        """``--channel lo`` 也不会被 down/up——那是参数打错了，不是接口不对。"""
        _interactive(monkeypatch, True)
        lines = []
        probe = _link_probe(IP_NOT_CAN, "lo")
        assert _common.ensure_can_link("lo", run=probe, out=lines.append) is False
        assert _sudo(probe.calls) == []
        assert "不是 CAN 接口" in "\n".join(lines)

    def test_a_missing_device_never_reaches_a_privileged_command(self, monkeypatch):
        _interactive(monkeypatch, True)
        lines = []
        probe = _ip_runner(_probe("can0", IP_UNPLUGGED, 1))
        assert _common.ensure_can_link("can0", run=probe, out=lines.append) is False
        assert _sudo(probe.calls) == []
        assert "读不到 can0" in "\n".join(lines)

    def test_a_restart_ms_rejection_retries_only_that_command(self, monkeypatch):
        """台架上这块 gs_usb 克隆不认 ``restart-ms``。原样重试 configure 会把接口留
        在 down（比原来更糟），所以只去掉这一项、只重试这一条，``up`` 照跑。"""
        _interactive(monkeypatch, True)
        runner = _ip_runner(
            _probe(text=IP_DOWN),
            _ip(["ip", "link", "set", "can0", "down"], 0),
            _ip(["ip", "link", "set", "can0", "type", "can", "bitrate",
                 "1000000", "restart-ms", "100", "fd off"], 1,
                "Error: argument \"restart-ms\" is wrong\n"),
            _ip(["ip", "link", "set", "can0", "type", "can", "bitrate",
                 "1000000", "fd off"], 0),
            _ip(["ip", "link", "set", "can0", "up"], 0),
            _probe(text=IP_HERE_HEALTHY),
        )
        assert _common.ensure_can_link("can0", run=runner, out=lambda *_: None)
        sudo = _sudo(runner.calls)
        assert "restart-ms" not in sudo[-2], sudo[-2]
        assert sudo[-1] == ["sudo", "ip", "link", "set", "can0", "up"]

    def test_any_other_failure_is_not_retried(self, monkeypatch):
        """只有名字里有 restart 的失败才重试。别的失败一并重试，会把偶发错误变成
        静默降级——接口是起来了，可再也不自动从 bus-off 恢复。"""
        _interactive(monkeypatch, True)
        lines = []
        runner = _ip_runner(
            _probe(text=IP_DOWN),
            _ip(["ip", "link", "set", "can0", "down"], 0),
            _ip(["ip", "link", "set", "can0", "type", "can", "bitrate",
                 "1000000", "restart-ms", "100", "fd off"], 1,
                "RTNETLINK answers: Operation not supported\n"),
        )
        assert _common.ensure_can_link("can0", run=runner,
                                       out=lines.append) is False
        assert len(_sudo(runner.calls)) == 2, "失败之后还往下跑了"
        assert "sudo ip link set can0 up" in "\n".join(lines)

    def test_a_repair_that_did_not_work_says_so(self, monkeypatch):
        """三条命令都成功、复查却还是不对：不能说「已就绪」。"""
        _interactive(monkeypatch, True)
        lines = []
        runner = _ip_runner(
            _probe(text=IP_BUS_OFF),
            _ip(["ip", "link", "set", "can0", "down"], 0),
            _ip(["ip", "link", "set", "can0", "type", "can", "bitrate",
                 "1000000", "restart-ms", "100", "fd off"], 0),
            _ip(["ip", "link", "set", "can0", "up"], 0),
            _probe(text=IP_BUS_OFF),
        )
        assert _common.ensure_can_link("can0", run=runner, out=lines.append) is False
        assert "复查仍然不对" in "\n".join(lines)

    def test_bus_off_is_named_before_the_repair(self, monkeypatch):
        _interactive(monkeypatch, True)
        lines = []
        runner = _repair_ok(from_text=IP_BUS_OFF)
        _common.ensure_can_link("can0", run=runner, out=lines.append)
        assert "BUS-OFF" in "\n".join(lines), "没告诉操作员这一帧都发不出去是为什么"

    def test_repair_false_only_probes(self, monkeypatch):
        """05 的 ``--status`` / 04 的 ``--passive`` 走这条：只看不动。替它们把接口
        改掉，恰好把 ``--status`` 要诊断的东西抹了。"""
        _interactive(monkeypatch, True)
        lines = []
        probe = _link_probe(IP_DOWN)
        assert _common.ensure_can_link("can0", repair=False, run=probe,
                                       out=lines.append) is False
        assert _sudo(probe.calls) == []
        assert "sudo ip link set can0 down" in "\n".join(lines)

    def test_a_non_interactive_stdin_only_prints(self, monkeypatch):
        """CI / 管道里 sudo 要不到密码，会一直挂着。"""
        _interactive(monkeypatch, False)
        lines = []
        probe = _link_probe(IP_DOWN)
        assert _common.ensure_can_link("can0", run=probe,
                                       out=lines.append) is False
        assert _sudo(probe.calls) == []
        assert "非交互" in "\n".join(lines)

    def test_without_sudo_it_prints_the_command(self, monkeypatch):
        _interactive(monkeypatch, True)
        lines = []
        probe = _link_probe(IP_DOWN)
        assert _common.ensure_can_link("can0", run=probe, which=lambda _: None,
                                       out=lines.append) is False
        assert _sudo(probe.calls) == []
        assert "sudo ip link set can0 up" in "\n".join(lines)

    def test_the_printed_commands_are_the_ones_it_would_have_run(self, monkeypatch):
        """打印出来备用的命令，要和真跑的那三条一模一样——否则操作员照着敲，敲出来
        的是另一件事。"""
        hint = _common.manual_can_hint("can0")
        _interactive(monkeypatch, True)
        runner = _repair_ok()
        _common.ensure_can_link("can0", run=runner)
        for argv in _sudo(runner.calls):
            assert " ".join(argv) in hint, f"{argv} 不在打印出来的命令里"


class TestDiagnosingAFailedEnable:
    """真机上撞到的那个错：``使能失败: [Errno 100] Network is down`` 被译成
    「夹爪可能处于错误状态或未上电」。它不是夹爪的问题。"""

    def test_errno_100_is_the_host_link_not_the_gripper(self):
        probe = _link_probe(IP_DOWN)
        text = _common.can_link_failure(
            OSError(100, "Network is down"), "can0", run=probe)
        assert "不是夹爪" in text
        assert "sudo ip link set can0 down" in text
        assert "未 up" in text

    def test_a_bus_off_interface_is_named_in_the_diagnosis(self):
        probe = _link_probe(IP_BUS_OFF)
        text = _common.can_link_failure(
            OSError(100, "Network is down"), "can0", run=probe)
        assert "BUS-OFF" in text

    def test_other_errors_are_left_alone(self):
        """不是链路错就别抢话：``None`` 让调用方照原样报。"""
        for exc in (ValueError("boom"), OSError(13, "Permission denied"),
                    RuntimeError("使能超时")):
            assert _common.can_link_failure(exc, "can0") is None

    def test_the_connect_message_does_not_blame_the_link_state(self):
        """``connect()`` 只建 socket 和 bind，这两步在没 up 的接口上也成功——所以
        这里的检查项里不该出现「接口没起来」。"""
        text = _common.connect_failure_message("can0")
        assert "bind" in text
        assert "ip -details link show can0" in text

    def test_a_bad_link_is_named_before_the_gripper(self):
        probe = _link_probe(IP_BUS_OFF)
        text = _common.enable_failure_message("can0", run=probe)
        assert "先修链路" in text
        assert "上电" not in text

    def test_a_good_link_keeps_the_gripper_wording(self):
        probe = _link_probe(IP_HERE_HEALTHY)
        text = _common.enable_failure_message("can0", run=probe)
        assert "夹爪可能处于错误状态或未上电" in text
        assert "已 up" in text, "没说清接口这时是什么状态"


# ── 连接路径：探测必须在建 LiteGrip 之前，且 repair 跟着 enable 走 ──────


class _FakeRealGripper:
    """真机夹爪的替身：只记下自己被怎么用了。"""

    def __init__(self, events):
        self.events = events
        self.config = SimpleNamespace(pos_closed_rad=0.114, pos_open_rad=-1.731,
                                      rad_to_mm=65.4, kp=1.0, kd=0.1)

    def connect(self):
        self.events.append("connect")
        return True

    def enable(self):
        self.events.append("enable")
        return True

    def disconnect(self):
        self.events.append("disconnect")

    def load_calibration(self, *_args, **_kwargs):
        return True


def _hardware_args(**overrides):
    parser = argparse.ArgumentParser()
    _common.add_hardware_args(parser)
    args = parser.parse_args([])
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


def _fake_sdk(events):
    def LiteGrip(**_kwargs):        # noqa: N802 — 这是 SDK 的名字
        events.append("LiteGrip")
        return _FakeRealGripper(events)

    return SimpleNamespace(LiteGrip=LiteGrip)


@pytest.fixture
def connected_events(monkeypatch):
    """把 ``open_real_gripper`` 里所有会碰真机/系统的接缝都换掉。

    只留一个**顺序**：探测 → 建 LiteGrip。顺序错了，接口不对时就已经先开了 socket。
    """
    events: list = []
    monkeypatch.setattr(_common, "choose_calibration_file",
                        lambda *a, **k: (None, False))
    monkeypatch.setattr(_common, "import_litegrip",
                        lambda: _fake_sdk(events))
    monkeypatch.setattr(_common, "check_sdk_api", lambda sdk: None)
    import litegrip_mujoco
    monkeypatch.setattr(litegrip_mujoco, "apply_calibration",
                        lambda *a, **k: _FakeRealGripper(events).config,
                        raising=False)
    return events


class TestOpenRealGripperProbesTheCanLink:
    """``repair=enable`` 和 ``--no-can-setup`` 这两处映射只有在这里才看得出来。"""

    def _record_probe(self, monkeypatch, events, state=None):
        state = state if state is not None else _common.parse_can_link(
            IP_HERE_HEALTHY)

        def probe(channel, **_kwargs):
            events.append("probe")
            return state

        monkeypatch.setattr(_common, "probe_can_link", probe)

    def test_the_probe_comes_before_the_gripper_is_built(
            self, monkeypatch, connected_events):
        self._record_probe(monkeypatch, connected_events)
        _common.open_real_gripper(_hardware_args(), enable=True)
        assert connected_events.index("probe") < connected_events.index("LiteGrip")

    def test_status_and_passive_never_repair_the_interface(
            self, monkeypatch, connected_events):
        """``enable=False``（05 的 ``--status``、04 的 ``--passive``）只读不修：把
        接口改掉，恰好把这两条路径要诊断的东西抹了。"""
        self._record_probe(monkeypatch, connected_events)
        repaired = []
        monkeypatch.setattr(_common, "ensure_can_link",
                            lambda channel, **kw: repaired.append(kw) or True)
        _common.open_real_gripper(_hardware_args(), enable=False)
        assert repaired == [{"repair": False}]

    def test_an_enabled_run_repairs_what_is_wrong(self, monkeypatch,
                                                  connected_events):
        self._record_probe(monkeypatch, connected_events)
        repaired = []
        monkeypatch.setattr(_common, "ensure_can_link",
                            lambda channel, **kw: repaired.append(kw) or True)
        _common.open_real_gripper(_hardware_args(), enable=True)
        assert repaired == [{"repair": True}]

    def test_no_can_setup_never_looks_at_the_interface(
            self, monkeypatch, connected_events):
        """逃生口：接口自己管，那就一条命令都不跑、也不弹密码。"""
        called = []
        monkeypatch.setattr(_common, "ensure_can_link",
                            lambda *a, **k: called.append(a) or True)
        monkeypatch.setattr(_common, "probe_can_link",
                            lambda *a, **k: called.append(a))
        _common.open_real_gripper(_hardware_args(no_can_setup=True), enable=True)
        assert called == []

    def test_dry_run_never_touches_the_interface(self, monkeypatch,
                                                 connected_events):
        """``--dry-run`` 的定义就是「不碰 CAN」，所以它也不该去探测接口。"""
        called = []
        monkeypatch.setattr(_common, "ensure_can_link",
                            lambda *a, **k: called.append(a))
        import litegrip_mujoco
        monkeypatch.setattr(litegrip_mujoco, "DryRunGripper",
                            lambda **k: _FakeRealGripper(connected_events),
                            raising=False)
        _common.open_real_gripper(_hardware_args(), dry_run=True)
        assert called == []


class TestArgParsers:
    def test_common_args(self):
        parser = argparse.ArgumentParser()
        _common.add_common_args(parser)
        args = parser.parse_args([])
        assert args.headless is False
        assert parser.parse_args(["--headless"]).headless
        assert parser.parse_args(["--no-render"]).headless, "旧名得还能用"

    def test_hardware_args_defaults(self):
        parser = argparse.ArgumentParser()
        _common.add_hardware_args(parser)
        args = parser.parse_args([])
        assert args.channel == "can0"
        assert args.can_id == 0x08
        assert args.mst_id == 0x18
        # ``None`` 表示「命令行没给」。它不是路径：``choose_calibration_file``
        # 会先把它换成 SDK 的出厂标定，出厂那份也读不出来才去问。
        assert args.calib is None
        assert args.no_can_setup is False

    def test_the_can_setup_can_be_turned_off(self):
        """逃生口：接口自己管。名字里带 ``no``，默认必须是「会探测」。"""
        parser = argparse.ArgumentParser()
        _common.add_hardware_args(parser)
        assert parser.parse_args([]).no_can_setup is False
        assert parser.parse_args(["--no-can-setup"]).no_can_setup is True

    def test_the_calibration_aliases_still_both_work(self):
        parser = argparse.ArgumentParser()
        _common.add_hardware_args(parser)
        assert parser.parse_args(["--calib", "x.json"]).calib == "x.json"
        assert parser.parse_args(["--calibration", "x.json"]).calib == "x.json"

    def test_hardware_args_accept_hex_and_decimal(self):
        parser = argparse.ArgumentParser()
        _common.add_hardware_args(parser)
        assert parser.parse_args(["--can-id", "0x0A"]).can_id == 10
        assert parser.parse_args(["--can-id", "10"]).can_id == 10
        assert parser.parse_args(["--mst-id", "0x20"]).mst_id == 0x20

    def test_safety_banner_warns_about_real_motion(self):
        assert "真机" in _common.SAFETY_BANNER
        assert "Esc" in _common.SAFETY_BANNER
