"""标定选择与来源校验的测试。

不给 ``--calibration`` 时用 SDK 包里那份出厂标定；换标定只有一个办法：显式给出
文件的路径。这一组用例钉的就是这两档，以及它周围那些**看起来能省事、实则会动错
真机**的捷径——尤其是「悄悄扫盘挑一份」。

全部是纯仿真的：不需要 CAN、不需要 `litegrip` SDK、不需要 TTY。需要真机的路径
用 `FakeReal`（一个记录调用顺序的鸭子类型替身）覆盖。
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time

import pytest

from litegrip_mujoco import constants as C
from litegrip_mujoco import calibration as cal
from litegrip_mujoco._litegrip import GripperConfig

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLES = os.path.join(REPO_ROOT, "examples")


# ══════════════════════════════════════════════════════════════════════════
# 工具
# ══════════════════════════════════════════════════════════════════════════


def write_cal(directory, name="cal.json", closed=1.775959, open_=-0.064279,
              rad_to_mm=None, **extra):
    """写一份合法的标定文件，返回绝对路径。"""
    if rad_to_mm is None:
        rad_to_mm = C.MM_SCALE / (closed - open_)
    data = {
        "zero_position_rad": closed,
        "max_position_rad": open_,
        "rad_to_mm": rad_to_mm,
    }
    data.update(extra)
    path = os.path.join(str(directory), name)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(data, handle)
    return path


def make_home(tmp_path, monkeypatch):
    """把 HOME 指到 tmp_path，并清掉可能存在的 LITEGRIP_CALIB。"""
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.delenv("LITEGRIP_CALIB", raising=False)
    monkeypatch.chdir(tmp_path)
    return tmp_path


class FakeReal:
    """`litegrip.LiteGrip` 的鸭子类型替身，记录调用顺序。

    它**不是**仿真设备（``IS_SIMULATED = False``），所以会走真机那一整套强制。
    """

    IS_SIMULATED = False

    def __init__(self, closed=1.775959, open_=-0.064279, accept=True,
                 load_values=None):
        self.config = GripperConfig(
            pos_closed_rad=closed,
            pos_open_rad=open_,
            max_stroke_mm=C.MM_SCALE,
        )
        self.calls = []
        self.is_connected = False
        self.is_enabled = False
        self._position_rad = closed
        self._accept = accept
        self._load_values = load_values

    # ── 生命周期 ──
    def connect(self):
        self.calls.append("connect")
        self.is_connected = True
        return True

    def disconnect(self):
        self.calls.append("disconnect")
        self.is_connected = False

    def enable(self):
        self.calls.append("enable")
        self.is_enabled = True
        return True

    def disable(self):
        self.calls.append("disable")
        self.is_enabled = False
        return True

    # ── 标定 ──
    def load_calibration(self, path=None):
        self.calls.append(("load_calibration", path))
        if not self._accept:
            return False
        if self._load_values is not None:
            # 模拟 SDK 的静默回退：装作读了 path，实际写进别的数
            self.config.pos_closed_rad = self._load_values[0]
            self.config.pos_open_rad = self._load_values[1]
            return True
        with open(path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
        self.config.pos_closed_rad = float(data["zero_position_rad"])
        self.config.pos_open_rad = float(data["max_position_rad"])
        return True

    # ── 运动与状态 ──
    def open(self, **kwargs):
        self.calls.append("open")
        self._position_rad = self.config.pos_open_rad
        return True

    def close(self, **kwargs):
        self.calls.append("close")
        self._position_rad = self.config.pos_closed_rad
        return True

    def grasp(self, **kwargs):
        self.calls.append("grasp")
        return True

    def goto(self, position_mm, **kwargs):
        self.calls.append(("goto", position_mm))
        return True

    def goto_rad(self, position_rad, **kwargs):
        self.calls.append(("goto_rad", position_rad))
        self._position_rad = float(position_rad)
        return True

    def get_position_rad(self):
        return self._position_rad

    def get_position(self):
        return 0.0

    def get_state(self):
        raise NotImplementedError

    def stop(self):
        self.calls.append("stop")
        return True


class NoLoader(FakeReal):
    """没有 ``load_calibration()`` 的设备。"""

    load_calibration = None


# ══════════════════════════════════════════════════════════════════════════
# 候选发现
# ══════════════════════════════════════════════════════════════════════════


class TestDiscovery:
    def test_scans_home_and_cwd(self, tmp_path, monkeypatch):
        """~/.litegrip 与当前目录都会被扫到，且 ~/.litegrip 排在前面。"""
        home = make_home(tmp_path, monkeypatch)
        (home / ".litegrip").mkdir()
        home_cal = write_cal(home / ".litegrip", "a.json")
        cwd = tmp_path / "work"
        cwd.mkdir()
        monkeypatch.chdir(cwd)
        cwd_cal = write_cal(cwd, "b.json")

        found = cal.discover_calibrations()
        assert found == [home_cal, cwd_cal]

    def test_dedupes_by_realpath(self, tmp_path, monkeypatch):
        """同一份文件经由符号链接出现两次时只列一次。"""
        home = make_home(tmp_path, monkeypatch)
        (home / ".litegrip").mkdir()
        real = write_cal(home / ".litegrip", "a.json")
        link = tmp_path / "link.json"
        try:
            os.symlink(real, link)
        except (OSError, NotImplementedError):  # pragma: no cover - 平台不支持
            pytest.skip("这个平台不支持符号链接")

        found = cal.discover_calibrations()
        assert found.count(real) == 1
        assert str(link) not in found

    def test_explicit_first(self, tmp_path, monkeypatch):
        """显式给的路径排在最前，哪怕它不在任何扫描目录里。"""
        home = make_home(tmp_path, monkeypatch)
        (home / ".litegrip").mkdir()
        write_cal(home / ".litegrip", "a.json")
        elsewhere = tmp_path / "elsewhere"
        elsewhere.mkdir()
        explicit = write_cal(elsewhere, "mine.json")

        found = cal.discover_calibrations(explicit)
        assert found[0] == explicit

    def test_explicit_missing_raises(self, tmp_path, monkeypatch):
        make_home(tmp_path, monkeypatch)
        with pytest.raises(cal.CalibrationFileError, match="不存在"):
            cal.discover_calibrations(str(tmp_path / "nope.json"))

    def test_ignores_non_json_and_dirs(self, tmp_path, monkeypatch):
        """`.bak`、非 json 文件与子目录都不算候选。"""
        home = make_home(tmp_path, monkeypatch)
        (home / ".litegrip").mkdir()
        good = write_cal(home / ".litegrip", "a.json")
        (home / ".litegrip" / "a.json.20260928.bak").write_text("{}")
        (home / ".litegrip" / "notes.txt").write_text("hi")
        (home / ".litegrip" / "subdir").mkdir()

        assert cal.discover_calibrations() == [good]

    def test_missing_home_directory_is_not_an_error(self, tmp_path, monkeypatch):
        """~/.litegrip 不存在时只扫当前目录，不抛异常（CI 与 --dry-run 都靠这条）。"""
        home = make_home(tmp_path, monkeypatch)
        assert not (home / ".litegrip").exists()
        cwd_cal = write_cal(home, "here.json")
        assert cal.discover_calibrations() == [cwd_cal]

    def test_excludes_sdk_factory_file(self, tmp_path, monkeypatch):
        """SDK 自带的出厂标定不出现在候选里。"""
        home = make_home(tmp_path, monkeypatch)
        factory = write_cal(home, "factory_calibration.json")
        monkeypatch.setattr(cal, "sdk_factory_calibration_path", lambda: factory)
        other = write_cal(home, "mine.json")

        found = cal.discover_calibrations()
        assert factory not in found
        assert other in found

    def test_newest_first_within_a_directory(self, tmp_path, monkeypatch):
        """同一个目录里按修改时间倒序 —— 上位机刚写的那份应该排最前。"""
        home = make_home(tmp_path, monkeypatch)
        old = write_cal(home, "old.json")
        new = write_cal(home, "new.json")
        os.utime(old, (time.time() - 3600, time.time() - 3600))
        os.utime(new, (time.time(), time.time()))

        assert cal.discover_calibrations() == [new, old]


# ══════════════════════════════════════════════════════════════════════════
# 文件校验
# ══════════════════════════════════════════════════════════════════════════


class TestLoadFile:
    def test_reads_endpoints_and_optional_fields(self, tmp_path):
        path = write_cal(tmp_path, "c.json", can_id=0x09, channel="can1", kp=120.0)
        loaded = cal.load_calibration_file(path)
        assert loaded.pos_closed_rad == pytest.approx(1.775959)
        assert loaded.pos_open_rad == pytest.approx(-0.064279)
        assert loaded.span_rad == pytest.approx(1.840238)
        assert loaded.travel_mm == pytest.approx(C.MM_SCALE, abs=0.05)
        assert loaded.raw["can_id"] == 0x09
        assert loaded.raw["channel"] == "can1"
        assert loaded.path == os.path.abspath(path)

    @pytest.mark.parametrize("missing", ["zero_position_rad", "max_position_rad", "rad_to_mm"])
    def test_rejects_missing_required_key(self, tmp_path, missing):
        """缺字段必须在 SDK 之前拦下 —— SDK 读 rad_to_mm 时会直接 KeyError。"""
        data = {
            "zero_position_rad": 1.0,
            "max_position_rad": -0.5,
            "rad_to_mm": 60.0,
        }
        del data[missing]
        path = tmp_path / "missing.json"
        path.write_text(json.dumps(data))
        with pytest.raises(cal.CalibrationFileError, match=missing):
            cal.load_calibration_file(str(path))

    def test_rejects_inverted_endpoints(self, tmp_path):
        """SDK 默认值那副形状（closed <= open）就是未标定。"""
        path = write_cal(tmp_path, "inverted.json", closed=0.0, open_=1.14,
                         rad_to_mm=105.26)
        with pytest.raises(cal.CalibrationFileError, match="不变量"):
            cal.load_calibration_file(path)

    def test_rejects_equal_endpoints(self, tmp_path):
        path = write_cal(tmp_path, "flat.json", closed=0.5, open_=0.5,
                         rad_to_mm=60.0)
        with pytest.raises(cal.CalibrationFileError):
            cal.load_calibration_file(path)

    def test_rejects_tiny_span(self, tmp_path):
        path = write_cal(tmp_path, "tiny.json", closed=1.0, open_=1.0 - 1e-9)
        with pytest.raises(cal.CalibrationFileError, match="量程"):
            cal.load_calibration_file(path)

    @pytest.mark.parametrize("value", [0.0, -3.0])
    def test_rejects_non_positive_rad_to_mm(self, tmp_path, value):
        path = write_cal(tmp_path, "zero.json", rad_to_mm=value)
        with pytest.raises(cal.CalibrationFileError, match="rad_to_mm"):
            cal.load_calibration_file(path)

    @pytest.mark.parametrize("value", ["abc", None, [1.0]])
    def test_rejects_non_numeric_field(self, tmp_path, value):
        data = {"zero_position_rad": value, "max_position_rad": -0.5,
                "rad_to_mm": 60.0}
        path = tmp_path / "bad.json"
        path.write_text(json.dumps(data))
        with pytest.raises(cal.CalibrationFileError, match="不是数字"):
            cal.load_calibration_file(str(path))

    def test_rejects_non_finite_field(self, tmp_path):
        path = tmp_path / "nan.json"
        path.write_text('{"zero_position_rad": NaN, "max_position_rad": -0.5,'
                        ' "rad_to_mm": 60.0}')
        with pytest.raises(cal.CalibrationFileError, match="有限数"):
            cal.load_calibration_file(str(path))

    def test_rejects_bad_json(self, tmp_path):
        path = tmp_path / "broken.json"
        path.write_text("{not json")
        with pytest.raises(cal.CalibrationFileError, match="合法 JSON"):
            cal.load_calibration_file(str(path))

    def test_rejects_non_object_toplevel(self, tmp_path):
        path = tmp_path / "list.json"
        path.write_text("[1, 2, 3]")
        with pytest.raises(cal.CalibrationFileError, match="JSON 对象"):
            cal.load_calibration_file(str(path))

    def test_rejects_missing_file_and_directory(self, tmp_path):
        with pytest.raises(cal.CalibrationFileError, match="不存在"):
            cal.load_calibration_file(str(tmp_path / "nope.json"))
        with pytest.raises(cal.CalibrationFileError, match="目录"):
            cal.load_calibration_file(str(tmp_path))

    def test_rejects_empty_path(self):
        with pytest.raises(cal.CalibrationFileError):
            cal.load_calibration_file(None)
        with pytest.raises(cal.CalibrationFileError):
            cal.load_calibration_file("   ")

    def test_describe_candidate_reports_the_reason(self, tmp_path):
        """不可用的候选也要能被描述出来，而不是被悄悄过滤掉。"""
        good = write_cal(tmp_path, "good.json")
        bad = tmp_path / "bad.json"
        bad.write_text("{}")

        assert "travel" in cal.describe_candidate(good)
        described = cal.describe_candidate(str(bad))
        assert "不可用" in described
        assert "zero_position_rad" in described

    def test_describe_candidate_flags_the_sdk_default_path(self, tmp_path, monkeypatch):
        home = make_home(tmp_path, monkeypatch)
        (home / ".litegrip").mkdir()
        path = write_cal(home / ".litegrip", "litegrip_calibration.json")
        described = cal.describe_candidate(path)
        assert "SDK 默认用户路径" in described
        assert "不会被自动选中" in described   # 一句话说清它只是候选，不是默认值


# ══════════════════════════════════════════════════════════════════════════
# 选择：两档
# ══════════════════════════════════════════════════════════════════════════


def fake_factory(tmp_path, monkeypatch, **fields):
    """造一份「SDK 自带的出厂标定」并把库层的探测指到它。

    真实的探测会去看 ``sys.path`` 上的 ``litegrip``；跑测试的机器上可能真装着一
    份（这台 bench 上就有），用例的结论不该因此改变。
    """
    path = write_cal(tmp_path, "factory_calibration.json", **fields)
    monkeypatch.setattr(cal, "sdk_factory_calibration_path", lambda: path)
    return path


class TestResolve:
    """``resolve_calibration_path`` 的两档，顺序就是优先级。"""

    def test_explicit_wins_and_is_validated(self, tmp_path, monkeypatch):
        make_home(tmp_path, monkeypatch)
        factory = fake_factory(tmp_path, monkeypatch)
        path = write_cal(tmp_path, "mine.json")
        chosen = cal.resolve_calibration_path(path)
        assert chosen == path
        assert chosen != factory

    def test_explicit_invalid_raises_immediately(self, tmp_path, monkeypatch):
        make_home(tmp_path, monkeypatch)
        fake_factory(tmp_path, monkeypatch)     # 有出厂标定也不许拿它顶替
        bad = tmp_path / "bad.json"
        bad.write_text("{}")
        with pytest.raises(cal.CalibrationFileError):
            cal.resolve_calibration_path(str(bad))

    def test_no_path_uses_the_sdk_factory_calibration(self, tmp_path, monkeypatch):
        """不给路径时的默认值：SDK 包里那份出厂标定。

        它跟着包目录解析，所以一台新机器不用先去找标定文件就能跑。
        """
        make_home(tmp_path, monkeypatch)
        factory = fake_factory(tmp_path, monkeypatch)
        assert cal.resolve_calibration_path(None) == factory

    def test_never_uses_the_default_user_path(self, tmp_path, monkeypatch):
        """``~/.litegrip/litegrip_calibration.json`` 上放着一份**可用**的标定，
        也不许自己用 —— 那份是 SDK 的默认用户路径，不是这台夹爪的证明。"""
        home = make_home(tmp_path, monkeypatch)
        (home / ".litegrip").mkdir()
        write_cal(home / ".litegrip", "litegrip_calibration.json")
        factory = fake_factory(tmp_path, monkeypatch)

        assert cal.resolve_calibration_path(None) == factory

    def test_no_factory_file_raises_and_names_the_flag(self, tmp_path, monkeypatch):
        """出厂文件都找不到 ⇒ 停下，并说清用哪个开关换一份。

        以前这里是「列出候选让操作员选」；现在不扫盘也不提问——猜哪份 JSON 是哪
        台夹爪的，不是这一层能做的事。
        """
        make_home(tmp_path, monkeypatch)
        monkeypatch.setattr(cal, "sdk_factory_calibration_path", lambda: None)

        with pytest.raises(cal.CalibrationRequiredError) as info:
            cal.resolve_calibration_path(None)
        message = str(info.value)
        assert "--calibration" in message
        assert "--list-calibrations" in message, "没说怎么去找候选"

    def test_an_unreadable_factory_file_raises(self, tmp_path, monkeypatch):
        """出厂文件在那儿但读不出来（SDK 装得残缺）也是同一档：停下。"""
        home = make_home(tmp_path, monkeypatch)
        (home / ".litegrip").mkdir()
        write_cal(home / ".litegrip", "litegrip_calibration.json")   # 有候选也不许用
        broken = tmp_path / "factory_calibration.json"
        broken.write_text("{not json")
        monkeypatch.setattr(cal, "sdk_factory_calibration_path", lambda: str(broken))

        with pytest.raises(cal.CalibrationRequiredError) as info:
            cal.resolve_calibration_path(None)
        assert "--calibration" in str(info.value)


# ══════════════════════════════════════════════════════════════════════════
# 套用与校验
# ══════════════════════════════════════════════════════════════════════════


class TestApply:
    def test_applies_and_records_provenance(self, tmp_path):
        path = write_cal(tmp_path, "c.json")
        device = FakeReal()
        applied = cal.apply_calibration(device, path)

        assert applied.path == os.path.abspath(path)
        assert device.config.pos_closed_rad == pytest.approx(1.775959)
        assert [c for c in device.calls if isinstance(c, tuple)][0][0] == "load_calibration"
        assert cal.applied_calibration(device) == applied
        assert cal.is_calibrated(device) is True

    def test_detects_the_silent_factory_fallback(self, tmp_path):
        """SDK 静默换了另一份标定却返回 True —— 这里必须当场抓住。

        这是整个模块存在的理由：只看 ``load_calibration()`` 的返回值是看不出
        端点到底来自哪个文件的。
        """
        path = write_cal(tmp_path, "mine.json")
        device = FakeReal(load_values=(0.114, -1.491))  # 出厂标定的数

        with pytest.raises(cal.CalibrationVerificationError) as info:
            cal.apply_calibration(device, path)
        message = str(info.value)
        assert "0.114" in message and "-1.491" in message       # 实际值
        assert "1.775959" in message and "-0.064279" in message  # 要求值
        assert cal.applied_calibration(device) is None

    def test_the_factory_path_is_applied_like_any_other_file(self, tmp_path, monkeypatch):
        """出厂标定不走特殊通道：它就是一份普通的标定文件。

        以前这里按路径拒绝它（要 ``allow_factory=True`` 才放行），等于把 SDK 一
        装好就摆在眼前、且经过验证的那份参数当成嫌疑对象。现在按文件校验、按载
        入结果校验，两条都不放松。
        """
        factory = fake_factory(tmp_path, monkeypatch,
                               closed=0.052071, open_=-1.357481)

        device = FakeReal()
        applied = cal.apply_calibration(device, factory)
        assert applied.path == os.path.abspath(factory)
        assert applied.pos_closed_rad == pytest.approx(0.052071)
        assert cal.applied_calibration(device) == applied

        # 出厂标定照样要过载入后校验：设备实际给的端点不是这份文件里的，就抓。
        drifted = FakeReal(load_values=(1.775959, -0.064279))
        with pytest.raises(cal.CalibrationVerificationError):
            cal.apply_calibration(drifted, factory)
        assert cal.applied_calibration(drifted) is None

    def test_false_return_is_a_failure(self, tmp_path):
        path = write_cal(tmp_path, "c.json")
        device = FakeReal(accept=False)
        with pytest.raises(cal.CalibrationVerificationError, match="返回 False"):
            cal.apply_calibration(device, path)

    def test_device_without_a_loader(self, tmp_path):
        path = write_cal(tmp_path, "c.json")
        with pytest.raises(cal.CalibrationVerificationError, match="load_calibration"):
            cal.apply_calibration(NoLoader(), path)

    def test_device_without_config_endpoints(self, tmp_path):
        """设备载入后仍读不到端点 —— 无从确认标定生效，必须报错。"""
        path = write_cal(tmp_path, "c.json")

        class NoEndpoints(FakeReal):
            def load_calibration(self, path=None):
                self.calls.append(("load_calibration", path))
                self.config = None
                return True

        with pytest.raises(cal.CalibrationVerificationError, match="pos_closed_rad"):
            cal.apply_calibration(NoEndpoints(), path)

    def test_simulated_device_is_refused(self, tmp_path):
        """仿真设备的端点是解析真值，套真机标定只会把它弄错。"""
        from litegrip_mujoco import DryRunGripper

        path = write_cal(tmp_path, "c.json")
        dry = DryRunGripper(realtime=False, noise=False)
        with pytest.raises(cal.CalibrationFileError, match="仿真设备"):
            cal.apply_calibration(dry, path)
        assert cal.applied_calibration(dry) is None


class TestRequireCalibration:
    """``require_calibration`` 是库层自己的入口：真机必须带着标定才准动。"""

    def test_applies_the_given_path(self, tmp_path):
        path = write_cal(tmp_path, "c.json")
        device = FakeReal()
        applied = cal.require_calibration(device, path)
        assert applied is not None
        assert applied.path == os.path.abspath(path)
        assert device.config.pos_closed_rad == pytest.approx(1.775959)

    def test_no_path_means_the_factory_calibration(self, tmp_path, monkeypatch):
        """真机不给路径时用出厂标定，不再是「报错让调用方自己想办法」。"""
        make_home(tmp_path, monkeypatch)
        factory = fake_factory(tmp_path, monkeypatch)
        device = FakeReal()

        applied = cal.require_calibration(device)
        assert applied is not None
        assert applied.path == os.path.abspath(factory)
        assert cal.applied_calibration(device) == applied

    def test_no_factory_file_still_refuses_the_real_device(self, tmp_path, monkeypatch):
        """出厂文件都读不出来时，真机不能被放过去 —— 这是最后一层闸。"""
        make_home(tmp_path, monkeypatch)
        monkeypatch.setattr(cal, "sdk_factory_calibration_path", lambda: None)
        device = FakeReal()

        with pytest.raises(cal.CalibrationRequiredError):
            cal.require_calibration(device)
        assert device.calls == []

    def test_simulated_device_needs_nothing(self, tmp_path):
        from litegrip_mujoco import DryRunGripper

        dry = DryRunGripper(realtime=False, noise=False)
        assert cal.require_calibration(dry) is None
        assert cal.require_calibration(dry, write_cal(tmp_path, "c.json")) is None

    def test_a_bad_path_fails_before_the_device_is_touched(self, tmp_path):
        """文件有问题就不该等到连上 CAN 才报——连之前就把错的挡掉。"""
        device = FakeReal()
        with pytest.raises(cal.CalibrationError):
            cal.require_calibration(device, str(tmp_path / "missing.json"))
        assert device.calls == []


class TestProvenance:
    def test_mark_calibrated_with_a_path(self, tmp_path):
        device = FakeReal()
        path = write_cal(tmp_path, "c.json")
        recorded = cal.mark_calibrated(device, path, reason="test")
        assert recorded is not None
        assert cal.applied_calibration(device) == recorded

    def test_mark_calibrated_without_a_path_is_an_opt_out(self):
        device = FakeReal()
        assert cal.mark_calibrated(device, None, reason="opt out") is None
        assert cal.applied_calibration(device) is None
        assert cal.is_calibrated(device) is True

    def test_tampering_after_apply_is_detected(self, tmp_path):
        device = FakeReal()
        cal.apply_calibration(device, write_cal(tmp_path, "c.json"))
        device.config.pos_open_rad = -0.5  # 例如又跑了一次 calibrate()

        with pytest.raises(cal.CalibrationVerificationError, match="改动"):
            cal.require_usable_device(device, action="测试")

    def test_simulated_devices_are_always_calibrated(self):
        from litegrip_mujoco import DryRunGripper, MujocoGripper

        assert cal.is_simulated_device(MujocoGripper) is True
        assert cal.is_simulated_device(DryRunGripper(realtime=False)) is True
        assert cal.is_simulated_device(FakeReal()) is False
        assert cal.is_calibrated(DryRunGripper(realtime=False)) is True


# ══════════════════════════════════════════════════════════════════════════
# SDK 探测：认 sys.modules，不认第一次的结论
# ══════════════════════════════════════════════════════════════════════════


def fake_sdk(directory, name="litegrip"):
    """造一个能骗过 ``sdk_factory_calibration_path`` 的假 ``litegrip`` 模块。

    它只需要一个 ``gripper`` 子模块，子模块的 ``__file__`` 指到 ``directory``
    下的 ``gripper.py``（真实的 SDK 就是这个布局：出厂标定与 ``gripper.py``
    同目录）。不装真的 SDK、不碰 CAN。
    """
    import types

    package = types.ModuleType(name)
    package.__path__ = [str(directory)]
    gripper = types.ModuleType(f"{name}.gripper")
    gripper.__file__ = os.path.join(str(directory), "gripper.py")
    package.gripper = gripper
    return package


class TestSdkProbe:
    """``_litegrip`` 眼里的 SDK 是 ``sys.modules["litegrip"]``。

    「第一次探测的结论」和「本进程真正的 litegrip」会不一样，而且必须让后者让路：
    例程层的 ``import_litegrip()`` 按 ``$LITEGRIP_SDK_DIR`` / 同级检出定位目录再用
    importlib 显式加载，不靠 ``sys.path`` —— 所以它常常在 ``_litegrip`` 第一次探测
    **失败之后**才把 SDK 装进来。那次失败只说明「那会儿 ``sys.path`` 上没有」。
    把它当终局的话，SDK 明明能用而 ``HAS_SDK`` 一直是 ``False``，于是
    :func:`~litegrip_mujoco.calibration.sdk_factory_calibration_path` 返回
    ``None``——不给 ``--calibration`` 时该用的那份出厂标定就这样静默找不到了。
    """

    def test_adopts_an_sdk_loaded_after_a_failed_probe(self, tmp_path, monkeypatch):
        from litegrip_mujoco import _litegrip as shim

        sdk = fake_sdk(tmp_path)
        monkeypatch.setattr(shim, "_sdk_module", None)
        monkeypatch.setattr(shim, "_sdk_error", ImportError("当初没装"))
        monkeypatch.setattr(shim, "HAS_SDK", False)
        monkeypatch.setitem(sys.modules, "litegrip", sdk)

        assert shim._probe() is sdk
        assert shim.HAS_SDK is True
        assert shim.sdk_unavailable_reason() is None
        # 出厂标定的路径要跟着**这一份**解析 —— 这正是探测失败的代价
        assert cal.sdk_factory_calibration_path() == os.path.join(
            str(tmp_path), "factory_calibration.json")

    def test_follows_the_sdk_that_replaced_the_old_one(self, tmp_path, monkeypatch):
        """被换掉的那份不能再攥着：两边的 GripperState 会变成两个类。"""
        from litegrip_mujoco import _litegrip as shim

        old = fake_sdk(tmp_path / "old")
        new = fake_sdk(tmp_path / "new")
        monkeypatch.setattr(shim, "_sdk_module", old)
        monkeypatch.setattr(shim, "_sdk_error", None)
        monkeypatch.setattr(shim, "HAS_SDK", False)
        monkeypatch.setitem(sys.modules, "litegrip", new)

        assert shim._probe() is new
        assert shim.load_litegrip() is new

    def test_a_failed_probe_is_still_cached_and_still_fails(self, monkeypatch):
        """没有 SDK 就是没有 —— 失败照旧缓存，不会每次调用都重试一遍 import。"""
        from litegrip_mujoco import _litegrip as shim

        monkeypatch.setattr(shim, "_sdk_module", None)
        monkeypatch.setattr(shim, "_sdk_error", ImportError("没装"))
        monkeypatch.delitem(sys.modules, "litegrip", raising=False)

        assert shim._probe() is None
        assert shim.sdk_unavailable_reason() is not None

    def test_a_module_named_litegrip_is_taken_at_face_value(self, monkeypatch):
        """判据只有「``sys.modules`` 里那一份」，没有额外的体检。

        ``import litegrip`` 拿到什么就是什么——本来就是这条规则。加体检的话，
        装了一半的 SDK 会在 ``import`` 成功的前提下被判成不可用，而调用方并没有
        第二个判据可用。
        """
        import types

        from litegrip_mujoco import _litegrip as shim

        monkeypatch.setattr(shim, "_sdk_module", None)
        monkeypatch.setattr(shim, "_sdk_error", ImportError("没装"))
        monkeypatch.setattr(shim, "HAS_SDK", False)
        stub = types.ModuleType("litegrip")
        monkeypatch.setitem(sys.modules, "litegrip", stub)

        assert shim._probe() is stub
        assert shim.load_litegrip() is stub


# ══════════════════════════════════════════════════════════════════════════
# 换算函数：默认严格
# ══════════════════════════════════════════════════════════════════════════


class TestStrictConversions:
    def _bad_dry(self):
        """θ 端点不满足不变量的虚拟夹爪（SDK 默认值那副形状）。"""
        from litegrip_mujoco import DryRunGripper

        bad = GripperConfig(pos_closed_rad=C.REAL_POS_CLOSED_RAD,
                            pos_open_rad=C.REAL_POS_CLOSED_RAD + 1.0)
        device = DryRunGripper(realtime=False, noise=False, config=bad)
        device.connect()
        return device

    def test_read_raises_on_unusable_endpoints(self):
        from litegrip_mujoco.mirror import read_frac_open

        device = self._bad_dry()
        try:
            with pytest.raises(cal.UncalibratedDeviceError):
                read_frac_open(device)
        finally:
            device.disconnect()

    def test_read_raises_without_provenance(self):
        from litegrip_mujoco.mirror import read_frac_open

        with pytest.raises(cal.UncalibratedDeviceError):
            read_frac_open(FakeReal())

    def test_read_strict_false_keeps_the_old_fallback(self):
        """旧行为仍可达，但必须显式开口，并且要给警告。"""
        from litegrip_mujoco.mirror import read_frac_open

        device = self._bad_dry()
        try:
            with pytest.warns(RuntimeWarning):
                frac = read_frac_open(device, strict=False)
            assert 0.0 <= frac <= 1.0
        finally:
            device.disconnect()

    def test_warn_keyword_is_a_deprecated_alias(self):
        """`warn=` 保留为兼容别名，不构成签名上的破坏性变更。"""
        from litegrip_mujoco.mirror import read_frac_open

        device = self._bad_dry()
        try:
            with pytest.warns(DeprecationWarning):
                with pytest.warns(RuntimeWarning):
                    frac = read_frac_open(device, warn=True)
            assert 0.0 <= frac <= 1.0
        finally:
            device.disconnect()

    def test_write_raises_and_does_not_move(self):
        """写入是会让真机动起来的一侧 —— 拒绝时必须一个指令都没发出去。"""
        from litegrip_mujoco.mirror import write_frac_open

        device = FakeReal()
        with pytest.raises(cal.UncalibratedDeviceError):
            write_frac_open(device, 0.5)
        assert device.calls == []

    def test_write_strict_false_uses_the_mm_path(self):
        """端点不可用时才退回 mm；这一支现在是显式开口才走得到的逃生门。"""
        from litegrip_mujoco.mirror import write_frac_open

        device = FakeReal(closed=0.0, open_=1.14)  # 违反不变量
        assert write_frac_open(device, 0.5, strict=False) is True
        assert device.calls == [("goto", 0.5 * C.MM_SCALE)]

    def test_provenance_unlocks_both_directions(self, tmp_path):
        from litegrip_mujoco.mirror import read_frac_open, write_frac_open

        device = FakeReal()
        path = write_cal(tmp_path, "c.json")
        cal.apply_calibration(device, path)

        write_frac_open(device, 0.25)
        assert device.calls[-1][0] == "goto_rad"
        assert read_frac_open(device) == pytest.approx(0.25, abs=1e-6)

    def test_opt_out_unlocks_without_a_file(self):
        from litegrip_mujoco.mirror import read_frac_open, write_frac_open

        device = FakeReal()
        cal.mark_calibrated(device, None, reason="test")
        write_frac_open(device, 1.0)
        assert read_frac_open(device) == pytest.approx(1.0, abs=1e-6)

    def test_simulated_devices_are_unaffected(self):
        """回归：仿真侧不该被这道闸门碰到。"""
        from litegrip_mujoco import DryRunGripper
        from litegrip_mujoco.mirror import read_frac_open, write_frac_open

        device = DryRunGripper(realtime=False, noise=False)
        device.connect()
        try:
            for frac in (0.0, 0.3, 1.0):
                write_frac_open(device, frac)
                assert read_frac_open(device) == pytest.approx(frac, abs=1e-3)
        finally:
            device.disconnect()


# ══════════════════════════════════════════════════════════════════════════
# 库层强制：DualGripper / MirrorMode
# ══════════════════════════════════════════════════════════════════════════


class TestDualGripperGuard:
    def test_a_real_device_defaults_to_the_factory_calibration(self, tmp_path, monkeypatch):
        """不给 ``calibration`` 时用出厂标定 —— 真机路径不再要求先选一份。"""
        from litegrip_mujoco import DualGripper

        make_home(tmp_path, monkeypatch)
        factory = fake_factory(tmp_path, monkeypatch)
        device = FakeReal()

        dual = DualGripper(real=device, render=False, mirror_first=False)
        try:
            assert device.calls == []          # 构造只读文件，不碰硬件
            dual.start()
        finally:
            dual.disconnect()

        assert dual.calibration is not None
        assert dual.calibration.path == os.path.abspath(factory)
        assert cal.is_calibrated(device) is True

    def test_a_missing_factory_file_stops_the_real_device(self, tmp_path, monkeypatch):
        """出厂标定都没有时，真机停在构造阶段，一台硬件都不碰。"""
        from litegrip_mujoco import DualGripper

        make_home(tmp_path, monkeypatch)
        monkeypatch.setattr(cal, "sdk_factory_calibration_path", lambda: None)
        device = FakeReal()

        with pytest.raises(cal.CalibrationRequiredError):
            DualGripper(real=device, render=False)
        assert device.calls == []

    def test_the_user_default_path_is_not_used_even_when_it_exists(self, tmp_path, monkeypatch):
        """``~/.litegrip/litegrip_calibration.json`` 上有一份可用的标定，也不许拿它顶
        替 —— 那是 SDK 的**默认用户路径**，谁写的、属于哪台夹爪都不知道。"""
        from litegrip_mujoco import DualGripper

        home = make_home(tmp_path, monkeypatch)
        (home / ".litegrip").mkdir()
        write_cal(home / ".litegrip", "litegrip_calibration.json")
        factory = fake_factory(tmp_path, monkeypatch)
        device = FakeReal()

        dual = DualGripper(real=device, render=False, mirror_first=False)
        try:
            dual.start()
        finally:
            dual.disconnect()

        assert dual.calibration is not None
        assert dual.calibration.path == os.path.abspath(factory)

    def test_apply_order_is_connect_load_enable(self, tmp_path, monkeypatch):
        from litegrip_mujoco import DualGripper

        make_home(tmp_path, monkeypatch)
        path = write_cal(tmp_path, "c.json")
        device = FakeReal()

        dual = DualGripper(real=device, render=False, mirror_first=False,
                           calibration=path)
        try:
            assert device.calls == []          # 构造只选文件，不碰硬件
            dual.start()
        finally:
            dual.disconnect()

        order = [c if isinstance(c, str) else c[0] for c in device.calls]
        assert order[:3] == ["connect", "load_calibration", "enable"]
        assert dual.calibration is not None
        assert dual.calibration.path == os.path.abspath(path)

    def test_verification_failure_leaves_no_enabled_motor(self, tmp_path, monkeypatch):
        from litegrip_mujoco import DualGripper

        make_home(tmp_path, monkeypatch)
        path = write_cal(tmp_path, "c.json")
        device = FakeReal(load_values=(0.114, -1.491))

        dual = DualGripper(real=device, render=False, mirror_first=False,
                           calibration=path)
        with pytest.raises(cal.CalibrationVerificationError):
            dual.start()

        assert "enable" not in device.calls
        assert "disconnect" in device.calls

    def test_allow_uncalibrated_is_explicit(self, tmp_path, monkeypatch):
        from litegrip_mujoco import DualGripper

        make_home(tmp_path, monkeypatch)
        device = FakeReal()
        dual = DualGripper(real=device, render=False, mirror_first=False,
                           allow_uncalibrated=True)
        try:
            dual.start()
            assert dual.calibration is None
        finally:
            dual.disconnect()
        assert "enable" in device.calls

    def test_simulated_injection_needs_nothing(self, tmp_path, monkeypatch):
        """回归：`DualGripper(real=DryRunGripper(), render=False)` 必须照旧可用。"""
        from litegrip_mujoco import DryRunGripper, DualGripper

        make_home(tmp_path, monkeypatch)
        dual = DualGripper(real=DryRunGripper(realtime=False), render=False,
                           mirror_first=False)
        try:
            dual.start()
            dual.move_to_frac(0.5, duration=0.2)
        finally:
            dual.disconnect()
        assert dual.calibration is None

    def test_already_calibrated_device_is_not_re_prompted(self, tmp_path, monkeypatch):
        from litegrip_mujoco import DualGripper

        make_home(tmp_path, monkeypatch)
        device = FakeReal()
        cal.apply_calibration(device, write_cal(tmp_path, "c.json"))

        dual = DualGripper(real=device, render=False, mirror_first=False)
        try:
            dual.start()
        finally:
            dual.disconnect()
        assert dual.calibration is not None


class TestMirrorModeGuard:
    def test_real_device_defaults_to_the_factory_calibration(self, tmp_path, monkeypatch):
        from litegrip_mujoco import MirrorMode, MujocoGripper

        make_home(tmp_path, monkeypatch)
        factory = fake_factory(tmp_path, monkeypatch)
        sim = MujocoGripper(render=False)
        mirror = MirrorMode(FakeReal(), sim)

        assert mirror.calibration is not None
        assert mirror.calibration.path == os.path.abspath(factory)

    def test_a_missing_factory_file_stops_the_real_device(self, tmp_path, monkeypatch):
        from litegrip_mujoco import MirrorMode, MujocoGripper

        make_home(tmp_path, monkeypatch)
        monkeypatch.setattr(cal, "sdk_factory_calibration_path", lambda: None)
        sim = MujocoGripper(render=False)
        device = FakeReal()

        with pytest.raises(cal.CalibrationRequiredError):
            MirrorMode(device, sim)
        assert device.calls == []

    def test_explicit_calibration_is_applied(self, tmp_path, monkeypatch):
        from litegrip_mujoco import MirrorMode, MujocoGripper

        make_home(tmp_path, monkeypatch)
        path = write_cal(tmp_path, "c.json")
        device = FakeReal()
        sim = MujocoGripper(render=False)
        mirror = MirrorMode(device, sim, calibration=path)
        assert mirror.calibration is not None
        assert mirror.calibration.path == os.path.abspath(path)

    def test_allow_uncalibrated_starts_and_marks(self, tmp_path, monkeypatch):
        from litegrip_mujoco import MirrorMode, MujocoGripper

        make_home(tmp_path, monkeypatch)
        device = FakeReal()
        sim = MujocoGripper(render=False)
        sim.connect()
        mirror = MirrorMode(device, sim, allow_uncalibrated=True, rate_hz=100.0)
        try:
            mirror.start()
            time.sleep(0.15)
            assert mirror.samples > 0
            assert mirror.last_error is None
        finally:
            mirror.stop()
            sim.disconnect()

    def test_start_refuses_when_calibration_was_lost(self, tmp_path, monkeypatch):
        from litegrip_mujoco import MirrorMode, MujocoGripper

        make_home(tmp_path, monkeypatch)
        device = FakeReal()
        cal.apply_calibration(device, write_cal(tmp_path, "c.json"))
        sim = MujocoGripper(render=False)
        mirror = MirrorMode(device, sim)

        device.config.pos_open_rad = -0.9  # 中途被改掉
        with pytest.raises(cal.CalibrationVerificationError):
            mirror.start()
        assert mirror.is_running is False

    def test_loop_stops_and_reports_instead_of_spinning(self, tmp_path, monkeypatch):
        """标定失效时镜像线程要停表并记下原因，不能 50 Hz 静默空转。"""
        from litegrip_mujoco import MirrorMode, MujocoGripper

        make_home(tmp_path, monkeypatch)
        device = FakeReal()
        cal.apply_calibration(device, write_cal(tmp_path, "c.json"))
        sim = MujocoGripper(render=False)
        sim.connect()
        mirror = MirrorMode(device, sim, rate_hz=200.0)
        try:
            mirror.start()
            assert mirror.is_running is True
            device.config.pos_open_rad = -0.9  # 线程跑到一半标定被改
            deadline = time.monotonic() + 5.0
            while mirror.is_running and time.monotonic() < deadline:
                time.sleep(0.02)
            assert mirror.is_running is False
            assert isinstance(mirror.last_error, cal.CalibrationVerificationError)
        finally:
            mirror.stop()
            sim.disconnect()

    def test_simulated_real_is_exempt(self):
        from litegrip_mujoco import DryRunGripper, MirrorMode, MujocoGripper

        real = DryRunGripper(realtime=False, noise=False)
        real.connect()
        sim = MujocoGripper(render=False)
        sim.connect()
        mirror = MirrorMode(real, sim, rate_hz=100.0)
        try:
            mirror.start()
            time.sleep(0.2)
            assert mirror.samples > 0
            assert mirror.last_error is None
        finally:
            mirror.stop()
            real.disconnect()
            sim.disconnect()


# ══════════════════════════════════════════════════════════════════════════
# 例程
# ══════════════════════════════════════════════════════════════════════════


def run_example(name, *args, home, env=None):
    """在子进程里跑一个例程。

    用 ``LITEGRIP_CALIB`` 而不是 ``HOME`` 来搬走"默认标定路径"：改 ``HOME``
    会连带把解释器的 user site-packages 也搬走，numpy 就 import 不到了。
    ``LITEGRIP_CALIB`` 本来就是 SDK 用来改默认路径的开关，语义正好。

    ``env`` 里的键覆盖上面这层默认值——用来把 ``LITEGRIP_SDK_DIR`` 指到一个
    空目录或一份自造的 SDK 上，这样用例不必依赖这台机器装没装 SDK。
    """
    environment = dict(os.environ)
    environment["LITEGRIP_CALIB"] = str(home / "litegrip_calibration.json")
    for key, value in (env or {}).items():
        if value is None:
            environment.pop(key, None)
        else:
            environment[key] = str(value)
    return subprocess.run(
        [sys.executable, os.path.join(EXAMPLES, name), *args],
        cwd=REPO_ROOT,
        env=environment,
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        timeout=300,
    )


def sdk_env(directory):
    """把一次运行完整钉在这份 SDK 检出上（见 :func:`fake_sdk_dir`）。

    ``PYTHONPATH`` 是关键的一半：``litegrip`` 是个**包名**，光换
    ``LITEGRIP_SDK_DIR`` 挡不住开发机上另装的那一份——库层的探测走的是
    ``import litegrip``，两条路径会各说各话。把它顶到 ``sys.path`` 最前面，
    这次运行的「SDK」才只有一个答案。
    """
    return {"LITEGRIP_SDK_DIR": directory, "PYTHONPATH": directory}


def fake_sdk_dir(tmp_path, **fields):
    """造一份最小可用的 SDK 检出：``litegrip/`` 包 + 出厂标定。

    例程不导入它——``--dry-run`` 不碰 CAN，只要出厂标定文件在那个位置。不过库层
    的 :func:`~litegrip_mujoco.calibration.sdk_factory_calibration_path` 是顺着
    ``litegrip.gripper`` 的所在目录去认这份文件的，所以 ``gripper.py`` 也得在。
    """
    directory = tmp_path / "sdk"
    package = directory / "litegrip"
    package.mkdir(parents=True, exist_ok=True)
    (package / "__init__.py").write_text("")
    (package / "gripper.py").write_text(
        "import os\n"
        "_FILE = os.path.join(os.path.dirname(__file__), "
        '"factory_calibration.json")\n'
    )
    data = {"zero_position_rad": 0.052071, "max_position_rad": -1.357481,
            "rad_to_mm": 61.01229326764816}
    data.update(fields)
    (package / "factory_calibration.json").write_text(json.dumps(data))
    return directory


def no_sdk(tmp_path):
    """这条路走起来像「这台机器根本没装 SDK」：包名在，但 import 就失败。

    ``__init__.py`` 里**抛 ImportError** 而不是空着：空包会让
    ``import litegrip.gripper`` 绕过它、落到别的检出上（开发机装的那份），于是
    这台机器上有没有 SDK 会改变用例的结论——而 CI 上恰好一份都没有。
    """
    directory = tmp_path / "no-sdk"
    package = directory / "litegrip"
    package.mkdir(parents=True, exist_ok=True)
    (package / "__init__.py").write_text(
        'raise ImportError("这个检出里没有 litegrip SDK")\n')
    return sdk_env(directory)


class TestExamples:
    @pytest.mark.parametrize("name", ["04_mirror_real.py", "05_dual_control.py"])
    def test_refuses_to_start_without_a_calibration(self, tmp_path, name):
        """找不到出厂标定、也没给 --calib ⇒ 退出码 1，且说清怎么修。

        退出码 1（``SystemExit(message)``）而不是 2：2 留给 argparse 自己的用法
        错误。两份真机样例与 pybullet 那套用的是同一条约定。
        """
        result = run_example(name, home=tmp_path, env=no_sdk(tmp_path))
        assert result.returncode == 1, result.stdout + result.stderr
        combined = result.stdout + result.stderr
        assert "--calibration" in combined
        assert "标定" in combined

    @pytest.mark.parametrize("name", ["04_mirror_real.py", "05_dual_control.py"])
    def test_help_lists_the_calibration_flags(self, tmp_path, name):
        result = run_example(name, "--help", home=tmp_path)
        assert result.returncode == 0
        assert "--calibration" in result.stdout
        assert "--list-calibrations" in result.stdout

    @pytest.mark.parametrize("name", ["04_mirror_real.py", "05_dual_control.py"])
    def test_dry_run_does_not_ask_for_calibration(self, tmp_path, name):
        """--dry-run 不接触真机：缺标定也照跑，而且**不问人**。

        必须给 ``--duration``：这两个例程是常驻的镜像/遥操作循环，跑到 Esc 或
        关窗口为止。
        """
        home = tmp_path / "home"
        home.mkdir()
        result = run_example(name, "--dry-run", "--no-render",
                             "--duration", "2.5",
                             home=home, env=no_sdk(tmp_path))
        assert result.returncode == 0, result.stdout + result.stderr
        assert "选择标定文件" not in result.stdout + result.stderr
        assert "也没找到 SDK 出厂标定" in result.stdout

    @pytest.mark.parametrize("name", ["04_mirror_real.py", "05_dual_control.py"])
    def test_uses_the_sdk_factory_calibration_by_default(self, tmp_path, name):
        """没给 --calib 时用 SDK 包里那份出厂标定，并说明它不是这台夹爪的。

        这是两档里的第 2 档，也是默认档。它必须**先于** SDK 导入就能定下来：
        SDK 未必装在 ``sys.path`` 上（这里就只存在于 ``LITEGRIP_SDK_DIR``），而
        这一档的默认行为就是走它。
        """
        sdk = fake_sdk_dir(tmp_path)
        result = run_example(name, "--dry-run", "--no-render",
                             "--duration", "2.5",
                             home=tmp_path / "home",
                             env=sdk_env(sdk))
        assert result.returncode == 0, result.stdout + result.stderr
        assert str(sdk / "litegrip" / "factory_calibration.json") in result.stdout
        assert "SDK 出厂标定" in result.stdout
        assert "不是这台夹爪自己量的" in result.stdout

    @pytest.mark.parametrize("name", ["04_mirror_real.py", "05_dual_control.py"])
    def test_explicit_calibration_is_reported(self, tmp_path, name):
        """指错文件要当场报「读不出来」，而不是先抱怨没装 SDK。"""
        result = run_example(name, "--calibration", str(tmp_path / "nope.json"),
                             home=tmp_path)
        assert result.returncode == 1, result.stdout + result.stderr
        assert "nope.json" in result.stdout + result.stderr

    def test_dry_run_still_refuses_an_explicitly_named_bad_file(self, tmp_path):
        """``--dry-run`` 原谅「没有标定」，不原谅「你点的那份读不出来」。

        两者不一样：前者是这台机器上没有可用的标定，后者多半是路径打错了字——
        静默跑完会让人以为那份文件是对的。
        """
        result = run_example("05_dual_control.py", "--dry-run", "--no-render",
                             "--calibration", str(tmp_path / "nope.json"),
                             home=tmp_path)
        assert result.returncode == 1, result.stdout + result.stderr
        assert "nope.json" in result.stdout + result.stderr

    @pytest.mark.parametrize("name", ["03_trajectory.py", "04_mirror_real.py",
                                      "05_dual_control.py"])
    def test_list_calibrations_runs_before_the_other_gates(self, tmp_path, name):
        """``--list-calibrations`` 是纯查询：跟 ``--dry-run``/``--headless``/
        ``--status`` 的组合不该改变它的结论，也不该先去 import SDK。"""
        home = tmp_path / "home"
        home.mkdir()
        planted = write_cal(home, "planted.json")

        for extra in ([], ["--headless"], ["--dry-run"]):
            result = run_example(name, "--list-calibrations", *extra,
                                 home=home, env=no_sdk(tmp_path))
            assert result.returncode == 0, result.stdout + result.stderr
            assert planted in result.stdout
            # 可用的排在前面：仓库根目录下的 .releaserc.json 之类不该挤掉真标定
            if ".releaserc.json" in result.stdout:
                assert (result.stdout.index(planted)
                        < result.stdout.index(".releaserc.json"))
