#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真专有能力的测试：循环骨架、窗口/输入层、世界查询、备用工件槽、轨迹。

这些都不是 SDK 对等面上的东西，而是例程要用的仿真机制。分五组：

  循环骨架     connected()/pump()/sim_time 在无窗口、无线程时也必须能问、能答，
               因为例程有 --headless 分支，测试也不该需要图形环境。
  窗口/输入    KeyQueue 是回调线程与主线程之间唯一的通道；无窗口时所有窗口 API
               一律 no-op 而不是抛异常。
  运动语义     command_fraction 必须**立刻返回**、限速、限力，且限力不能漏给
               下一条指令。
  世界与槽位   scene.xml 的 4 个槽位。模型是编译期的，"放一个工件"只能是搬运
               一个已有的，所以槽位用尽、被覆盖、被 reset 复位都要钉住。
  轨迹         ``.lgt`` 的字节布局对着**字面量**钉住（不是对着本模块自己的
               常量，那等于拿实现验实现）。装了 SDK 时还会真跑一遍互读；没装
               就跳过，但不是不测——字面量那条永远跑。
"""
from __future__ import annotations

import struct
import threading
import time

import numpy as np
import pytest

import mujoco
from litegrip_mujoco import (
    CONFIRM_KEYS,
    QUIT_KEYS,
    ZERO_GRAVITY_KEYS,
    KeyQueue,
    MujocoContact,
    MujocoGripper,
    Trajectory,
    TrajectoryBusyError,
    TrajectoryNotActiveError,
    TrajectoryRecordingError,
    TrajectorySample,
    constants as C,
    resolve_path,
    trajectory_dir,
    world,
)
from litegrip_mujoco import trajectory as T
from litegrip_mujoco import window as W
from litegrip_mujoco.gripper import _resolve_model_path

# ══════════════════════════════════════════════════════════════════════════
# 夹具
# ══════════════════════════════════════════════════════════════════════════


@pytest.fixture
def sim():
    """无窗口的纯夹爪模型。"""
    g = MujocoGripper(render=False)
    g.connect()
    g.enable()
    yield g
    g.disconnect()


@pytest.fixture
def scene():
    """无窗口的演示场景（带工件与 4 个备用槽位）。"""
    g = MujocoGripper(model_path="scene.xml", render=False)
    g.connect()
    g.enable()
    yield g
    g.disconnect()


def _until(predicate, timeout=5.0, interval=0.005):
    """自旋等到 ``predicate()`` 为真，返回是否等到了。"""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(interval)
    return False


# ══════════════════════════════════════════════════════════════════════════
# 循环骨架
# ══════════════════════════════════════════════════════════════════════════


class TestLoopSkeleton:
    def test_connected_tracks_the_thread(self, sim):
        """``is_connected`` 是逻辑标志，``connected()`` 还要求线程活着。"""
        assert sim.connected() and sim.is_connected
        sim.disconnect()
        assert not sim.connected() and not sim.is_connected

    def test_connected_is_false_before_connect(self):
        g = MujocoGripper(render=False)
        assert g.is_connected is False
        assert g.connected() is False

    def test_sim_time_counts_simulation_not_wallclock(self):
        """``realtime=False`` 时仿真时间会跑到墙钟前面 —— 它记的是仿真。"""
        g = MujocoGripper(render=False, realtime=False)
        try:
            start = g.sim_time
            g.step(2000)
            assert g.sim_time == pytest.approx(start + 2.0, abs=1e-9)
        finally:
            g.disconnect()

    def test_sim_time_survives_reset(self):
        """``reset()`` 会清 ``data.time``，但不该清这个累计时钟。"""
        g = MujocoGripper(render=False)
        try:
            g.step(500)
            before = g.sim_time
            g.reset("open")
            assert g.sim_time == pytest.approx(before)
        finally:
            g.disconnect()

    def test_pump_advances_without_a_thread(self):
        g = MujocoGripper(render=False, realtime=False)
        try:
            assert g.pump() is True
            assert g.sim_time > 0.0
        finally:
            g.disconnect()

    def test_pump_is_false_after_stop(self, sim):
        assert sim.pump() is True
        sim.stop()
        assert sim.pump() is False

    def test_pump_does_not_double_step(self, sim):
        """线程在跑时 ``pump()`` 只等，不自己推进 —— 否则仿真会走两倍快。"""
        time.sleep(0.2)
        t0 = sim.sim_time
        for _ in range(20):
            sim.pump()
        advanced = sim.sim_time - t0
        assert advanced < 0.2, f"pump() 把物理推进了 {advanced:.3f} s，超出一拍"

    def test_gui_is_false_without_a_viewer(self, sim):
        assert sim.gui is False


# ══════════════════════════════════════════════════════════════════════════
# 窗口与输入层
# ══════════════════════════════════════════════════════════════════════════


class TestInputLayer:
    def test_key_codes_match_glfw(self):
        """字面量必须和真 GLFW 一致，否则按键会认错。

        没装 glfw 时（无图形环境）跳过 —— 这条只是防止字面量悄悄漂移。
        """
        try:
            from mujoco.viewer import glfw  # type: ignore
        except Exception:
            pytest.skip("没有 glfw，无法对照")
        codes = W.key_codes()
        for name, value in codes.items():
            assert value == getattr(glfw, f"KEY_{name}"), f"{name} 与 glfw 不一致"

    def test_key_codes_falls_back_without_glfw(self, monkeypatch):
        import sys
        import types

        monkeypatch.setitem(sys.modules, "mujoco.viewer", types.ModuleType("mujoco.viewer"))
        W.key_codes.cache_clear()
        try:
            assert W.key_codes()["ESCAPE"] == 256
        finally:
            W.key_codes.cache_clear()

    def test_pressed_reads_a_press(self):
        assert W.pressed({256: W.KEY_PRESSED}, QUIT_KEYS) is True
        assert W.pressed({}, QUIT_KEYS) is False
        assert W.pressed({70: W.KEY_PRESSED}, QUIT_KEYS) is False

    def test_pressed_ignores_a_zero_event(self):
        """事件值为 0 表示"没按下"，不能当成按下。"""
        assert W.pressed({256: 0}, QUIT_KEYS) is False

    def test_held_still_means_pressed(self):
        """查看器不报释放事件，所以 held() 只能是 pressed()。"""
        assert W.held({256: W.KEY_PRESSED}, QUIT_KEYS) is True

    def test_clicked_is_always_false(self):
        """MuJoCo 被动查看器没有鼠标回调，这个哑元永远返回 False。"""
        assert W.clicked([]) is False
        assert W.clicked([object()]) is False
        assert W.clicked([(1, 2, 3)]) is False

    def test_mouse_events_is_always_empty(self, sim):
        assert sim.mouse_events() == []

    def test_key_label_never_raises(self):
        assert W.key_label(256) == "Esc"
        assert W.key_label(9999).startswith("key "), "未知键要有个兜底名字"

    def test_key_tables_are_disjoint_where_it_matters(self):
        """退出键里不能混进遥操作键，否则一按键就退出。"""
        teleop = {k for keys in W.TELEOP_KEYS.values() for k in keys}
        assert not (set(QUIT_KEYS) & teleop)
        assert set(ZERO_GRAVITY_KEYS).isdisjoint(teleop)
        assert set(CONFIRM_KEYS) & set(W.TELEOP_KEYS["stop"])

    def test_key_queue_round_trip(self):
        q = KeyQueue()
        q.feed(256)
        q.feed(81)
        q.feed(256)
        assert len(q) == 3
        assert q.count(256) == 2
        events = q.drain()
        assert set(events) == {256, 81}
        assert len(q) == 0
        assert q.drain() == {}

    def test_key_queue_peek_keeps_events(self):
        q = KeyQueue()
        q.feed(256)
        assert q.peek() == {256: W.KEY_PRESSED}
        assert len(q) == 1

    def test_key_queue_is_safe_across_threads(self):
        """回调线程只管 append。这条测的是"不会丢、不会炸"。"""
        q = KeyQueue()
        n = 500

        def feeder():
            for i in range(n):
                q.feed(i)

        t = threading.Thread(target=feeder)
        t.start()
        t.join()
        assert len(q) == n

    def test_keyboard_events_drains_the_queue(self, sim):
        sim._keys.feed(256)
        assert sim.keyboard_events() == {256: W.KEY_PRESSED}
        assert sim.keyboard_events() == {}

    def test_window_api_is_a_noop_without_a_viewer(self, sim):
        """无窗口时窗口 API 返回"没做成"，而不是抛异常。"""
        assert sim.status_text(["hello"]) is False
        assert sim.focus_camera(distance=1.0) is False
        assert sim.gui is False


# ══════════════════════════════════════════════════════════════════════════
# 运动语义
# ══════════════════════════════════════════════════════════════════════════


class TestCommandFraction:
    def test_returns_immediately(self, sim):
        """非阻塞是它的全部意义 —— 阻塞版会把控制回路自己卡住。"""
        t0 = time.monotonic()
        sim.command_fraction(0.0)
        assert time.monotonic() - t0 < 0.05
        assert _until(lambda: sim.frac_open() < 0.05)

    def test_reaches_the_requested_fraction(self, sim):
        for frac in (0.25, 0.75, 0.0, 1.0):
            sim.command_fraction(frac)
            assert _until(lambda: abs(sim.frac_open() - frac) < 0.002), (
                f"没有走到 {frac}，停在 {sim.frac_open():.4f}"
            )

    def test_clamps_out_of_range_fractions(self, sim):
        sim.command_fraction(3.0)
        assert _until(lambda: sim.frac_open() > 0.99)
        sim.command_fraction(-2.0)
        assert _until(lambda: sim.frac_open() < 0.01)

    def test_speed_limit_is_respected(self, sim):
        """限速是单指线速度，全行程走完约 1 秒。"""
        sim.command_fraction(0.0, velocity_m_s=C.DEFAULT_VELOCITY_M_S)
        t0 = time.monotonic()
        assert _until(lambda: sim.frac_open() < 0.01, timeout=5.0)
        elapsed = time.monotonic() - t0
        assert 0.7 < elapsed < 2.0, f"全行程用了 {elapsed:.3f} s，限速没生效"

    def test_force_limit_lowers_the_torque_ceiling(self, sim):
        sim.command_fraction(1.0, force_n=2.0)
        assert sim._ctrl.tau_max == pytest.approx(C.n_to_nm(2.0))

    def test_force_limit_does_not_leak_into_the_next_move(self, sim):
        """一次限力运动不能把限幅漏给后面所有指令。"""
        sim.command_fraction(1.0, force_n=2.0)
        assert sim._ctrl.tau_max == pytest.approx(C.n_to_nm(2.0))
        sim.command_fraction(0.0)
        assert sim._ctrl.tau_max == pytest.approx(C.TAU_MAX)

    def test_stop_clears_the_torque_ceiling(self, sim):
        sim.command_fraction(1.0, force_n=2.0)
        sim.stop()
        assert sim._ctrl.tau_max == pytest.approx(C.TAU_MAX)

    def test_requires_connect_and_enable(self):
        g = MujocoGripper(render=False)
        try:
            with pytest.raises(Exception):
                g.command_fraction(0.5)
        finally:
            g.disconnect()


class TestSettle:
    def test_settle_with_seconds_runs_the_full_time(self, sim):
        """带参数的 settle 是"跑满"，哪怕指爪早就到位 —— 时间要留给别的东西。"""
        t0 = time.monotonic()
        elapsed, reached = sim.settle(0.3)
        wall = time.monotonic() - t0
        assert wall >= 0.29, f"只跑了 {wall:.3f} s"
        assert elapsed == pytest.approx(0.3, abs=0.05)
        assert reached is True

    def test_settle_without_seconds_waits_for_the_travel(self, sim):
        sim.command_fraction(0.0)
        elapsed, reached = sim.settle()
        assert reached is True
        assert elapsed > 0.5, f"还没走完就返回了（{elapsed:.3f} s 仿真时间）"

    def test_settle_reports_stall_as_not_reached(self, scene):
        """撞上工件停在半路 = 没到位，但用时远小于超时。"""
        scene.reset("fixture")
        scene.close(duration=1.0)
        elapsed, reached = scene.settle()
        assert reached is False, "撞在工件上却被判成到位"
        assert elapsed < C.DEFAULT_SETTLE_TIMEOUT_S, "应当提前收手，而不是干等超时"

    def test_settle_returns_immediately_when_already_there(self, sim):
        sim.goto(C.MM_SCALE, duration=0.5)
        elapsed, reached = sim.settle()
        assert reached is True
        assert elapsed < 0.1

    def test_settle_does_not_report_arrival_mid_ramp(self, sim):
        """沿斜坡匀速走的时候位置误差贴着零，只看误差会误判成"到位"。"""
        sim.goto(0.0, duration=1.0)
        elapsed, reached = sim.settle()
        assert reached is True
        assert sim.get_position() == pytest.approx(0.0, abs=0.1)


# ══════════════════════════════════════════════════════════════════════════
# 世界查询
# ══════════════════════════════════════════════════════════════════════════


class TestWorldQueries:
    def test_grasp_center_is_between_the_pads(self, sim):
        """中心 = 两个夹持面中心的中点。

        不假设哪根手指在 +x：``finger_left`` 其实站在 +x 一侧，"left" 是随本体
        坐标系叫的，不是随世界系。
        """
        centre = sim.grasp_center()
        left, right = sim.pad_aabbs()
        left_c = 0.5 * (left[0] + left[1])
        right_c = 0.5 * (right[0] + right[1])
        assert centre == pytest.approx(0.5 * (left_c + right_c), abs=1e-12)
        for axis in range(3):
            assert min(left[0][axis], right[0][axis]) < centre[axis]
            assert centre[axis] < max(left[1][axis], right[1][axis])

    def test_grasp_center_tracks_the_opening(self, sim):
        """开口变，中心不变 —— 中心取的是两指中点，不是包围盒中点。"""
        sim.goto(C.MM_SCALE, duration=0.5)
        wide = sim.grasp_center()
        sim.goto(0.0, duration=0.5)
        narrow = sim.grasp_center()
        assert wide[2] == pytest.approx(narrow[2], abs=1e-6)
        assert wide[0] == pytest.approx(0.0, abs=1e-6)
        assert narrow[0] == pytest.approx(0.0, abs=1e-6)

    def test_link_aabb_is_exact_for_a_box(self, scene):
        lo, hi = scene.link_aabb("object")
        assert hi - lo == pytest.approx(np.array([0.02, 0.02, 0.03]), abs=1e-9)

    def test_link_aabb_rejects_unknown_bodies(self, sim):
        with pytest.raises(KeyError):
            sim.link_aabb("no_such_body")

    def test_contacts_finds_the_floor(self, sim):
        sim.settle(0.05)
        items = sim.contacts()
        assert all(isinstance(c, MujocoContact) for c in items)

    def test_contacts_filter_by_name(self, scene):
        scene.reset("fixture")
        scene.settle(0.05)
        everything = scene.contacts(with_force=False)
        only_object = scene.contacts(only=("object",), with_force=False)
        assert len(only_object) <= len(everything)
        assert all(c.involves("object") for c in only_object)

    def test_contacts_carry_a_force(self, scene):
        scene.reset("fixture")
        scene.close(duration=1.0)
        scene.settle(0.2)
        touched = scene.contacts(only=("object",))
        assert touched, "夹到工件了却没有接触点"
        assert max(c.force_n for c in touched) > 0.0


# ══════════════════════════════════════════════════════════════════════════
# 备用工件槽
# ══════════════════════════════════════════════════════════════════════════


class TestSpawnSlots:
    def test_pure_gripper_model_has_no_slots(self, sim):
        assert sim.box_slots() == []

    def test_scene_has_four_slots(self, scene):
        assert len(scene.box_slots()) == 4

    def test_slots_start_parked_on_the_floor(self, scene):
        """应用 open 键位之后，槽位必须在停放位，而不是世界原点。

        MuJoCo 对键位里没写到的自由度补零，而补零的自由关节就是"位于原点"——
        也就是夹爪底座里面。这条钉的就是那个补位。
        """
        floor_z = -0.08
        for name in scene.box_slots():
            bid = mujoco.mj_name2id(scene.model, mujoco.mjtObj.mjOBJ_BODY, name)
            pos = np.asarray(scene.data.xpos[bid])
            assert np.linalg.norm(pos) > 0.1, f"{name} 停在原点附近：{pos}"
            assert pos[2] > floor_z, f"{name} 陷到地板下面：{pos}"

    def test_add_box_moves_the_slot(self, scene):
        name = scene.add_box(0)
        bid = mujoco.mj_name2id(scene.model, mujoco.mjtObj.mjOBJ_BODY, name)
        centre = scene.grasp_center()
        assert np.asarray(scene.data.xpos[bid]) == pytest.approx(centre, abs=1e-6)

    def test_add_box_resizes_and_reweighs(self, scene):
        name = scene.add_box(0, size=(0.02, 0.02, 0.02))
        bid = mujoco.mj_name2id(scene.model, mujoco.mjtObj.mjOBJ_BODY, name)
        assert scene.model.body_mass[bid] == pytest.approx(
            world.box_mass((0.02, 0.02, 0.02))
        )
        gid = int(scene.model.body_geomadr[bid])
        assert scene.model.geom_size[gid] == pytest.approx([0.02, 0.02, 0.02])

    def test_add_box_leaves_the_gripper_alone(self, scene):
        """mj_setConst 会把 qpos 复位成 qpos0，add_box 必须把它恢复回来。"""
        scene.reset("fixture")
        scene.goto(20.0, duration=0.5)
        before = scene.data.qpos.copy()
        scene.add_box(2)
        assert scene.data.qpos[:2] == pytest.approx(before[:2])
        assert scene.data.qpos[2:9] == pytest.approx(before[2:9])

    def test_add_box_overwrites_the_previous_occupant(self, scene):
        scene.add_box(0, size=(0.005, 0.005, 0.005))
        name = scene.add_box(0, size=(0.015, 0.015, 0.015))
        bid = mujoco.mj_name2id(scene.model, mujoco.mjtObj.mjOBJ_BODY, name)
        assert scene.model.body_mass[bid] == pytest.approx(
            world.box_mass((0.015, 0.015, 0.015))
        )

    def test_add_box_rejects_a_bad_index(self, scene):
        with pytest.raises(IndexError):
            scene.add_box(4)
        with pytest.raises(IndexError):
            scene.add_box(-1)

    def test_add_box_rejects_a_model_without_slots(self, sim):
        with pytest.raises(IndexError):
            sim.add_box(0)

    def test_reset_reparks_every_slot(self, scene):
        name = scene.add_box(1)
        bid = mujoco.mj_name2id(scene.model, mujoco.mjtObj.mjOBJ_BODY, name)
        scene.reset("fixture")
        assert np.asarray(scene.data.xpos[bid]) != pytest.approx(
            scene.grasp_center(), abs=1e-3
        )

    def test_a_slot_can_be_gripped(self, scene):
        """端到端：搬一个方块进指间、合拢，指爪停在方块上而不是走到闭合位。

        ⚠ 合拢必须**快**。夹爪是固定安装的，方块在两指之间做自由落体 —— 这和
        scene.xml 里那个工件必须靠夹具托着是同一件事。实测合拢时长的影响：

            0.05 s → 开口停在 19.67 mm（夹住了）
            0.10 s → 开口停在 19.67 mm（夹住了，但方块已下滑 21 mm）
            0.15 s → 开口走到 16.57 mm（方块掉出去了）

        所以这里取 0.05 s，并同时断言"没掉到地板上"。
        """
        scene.reset("fixture")
        scene.settle(0.05)
        scene.release_fixture()          # 让原来的工件落到地板，腾出位置
        scene.settle(1.0)
        name = scene.add_box(0, size=(0.010, 0.010, 0.015))
        bid = mujoco.mj_name2id(scene.model, mujoco.mjtObj.mjOBJ_BODY, name)

        scene.close(duration=0.05, force_n=10.0)
        scene.settle(1.0)

        # 与 test_workpiece_blocks_closing 同一个数：指爪压进方块约 0.33 mm。
        assert scene.gap_mm() == pytest.approx(19.67, abs=0.1), "指爪没有停在方块上"
        assert float(scene.data.xpos[bid][2]) > 0.05, "方块从指间掉出去了"

    def test_a_spawned_box_falls_and_lands(self, scene):
        """方块是真刚体：悬空放会掉，落到地板停在半高位置。"""
        scene.release_fixture()
        name = scene.add_box(0, pos=(0.05, 0.05, 0.20))
        bid = mujoco.mj_name2id(scene.model, mujoco.mjtObj.mjOBJ_BODY, name)
        scene.settle(1.5)
        # 地板 z=-0.08，半高 0.01。
        assert float(scene.data.xpos[bid][2]) == pytest.approx(-0.07, abs=0.005)


# ══════════════════════════════════════════════════════════════════════════
# 轨迹：.lgt 格式与录制/回放
# ══════════════════════════════════════════════════════════════════════════


def _two_sample_trajectory():
    return Trajectory(
        samples=[
            TrajectorySample(t=0.0, openness=1.0, position_rad=0.0),
            TrajectorySample(t=1.0, openness=0.5, position_rad=-0.5,
                             velocity_rad_s=0.1, torque_nm=0.2),
        ],
        sample_hz=100.0, created=1234.5, can_id=8,
        pos_closed_rad=0.0, pos_open_rad=-1.14, rad_to_mm=74.958,
        mount="reverse",
    )


#: ``.lgt`` 的字节布局，按格式说明**手写**成字面量。
#:
#: 刻意不复用本模块的 ``_HEADER``/``_SAMPLE``：那等于拿实现验实现，常量被改错
#: 时两边一起错。这一份是给"跨实现互读"兜底的 —— 硬件的 SDK 写的文件必须能被
#: 这里读出来，反之亦然，所以布局只能有一个定义，而它就是这个字面量。
_EXPECTED_HEADER_SIZE = 66
_EXPECTED_SAMPLE_SIZE = 40
_EXPECTED_BYTES = (
    struct.pack("<8sHI5dI8s", b"LGRTRJ01", 1, 2, 100.0, 1234.5, 0.0, -1.14,
                74.958, 8, b"reverse\x00")
    + struct.pack("<5d", 0.0, 1.0, 0.0, 0.0, 0.0)
    + struct.pack("<5d", 1.0, 0.5, -0.5, 0.1, 0.2)
)


def _sdk_trajectory_module():
    """装了 SDK 就返回它的 ``trajectory`` 模块，否则 None。"""
    try:
        import litegrip.trajectory as sdk  # type: ignore
    except Exception:
        return None
    return sdk


class TestLgtFormat:
    def test_struct_sizes_and_magic(self):
        """头部 66B、每拍 40B、magic 与版本号 —— 格式的三个硬数字。"""
        assert T._HEADER.size == _EXPECTED_HEADER_SIZE
        assert T._SAMPLE.size == _EXPECTED_SAMPLE_SIZE
        assert T._MAGIC == b"LGRTRJ01"
        assert T._VERSION == 1

    def test_to_bytes_matches_the_hand_written_layout(self):
        assert _two_sample_trajectory().to_bytes() == _EXPECTED_BYTES

    def test_from_bytes_reads_the_hand_written_layout(self):
        back = Trajectory.from_bytes(_EXPECTED_BYTES)
        assert len(back) == 2
        assert back.sample_hz == 100.0
        assert back.can_id == 8
        assert back.mount == "reverse"
        assert back.samples[1] == TrajectorySample(
            t=1.0, openness=0.5, position_rad=-0.5, velocity_rad_s=0.1,
            torque_nm=0.2)
        assert back.to_bytes() == _EXPECTED_BYTES, "读回来再写出去必须字节不变"

    def test_a_truncated_file_is_refused(self):
        """长度与头部声明的拍数不符 ⇒ 拒绝，而不是解析出半截轨迹。"""
        with pytest.raises(T.TrajectoryFormatError, match="truncated|trailing"):
            Trajectory.from_bytes(_EXPECTED_BYTES[:-1])
        with pytest.raises(T.TrajectoryFormatError, match="truncated|trailing"):
            Trajectory.from_bytes(_EXPECTED_BYTES + b"\x00" * 40)

    def test_a_foreign_file_is_refused(self):
        with pytest.raises(T.TrajectoryFormatError, match="magic"):
            Trajectory.from_bytes(b"NOPE0001" + _EXPECTED_BYTES[8:])
        with pytest.raises(T.TrajectoryFormatError, match="too short"):
            Trajectory.from_bytes(b"LGRTRJ01")

    def test_an_out_of_range_openness_is_refused(self):
        """openness 必须在 [0, 1] —— 这条正是录制端要做 clamp 的原因。"""
        bad = (_EXPECTED_BYTES[:_EXPECTED_HEADER_SIZE]
               + struct.pack("<5d", 0.0, -3.7e-06, 0.0, 0.0, 0.0)
               + struct.pack("<5d", 1.0, 0.5, -0.5, 0.1, 0.2))
        with pytest.raises(T.TrajectoryFormatError, match="outside"):
            Trajectory.from_bytes(bad)

    def test_duration_is_the_span_not_the_end_stamp(self):
        """首拍不在 t=0 的轨迹，时长仍是自己覆盖的那一段。"""
        far = Trajectory(samples=[TrajectorySample(10.0, 1.0, 0.0),
                                  TrajectorySample(12.0, 0.0, -1.14)],
                         rad_to_mm=74.958, pos_open_rad=-1.14)
        assert far.duration == pytest.approx(2.0)

    def test_openness_at_interpolates_and_clamps(self):
        traj = Trajectory(samples=[TrajectorySample(1.0, 0.0, 0.0),
                                   TrajectorySample(3.0, 1.0, -1.14)])
        assert traj.openness_at(0.0) == pytest.approx(0.0), "首拍之前夹住"
        assert traj.openness_at(2.0) == pytest.approx(0.5)
        assert traj.openness_at(99.0) == pytest.approx(1.0), "末拍之后夹住"

    def test_bare_names_land_in_the_trajectory_dir(self, tmp_path,
                                                   monkeypatch):
        monkeypatch.setenv("LITEGRIP_TRAJ_DIR", str(tmp_path))
        assert trajectory_dir() == str(tmp_path)
        assert resolve_path("pick") == str(tmp_path / "pick.lgt")
        assert resolve_path("pick.lgt") == str(tmp_path / "pick.lgt")
        assert resolve_path("sub/pick.lgt") == "sub/pick.lgt"
        assert resolve_path("/tmp/pick.lgt") == "/tmp/pick.lgt"

    def test_save_and_load_round_trip(self, tmp_path, monkeypatch):
        monkeypatch.setenv("LITEGRIP_TRAJ_DIR", str(tmp_path))
        written = _two_sample_trajectory().save("round_trip")
        assert written == str(tmp_path / "round_trip.lgt")
        back = Trajectory.load("round_trip")
        assert back.samples == _two_sample_trajectory().samples
        assert back.mount == "reverse"

    def test_the_sdk_reads_our_bytes_and_we_read_its(self, tmp_path):
        """跨实现互读。SDK 没装就跳过 —— 但**不是**不测：上面那几条字面量
        用例永远跑，钉的就是这个布局。"""
        sdk = _sdk_trajectory_module()
        if sdk is None:
            pytest.skip("litegrip SDK 未安装（布局已由字面量用例钉住）")

        ours = _two_sample_trajectory().to_bytes()
        theirs = sdk.Trajectory.from_bytes(ours)
        assert len(theirs) == 2
        assert theirs.mount == "reverse"
        assert [s.openness for s in theirs.samples] == [1.0, 0.5]

        # 反向：SDK 写、本模块读。SDK 的 Trajectory 与本地的是**两个类**，
        # 靠的就是字节流这一层。
        blob = theirs.to_bytes()
        assert blob == ours, "SDK 重写一遍也必须字节相同"
        assert Trajectory.from_bytes(blob).samples[1].torque_nm == pytest.approx(0.2)


class TestTrajectorySession:
    def test_recording_is_exclusive(self, sim):
        sim.record_start(rate_hz=20.0, zero_gravity=False)
        try:
            with pytest.raises(TrajectoryBusyError):
                sim.record_start(rate_hz=20.0, zero_gravity=False)
        finally:
            sim.record_stop(allow_empty=True)
        assert sim.trajectory_status() == {"active": False, "kind": None}

    def test_stop_without_start_raises(self, sim):
        with pytest.raises(TrajectoryNotActiveError):
            sim.record_stop()

    def test_blocking_play_refuses_to_loop(self, sim):
        with pytest.raises(ValueError, match="loop"):
            sim.play(_two_sample_trajectory(), loop=True)

    def test_an_empty_trajectory_cannot_be_played(self, sim):
        with pytest.raises(T.TrajectoryEmptyError):
            sim.play_start(Trajectory())

    def test_status_keys_match_the_sdk(self, sim):
        """状态字典的键名与 SDK 逐字一致 —— 日志解析不该关心是仿真还是真机。"""
        sim.record_start(rate_hz=20.0, zero_gravity=False)
        try:
            assert set(sim.trajectory_status()) == {
                "active", "kind", "samples", "rate_hz", "zero_gravity",
                "loop_hz", "error"}
        finally:
            sim.record_stop(allow_empty=True)

        traj = _two_sample_trajectory()
        sim.play_start(traj, align=False)
        try:
            status = sim.trajectory_status()
            assert set(status) == {
                "active", "kind", "samples", "frames", "speed", "loop",
                "completed", "openness", "loop_hz", "error"}
            assert status["kind"] == "play"
        finally:
            sim.play_stop()

    def test_a_stopped_clock_aborts_the_recorder(self, sim):
        """时钟不前进 ⇒ 报错，而不是在停住的时钟上无限追加采样。"""
        recorder = T.TrajectoryRecorder(sim, rate_hz=100.0, zero_gravity=False,
                                        monotonic_fn=lambda: 0.0)
        recorder.start()
        try:
            with pytest.raises(TrajectoryRecordingError, match="did not advance"):
                recorder.wait_for(2, timeout=2.0)
        finally:
            recorder.stop()
        assert recorder.sample_count == 1, "只该有 start() 那一拍参考样本"

    def test_a_dead_loop_is_not_returned_as_a_whole_recording(self, sim):
        recorder = T.TrajectoryRecorder(sim, rate_hz=100.0, zero_gravity=False,
                                        monotonic_fn=lambda: 0.0)
        recorder.start()
        try:
            recorder.wait_for(2, timeout=2.0)
        except TrajectoryRecordingError:
            pass
        recorder.stop()
        with pytest.raises(TrajectoryRecordingError):
            recorder.result()

    def test_record_then_replay_reproduces_the_move(self, sim):
        """端到端：主线程驱动一遍移动，录下来，挪走，再放回去。

        录制用 ``zero_gravity=False`` —— 仿真里手指推不动（查看器把鼠标留着
        控制相机），所以录的必然是"另起一个线程驱动、本线程只读"这一路，也正是
        SDK 为程序化录制准备的那一路。

        终点用**录到的那一拍**去比，而不是比一个想象中的目标位：``play_stop()``
        会就地保持（SDK 的 ``_hold_position()`` 也是就地保持），所以回放的物理
        终点会比最后一帧的目标差一点点。实测 40 mm 处的移动差 0.06 mm。
        """
        target_mm = 40.0
        assert sim.gap_mm() == pytest.approx(86.98, abs=0.1)

        sim.record_start(rate_hz=50.0, zero_gravity=False, max_samples=80)
        sim.goto(target_mm, duration=0.6)    # 阻塞到走完
        traj = sim.record_stop()
        sim.settle(0.2)
        taught_gap = sim.gap_mm()

        assert len(traj) > 10, "0.6s 的移动不该只录到个位数拍"
        assert traj.samples[0].openness == pytest.approx(1.0, abs=1e-3)
        assert traj.samples[-1].openness == pytest.approx(0.468, abs=0.01)
        assert traj.mount == "normal", "仿真端点闭合位数值更大"
        assert traj.rad_to_mm == pytest.approx(C.MM_SCALE / 1.14, rel=1e-6)
        assert taught_gap == pytest.approx(C.GAP_CLOSED_MM + target_mm, abs=0.1)

        # 挪回张开位再放，否则"回到教过的位置"这件事没有可观测的变化
        sim.goto(C.MM_SCALE, duration=0.6)
        assert sim.gap_mm() == pytest.approx(86.98, abs=0.5)

        status = sim.play(traj, align=True)
        assert status["completed"] is True
        assert status["error"] is None
        assert status["frames"] > 10
        sim.settle(0.3)
        assert sim.gap_mm() == pytest.approx(taught_gap, abs=0.3), (
            "回放应当把夹爪带回教过的位置")
        assert sim.trajectory_status() == {"active": False, "kind": None}

    def test_openness_is_clamped_to_the_format_range(self, sim):
        """两端的换算都夹在 [0, 1] —— 与 SDK 的 ``rad_to_openness`` 一致。

        这条不是锦上添花：``from_bytes`` 只接受 ``[0, 1]``，而夹爪顶到限位时
        PD 会轻微过冲（实测算出过 ``-3.7e-06``）。不夹的话录制端会写出自己的
        读取端拒收的文件。
        """
        cfg = sim.config
        assert T._frac_from_theta(C.POS_CLOSED_RAD, cfg) == pytest.approx(0.0)
        assert T._frac_from_theta(C.POS_OPEN_RAD, cfg) == pytest.approx(1.0)

        # "越过限位"的方向取决于端点的大小关系（反向安装会反过来），所以从
        # 端点自己推，不写死符号。
        outward = 1.0 if C.POS_CLOSED_RAD > C.POS_OPEN_RAD else -1.0
        past_closed = C.POS_CLOSED_RAD + outward * 0.05
        past_open = C.POS_OPEN_RAD - outward * 0.05
        assert T._frac_from_theta(past_closed, cfg) == 0.0, "越过闭合位要夹到 0"
        assert T._frac_from_theta(past_open, cfg) == 1.0, "越过张开位要夹到 1"

        assert T._theta_from_frac(-1.0, cfg) == pytest.approx(C.POS_CLOSED_RAD)
        assert T._theta_from_frac(2.0, cfg) == pytest.approx(C.POS_OPEN_RAD)

    def test_a_recording_can_always_be_loaded_back(self, sim, tmp_path,
                                                  monkeypatch):
        """端到端：贴着两端录一遍，写出来的文件必须自己读得回来。"""
        monkeypatch.setenv("LITEGRIP_TRAJ_DIR", str(tmp_path))
        sim.record_start(rate_hz=100.0, zero_gravity=False, max_samples=80)
        sim.close(duration=0.3)              # 快合拢，最容易过冲
        traj = sim.record_stop()

        assert all(0.0 <= s.openness <= 1.0 for s in traj.samples)
        assert len(Trajectory.load(traj.save("clamped"))) == len(traj)

    def test_disconnect_stops_a_running_replay(self, sim):
        """回放在自己的线程里下指令，断开时必须连它一起收掉。

        不收的话回放线程会在断开之后继续发帧、一路抛"未连接"直到自己停下 ——
        那是一堆噪声，不是错误处理。
        """
        sim.play_start(_two_sample_trajectory(), align=False, loop=True)
        player = sim._player
        assert player is not None
        assert _until(lambda: player.status()["frames"] > 0)

        sim.disconnect()

        assert _until(lambda: not player.is_playing)
        assert player.status()["error"] is None, "断开该是一次干净的停止"
        assert sim.trajectory_status() == {"active": False, "kind": None}
