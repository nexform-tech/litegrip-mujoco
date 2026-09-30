#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真专有能力的测试：循环骨架、窗口/输入层、世界查询、备用工件槽。

这些都不是 SDK 对等面上的东西，而是例程要用的仿真机制。分四组：

  循环骨架     connected()/pump()/sim_time 在无窗口、无线程时也必须能问、能答，
               因为例程有 --headless 分支，测试也不该需要图形环境。
  窗口/输入    KeyQueue 是回调线程与主线程之间唯一的通道；无窗口时所有窗口 API
               一律 no-op 而不是抛异常。
  运动语义     command_fraction 必须**立刻返回**、限速、限力，且限力不能漏给
               下一条指令。
  世界与槽位   scene.xml 的 4 个槽位。模型是编译期的，"放一个工件"只能是搬运
               一个已有的，所以槽位用尽、被覆盖、被 reset 复位都要钉住。
"""
from __future__ import annotations

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
    constants as C,
    world,
)
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
