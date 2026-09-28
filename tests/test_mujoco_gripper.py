#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MujocoGripper 的测试。

分三层：

  几何与单位   直接从 STL / MJCF 读数据，不经过 MujocoGripper —— 这些断言
               锁的是**模型本身**（网格是米还是毫米、开口是不是 87.000 mm、
               关节值扫一遍会不会穿模）。任何一条挂掉都说明 MJCF 被改坏了。
  行为         通过 MujocoGripper 的公开 API 验证仿真语义，并与 SDK 对齐。
  物理保真     那些"看起来像 bug、其实是照抄真机"的行为（close 不限力、
               grasp 超出请求力、空夹误报）固化成测试，防止后人"顺手修好"。
"""
from __future__ import annotations

import dataclasses
import enum
import inspect
import sys
import threading
import time
import types
from typing import List

import numpy as np
import pytest

import mujoco
from litegrip_mujoco import MujocoGripper, constants as C
# 从 shim 取，不要从 _fallback 取：装了 SDK 之后 shim 会把这个名字解析成
# `litegrip.exceptions.NotInitializedError`，而 _fallback 里那个是**另一个类对象**。
# 名字一样不代表 `except` 接得住 —— 这条正是对等性的要点。
from litegrip_mujoco._litegrip import NotInitializedError
from litegrip_mujoco.gripper import _resolve_model_path


def _sdk_exceptions():
    """装了 SDK 就返回它的 exceptions 模块，否则 None。"""
    try:
        from litegrip import exceptions  # type: ignore

        return vars(exceptions)
    except Exception:
        return None

# ══════════════════════════════════════════════════════════════════════════
# 夹具
# ══════════════════════════════════════════════════════════════════════════


@pytest.fixture
def raw():
    """裸的 (MjModel, MjData)，停在张开位。"""
    model = mujoco.MjModel.from_xml_path(_resolve_model_path(None))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


@pytest.fixture
def raw_scene():
    """演示场景的裸 (MjModel, MjData)。"""
    model = mujoco.MjModel.from_xml_path(_resolve_model_path("scene.xml"))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "fixture")
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)
    return model, data


@pytest.fixture
def sim():
    """无窗口仿真，用完自动断开。"""
    g = MujocoGripper(render=False)
    g.connect()
    g.enable()
    yield g
    g.disconnect()


@pytest.fixture
def scene():
    """无窗口演示场景（带工件与夹具）。"""
    g = MujocoGripper(model_path="scene.xml", render=False)
    g.connect()
    g.enable()
    yield g
    g.disconnect()


def _verts(model, name):
    """取某个网格的顶点数组。"""
    mid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MESH, name)
    assert mid >= 0, f"模型里没有网格 {name!r}"
    adr = int(model.mesh_vertadr[mid])
    num = int(model.mesh_vertnum[mid])
    return np.array(model.mesh_vert[adr:adr + num])


def _geom_id(model, name):
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    assert gid >= 0, f"模型里没有 geom {name!r}"
    return gid


def _set_q(model, data, q):
    """把两个指关节都设成 q，重算前向运动学。"""
    for a in range(model.nu):
        jid = int(model.actuator_trnid[a, 0])
        data.qpos[int(model.jnt_qposadr[jid])] = q
    mujoco.mj_forward(model, data)


def _pad_gap(model, data):
    """两指夹持面之间的开口 (m)，直接由 geom 位置算，不经过 MujocoGripper。"""
    r = _geom_id(model, "finger_right_col_pad")
    l = _geom_id(model, "finger_left_col_pad")
    right_face = data.geom_xpos[r][0] + model.geom_size[r][0]
    left_face = data.geom_xpos[l][0] - model.geom_size[l][0]
    return left_face - right_face


# ══════════════════════════════════════════════════════════════════════════
# 几何与单位
# ══════════════════════════════════════════════════════════════════════════


class TestGeometryAndUnits:
    def test_mesh_units(self, raw):
        """STL 的单位是米，不是毫米。

        这是最容易被静默搞错的一条：STL 没有单位，如果导出时用了毫米，
        模型会大一千倍 —— 但 MuJoCo 照样能编译、照样能跑，只是所有数字都错。
        """
        model, _ = raw
        for name in ("base_link", "finger_right_mesh", "finger_left_mesh"):
            v = _verts(model, name)
            span = v.max(axis=0) - v.min(axis=0)
            assert span.max() < 0.5, (
                f"{name} 的包围盒跨度 {span.max():.4f} —— 像是毫米单位"
            )
            assert span.max() > 0.01, (
                f"{name} 的包围盒跨度 {span.max():.6f} —— 太小了"
            )

    def test_mesh_bbox_matches_urdf(self, raw):
        """三个网格的包围盒与 litegrip-urdf 里的一致（0.1% 容差）。"""
        model, _ = raw
        expected = {
            "base_link": ([-0.02868784, -0.04861952, -0.07421798],
                          [0.02864786, 0.04276557, 0.07354394]),
            "finger_right_mesh": ([-0.01113566, -0.02806435, -0.02852861],
                                  [0.01304337, 0.03344524, 0.06688434]),
            "finger_left_mesh": ([-0.01304337, -0.03344524, -0.02852861],
                                 [0.01113566, 0.02806435, 0.06688434]),
        }
        for name, (lo, hi) in expected.items():
            v = _verts(model, name)
            np.testing.assert_allclose(v.min(axis=0), lo, rtol=1e-3, atol=1e-6)
            np.testing.assert_allclose(v.max(axis=0), hi, rtol=1e-3, atol=1e-6)

    def test_finger_mirror(self, raw):
        """左右指是 x/y 双取反的镜像，且两个文件内容不同（不能复用同一个 STL）。

        注意**不能逐顶点比对**：两个 STL 的顶点顺序不同（md5 不同），
        逐点比会得到 0.023 m 的"误差"。正确做法是当成点集比。
        """
        model, _ = raw
        r = _verts(model, "finger_right_mesh")
        l = _verts(model, "finger_left_mesh")

        assert r.shape == l.shape
        assert not np.allclose(r, l), "左右指用了同一个网格 —— URDF 里它们是两个文件"

        np.testing.assert_allclose(np.sort(l[:, 0]), np.sort(-r[:, 0]), atol=1e-9)
        np.testing.assert_allclose(np.sort(l[:, 1]), np.sort(-r[:, 1]), atol=1e-9)
        np.testing.assert_allclose(np.sort(l[:, 2]), np.sort(r[:, 2]), atol=1e-9)

    def test_visual_mesh_not_rotated_by_mujoco(self, raw):
        """视觉网格的 geom 姿态是单位阵。

        MuJoCo 编译期会对网格做主轴对齐，并把 geom 的默认 pos/quat 设成同一个
        变换来抵消。一旦手写 pos/quat 覆盖掉这个补偿，零件就会歪掉 —— 而且
        外观上可能只是"稍微不对"，很难发现。这条断言守住它。
        """
        model, data = raw
        for name in ("base_vis", "finger_right_vis", "finger_left_vis"):
            gid = _geom_id(model, name)
            assert model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_MESH
            # mesh 的编译期变换与 geom 的应当一致（MuJoCo 就是这么补偿的）
            mid = int(model.geom_dataid[gid])
            np.testing.assert_allclose(
                model.mesh_quat[mid], model.geom_quat[gid], atol=1e-9,
                err_msg=f"{name} 的 geom_quat 覆盖了网格的对齐补偿",
            )
            np.testing.assert_allclose(
                model.mesh_pos[mid], model.geom_pos[gid], atol=1e-9,
                err_msg=f"{name} 的 geom_pos 覆盖了网格的对齐补偿",
            )

    def test_jaw_geometry(self, raw):
        """张口 87.000 mm，行程两端都是精确的。

        87.000 来自网格几何（两指内侧面在 q=0 时位于 x=±0.0435），而 URDF
        注释里的 85.452 是卡尺实测的**行程**。两者相差 0.04 mm 是已知的、
        刻意保留的矛盾，见 assets/litegrip.xml 头部。
        """
        model, data = raw

        _set_q(model, data, 0.0)
        assert _pad_gap(model, data) == pytest.approx(0.087000, abs=1e-9)

        _set_q(model, data, C.STROKE)
        assert _pad_gap(model, data) == pytest.approx(0.001548, abs=1e-6)

        # 行程 = 开口的变化量，不是开口本身
        assert C.MM_SCALE == pytest.approx(85.452, abs=1e-9)

    def test_jaw_gap_is_linear_in_q(self, raw):
        """开口随 q 线性闭合，斜率 2（两个指各走一份）。"""
        model, data = raw
        _set_q(model, data, 0.0)
        gap0 = _pad_gap(model, data)
        _set_q(model, data, C.STROKE / 2)
        gap_half = _pad_gap(model, data)
        _set_q(model, data, C.STROKE)
        gap1 = _pad_gap(model, data)

        assert (gap0 - gap_half) == pytest.approx(gap_half - gap1, abs=1e-9)
        assert (gap0 - gap1) == pytest.approx(2 * C.STROKE, abs=1e-9)

    def test_boot_pose_no_contact(self, raw):
        """上电姿态（张开）下没有任何接触。"""
        model, data = raw
        assert data.ncon == 0, (
            f"张开位有 {data.ncon} 个接触 —— 视觉网格被当成了碰撞体？"
        )

    def test_visual_geoms_do_not_collide(self, raw):
        """视觉网格必须关掉碰撞。

        MuJoCo 没有凹网格碰撞（一律凸化），本夹爪凸化后本体恰好填满指的滑道，
        于是任意关节值下都恒定产生 6 个接触、5.25 mm 穿透。这不是调参能救的。
        """
        model, _ = raw
        for name in ("base_vis", "finger_right_vis", "finger_left_vis"):
            gid = _geom_id(model, name)
            assert model.geom_contype[gid] == 0
            assert model.geom_conaffinity[gid] == 0

        for name in ("finger_right_col_body", "finger_right_col_pad",
                     "finger_left_col_body", "finger_left_col_pad"):
            gid = _geom_id(model, name)
            assert model.geom_contype[gid] == 1
            assert model.geom_conaffinity[gid] == 1

    def test_no_penetration_across_stroke(self, raw):
        """整个行程扫一遍，任何两个碰撞体都不许互相穿透。

        直接锁死上面那条坑：如果哪天有人把视觉网格的 contype 打开，
        这里会立刻炸。
        """
        model, data = raw
        worst = 0.0
        offenders = 0
        for q in np.linspace(0.0, C.STROKE, 200):
            _set_q(model, data, float(q))
            for i in range(data.ncon):
                dist = float(data.contact[i].dist)
                worst = min(worst, dist)
                if dist < -1e-6:
                    offenders += 1
        assert offenders == 0, f"{offenders} 个接触穿透，最深 {worst * 1e3:.3f} mm"

    def test_contact_geoms_span_the_pad(self, raw):
        """夹持面 collision box 的 +x 面精确落在网格的 x=0.0235。"""
        model, data = raw
        gid = _geom_id(model, "finger_right_col_pad")
        pad = float(model.geom_size[gid][0])
        # finger_right 的关节原点 x=-0.067；q=0 时夹持面应在世界系 -0.0435
        body = int(model.geom_bodyid[gid])
        assert model.body_pos[body][0] == pytest.approx(-0.067, abs=1e-12)
        assert model.geom_pos[gid][0] + pad == pytest.approx(0.0235, abs=1e-9)

    def test_finger_coupling_is_rigid(self, raw):
        """两指由刚性等式耦合 —— 真机只有一台电机。

        缺了这条约束，模型描述的就是一台双电机夹爪：夹偏心工件时两指会各自
        退让、不去把工件拨正，抓取行为与真机不同。
        """
        model, _ = raw
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EQUALITY, i)
                 for i in range(model.neq)]
        assert "finger_coupling" in names, f"等式列表是 {names}"
        eid = names.index("finger_coupling")
        assert model.eq_type[eid] == mujoco.mjtEq.mjEQ_JOINT
        np.testing.assert_allclose(model.eq_data[eid][:5], [0, 1, 0, 0, 0],
                                   atol=1e-12)


# ══════════════════════════════════════════════════════════════════════════
# 单位换算
# ══════════════════════════════════════════════════════════════════════════


class TestUnits:
    def test_mm_mapping_roundtrip(self):
        """mm ↔ q ↔ θ 的往返换算必须精确。"""
        for mm in np.linspace(0.0, C.MM_SCALE, 51):
            q = C.mm_to_q(float(mm))
            assert C.q_to_mm(q) == pytest.approx(mm, abs=1e-9)
            th = C.mm_to_theta(float(mm))
            assert C.theta_to_mm(th) == pytest.approx(mm, abs=1e-9)

    def test_endpoints(self):
        """两个端点的对应关系。"""
        assert C.mm_to_q(0.0) == pytest.approx(C.STROKE, abs=1e-12)
        assert C.mm_to_q(C.MM_SCALE) == pytest.approx(0.0, abs=1e-12)
        assert C.mm_to_theta(0.0) == pytest.approx(C.POS_CLOSED_RAD, abs=1e-12)
        assert C.mm_to_theta(C.MM_SCALE) == pytest.approx(C.POS_OPEN_RAD, abs=1e-12)

    def test_gap_mapping(self):
        """开口 mm ↔ q：闭合时仍有 1.548 mm 的机械间隙。"""
        assert C.gap_mm_to_q(C.GAP_OPEN_MM) == pytest.approx(0.0, abs=1e-12)
        assert C.gap_mm_to_q(C.GAP_CLOSED_MM) == pytest.approx(C.STROKE, abs=1e-9)
        assert C.q_to_gap_mm(0.0) == pytest.approx(C.GAP_OPEN_MM, abs=1e-9)
        # 20 mm 工件的闭合位
        assert C.gap_mm_to_q(20.0) == pytest.approx(0.033500, abs=1e-6)

    def test_frac_open_range(self):
        """frac_open ∈ [0, 1]，0 = 闭合。"""
        assert C.frac_open_from_q(C.STROKE) == pytest.approx(0.0, abs=1e-12)
        assert C.frac_open_from_q(0.0) == pytest.approx(1.0, abs=1e-12)
        assert C.q_from_frac_open(1.0) == pytest.approx(0.0, abs=1e-12)
        assert C.q_from_frac_open(0.0) == pytest.approx(C.STROKE, abs=1e-12)

    def test_stroke_basis_is_85_452(self):
        """行程口径是 85.452 mm，不是 SDK 名义的 120 mm。

        两个数差 40%。选 85.452 的意义在于：真机把 max_stroke_mm 设成 85.452
        并重新标定后，两侧报出的毫米就是同一个量，跨设备零换算。
        """
        assert C.MM_SCALE == pytest.approx(85.452, abs=1e-9)
        assert C.STROKE * 2 * 1000 == pytest.approx(C.MM_SCALE, abs=1e-6)
        # R 是"每米关节位移对应多少弧度电机角"（丝杠比），≈26.68 rad/m。
        # 容易写反成它的倒数 —— 写反了 θ 换算会差 700 倍。
        assert C.R == pytest.approx(
            (C.POS_CLOSED_RAD - C.POS_OPEN_RAD) / C.STROKE, rel=1e-12)
        assert C.R == pytest.approx(26.6816, rel=1e-4)

    def test_theta_convention_invariant(self):
        """仿真侧的 θ 端点满足 closed > open。

        SDK 的默认值 POS_OPEN_RAD=+1.14 / POS_CLOSED_RAD=0.0 违反这个不变量，
        并让 goto_rad() 的 clamp 对任意输入都返回 1.14。仿真侧不复刻这个 bug。
        """
        assert C.POS_CLOSED_RAD > C.POS_OPEN_RAD
        assert C.clamp_theta(999.0) == pytest.approx(C.POS_CLOSED_RAD, abs=1e-12)
        assert C.clamp_theta(-999.0) == pytest.approx(C.POS_OPEN_RAD, abs=1e-12)

    def test_force_units(self):
        """力 ↔ 力矩：SDK 的 NM_TO_N = 10。"""
        assert C.n_to_nm(10.0) == pytest.approx(1.0, abs=1e-12)
        assert C.nm_to_n(1.0) == pytest.approx(10.0, abs=1e-12)


# ══════════════════════════════════════════════════════════════════════════
# 执行器与物理量纲
# ══════════════════════════════════════════════════════════════════════════


class TestActuator:
    def test_actuator_units(self, raw):
        """gear=10 让 ctrl 的数值等于 SDK 的 tau(Nm)。

        ``actuator_force`` 是**过 gear 之前**的值，``qfrc_actuator`` 是之后。
        搞混这两个会得到 10 倍的力误差。
        """
        model, data = raw
        assert model.nu == 2
        np.testing.assert_allclose(model.actuator_gear[:, 0], [10.0, 10.0], atol=1e-12)
        np.testing.assert_allclose(model.actuator_ctrlrange, [[-10, 10], [-10, 10]],
                                   atol=1e-12)
        np.testing.assert_allclose(model.actuator_forcerange, [[-100, 100], [-100, 100]],
                                   atol=1e-12)

        data.ctrl[:] = 1.0
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(data.actuator_force[:2], [1.0, 1.0], atol=1e-12)
        np.testing.assert_allclose(data.qfrc_actuator[:2], [10.0, 10.0], atol=1e-12)

    def test_armature_is_present(self, raw):
        """armature 不能缺。

        DM4310 转子惯量经 10:1 减速 + 26.68 rad/m 的丝杠折算约 0.85 kg，
        是指自身质量的 24 倍。缺了它，dt=2ms、kp=150 下 vmax 冲到 2.93 m/s
        且不收敛。
        """
        model, _ = raw
        for a in range(model.nu):
            dof = int(model.jnt_dofadr[model.actuator_trnid[a, 0]])
            assert model.dof_armature[dof] == pytest.approx(0.85, abs=1e-12)

    def test_timestep(self, raw):
        """dt=0.001。实测 dt=2ms 时 kd>=5 不收敛。"""
        model, _ = raw
        assert model.opt.timestep == pytest.approx(0.001, abs=1e-12)
        assert model.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        assert model.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC

    def test_free_joint_does_not_confuse_joint_count(self, raw_scene):
        """scene.xml 的工件带 freejoint，nv=8 但执行器只有 2 个。

        按 nv 建控制器会把 8 个力矩往 2 个执行器里塞 —— 这是实际踩过的坑。
        """
        model, _ = raw_scene
        assert model.nv == 8
        assert model.nu == 2

        g = MujocoGripper(model_path="scene.xml", render=False)
        try:
            assert g._n_joints == 2
            assert len(set(g._qpos_idx)) == 2
            assert max(g._qpos_idx) < len(g._data.qpos)
        finally:
            g.disconnect()


# ══════════════════════════════════════════════════════════════════════════
# 生命周期与状态
# ══════════════════════════════════════════════════════════════════════════


class TestLifecycle:
    def test_context_manager(self):
        with MujocoGripper(render=False) as g:
            assert g.is_connected
            assert g.is_enabled
        assert not g.is_connected

    def test_repr(self):
        g = MujocoGripper(render=False)
        try:
            assert "MujocoGripper" in repr(g)
        finally:
            g.disconnect()

    def test_requires_connect(self):
        """未连接时抛 ``NotInitializedError`` —— 与 SDK 同一个异常类。"""
        g = MujocoGripper(render=False)
        try:
            with pytest.raises(NotInitializedError):
                g.get_position()
        finally:
            g.disconnect()

    def test_requires_enable(self):
        g = MujocoGripper(render=False)
        g.connect()
        try:
            with pytest.raises(NotInitializedError):
                g.open()
        finally:
            g.disconnect()

    def test_exception_types_match_sdk(self):
        """异常类型必须与 SDK 一致，否则同一段 try/except 在两侧行为不同。"""
        from litegrip_mujoco._litegrip import _fallback as fb

        assert issubclass(fb.NotInitializedError, fb.LiteGripError)
        assert issubclass(fb.LiteGripError, Exception)

        sdk_exc = _sdk_exceptions()
        if sdk_exc is None:
            pytest.skip("litegrip SDK 未安装")
        for name in ("LiteGripError", "NotInitializedError"):
            assert name in sdk_exc, f"SDK 里没有 {name}"

        # 同名还不够，必须是**同一个类对象**。真机上 `except litegrip.LiteGripError`
        # 要能接住仿真抛出的异常，靠的就是 shim 把这个名字指向 SDK 的类。
        import litegrip as _sdk_pkg

        from litegrip_mujoco import _litegrip as shim

        assert shim.HAS_SDK
        assert shim.NotInitializedError is _sdk_pkg.NotInitializedError, (
            "shim 没有把 NotInitializedError 解析到 SDK 的类上 —— "
            "两侧的 except 子句会各接各的"
        )

        # 抛出来的那个必须就是它。上面 test_requires_connect 用 pytest.raises
        # 验的是同一个契约，这里再钉一次抛出侧，防止实现改回本地异常类。
        g = MujocoGripper(render=False)
        try:
            with pytest.raises(_sdk_pkg.NotInitializedError):
                g.get_position()
        finally:
            g.disconnect()

    def test_bad_model_path(self):
        with pytest.raises((FileNotFoundError, ValueError)):
            MujocoGripper(model_path="nope_does_not_exist.xml", render=False)

    def test_disconnect_is_idempotent(self):
        g = MujocoGripper(render=False)
        g.connect()
        g.disconnect()
        g.disconnect()
        assert not g.is_connected


# ══════════════════════════════════════════════════════════════════════════
# 查看器的生命周期
#
# 这一组**不需要显示环境**：mujoco.viewer 被替换成假模块，所以 CI 上也能跑。
# 被测的是顺序与所有权，不是渲染本身。
# ══════════════════════════════════════════════════════════════════════════


class _FakeViewer:
    """够用的假查看器：只记录被 sync/close 过没有。"""

    def __init__(self):
        self.synced = 0
        self.closed = False
        self.running = True

    def is_running(self) -> bool:
        return self.running

    def sync(self) -> None:
        self.synced += 1

    def close(self) -> None:
        self.closed = True
        self.running = False


class TestViewerLifecycle:
    """查看器必须在仿真线程**启动之前**建好。

    `launch_passive()` 内部会对同一个 mjData 调 `mj_forward()`，而 `_sim_loop`
    同时在 `mj_step()` 同一个 mjData。两者重叠会让 mjData 的 arena 无法扩容，
    报 `mj_makeConstraint: nefc under-allocation`，更糟时直接段错误 —— 而且
    是间歇性的（碰不上就没事），所以必须有测试钉住顺序，不能靠"跑一次没崩"。
    """

    @staticmethod
    def _install(monkeypatch, record: List[List[str]]):
        """把 mujoco.viewer 换成假模块，记录调用时活着的仿真线程。

        ``sys.modules`` 和 ``mujoco.viewer`` 属性都要换：``import mujoco.viewer``
        之后代码访问的是 ``mujoco.viewer`` 这个**属性**，只塞 sys.modules 不够。
        """
        mod = types.ModuleType("mujoco.viewer")

        def launch_passive(model, data, **kwargs):
            record.append(
                [t.name for t in threading.enumerate() if t.name == "litegrip_sim"]
            )
            return _FakeViewer()

        mod.launch_passive = launch_passive
        monkeypatch.setitem(sys.modules, "mujoco.viewer", mod)
        monkeypatch.setattr(mujoco, "viewer", mod, raising=False)

    def test_no_sim_thread_when_viewer_is_created(self, monkeypatch):
        """回归：建窗口时不允许已经有仿真线程在跑。"""
        seen: List[List[str]] = []
        self._install(monkeypatch, seen)

        g = MujocoGripper(render=True)
        try:
            assert len(seen) == 1, "查看器没有被建出来"
            assert seen[0] == [], (
                f"建查看器时已经有仿真线程在跑：{seen[0]} —— "
                "launch_passive 的 mj_forward 会和 _sim_loop 的 mj_step 抢 mjData"
            )
        finally:
            g.disconnect()

    def test_render_does_not_imply_connected(self, monkeypatch):
        """``render=True`` 只开窗，不代替 ``connect()``。"""
        self._install(monkeypatch, [])
        g = MujocoGripper(render=True)
        try:
            assert not g.is_connected
            assert g._viewer is not None
            g.connect()
            assert g.is_connected
        finally:
            g.disconnect()

    def test_launch_viewer_is_idempotent(self, monkeypatch):
        """重复开窗不会把旧窗口漏掉（旧窗口会变成没人 sync 的孤儿）。"""
        seen: List[List[str]] = []
        self._install(monkeypatch, seen)
        g = MujocoGripper(render=True)
        try:
            g.launch_viewer()
            g.launch_viewer()
            assert len(seen) == 1
        finally:
            g.disconnect()

    def test_disconnect_closes_viewer(self, monkeypatch):
        """正常路径：线程退干净了就关窗。"""
        self._install(monkeypatch, [])
        g = MujocoGripper(render=True)
        g.connect()
        viewer = g._viewer
        g.disconnect()
        assert viewer.closed

    def test_disconnect_skips_close_when_thread_wedged(self, monkeypatch):
        """线程卡住时**不要**关窗。

        join 超时说明它可能正卡在 ``sync()`` 里，此时关窗会和它抢 mjData，
        制造一个"退出瞬间段错误"。宁可把窗口留给进程退出回收。
        """
        self._install(monkeypatch, [])
        wedged = threading.Event()

        def wedged_loop(self):  # 无视 _running，永不退出
            wedged.wait(timeout=10.0)

        monkeypatch.setattr(MujocoGripper, "_sim_loop", wedged_loop)
        g = MujocoGripper(render=True)
        g.connect()
        viewer = g._viewer
        try:
            g.disconnect()
            assert not viewer.closed, (
                "线程还活着就关窗 —— 这会在退出瞬间和 sync() 抢 mjData"
            )
        finally:
            wedged.set()
            g.disconnect()

    def test_viewer_failure_raises_runtimeerror(self, monkeypatch):
        """开不出窗口要抛 RuntimeError，例程靠它退回无窗口。"""
        mod = types.ModuleType("mujoco.viewer")

        def launch_passive(model, data, **kwargs):
            raise mujoco.FatalError("no GL context")

        mod.launch_passive = launch_passive
        monkeypatch.setitem(sys.modules, "mujoco.viewer", mod)
        monkeypatch.setattr(mujoco, "viewer", mod, raising=False)

        with pytest.raises(RuntimeError):
            MujocoGripper(render=True)


class TestState:
    def test_get_state(self, sim):
        st = sim.get_state()
        assert st.is_enabled
        assert not st.is_error
        assert st.position_mm == pytest.approx(C.MM_SCALE, abs=0.05)
        assert st.position_rad == pytest.approx(C.POS_OPEN_RAD, abs=1e-3)
        assert st.force_n == pytest.approx(0.0, abs=0.5)

    def test_position_never_negative(self, sim):
        """闭合位不能报出负行程。

        软关节限位在撞到闭合位时会过冲约 2 µm —— 不夹的话 position_mm
        会是 -0.002，真机不可能出现这个值。
        """
        sim.close(duration=0.5)
        assert sim.get_position() >= 0.0
        assert sim.get_position() == pytest.approx(0.0, abs=0.01)

    def test_temperature_is_plausible(self, sim):
        mos, coil = sim.get_temperature()
        assert 20 <= mos <= 120
        assert 20 <= coil <= 120
        assert mos >= 25.0

    def test_get_info(self, sim):
        info = sim.get_info()
        assert info is not None

    def test_is_grasped_false_when_idle(self, sim):
        assert sim.is_grasped() is False

    def test_state_is_never_stale(self, sim):
        """仿真态永远是最新的：``is_stale`` 必须为 False。

        SDK 把 ``GripperState.data_age_s`` 的默认值设成 ``inf``，因为真机上
        "没有数据"是真的会发生 —— 失能的电机不主动发状态帧，``get_state()``
        返回的是缓存或构造默认值。**仿真不能照抄这个默认值**：状态是从
        ``mjData`` 直接算出来的，定义上瞬时且精确。照抄的话 ``is_stale``
        恒为 True，`if state.is_stale: 丢弃读数` 会把每一次好读数都丢掉。

        回归测试：``get_state()`` 构造 ``GripperState`` 时必须显式传
        ``data_age_s=0.0``。SDK 加这个字段时这里就漏过一次。
        """
        st = sim.get_state()
        assert st.data_age_s == 0.0
        assert st.is_stale is False
        assert st.has_data is True

    def test_state_stays_fresh_while_disabled(self, sim):
        """失能状态下读数同样不陈旧 —— 真机恰恰是反过来的。"""
        sim.disable()
        st = sim.get_state()
        assert st.is_stale is False
        assert st.data_age_s == 0.0

    def test_refresh_status_is_always_satisfied(self, sim):
        """``refresh_status()`` 在仿真里恒 True，且失能时也成立。

        真机上这个方法是为了**失能时也能读到位置**（发 0xCC 主动要一帧）。
        仿真没有 CAN 链路，状态随时可读，所以直接返回 True；但方法必须存在，
        否则真机代码换到仿真会变成 ``AttributeError``。
        """
        assert sim.refresh_status() is True
        assert sim.refresh_status(timeout_s=0.01) is True
        sim.disable()
        assert sim.refresh_status() is True

    def test_dry_run_gripper_also_has_refresh_status(self):
        """虚拟夹爪同样是 LiteGrip 形态的，不能缺这个方法。"""
        from litegrip_mujoco import DryRunGripper

        dry = DryRunGripper(realtime=False, noise=False)
        dry.connect()
        dry.enable()
        try:
            assert dry.get_state().is_stale is False
            assert dry.refresh_status() is True
        finally:
            dry.disconnect()


# ══════════════════════════════════════════════════════════════════════════
# 运动
# ══════════════════════════════════════════════════════════════════════════


class TestMotion:
    def test_open_close_reach_endpoints(self, sim):
        sim.close(duration=0.6)
        assert sim.get_position() == pytest.approx(0.0, abs=0.05)
        assert sim.gap_mm() == pytest.approx(C.GAP_CLOSED_MM, abs=0.15)

        sim.open(duration=0.6)
        assert sim.get_position() == pytest.approx(C.MM_SCALE, abs=0.05)
        assert sim.gap_mm() == pytest.approx(C.GAP_OPEN_MM, abs=0.15)

    def test_goto(self, sim):
        for mm in (0.0, 20.0, 42.726, 85.452):
            sim.goto(mm, duration=0.5)
            assert sim.get_position() == pytest.approx(mm, abs=0.05)

    def test_goto_matches_gap(self, sim):
        """行程口径与开口口径差一个恒定的 1.548 mm。"""
        for mm in (0.0, 20.0, 40.0):
            sim.goto(mm, duration=0.5)
            assert sim.gap_mm() == pytest.approx(mm + C.GAP_CLOSED_MM, abs=0.1)

    def test_duration_is_wallclock(self, sim):
        """``duration`` 是墙上秒，不是仿真秒。

        仿真比实时快很多倍，如果照抄 SDK 的"睡 1 秒"，``open(duration=1.0)``
        会在几毫秒内返回，真机与仿真的时间语义就分叉了。
        """
        sim.goto(0.0, duration=0.4)
        t0 = time.monotonic()
        sim.open(duration=1.0)
        elapsed = time.monotonic() - t0
        assert elapsed == pytest.approx(1.0, abs=0.08), f"实际用了 {elapsed:.3f}s"

    def test_move_at_speed_is_constant(self, sim):
        """恒速运动：路程 / 用时 ≈ 请求速度。"""
        for speed in (80.0, 20.0):
            sim.goto(C.MM_SCALE, duration=0.6)
            sim.settle(0.2)
            start = sim.get_position()
            t0 = time.monotonic()
            sim.move_at_speed(0.0, speed_mm_s=speed)
            dt = time.monotonic() - t0
            travelled = start - sim.get_position()
            assert travelled / dt == pytest.approx(speed, rel=0.10)

    def test_theta_and_mm_agree(self, sim):
        """get_position_rad() 与 get_position() 是同一个量的两种写法。"""
        sim.goto(30.0, duration=0.5)
        assert C.theta_to_mm(sim.get_position_rad()) == pytest.approx(
            sim.get_position(), abs=1e-3)

    def test_set_frac_open(self, sim):
        for frac in (0.0, 0.25, 0.5, 1.0):
            sim.set_frac_open(frac)
            assert sim.frac_open() == pytest.approx(frac, abs=1e-6)

    def test_reset_and_settle(self, sim):
        sim.close(duration=0.5)
        sim.reset("open")
        sim.settle(0.1)
        assert sim.get_position() == pytest.approx(C.MM_SCALE, abs=0.05)

    def test_stop_interrupts_motion(self, sim):
        sim.goto(C.MM_SCALE, duration=0.5)
        sim.goto(0.0, duration=5.0)  # 起一段很长的运动
        sim.stop()
        assert not sim.is_moving()


# ══════════════════════════════════════════════════════════════════════════
# 力的语义（照抄 SDK 的行为，不是缺陷）
# ══════════════════════════════════════════════════════════════════════════


class TestForceSemantics:
    def test_close_cannot_limit_force(self, scene):
        """``close(force_n=…)`` 限不住力 —— 固化成测试，别"顺手修好"。

        目标一路指向闭合位，指爪停在工件表面时留下约 9 mm 的位置误差，
        kp·Δθ 远大于前馈 n_to_nm(force_n)，执行器直接饱和。真机同理。
        """
        for n in (0.0, 5.0, 10.0, 20.0):
            scene.reset("fixture")
            scene.settle(0.1)
            scene.close(force_n=n, duration=0.8)
            assert scene.get_torque() == pytest.approx(C.TAU_MAX, abs=0.05), (
                f"close(force_n={n}) 居然没有饱和 —— 力控语义变了"
            )

    def test_grasp_force_exceeds_request(self, scene):
        """``grasp(force_n)`` 的实际夹持力**大于**请求值。

        堵转确认窗口（5×10 ms）里指爪还在往前走，``q_target`` 被改写到窗口
        末尾的位置，于是留下一段固定的位置误差。真机同构（同样的 5 cycles
        × 10 ms），所以这是**照抄**来的行为。

        这里锁的是 assets/litegrip.xml 头部那张实测表的数值；如果谁动了
        solref / 接触刚度，这条会立刻指出该同步更新文档。
        """
        expected = {5.0: 10.8, 10.0: 15.5, 20.0: 24.9}
        measured = {}
        for n in sorted(expected):
            scene.reset("fixture")
            scene.settle(0.1)
            scene.grasp(force_n=n, duration=3.0)
            measured[n] = scene.get_force()
            assert measured[n] == pytest.approx(expected[n], rel=0.10), (
                f"grasp({n}) 得到 {measured[n]:.3f} N，与文档里的 "
                f"{expected[n]} N 不符；改过接触参数就要同步更新 XML 里的表"
            )
            assert measured[n] > n, "实际力不应小于请求值"

        # 力对 force_n 单调 —— 所以它可以当带偏置的开环力控用
        assert measured[5.0] < measured[10.0] < measured[20.0]

    def test_set_force_first_call_carries_preload_error(self, scene):
        """``set_force(N)`` 的**第一次**调用会带上一次受力的位置误差。

        目标在调用瞬间被改写成当前位置。此时指爪还被上一次的力压在工件上
        （这里是 grasp(10 N) 留下的 15.5 N），工件被压陷；力矩一降，工件把
        指爪顶回去，而目标还停在旧位置，于是留下一段位置误差 kp·Δθ。
        实测请求 2 N 得到 2.878 N（+44%），请求 20 N 得到 19.35 N（−3%）。

        真机同构 —— 这不是仿真缺陷，是"目标=当前"这个语义的固有后果。
        """
        scene.reset("fixture")
        scene.settle(0.3)
        scene.grasp(force_n=10.0, duration=3.0)
        scene.release_fixture()
        scene.settle(2.0)
        assert scene.get_force() > 15.0, "起点应当带着 grasp(10 N) 的预载"

        scene.set_force(2.0, duration=0.8)
        first = scene.get_force()
        assert first > 2.0 * 1.2, (
            f"第一次 set_force(2.0) 得到 {first:.3f} N —— 预载误差被抹平了？"
        )

    def test_set_force_converges_on_repeat(self, scene):
        """重复下发同一个 ``set_force(N)`` 会收敛到请求值（0.5% 以内）。

        第二次调用是在"已经被 2 N 力压着"的状态下重新取当前位置，误差不再
        累积。所以 ``set_force`` 的正确用法是**连续下发**，而不是发一次就完事。
        """
        scene.reset("fixture")
        scene.settle(0.3)
        scene.grasp(force_n=10.0, duration=3.0)
        scene.release_fixture()
        scene.settle(2.0)

        for n in (2.0, 5.0, 10.0, 20.0):
            for _ in range(3):
                scene.set_force(n, duration=0.8)
            measured = scene.get_force()
            assert measured == pytest.approx(n, rel=0.005), (
                f"set_force({n}) 收敛到 {measured:.3f} N"
            )

    def test_grasp_holds_part_after_fixture_release(self, scene):
        """抓取是否真的成立，用"松开夹具后工件掉不掉"来验证。

        这是本仓库里唯一一条**端到端**的抓取断言：它不是读力矩推算值，
        而是让工件真的只靠摩擦力留在指间。
        """
        obj = scene.model.body("object").id
        scene.reset("fixture")
        scene.settle(0.2)
        z0 = float(scene.data.xpos[obj][2])

        assert scene.grasp(force_n=10.0, duration=3.0) is True
        scene.release_fixture()
        scene.settle(2.0)

        dz = abs(float(scene.data.xpos[obj][2]) - z0)
        assert dz < 0.005, f"工件掉了 {dz * 1e3:.3f} mm"

    def test_open_drops_part(self, scene):
        """张开后工件落到地板上（地板 z=-80 mm + 半高 15 mm）。"""
        obj = scene.model.body("object").id
        scene.reset("fixture")
        scene.settle(0.2)
        scene.grasp(force_n=10.0, duration=3.0)
        scene.release_fixture()
        scene.settle(0.5)
        scene.open(duration=1.0)
        scene.settle(2.0)
        assert float(scene.data.xpos[obj][2]) == pytest.approx(-0.065, abs=0.002)

    def test_empty_grasp_reports_true(self, sim):
        """空夹时 ``grasp()`` 也返回 True —— 与 SDK 行为一致，不要"改进"。

        指爪顶到闭合限位同样是"停住了"，SDK 就是这么判的。改了会让仿真与
        真机分叉：真机上空夹返回 True，仿真里返回 False。
        """
        assert sim.grasp(force_n=10.0, duration=2.0) is True

    def test_get_force_is_torque_derived(self, sim):
        """``get_force()`` 是由力矩推算的估计，不是接触力测量。

        ``qfrc_constraint`` 不能用来测夹持力：实测工件放在地板上、ctrl=1.0 时
        它仍然读到精确的 -5.000 N（闭合限位提供了反力）。所以必须沿用 SDK 的
        ``force_n = torque_nm × NM_TO_N``。
        """
        sim.goto(0.0, duration=0.5)
        sim.settle(0.2)
        st = sim.get_state()
        assert st.force_n == pytest.approx(st.torque_nm * C.NM_TO_N, rel=1e-12)

    def test_fixture_toggle(self, scene):
        obj = scene.model.body("object").id
        scene.reset("fixture")
        scene.settle(0.1)
        z0 = float(scene.data.xpos[obj][2])

        assert scene.release_fixture() is True
        scene.settle(1.0)
        assert float(scene.data.xpos[obj][2]) < z0 - 0.01, "工件应当落到地板上"

        assert scene.hold_fixture() is True
        scene.reset("fixture")
        scene.settle(0.1)
        assert float(scene.data.xpos[obj][2]) == pytest.approx(z0, abs=1e-4)

    def test_unknown_equality_raises(self, sim):
        with pytest.raises(KeyError):
            sim.release_fixture("no_such_equality")


# ══════════════════════════════════════════════════════════════════════════
# API 对等
# ══════════════════════════════════════════════════════════════════════════

#: `litegrip.LiteGrip` 的全部公开成员（44 个；用 ast 从 SDK 源码抽取的快照）。
#: SDK 未安装时用它兜底，装了就直接对着真类比 —— 下面 test_matches_installed_sdk
#: 会双向核对这份快照，两边任何一个多出成员都会让它失败。
LITEGRIP_PUBLIC_API: List[str] = [
    "calibrate", "calibrate_guided", "calibrate_manual", "can_id", "channel",
    "clear_fault", "close", "config", "connect", "disable", "disconnect",
    "enable", "enter_zero_gravity", "exit_zero_gravity", "get_error",
    "get_force", "get_info", "get_position", "get_position_rad", "get_state",
    "get_temperature", "get_torque", "goto", "goto_rad", "grasp", "home",
    "is_connected", "is_enabled", "is_grasped", "is_moving", "load_calibration",
    "move_at_speed", "move_at_speed_rad", "move_to", "mst_id", "open", "poll",
    "read_param", "refresh_status", "save_calibration", "send_mit_frame",
    "set_force", "stop", "wait_for_ready",
]


def _sdk_litegrip_class():
    """装了 SDK 就返回真的 LiteGrip 类，否则 None。"""
    try:
        from litegrip import LiteGrip  # type: ignore

        return LiteGrip
    except Exception:
        return None


#: shim 会从 SDK 转出的数据模型（`_litegrip/__init__.py` 的 `_SDK_NAMES`）。
MIRRORED_TYPES = (
    "GripperState", "GripperConfig", "GripperInfo", "GripperStatus",
    "GripperMode", "CalibrationData", "GripperParams", "UnitConversion",
    "ErrorCode", "DM_Motor_Type", "Control_Mode",
)

#: **有意的**偏离：仿真用真实毫米（85.452，非 SDK 名义的 120）与自洽的 θ 端点
#: 符号（SDK 的 POS_OPEN_RAD 违反它自己文档的不变量）。理由见 constants.py。
#: 注意 `GripperConfig.pos_closed_rad` 不在里面 —— 两边都是 0.0，本来就一致。
DELIBERATE_DIVERGENCE = {
    "GripperConfig": {"pos_open_rad", "max_stroke_mm", "rad_to_mm"},
    "CalibrationData": {"max_position", "rad_to_mm"},
    "UnitConversion": {"RAD_TO_MM", "MM_TO_RAD"},
    # GripperParams 是同一组 θ 端点的第三份拷贝（另两份是 GripperConfig /
    # CalibrationData），同样只差 POS_OPEN_RAD 的符号；POS_CLOSED_RAD 两边
    # 都是 0.0，本来就不算偏离，别写进来（下一条用例要求列出的**确实**不同）。
    "GripperParams": {"POS_OPEN_RAD"},
}


def _mirror_surface(cls):
    """把 dataclass 字段 / Enum 成员 / 普通类属性统一成 {名字: 值的字面量}。

    普通类属性这一支不能省：`ErrorCode` 既不是 dataclass 也不是 Enum，
    早先只比 dataclass 字段的审计脚本因此漏掉了它缺三个错误码。
    """
    if dataclasses.is_dataclass(cls):
        out = {}
        for f in dataclasses.fields(cls):
            out[f.name] = (
                "<无默认>" if f.default is dataclasses.MISSING else repr(f.default)
            )
        return out
    if isinstance(cls, type) and issubclass(cls, enum.Enum):
        return {m.name: repr(m.value) for m in cls}
    return {
        n: repr(v)
        for n, v in vars(cls).items()
        if not n.startswith("_")
        and not isinstance(v, (property, staticmethod, classmethod))
        and not inspect.isfunction(v)
    }


def _mirror_members(cls):
    """公开的属性与方法的**名字**（`GripperState` 的 has_data / is_stale 这类）。"""
    if not inspect.isclass(cls):
        return set()
    return {
        n
        for n, v in vars(cls).items()
        if not n.startswith("_")
        and (isinstance(v, (property, staticmethod, classmethod))
             or inspect.isfunction(v))
    }


class TestApiParity:
    def test_all_methods_exist(self):
        """MujocoGripper 覆盖了 LiteGrip 的全部公开成员。

        这是本包的核心承诺：同一段控制代码改一行 import 就能从仿真切到真机。
        """
        have = set(dir(MujocoGripper))
        missing = [n for n in LITEGRIP_PUBLIC_API if n not in have]
        assert not missing, f"MujocoGripper 缺少 LiteGrip 的这些成员: {missing}"

    def test_matches_installed_sdk(self):
        """装了 SDK 就对着真类比 —— 上面那份快照可能是过期的。"""
        cls = _sdk_litegrip_class()
        if cls is None:
            pytest.skip("litegrip SDK 未安装")

        sdk_api = {n for n in dir(cls) if not n.startswith("_")}
        missing = sorted(sdk_api - set(dir(MujocoGripper)))
        assert not missing, f"SDK 里有而 MujocoGripper 没有: {missing}"

        # 快照本身也该与真类一致，否则下一个人会拿一份过期的清单去核对
        stale = sorted(set(LITEGRIP_PUBLIC_API) ^ sdk_api)
        assert not stale, f"LITEGRIP_PUBLIC_API 快照已过期，差异: {stale}"

    def test_mirrors_match_installed_sdk(self):
        """装了 SDK 就把 `_fallback` 的每个镜像类型逐字段对着真类比。

        上面那个用例只管 `LiteGrip` 的**方法**，管不到 dataclass 的字段 ——
        SDK 给 `GripperState` 加 `data_age_s`、给 `ErrorCode` 加三个错误码时，
        就是从这个空子钻过去的：仿真照常能跑，只是那些名字悄悄缺了或吃了
        错的默认值。这条用例把整张表都钉住，装了 SDK 就会自动跟着比。
        """
        from litegrip_mujoco._litegrip import _fallback as fb

        if _sdk_litegrip_class() is None:
            pytest.skip("litegrip SDK 未安装")

        import litegrip

        for name in MIRRORED_TYPES:
            sdk_cls = getattr(litegrip, name)
            loc_cls = getattr(fb, name)
            skip = DELIBERATE_DIVERGENCE.get(name, set())

            sdk_items, loc_items = _mirror_surface(sdk_cls), _mirror_surface(loc_cls)

            missing = sorted(set(sdk_items) - set(loc_items))
            assert not missing, (
                f"{name}: 兜底镜像缺少 {missing}（SDK 有而 _fallback 没有）"
            )

            for key, sdk_val in sdk_items.items():
                if key in skip:
                    continue
                assert loc_items[key] == sdk_val, (
                    f"{name}.{key}: SDK={sdk_val} 兜底={loc_items[key]}"
                )

            # 字段顺序也算契约：dataclass 可以按位置构造，顺序错了会静默串位
            if dataclasses.is_dataclass(sdk_cls):
                sdk_order = [f.name for f in dataclasses.fields(sdk_cls)]
                loc_order = [f.name for f in dataclasses.fields(loc_cls)]
                assert loc_order == sdk_order, (
                    f"{name}: 字段顺序与 SDK 不一致\n"
                    f"  SDK : {sdk_order}\n  兜底: {loc_order}"
                )

            missing_members = sorted(_mirror_members(sdk_cls) - _mirror_members(loc_cls))
            assert not missing_members, (
                f"{name}: 兜底镜像缺少成员 {missing_members}"
            )

    def test_deliberate_divergences_still_diverge(self):
        """有意的偏离必须**仍然**偏离，否则说明它们被谁"顺手对齐"了。

        这条是反向断言：如果哪天有人把 `max_stroke_mm` 改成 SDK 的 120，
        上面那条用例会照样通过（因为它跳过这个键），只有这条会失败。
        """
        from litegrip_mujoco._litegrip import _fallback as fb

        # 与 SDK 无关，纯仿真侧也该成立：毫米基准是真实行程
        assert fb.GripperConfig.max_stroke_mm == C.MM_SCALE == 85.452
        assert fb.GripperConfig.pos_open_rad < fb.GripperConfig.pos_closed_rad, (
            "θ 端点必须满足 SDK 自己文档写的不变量：闭合位数值更大"
        )

        if _sdk_litegrip_class() is None:
            pytest.skip("litegrip SDK 未安装")

        import litegrip

        for name, keys in DELIBERATE_DIVERGENCE.items():
            sdk_cls, loc_cls = getattr(litegrip, name), getattr(fb, name)
            for key in keys:
                sdk_val = _mirror_surface(sdk_cls).get(key)
                loc_val = _mirror_surface(loc_cls).get(key)
                assert sdk_val != loc_val, (
                    f"{name}.{key} 与 SDK 变成一样了（{sdk_val}）—— "
                    f"如果这是有意的，请一并更新 constants.py 与开发文档"
                )

    def test_exception_family_is_the_sdk_family(self):
        """7 个异常类必须与 SDK 的**同一个类对象**，不只是同名。

        仿真只抛 NotInitializedError，但共用代码里的 `except CommError:`
        在纯仿真环境不能 ImportError，也不能接不住。
        """
        from litegrip_mujoco import _litegrip as shim

        names = (
            "LiteGripError", "NotInitializedError", "ConnectError", "CommError",
            "CANTimeoutError", "HardwareError", "CommandError",
        )
        for name in names:
            assert hasattr(shim, name), f"shim 没有转出 {name}"

        if _sdk_litegrip_class() is None:
            # 没装 SDK 时至少要保证兜底自己的层级是对的
            for name in names[1:]:
                assert issubclass(getattr(shim, name), shim.LiteGripError)
            pytest.skip("litegrip SDK 未安装")

        import litegrip

        for name in names:
            assert getattr(shim, name) is getattr(litegrip, name), (
                f"{name} 不是 SDK 的类对象 —— 两侧的 except 会各接各的"
            )

    def test_describe_error_covers_every_error_code(self):
        """`describe_error` 要认全 `ErrorCode` 里的每一个码。

        SDK 加了 OV_FAULT / COMM_LOSS / OVERLOAD 三个码，镜像不能只加码
        不加描述 —— 否则故障会显示成"未知错误"。SDK 没有导出
        ``ERROR_DESCRIPTIONS`` 这个表（只导出函数），所以表是在兜底里维护的，
        下面两个配置都要过：装了 SDK 时用 SDK 的函数验，没装时装不了的那支
        用兜底自己的表验。
        """
        from litegrip_mujoco._litegrip import (
            ErrorCode,
            describe_error,
            sdk_unavailable_reason,
        )

        # 码是不是**齐**由上一条用例（对着 SDK 比 surface）保证，这里只管
        # "每个已知码都有描述"。少码不会漏 —— 漏了上一条会报缺名。
        for name, value in vars(ErrorCode).items():
            if name.startswith("_"):
                continue
            assert not describe_error(value).startswith("未知错误"), (
                f"ErrorCode.{name} = {value:#x} 没有对应的描述"
            )

        # 未收录的码要有兜底文案，不能抛
        assert "0x99" in describe_error(0x99)

        # 描述文案本身也得两边一致 —— 上面验的是"有没有"，这里验"对不对"
        from litegrip_mujoco._litegrip import _fallback as fb

        if sdk_unavailable_reason() is not None:
            pytest.skip("litegrip SDK 未安装 —— 表已由上面的用例验过")

        import litegrip

        for code, text in fb.ERROR_DESCRIPTIONS.items():
            assert litegrip.describe_error(code) == text, (
                f"{code:#x}: 兜底={text!r} SDK={litegrip.describe_error(code)!r}"
            )

    def test_sim_only_extras_are_documented(self):
        """仿真独有的成员必须与 SDK 的名字不冲突。"""
        cls = _sdk_litegrip_class()
        if cls is None:
            return
        sim_only = {"step", "settle", "reset", "release_fixture", "hold_fixture",
                    "gap_mm", "frac_open", "set_frac_open", "launch_viewer",
                    "sync_viewer", "model", "data", "model_path"}
        overlap = sim_only & {n for n in dir(cls) if not n.startswith("_")}
        assert not overlap, f"仿真独有成员与 SDK 撞名: {sorted(overlap)}"

    def test_read_param_raises(self, sim):
        """``read_param()`` 读的是 DM 驱动器寄存器，仿真里没有。"""
        with pytest.raises(NotImplementedError):
            sim.read_param(0)

    def test_send_mit_frame_sets_control_law(self, sim):
        """``send_mit_frame`` 直接设控制律，不经过斜坡。"""
        sim.send_mit_frame(C.POS_OPEN_RAD, kp=100.0, kd=2.0, dq=0.0, tau=0.0)
        sim.settle(0.2)
        assert sim.get_position() == pytest.approx(C.MM_SCALE, abs=0.1)
        sim.send_mit_frame(C.POS_CLOSED_RAD, kp=100.0, kd=2.0, dq=0.0, tau=0.0)
        sim.settle(0.5)
        assert sim.get_position() == pytest.approx(0.0, abs=0.1)

    def test_zero_gravity(self, sim):
        sim.goto(0.0, duration=0.5)
        assert sim.enter_zero_gravity() is True
        sim.settle(0.2)
        assert sim.get_torque() == pytest.approx(0.0, abs=1e-9)
        assert sim.exit_zero_gravity() is True


# ══════════════════════════════════════════════════════════════════════════
# 跨设备口径
# ══════════════════════════════════════════════════════════════════════════


class TestCrossDevice:
    def test_frac_open_is_device_independent(self):
        """``frac_open`` 与标定口径无关 —— 这是跨设备唯一安全的量。

        θ 的零点和量程都是设备相关的（仿真 θ∈[-1.14, 0]，真机标定后
        θ∈[-0.064279, 1.775959]）。同一个 position_rad 在两侧指的是完全不同的
        开度，直接对拷必然错位。
        """
        from litegrip_mujoco import DryRunGripper
        from litegrip_mujoco.mirror import read_frac_open, write_frac_open

        dry = DryRunGripper(realtime=False, noise=False)
        dry.connect()
        try:
            assert dry.config.pos_closed_rad != pytest.approx(C.POS_CLOSED_RAD)
            for frac in (0.0, 0.3, 0.75, 1.0):
                write_frac_open(dry, frac)
                assert read_frac_open(dry, warn=False) == pytest.approx(frac, abs=1e-3)
        finally:
            dry.disconnect()

    def test_write_frac_open_clamps(self):
        from litegrip_mujoco import DryRunGripper
        from litegrip_mujoco.mirror import read_frac_open, write_frac_open

        dry = DryRunGripper(realtime=False, noise=False)
        dry.connect()
        try:
            write_frac_open(dry, 5.0)
            assert read_frac_open(dry, warn=False) == pytest.approx(1.0, abs=1e-3)
            write_frac_open(dry, -5.0)
            assert read_frac_open(dry, warn=False) == pytest.approx(0.0, abs=1e-3)
        finally:
            dry.disconnect()

    def test_read_frac_open_warns_without_calibration(self):
        """θ 端点不可信时退回 mm 口径并发出 RuntimeWarning。"""
        from litegrip_mujoco._litegrip._fallback import GripperConfig
        from litegrip_mujoco.mirror import read_frac_open
        from litegrip_mujoco import DryRunGripper

        # SDK 的出厂默认值本身就违反 closed > open 的不变量
        bad = GripperConfig(pos_closed_rad=C.REAL_POS_CLOSED_RAD,
                            pos_open_rad=C.REAL_POS_CLOSED_RAD + 1.0)
        dry = DryRunGripper(realtime=False, noise=False, config=bad)
        dry.connect()
        try:
            with pytest.warns(RuntimeWarning):
                frac = read_frac_open(dry, warn=True)
            assert 0.0 <= frac <= 1.0
        finally:
            dry.disconnect()

    def test_dual_gripper_dry_run(self):
        """``DualGripper`` 在无硬件下走通全流程，两侧开度一致。"""
        from litegrip_mujoco import DualGripper

        dual = DualGripper(render=False, dry_run=True)
        try:
            dual.start()
            for frac in (0.0, 0.5, 1.0):
                dual.move_to_frac(frac, duration=0.4)
                time.sleep(0.2)
                cmp = dual.compare()
                assert abs(cmp["delta"]) < 0.05, f"两侧开度对不上: {cmp}"
        finally:
            dual.disconnect()

    def test_mirror_mode_follows(self):
        """``MirrorMode`` 把真机开度搬到仿真上。"""
        from litegrip_mujoco import DryRunGripper, MirrorMode

        real = DryRunGripper(realtime=False, noise=False)
        sim = MujocoGripper(render=False)
        sim.connect()
        real.connect()
        real.enable()
        mirror = MirrorMode(real, sim, rate_hz=100.0)
        try:
            mirror.start()
            real.goto(20.0, duration=0.4)
            time.sleep(0.6)
            assert abs(sim.frac_open() - real.get_position() / C.MM_SCALE) < 0.05
            assert mirror.samples > 0
        finally:
            mirror.stop()
            real.disconnect()
            sim.disconnect()


# ══════════════════════════════════════════════════════════════════════════
# 控制器与轨迹
# ══════════════════════════════════════════════════════════════════════════


class TestController:
    def test_pd_torque_sign(self):
        from litegrip_mujoco.controller import GripperPDController

        c = GripperPDController(kp=100.0, kd=2.0, n_joints=2)
        c.set_target(0.0)
        tau = c.compute(0.5, 0.0)
        assert tau.shape == (2,)
        assert np.all(tau < 0), "目标小于当前角，力矩应当为负"

    def test_torque_is_clamped(self):
        from litegrip_mujoco.controller import GripperPDController

        c = GripperPDController(kp=10000.0, kd=2.0, n_joints=2)
        c.set_target(0.0)
        assert np.all(np.abs(c.compute(10.0, 0.0)) <= C.TAU_MAX + 1e-12)

    def test_hold_current_zeroes_error(self):
        from litegrip_mujoco.controller import GripperPDController

        c = GripperPDController(kp=100.0, kd=2.0, n_joints=2)
        c.hold_current(0.3, 0.4)
        tau = c.compute(0.3, 0.0)
        assert np.allclose(tau, 0.4, atol=1e-12), "位置误差归零后只剩前馈"

    def test_min_jerk_endpoints_and_bounds(self):
        from litegrip_mujoco.controller import MinJerkRamp

        r = MinJerkRamp(0.0, 1.0, 0.5)
        vals = []
        while not r.done:
            v, _ = r.advance(0.001)
            vals.append(v)
        # 第一个采样点在 u=0.002（不是 0）—— s(0.002)≈8e-8，这是 min-jerk
        # 五次曲线在起点附近的三阶平坦性，不是误差。
        assert vals[0] == pytest.approx(0.0, abs=1e-6)
        assert vals[-1] == pytest.approx(1.0, abs=1e-9)
        assert all(-1e-9 <= v <= 1 + 1e-9 for v in vals), "min-jerk 不该过冲"
        assert all(b >= a - 1e-12 for a, b in zip(vals, vals[1:])), "应当单调"

    def test_minimum_jerk_trajectory(self):
        from litegrip_mujoco.controller import minimum_jerk_trajectory

        pts = minimum_jerk_trajectory(0.0, 1.0, duration=0.5, dt=0.01)
        assert pts[0] == pytest.approx(0.0, abs=1e-9)
        assert pts[-1] == pytest.approx(1.0, abs=1e-9)
        assert len(pts) == pytest.approx(50, abs=2)

    def test_resample(self):
        from litegrip_mujoco.controller import resample

        # 4 个点、间隔 1 s ⇒ 跨度 3 s
        assert len(resample([0.0, 1.0, 2.0, 3.0], 1.0, 0.5)) == 7   # 加密到 0.5 s
        assert len(resample([0.0, 1.0, 2.0, 3.0], 1.0, 2.0)) == 3   # 抽稀到 2 s

        out = resample([0.0, 1.0, 2.0, 3.0], 1.0, 0.5)
        assert out[0] == pytest.approx(0.0)
        assert out[-1] == pytest.approx(3.0)
        assert out[1] == pytest.approx(0.5), "线性插值应当取到中点"

    def test_linear_ramp_hits_speed(self):
        from litegrip_mujoco.controller import LinearRamp

        r = LinearRamp(0.0, 1.0, 1.0)   # 1 rad/s
        t = 0.0
        while not r.done:
            r.advance(0.001)
            t += 0.001
        assert t == pytest.approx(1.0, abs=0.01)


# ══════════════════════════════════════════════════════════════════════════
# 场景文件
# ══════════════════════════════════════════════════════════════════════════


class TestScene:
    def test_scene_compiles(self, raw_scene):
        model, data = raw_scene
        assert model.nbody >= 3
        assert model.neq == 2, "scene.xml 应当同时有 finger_coupling 与 fixture"

    def test_equalities_resolved_by_name(self, raw_scene):
        """按**名字**解析等式 —— 按索引会拿错。

        scene.xml 用 <include> 引入 litegrip.xml，两个文件的 <equality> 会合并，
        实测合并后索引 0 是 finger_coupling 而不是 fixture。按索引写会让
        release_fixture() 去动耦合约束。
        """
        model, _ = raw_scene
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EQUALITY, i)
                 for i in range(model.neq)]
        assert names == ["finger_coupling", "fixture"], f"等式顺序变了: {names}"

        g = MujocoGripper(model_path="scene.xml", render=False)
        try:
            assert set(g._eq_ids) == {"finger_coupling", "fixture"}
            assert g._eq_ids["fixture"] == 1
            assert g._eq_ids["finger_coupling"] == 0
        finally:
            g.disconnect()

    def test_workpiece_blocks_closing(self, scene):
        """工件挡在两指之间，close() 停在工件表面而不是闭合位。

        这是仿真该有的行为：夹爪是固定安装的，工件由夹具托在两指之间。
        """
        scene.reset("fixture")
        scene.settle(0.1)
        scene.close(duration=1.0)
        # 指爪停在工作表面，但接触是软的：每指再压陷约 0.16 mm，
        # 所以开口比工件的 20.000 mm 略小。
        assert scene.gap_mm() == pytest.approx(19.67, abs=0.1)
        # 行程口径 = 开口 − 闭合间隙，所以是 18.12 而不是 0（闭合位）。
        assert scene.get_position() == pytest.approx(18.12, abs=0.1)
        assert scene.get_position() > 10.0, "没有停在工件上，而是走到了闭合位"

    def test_workpiece_rests_on_floor(self, raw_scene):
        """工件从夹具上落到地板后停在 -65 mm。"""
        model, data = raw_scene
        key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "fixture")
        mujoco.mj_resetDataKeyframe(model, data, key)
        # 关掉夹具，让工件自由落体
        data.eq_active[key_eq(model)] = 0
        for _ in range(4000):
            mujoco.mj_step(model, data)
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")
        assert float(data.xpos[body][2]) == pytest.approx(-0.065, abs=0.002)


def key_eq(model):
    """scene.xml 里 fixture 等式的索引。"""
    for i in range(model.neq):
        if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EQUALITY, i) == "fixture":
            return i
    raise AssertionError("scene.xml 里没有 fixture 等式")
