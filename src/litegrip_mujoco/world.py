"""Ask the MuJoCo world where things are and what is touching what.

Everything here is a plain function over ``(model, data)``, so it can be tested
against a raw ``MjModel``/``MjData`` without constructing a
:class:`~litegrip_mujoco.gripper.MujocoGripper`. ``MujocoGripper`` exposes the
same operations as thin methods.

Why the spare boxes exist
-------------------------

MuJoCo compiles a model once. There is no way to add a body or a geom to a
running simulation, so "put a workpiece in the gripper" cannot mean "create
one". ``assets/scene.xml`` therefore declares four spare 20 mm cubes
(``spawn_box_0`` .. ``spawn_box_3``) parked in a row on the floor, and
:func:`add_box` *moves* one of them where you want it.

That is a real difference from a scene format where objects are spawned on
demand, and it has three consequences worth knowing before you use it:

* Reusing a slot overwrites the previous occupant -- its size, its mass and its
  velocity are gone. There are four slots, so at most four boxes.
* ``reset()`` re-parks every slot. A box you placed does not survive a reset.
* The parked pose is not stored here. It is the body's own ``pos`` in the MJCF,
  read back through ``model.qpos0``, so the XML stays the single source of
  truth and this module cannot drift from it.
"""
from __future__ import annotations

import dataclasses
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

__all__ = [
    "BOX_DENSITY",
    "SPAWN_SLOT_PREFIX",
    "MujocoContact",
    "add_box",
    "body_aabb",
    "box_inertia",
    "box_mass",
    "contacts",
    "geom_aabb",
    "grasp_center",
    "pad_aabbs",
    "park_spawns",
    "slot_names",
    "slot_qpos_adr",
]

#: Body-name prefix of the spare workpiece slots in ``scene.xml``.
SPAWN_SLOT_PREFIX = "spawn_box_"

#: Density used to turn a box's half-extents into a mass (kg/m³). Matches the
#: MJCF default, so a box added here weighs what the same box would weigh if it
#: had been written into the model.
BOX_DENSITY = 1000.0


@dataclasses.dataclass(frozen=True)
class MujocoContact:
    """One active contact, with names resolved.

    MuJoCo reports contacts by integer index; a caller almost always wants to
    ask "is the left pad touching the object", so every field here is a name
    except the numbers.

    ``force`` is the contact force in world coordinates, already scaled by the
    timestep as MuJoCo stores it. It is only meaningful for ``condim >= 3``
    contacts -- for a frictionless point contact the first three entries are
    still the normal force, but the tangential components are zero.
    """

    geom1: str
    geom2: str
    body1: str
    body2: str
    dist: float
    pos: np.ndarray
    normal: np.ndarray
    force: np.ndarray

    def involves(self, name: str) -> bool:
        """True if ``name`` is either geom or either body of this contact."""
        return name in (self.geom1, self.geom2, self.body1, self.body2)

    @property
    def force_n(self) -> float:
        """Magnitude of the contact force (N)."""
        return float(np.linalg.norm(self.force))


def _name(model: Any, objtype: Any, index: int) -> str:
    """Object name, or ``"<unnamed>"`` -- MuJoCo returns ``None`` for no name."""
    import mujoco

    got = mujoco.mj_id2name(model, objtype, int(index))
    return got if got is not None else "<unnamed>"


def _body_id(model: Any, name: str) -> int:
    import mujoco

    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid < 0:
        raise KeyError(f"模型里没有名为 {name!r} 的 body")
    return int(bid)


def _geom_id(model: Any, name: str) -> int:
    import mujoco

    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if gid < 0:
        raise KeyError(f"模型里没有名为 {name!r} 的 geom")
    return int(gid)


def slot_names(model: Any) -> List[str]:
    """Names of the spare workpiece slots that exist, in index order.

    Returns an empty list for a model without slots, which is the normal case
    for ``litegrip.xml``. Callers that want "slot 2" should index this list
    rather than build the name themselves -- the two agree today, but only
    because the MJCF names them consecutively.
    """
    import mujoco

    found = []
    for i in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if name is not None and name.startswith(SPAWN_SLOT_PREFIX):
            found.append((i, name))
    found.sort()
    return [name for _, name in found]


def slot_qpos_adr(model: Any, name: str) -> int:
    """Index of a slot body's 7 free-joint values in ``data.qpos``.

    Raises ``KeyError`` if the body has no free joint -- a slot body without one
    could not be moved, and silently returning a wrong offset would corrupt some
    unrelated joint.
    """
    import mujoco

    bid = _body_id(model, name)
    jnt = int(model.body_jntadr[bid])
    if jnt < 0 or int(model.body_jntnum[bid]) != 1:
        raise KeyError(f"{name!r} 不是单关节 body，无法当作槽位使用")
    if int(model.jnt_type[jnt]) != int(mujoco.mjtJoint.mjJNT_FREE):
        raise KeyError(f"{name!r} 的关节不是 freejoint，无法当作槽位使用")
    return int(model.jnt_qposadr[jnt])


def park_spawns(model: Any, data: Any) -> int:
    """Put every slot back on the floor and stop it. Returns how many moved.

    The target pose comes from ``model.qpos0``, which MuJoCo fills from each
    body's ``pos`` attribute -- so the MJCF is the only place the parked pose is
    written down.

    This exists because a keyframe that does not mention the slots zero-fills
    them, and a zeroed free joint means "at the world origin" -- inside the
    gripper base. ``litegrip.xml``'s ``open`` and ``home`` keyframes predate the
    slots and cannot know about them, so
    :meth:`MujocoGripper._apply_keyframe` calls this right after applying one.
    """
    moved = 0
    for name in slot_names(model):
        adr = slot_qpos_adr(model, name)
        data.qpos[adr:adr + 7] = model.qpos0[adr:adr + 7]
        jnt = int(model.body_jntadr[_body_id(model, name)])
        dadr = int(model.jnt_dofadr[jnt])
        data.qvel[dadr:dadr + 6] = 0.0
        moved += 1
    return moved


def box_mass(size: Sequence[float], density: float = BOX_DENSITY) -> float:
    """Mass of a box (kg) from its half-extents and a density."""
    hx, hy, hz = (float(v) for v in size)
    return float(density) * 8.0 * hx * hy * hz


def box_inertia(size: Sequence[float], mass: float) -> np.ndarray:
    """Diagonal inertia of a solid box about its centre, in the body frame.

    ``I = m/12 * (H² + D²)`` for the full extents, which is ``m/3 * (h² + d²)``
    for the half-extents used here.
    """
    hx, hy, hz = (float(v) for v in size)
    m = float(mass)
    return np.array([
        m / 3.0 * (hy * hy + hz * hz),
        m / 3.0 * (hx * hx + hz * hz),
        m / 3.0 * (hx * hx + hy * hy),
    ])


def add_box(
    model: Any,
    data: Any,
    index: int,
    *,
    pos: Sequence[float],
    size: Sequence[float] = (0.010, 0.010, 0.010),
    quat: Optional[Sequence[float]] = None,
    mass: Optional[float] = None,
    density: float = BOX_DENSITY,
) -> str:
    """Move spare slot ``index`` to ``pos``, resized to ``size``. Returns its name.

    Args:
        model, data: the compiled model and its state.
        index: which slot, counting from zero over :func:`slot_names`.
        pos: centre position in world coordinates (m).
        size: half-extents (m). Default is a 20 mm cube.
        quat: orientation as ``(w, x, y, z)``. Default is upright.
        mass: override the mass (kg). Default is ``density`` × volume.
        density: used only when ``mass`` is not given.

    Returns:
        The slot's body name, so the caller can look up contacts against it.

    Note:
        The slot is **moved**, not created. Whatever occupied it before -- its
        size, mass and velocity -- is gone. Resizing rewrites ``body_mass`` and
        ``body_inertia`` and then calls ``mj_setConst``, because MuJoCo derives
        those at compile time from the geom size and does not notice a runtime
        change on its own.

        ``mj_setConst`` has a side effect worth knowing: it resets ``qpos``
        back to ``qpos0`` (measured on MuJoCo 3.11 -- ``qvel``, ``ctrl`` and
        ``time`` are left alone). Left unhandled it would quietly put the
        gripper and the workpiece back at the model's default pose every time a
        box was added, so this function saves and restores ``qpos`` around the
        call. The save happens before the resize, so the restore cannot undo
        the new box's pose -- which is written afterwards.

    Raises:
        IndexError: no such slot. ``litegrip.xml`` has none at all.
        KeyError: the slot body is not a single free joint.
        ValueError: the slot body does not have exactly one geom.
    """
    import mujoco

    names = slot_names(model)
    if not names:
        raise IndexError(
            "这个模型没有备用槽位（spawn_box_*）。请用 model_path='scene.xml' 打开场景模型。"
        )
    if not 0 <= int(index) < len(names):
        raise IndexError(f"槽位下标 {index} 越界，模型里只有 {len(names)} 个：{names}")

    name = names[int(index)]
    bid = _body_id(model, name)
    adr = slot_qpos_adr(model, name)
    if int(model.body_geomnum[bid]) != 1:
        raise ValueError(
            f"槽位 {name!r} 挂了 {int(model.body_geomnum[bid])} 个 geom，"
            "add_box 只会改一个，无法处理"
        )

    size_arr = np.asarray(size, dtype=np.float64)
    saved_qpos = data.qpos.copy()

    gid = int(model.body_geomadr[bid])
    model.geom_size[gid] = size_arr
    m = box_mass(size_arr, density) if mass is None else float(mass)
    model.body_mass[bid] = m
    model.body_inertia[bid] = box_inertia(size_arr, m)
    mujoco.mj_setConst(model, data)
    data.qpos[:] = saved_qpos

    data.qpos[adr:adr + 3] = np.asarray(pos, dtype=np.float64)
    data.qpos[adr + 3:adr + 7] = (
        np.array([1.0, 0.0, 0.0, 0.0]) if quat is None
        else np.asarray(quat, dtype=np.float64)
    )
    jnt = int(model.body_jntadr[bid])
    dadr = int(model.jnt_dofadr[jnt])
    data.qvel[dadr:dadr + 6] = 0.0

    mujoco.mj_forward(model, data)
    return name


def geom_aabb(model: Any, data: Any, geom: Any) -> Tuple[np.ndarray, np.ndarray]:
    """World-space axis-aligned bounding box of one geom: ``(lo, hi)``.

    Exact for boxes and spheres. For everything else -- meshes, capsules,
    cylinders -- it falls back to ``geom_rbound``, the bounding sphere MuJoCo
    precomputes at compile time, so the result is conservative rather than
    tight. Camera framing is the intended use; do not read a mesh AABB as a
    measurement.
    """
    import mujoco

    gid = geom if isinstance(geom, (int, np.integer)) else _geom_id(model, str(geom))
    gid = int(gid)
    centre = np.asarray(data.geom_xpos[gid], dtype=np.float64)
    gtype = int(model.geom_type[gid])
    size = np.asarray(model.geom_size[gid], dtype=np.float64)

    if gtype == int(mujoco.mjtGeom.mjGEOM_BOX):
        rot = np.asarray(data.geom_xmat[gid], dtype=np.float64).reshape(3, 3)
        half = np.abs(rot) @ size
    elif gtype == int(mujoco.mjtGeom.mjGEOM_SPHERE):
        half = np.full(3, size[0])
    else:
        half = np.full(3, float(model.geom_rbound[gid]))
    return centre - half, centre + half


def body_aabb(model: Any, data: Any, body: Any) -> Tuple[np.ndarray, np.ndarray]:
    """World-space AABB of a whole body: the union over its geoms.

    Bodies with no geoms (a bare frame or mount) raise ``ValueError``: there is
    nothing to measure, and returning an empty box would be worse than saying so.
    """
    bid = body if isinstance(body, (int, np.integer)) else _body_id(model, str(body))
    bid = int(bid)
    first = int(model.body_geomadr[bid])
    count = int(model.body_geomnum[bid])
    if count < 1:
        raise ValueError(f"body {bid} 没有 geom，量不出包围盒")

    los, his = [], []
    for gid in range(first, first + count):
        lo, hi = geom_aabb(model, data, gid)
        los.append(lo)
        his.append(hi)
    return np.min(los, axis=0), np.max(his, axis=0)


def pad_aabbs(model: Any, data: Any) -> Tuple[Tuple[np.ndarray, np.ndarray],
                                             Tuple[np.ndarray, np.ndarray]]:
    """AABBs of the two gripper pads, as ``(left, right)``.

    The pads are named ``finger_left_col_pad`` and ``finger_right_col_pad`` in
    ``litegrip.xml``. Asking for them by name rather than by body index keeps
    this working when ``scene.xml`` adds bodies around the gripper.
    """
    return (
        geom_aabb(model, data, "finger_left_col_pad"),
        geom_aabb(model, data, "finger_right_col_pad"),
    )


def grasp_center(model: Any, data: Any) -> np.ndarray:
    """Midpoint between the two pad centres, in world coordinates.

    This is where a workpiece belongs, and what a camera should look at. It is
    the midpoint of the pads rather than of their AABBs, because the AABBs grow
    with the opening and their midpoint does not.
    """
    left, right = pad_aabbs(model, data)
    return 0.5 * (0.5 * (left[0] + left[1]) + 0.5 * (right[0] + right[1]))


def contacts(
    model: Any,
    data: Any,
    *,
    only: Optional[Sequence[str]] = None,
    with_force: bool = True,
) -> List[MujocoContact]:
    """Active contacts, with names resolved.

    Args:
        model, data: the compiled model and its state.
        only: if given, keep only contacts involving one of these geom or body
            names. Filtering here rather than in the caller keeps the caller
            from rebuilding the same list on every control tick.
        with_force: call ``mj_contactForce`` for each kept contact. Turn it off
            when you only need to know *whether* something touches -- the force
            query is the expensive part, and it is only meaningful when the
            contact has just been evaluated by a step or a forward pass.

    Note:
        The list is rebuilt on every call, because MuJoCo reuses one contact
        buffer in place. Keeping the returned list is fine; keeping a reference
        into ``data.contact`` is not.
    """
    import mujoco

    wanted = None if only is None else set(only)
    out: List[MujocoContact] = []
    for i in range(int(data.ncon)):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        b1 = int(model.geom_bodyid[g1])
        b2 = int(model.geom_bodyid[g2])
        item = MujocoContact(
            geom1=_name(model, mujoco.mjtObj.mjOBJ_GEOM, g1),
            geom2=_name(model, mujoco.mjtObj.mjOBJ_GEOM, g2),
            body1=_name(model, mujoco.mjtObj.mjOBJ_BODY, b1),
            body2=_name(model, mujoco.mjtObj.mjOBJ_BODY, b2),
            dist=float(c.dist),
            pos=np.asarray(c.pos, dtype=np.float64).copy(),
            normal=np.asarray(c.frame[0:3], dtype=np.float64).copy(),
            force=np.zeros(6),
        )
        if wanted is not None:
            if not ({item.geom1, item.geom2, item.body1, item.body2} & wanted):
                continue
        if with_force:
            buf = np.zeros(6, dtype=np.float64)
            mujoco.mj_contactForce(model, data, i, buf)
            item = dataclasses.replace(item, force=buf)
        out.append(item)
    return out
