#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_common.py — LiteGrip MuJoCo 样例的共用启动样板。

五个样例（01–05）共享这里的东西：

  bootstrap_src()     让仓库 ``src/`` 下的 litegrip_mujoco 可导入（未 pip install 时）
  ensure_deps()       缺 mujoco/numpy 时自动改用仓库自带 .venv 重跑
  make_sim()          按 ``--headless`` 造一台仿真夹爪；开窗失败就退回无窗口
  import_litegrip()   导入真机 SDK（$LITEGRIP_SDK_DIR / 同级 litegrip-python
                      仓库 / 已安装的 litegrip）
  check_sdk_api()     核对 SDK 有没有本仓库依赖的公开接口，缺了就在启动时停下
  add_common_args()   --headless（别名 --no-render）
  add_hardware_args() --channel / --can-id / --mst-id / --calib（别名
                      --calibration）/ --list-calibrations / --no-can-setup
  ensure_can_link()  连接之前探测 CAN 接口；状态不对才用 sudo 把它拉起来
                     （照上位机 litegrip-studio 的流程；--no-can-setup 可关掉）
  choose_calibration_file() 定下这次用**哪一份**标定：--calib 指定 → SDK 出厂
                      标定；两个都没有就直接退出（不扫盘、不提问）
  list_calibrations() --list-calibrations：列出本机候选标定文件后退出（纯查询）
  open_real_gripper() 选标定 → 连接 → 载入并核实标定 → 使能，失败时给出可读的提示
  fresh_state()       等到一帧**新**的状态帧再读位置；等不到返回 None
                      （读真机位置只该走这里，别直接读 get_state() 的缓存）
  fraction_to_target_rad() / rad_to_fraction()  开度 ⇄ 电机目标角（按标定行程归一）
  status_line()       五个样例共用的那一行状态文本

三个真机样例（03/04/05）用**同一份** SDK 检出：带轨迹录制/回放的那份
（``nexform-tech/litegrip-python``）。它不在 PyPI 上，`pip install litegrip`
装到的是别的代码，所以要从检出装或把目录指出来——见 :func:`import_litegrip`。

04/05 会驱动真机！真机的两个手指会真的闭合。首次跑请：
  1) 把夹爪拿在手上或固定在台面上，**手指行程内不要放任何东西**；
  2) 手放在电源开关旁边；
  3) 先用 --dry-run 跑一遍看看流程。

不指定 ``--calib`` 时用的是 SDK 包里那份**出厂标定**——它是台架夹具的实测参数，
而标定的角度/毫米刻度本该是每台夹爪单独量的。换标定只有一个办法：用 ``--calib``
指这台夹爪自己的那份（上位机 ``litegrip-studio`` / ``litegrip-console`` 标定后
保存，或 SDK 自带的 ``tools/gui/litegrip_gui.py``）。样例**不扫盘**：出厂文件也
读不出来就直接退出；要列本机候选就用 ``--list-calibrations``。

真机跑之前确认 CAN 已配置好。这一步**不用你手动做**：三个真机样例在连接前会探测
接口，只在它真的不对时（没 up / 比特率不对 / 控制器 BUS-OFF）才用 sudo 配一次，
已经对了就一条命令都不跑、也不问密码——见 :func:`ensure_can_link`。想自己管接口
就加 ``--no-can-setup``。手动那条命令是：

    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 1000000 restart-ms 100 fd off
    sudo ip link set can0 up

与 pybullet 那套样例的关系
--------------------------

本文件是从 ``litegrip-pybullet/examples/_common.py`` 改出来的，函数名与判据逐条
对齐，所以五份样例的源码在两仓之间读起来是一个形状。三处**有意**不同：

* 仿真侧用的是 ``MujocoGripper`` + ``set_frac_open()``，没有 ``GripperSim`` 的
  ``reset_fraction()`` / ``aperture_mm()`` 这些名字；对应关系见
  ``examples/README.md`` 的「pybullet → mujoco 名称映射」。
* ``status_line()`` 的毫米数一律是**本仓的真实毫米**（``constants.MM_SCALE`` =
  85.452 mm 全行程），不引入 SDK 名义的 120 mm 刻度，所以没有 ``sdk_mm`` 那一栏。
* ``choose_calibration_file()`` 走库层的 :func:`apply_calibration`（它比 pybullet
  那版多一道「载入后核对端点」）；两档（``--calib`` → 出厂标定）在两边现在完全
  一致，出厂标定是一份普通文件，不再需要单独的开关放行。
"""
from __future__ import annotations

import argparse
import errno
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

__all__ = [
    "CAN_BITRATE",
    "CAN_BUS_OFF",
    "CAN_CHANNEL",
    "CAN_DEVICE_RE",
    "CAN_ERROR_ACTIVE",
    "CAN_PROBE_TIMEOUT_S",
    "CAN_RESTART_MS",
    "CAN_SETUP_TIMEOUT_S",
    "CanLinkState",
    "FRESH_WAIT_S",
    "MAX_GRIP_FORCE_N",
    "RATED_SPEED_MM_S",
    "REQUIRED_CALIB_KEYS",
    "REQUIRED_SDK_API",
    "SAFETY_BANNER",
    "STATUS_WAIT_S",
    "add_common_args",
    "add_hardware_args",
    "bootstrap_src",
    "can_link_failure",
    "check_calibration",
    "check_calibration_matches_args",
    "check_calibration_values",
    "check_sdk_api",
    "choose_calibration_file",
    "connect_failure_message",
    "enable_failure_message",
    "ensure_can_link",
    "ensure_deps",
    "factory_calibration_path",
    "fraction_to_gap_mm",
    "fraction_to_target_rad",
    "fresh_state",
    "import_litegrip",
    "is_sdk_factory_calibration",
    "list_calibrations",
    "make_sim",
    "manual_can_hint",
    "missing_sdk_api",
    "open_real_gripper",
    "parse_can_link",
    "probe_can_link",
    "rad_to_fraction",
    "sdk_dir",
    "status_line",
]

#: 真机样例开跑前打印的横幅。
SAFETY_BANNER = """\
即将驱动真机：夹爪两个手指会真实运动。
    请确认行程内无遮挡、人员远离，并让电源开关触手可及。
    再确认一次下面打印的那份标定文件：它是**这台夹爪**标出来的，还是 SDK 自带的
    出厂标定（台架夹具的实测参数）。用出厂那份驱动，行程端点可能与这台对不上。
    随时按 Esc / Q 停止（会等当前这条指令走完再退出）。"""

#: 夹持力上限 [N]。与 pybullet 那套样例同一个数，好让两边的 02/05 手感一致。
#: 本仓仿真侧的结构上限是 ``constants.FORCE_MAX``（= GEAR × TAU_MAX = 100 N），
#: 但真机上 40 N 已经远超这台夹爪的额定出力，所以命令行的力一律按这个夹住。
MAX_GRIP_FORCE_N = 40.0

#: 手指额定速度 [SDK 刻度 mm/s]，取自 SDK 的 ``move_at_speed`` 规格。
#: 05 的速度滑条以它为 100%。
RATED_SPEED_MM_S = 85.0

_REPO_ROOT = Path(__file__).resolve().parent.parent
_SRC = _REPO_ROOT / "src"
_VENV_PY = _REPO_ROOT / ".venv" / "bin" / "python"
_REEXEC_FLAG = "LITEGRIP_MUJOCO_REEXEC"


def _importable(name: str) -> bool:
    try:
        __import__(name)
    except Exception:  # noqa: BLE001 — 缺依赖、装坏了都算「用不了」
        return False
    return True


def bootstrap_src() -> None:
    """让仓库 ``src/`` 下的 litegrip_mujoco 可导入（未 pip install 时）。"""
    if str(_SRC) not in sys.path:
        sys.path.insert(0, str(_SRC))


def ensure_deps() -> None:
    """当前 python 缺 mujoco/numpy 时，自动改用仓库自带 .venv 重跑。

    这样 ``python3 examples/01_hello_sim.py`` 开箱即用，不必先 pip install。
    ``.venv`` 不存在时把话说清楚再退出——**不**替用户装东西。
    """
    if _REEXEC_FLAG in os.environ:  # 已经重跑过一次，别再套娃
        return
    missing = [name for name in ("mujoco", "numpy") if not _importable(name)]
    if not missing:
        return
    script = os.path.abspath(sys.argv[0])  # 被跑的样例脚本，不是 _common.py
    if _VENV_PY.exists() and Path(sys.executable).resolve() != _VENV_PY.resolve():
        print(f"[hint] 当前 python 缺 {'/'.join(missing)}，改用 {_VENV_PY}")
        os.environ[_REEXEC_FLAG] = "1"
        os.execv(str(_VENV_PY), [str(_VENV_PY), script] + sys.argv[1:])
    raise SystemExit(
        f"当前 python 缺 {'/'.join(missing)}：{sys.executable}\n"
        "   在仓库根目录装依赖：\n"
        '     python3 -m venv .venv && .venv/bin/pip install -e ".[dev]"\n'
        "   或直接装到当前解释器：python3 -m pip install -e .\n"
        f"   （{_VENV_PY} 存在时本文件会自动改用它重跑）"
    )


def sdk_dir() -> Path | None:
    """真机 SDK（``litegrip`` 包）所在目录，找不到返回 ``None``。

    顺序：``$LITEGRIP_SDK_DIR`` → 同级 ``litegrip-python`` 仓库 → 已安装的
    ``litegrip``。

    为什么同级检出排在**已安装的**前面：本仓库要的是带轨迹录制/回放的那份
    （``litegrip-python``），而机器上装着的 ``litegrip`` 可能是另一个仓库的同名
    包——两份的 ``__version__`` 都是 ``2.2.0``，光看版本号分不出来。同级目录就在
    眼前、名字点得很明确，优先信它。
    """
    env = os.environ.get("LITEGRIP_SDK_DIR")
    if env:
        return Path(env).expanduser()
    # 同级检出：``litegrip-python`` 是 src 布局（包在 src/litegrip），也接受把包
    # 直接放在仓库根下的布局——两种都试，免得只认一种。
    sibling = _REPO_ROOT.parent / "litegrip-python"
    for candidate in (sibling / "src", sibling):
        if (candidate / "litegrip" / "__init__.py").is_file():
            return candidate
    try:
        import litegrip  # noqa: F401

        return Path(litegrip.__file__).resolve().parent.parent
    except Exception:  # noqa: BLE001 — 没装就是没装
        return None


def _load_package_from(directory):
    """从指定目录显式加载 ``litegrip`` 包，不走 ``import litegrip``。

    为什么必须显式加载：``pip install -e`` 装的那份会注册一个 **meta path
    finder**，它的优先级高于 ``sys.path``——所以「把要用的那份插到 sys.path 最
    前面」在装了 editable 版的机器上一点用都没有，``import litegrip`` 拿到的还是
    装的哪份。同级目录 checkout 与已安装的包同名时，只能按目录点名加载。

    副作用是 ``sys.modules["litegrip"]`` 被换掉（包括它已经导入过的子模块）：这个
    进程从这一句起就用这一份，这正是想要的。
    """
    import importlib.util

    init = Path(directory) / "litegrip" / "__init__.py"
    if not init.is_file():
        return None
    for name in [m for m in sys.modules
                 if m == "litegrip" or m.startswith("litegrip.")]:
        del sys.modules[name]
    spec = importlib.util.spec_from_file_location(
        "litegrip", init, submodule_search_locations=[str(init.parent)])
    module = importlib.util.module_from_spec(spec)
    sys.modules["litegrip"] = module       # 子模块与 dataclass 都要先看到它
    try:
        spec.loader.exec_module(module)
    except Exception:                      # 导入一半失败：别留下半个包
        sys.modules.pop("litegrip", None)
        raise
    return module


def import_litegrip():
    """导入真机 SDK，失败时给出安装提示并退出。

    先找目录再**显式加载**，不能靠 ``sys.path`` 顺序：``pip install -e`` 装的那份
    会注册一个 meta path finder，优先级高于 ``sys.path``，把目录插到最前面也没用
    ——理由见 :func:`_load_package_from`。

    ``$LITEGRIP_SDK_DIR`` 指错目录时**不悄悄换一份**：本机装着的 ``litegrip`` 可能
    是另一个仓库的同名包（两份的 ``__version__`` 都是 2.2.0），悄悄换过去会让人以
    为「指了却能跑」，实际跑的是别的代码。

    Returns:
        已导入的 ``litegrip`` 模块。
    """
    directory = sdk_dir()
    if directory is not None:
        try:
            module = _load_package_from(directory)
        except Exception as exc:  # noqa: BLE001 — 那份检出自己炸了（语法错误等）
            raise SystemExit(f"从 {directory} 加载 litegrip 失败：{exc}") from exc
        if module is not None:
            return module
        requested = os.environ.get("LITEGRIP_SDK_DIR")
        if requested:
            raise SystemExit(
                f"找不到真机 SDK（litegrip 包）：LITEGRIP_SDK_DIR 指到 {requested}，"
                f"但那里没有 litegrip/__init__.py。\n"
                "   这个变量要指到**包所在目录**（src 布局就是 "
                "<仓库>/litegrip-python/src），不是仓库根、也不是包目录本身。\n"
                "   不想指定就 unset LITEGRIP_SDK_DIR；否则三种任选其一：\n"
                "   1) python3 -m pip install -e /path/to/litegrip-python\n"
                "   2) export LITEGRIP_SDK_DIR=/path/to/litegrip-python/src\n"
                "   3) 把 litegrip-python 仓库克隆到本仓库的同级目录"
            )
        # 走到这里说明是自己找到的目录（同级检出 / 已安装包）里没有包——同级目录
        # 名字没对上而已，落到下面的 import 再看。
    try:
        import litegrip
    except ImportError as exc:
        raise SystemExit(
            "找不到真机 SDK（litegrip 包）。\n"
            "   litegrip **没有发布到 PyPI**，`pip install litegrip` 装的不是它；\n"
            "   本仓库要的是带轨迹录制/回放的那份检出\n"
            "   （nexform-tech/litegrip-python）。三种任选其一：\n"
            "   1) python3 -m pip install -e /path/to/litegrip-python\n"
            "   2) export LITEGRIP_SDK_DIR=/path/to/litegrip-python/src\n"
            "   3) 把 litegrip-python 仓库克隆到本仓库的同级目录\n"
            f"   （原始错误：{exc}）"
        ) from exc
    return litegrip


#: 本仓库依赖的 SDK 公开接口。清单化而不是散在各调用处：缺哪一个就在**启动时**
#: 说清楚该换哪份 SDK，而不是在发帧的循环里抛 AttributeError。
#:
#: 三个真机样例（03/04/05）用的是**同一份**检出，所以这里是一份并集：轨迹那几个是
#: 03 要的，其余是 04/05 要的。
#:
#: 这些接口目前没有任何发行版带（litegrip 也不在 PyPI 上），所以
#: ``pip install litegrip`` 装到的那份一定缺它们——这正是要拦的情况。
#:
#: 写法是 ``名字.属性`` 或裸的模块级名字（如 ``trajectory_dir``），两种都认。
REQUIRED_SDK_API: tuple[tuple[str, str], ...] = (
    ("LiteGrip.connect", "连上 CAN 并注册夹爪"),
    ("LiteGrip.enable", "使能电机——不使能就一个运动指令都发不出去"),
    ("LiteGrip.disable", "退出前把电机放回失力状态"),
    ("LiteGrip.disconnect", "关掉 CAN 连接"),
    ("LiteGrip.poll",
     "等一帧**状态帧**；fresh_state() 靠它区分「刚量到的」和「缓存里的」"),
    ("LiteGrip.get_state", "读一次位置/速度/力矩/错误码快照"),
    ("LiteGrip.send_mit_frame", "发一帧 MIT 指令：锁位帧和零力矩帧都走它"),
    ("LiteGrip.read_param", "按 RID 读电机寄存器（只读诊断用）"),
    ("LiteGrip.clear_fault", "清锁存的故障码"),
    ("LiteGrip.enter_zero_gravity", "进零重力：03/04 靠它让人手拖动手指"),
    ("LiteGrip.exit_zero_gravity", "退出零重力并锁在当前位"),
    ("LiteGrip.move_at_speed", "按速度走一段（05 的额定速度就是它的规格）"),
    ("LiteGrip.load_calibration", "载入标定文件——毫米刻度和行程端点都从它来"),
    ("LiteGrip.record_start",
     "在后台开始录制；zero_gravity=True 时由它自己流零力矩帧"),
    ("LiteGrip.record_stop", "停止录制并取回轨迹"),
    ("LiteGrip.play_start",
     "在后台开始回放——主循环才腾得出手同步刷仿真（play() 会阻塞到放完）"),
    ("LiteGrip.play_stop", "停止回放，并把夹爪留在最后一个目标位上"),
    ("LiteGrip.trajectory_status",
     "录制/回放的进度：active / completed / openness"),
    ("GripperState.position_rad", "状态快照里的电机角，开度换算的输入"),
    ("GripperState.is_error", "故障是锁死的，只能从状态帧上看出来"),
    ("Trajectory.load", "读回一段 ``.lgt``（纯文件 I/O，不碰 CAN）"),
    ("Trajectory.openness_at", "按时间取归一化开度——离线 --play 靠它驱动仿真"),
    ("trajectory_dir", "轨迹默认存在哪（``~/.litegrip/trajectories``）"),
)


def missing_sdk_api(litegrip) -> list[str]:
    """已导入的 SDK 里缺哪些必需接口（按 :data:`REQUIRED_SDK_API` 的顺序）。"""
    missing: list[str] = []
    for path, _why in REQUIRED_SDK_API:
        owner, sep, attr = path.partition(".")
        if sep:
            found = hasattr(getattr(litegrip, owner, None), attr)
        else:
            found = hasattr(litegrip, owner)   # 模块级的名字，如 trajectory_dir
        if not found:
            missing.append(path)
    return missing


def check_sdk_api(litegrip) -> None:
    """缺必需接口就带着「该用哪份 SDK」退出（``SystemExit``）。

    **不保留降级路径**：这些接口没有替代品——录制是 SDK 在后台线程里按自己的节拍
    采样和发帧的，自己拿 ``send_mit_frame`` 拼一个循环只会得到一份节拍对不上的
    样本。真机样例宁可不跑，也不拿一个猜出来的位置去算目标角——那正是一条指向别处
    的阶跃指令的成因。
    """
    missing = missing_sdk_api(litegrip)
    if not missing:
        return
    why = dict(REQUIRED_SDK_API)
    directory = sdk_dir()
    raise SystemExit(
        "这份 litegrip SDK 缺少本仓库必须的公开接口：\n"
        + "".join(f"     • {path} —— {why[path]}\n" for path in missing)
        + "   litegrip **没有发布到 PyPI**（`pip install litegrip` 装到的不是这份"
          "代码）。\n"
          "   本仓库要的是带轨迹录制/回放的那份检出，请指到它：\n"
          "     python3 -m pip install -e /path/to/litegrip-python  # 或\n"
          "     export LITEGRIP_SDK_DIR=/path/to/litegrip-python/src\n"
        + (f"   （这次导入到的是：{directory}）" if directory is not None else "")
    )


# ═════════════════════════════════════════════════════════════════════════
# 仿真侧
# ═════════════════════════════════════════════════════════════════════════


def make_sim(headless: bool, **kwargs):
    """按 ``--headless`` 造一台 :class:`~litegrip_mujoco.MujocoGripper` 并连接。

    开窗失败（无显示、没有 GL、查看器太老）时**退回无窗口**并说清楚，而不是让整
    个样例崩在构造函数里——运动学完全一致，只有画面没有。这条路径在 CI 和 ssh
    里都会走到。

    Args:
        headless: 是否强制不开窗。
        **kwargs: 透传给 ``MujocoGripper`` 的其余参数（``model_path`` 等）。
    """
    from litegrip_mujoco import MujocoGripper

    if headless:
        sim = MujocoGripper(render=False, **kwargs)
        sim.connect()
        return sim
    try:
        sim = MujocoGripper(render=True, **kwargs)
    except RuntimeError as exc:
        print(f"   [警告] 开不了窗口（{exc}）→ 改为无窗口运行")
        sim = MujocoGripper(render=False, **kwargs)
    sim.connect()
    return sim


# ═════════════════════════════════════════════════════════════════════════
# 命令行
# ═════════════════════════════════════════════════════════════════════════


def add_common_args(parser: argparse.ArgumentParser) -> None:
    """加 ``--headless``（``--no-render`` 是它的旧名，保留作别名）。"""
    parser.add_argument(
        "--headless", "--no-render", dest="headless", action="store_true",
        help="不开可视化窗口（无显示环境必须加；运动学完全一致）",
    )


def add_hardware_args(parser: argparse.ArgumentParser) -> None:
    """加真机连接参数 ``--channel`` / ``--can-id`` / ``--mst-id`` / ``--calib``
    / ``--no-can-setup``。"""
    parser.add_argument(
        "--channel", default=CAN_CHANNEL,
        help=f"SocketCAN 接口名（默认 {CAN_CHANNEL}）",
    )
    parser.add_argument(
        "--can-id", default=0x08, type=lambda s: int(s, 0),
        help="夹爪的 CAN ID（默认 0x08）",
    )
    parser.add_argument(
        "--mst-id", default=0x18, type=lambda s: int(s, 0),
        help="主控（达妙电机 MIT 协议）ID（默认 0x18）",
    )
    parser.add_argument(
        "--calib", "--calibration", dest="calib", default=None, metavar="PATH",
        help="标定文件路径（--calibration 是同一个参数的旧名）。不给就用 SDK "
             "包里那份**出厂标定**（台架夹具的实测参数，跟着 SDK 包走，换电脑也"
             "指得到）；要按这**台**夹爪自己的尺寸驱动，就得给出它的标定。出厂"
             "文件也读不出来就直接退出，让你显式给 --calib——样例不去扫盘猜一份。"
             "标定文件由 litegrip-studio / litegrip-console 对着真机标定后保存得到",
    )
    parser.add_argument(
        "--list-calibrations", action="store_true",
        help="列出本机候选标定文件及其端点后退出（纯查询，不连真机、不发帧；"
             "把列出的路径喂给 --calib 即可换成那份）",
    )
    parser.add_argument(
        "--no-can-setup", action="store_true",
        help="不自动准备 CAN 口：接口由你自己管，连接前不再探测、也不弹 sudo 密码",
    )


def list_calibrations(out=print) -> int:
    """``--list-calibrations``：打印候选标定文件，返回退出码。

    **纯查询**：不连真机、不发帧，也不改变默认标定——默认永远是 SDK 包里那份
    出厂标定。它只有一个用途：把路径找出来，喂给 ``--calib``。
    """
    from litegrip_mujoco import describe_candidate, discover_calibrations

    candidates = discover_calibrations()
    if not candidates:
        out("没有找到候选标定文件。标定文件由上位机（GUI）标定后生成，")
        out("默认写在 ~/.litegrip/，也可以放在当前目录。")
        out("不给 --calib 时用的是 SDK 包里那份出厂标定，不必在这里选。")
        return 0
    out(f"候选标定文件（{len(candidates)} 个；SDK 自带的出厂标定不在其中——"
        "它就是不给 --calib 时的默认值）:")
    for index, path in enumerate(candidates, start=1):
        out(f"  {index}) {path}")
        out(f"     {describe_candidate(path)}")
    out("   要用其中一份：--calib <上面的路径>")
    return 0


# ═════════════════════════════════════════════════════════════════════════
# 标定
# ═════════════════════════════════════════════════════════════════════════

#: SDK ``load_calibration`` 里**无保护**索引的三个键（它直接 ``data[...]`` 取值）。
#: 库层的 :func:`load_calibration_file` 已经查过一遍，这里留着是为了在报错信息里
#: 说清楚「缺的是哪几个键」。
REQUIRED_CALIB_KEYS = ("zero_position_rad", "max_position_rad", "rad_to_mm")

#: SDK ``GripperConfig.max_stroke_mm`` 的默认值 [mm]。标定文件里**没有**这一项
#: （它是「名义行程」而不是量出来的尺寸），所以只读文件、拿不到 config 的
#: ``--dry-run`` 用这个值做自洽性检查。
#:
#: 开度换算**不**用它：归一化开度按标定行程归一，见 :func:`rad_to_fraction`。
NOMINAL_STROKE_MM = 120.0


def factory_calibration_path() -> Path | None:
    """SDK 包里那份**出厂标定**的路径；拿不到 SDK 时返回 ``None``。

    跟着 ``litegrip`` 包所在目录解析（``<包目录>/factory_calibration.json``），
    所以换电脑、换虚拟环境、换 SDK 检出都指得到——**不要**把它写成某个本机绝对
    路径，这正是这个函数存在的理由。库层的
    :func:`litegrip_mujoco.sdk_factory_calibration_path` 做的就是这件事（它还会认
    SDK 声明的 ``_FACTORY_CALIB``），这里只把 ``None`` 与「没有 SDK」分开说。

    这份文件是**台架夹具的实测参数**，不是每台夹爪各自量的：它是一份能用的默认
    值，不是「这台夹爪的标定」。要按这台夹爪自己的尺寸驱动，用 ``--calib`` 指
    上位机保存的那份。
    """
    from litegrip_mujoco import sdk_factory_calibration_path

    found = sdk_factory_calibration_path()
    if found is not None:
        return Path(found)

    # 库层的探测只在 ``sys.modules`` / ``sys.path`` 上看 ``litegrip``；而 SDK 完全
    # 可能只存在于 ``$LITEGRIP_SDK_DIR`` 或同级检出里——那条路径要等到
    # :func:`import_litegrip` 显式加载才会被看见，而那一步在选标定**之后**。这里
    # 直接按目录找文件，**不导入 SDK**：导入会连带拉起 SocketCAN 依赖，而这一步
    # 只想知道一个文件在哪。
    #
    # 找错的风险说清楚：SDK 若把出厂标定挪到别处（它声明的 ``_FACTORY_CALIB``），
    # 这里就会漏掉，于是退回库层的两档选择——它自己也按包目录找，同样漏掉的话就
    # 直接报错退出，是响的，不是错的。反过来找**对**了而库层不认识它（同一份
    # 文件、两条路径），库层会把它当普通文件校验，端点不符时照常报错。
    directory = sdk_dir()
    if directory is not None:
        candidate = directory / "litegrip" / "factory_calibration.json"
        if candidate.is_file():
            return candidate
    return None


def is_sdk_factory_calibration(path, factory) -> bool:
    """``path`` 是不是 SDK 包里那份出厂标定（``factory`` 由上面那个函数给出）。

    按 ``realpath`` 比，符号链接、``..`` 这些写法都算同一份。两边任一为 ``None``
    就是否——**没有出厂标定**与**用了出厂标定**不能混为一谈。
    """
    if path is None or factory is None:
        return False
    return os.path.realpath(str(path)) == os.path.realpath(str(factory))


def choose_calibration_file(requested=None, *, factory=None, out=print):
    """定下这次用哪份标定文件。两档，**顺序就是优先级**。

    * ``--calib <路径>``（``requested``）：直接用，只做校验。
    * ``factory`` 给了且读得出来：用 SDK 包里那份出厂标定，只打一行说明。这是
      默认路径——``factory`` 由调用方用 :func:`factory_calibration_path` 算出来，
      所以它跟着包走，换电脑也一样。

    两档都没成（``--calib`` 没给，出厂标定也读不出来）就**直接退出**，让人显式
    给出 ``--calib``。这里**不扫盘、不提问**：样例不去猜 ``~/.litegrip`` 下哪份
    JSON 是这台夹爪的，猜错了就是把另一台机器的尺寸驱动到真机上。想在这些文件里
    挑一份，用 ``--list-calibrations`` 看列表，再把路径喂给 ``--calib``。

    出厂标定是**台架夹具的实测参数**，不是每台夹爪各自量的：它是一份能用的默认
    值，不是「这台夹爪的标定」。要按这台夹爪自己的尺寸驱动，用 ``--calib`` 指
    上位机保存的那份——所以第 2 档那行提示必须把这句话说出来。

    Args:
        requested: ``--calib`` 的值（``None`` = 没给）。
        factory: 出厂标定文件的路径；``None`` 表示调用方拿不到 SDK。
        out: 打印函数（默认 ``print``）——测试注入用。

    Returns:
        选中的标定文件路径（已校验存在、可解析、字段齐、端点自洽）。

    Raises:
        SystemExit: 两档都没有可用文件；信息里给出 ``--calib`` 的用法。
    """
    from litegrip_mujoco import (
        CalibrationError,
        CalibrationRequiredError,
        load_calibration_file,
        resolve_calibration_path,
    )

    if requested:
        path = os.path.abspath(os.path.expanduser(str(requested)))
        try:
            load_calibration_file(path)
        except CalibrationError as exc:
            raise SystemExit(f"{exc}") from exc
        return path

    if factory is not None:
        path = os.path.abspath(os.path.expanduser(str(factory)))
        try:
            load_calibration_file(path)
        except CalibrationError as exc:
            # 出厂文件不在 / 坏了：这里**不**退回候选列表，直接说清楚怎么办。
            raise SystemExit(
                f"没有指定 --calib，SDK 自带的出厂标定也读不出来：{path}\n"
                f"   （{exc}）\n"
                "   样例不会去扫盘替你挑一份。请显式指定这台夹爪的标定：\n"
                "     --calib <路径>\n"
                "   要看看本机有哪些候选：--list-calibrations"
            ) from None
        out(f"未指定 --calib：使用 SDK 自带的出厂标定 {path}")
        out("   （台架夹具的实测参数，不是这台夹爪自己量的。"
            "换 --calib <路径> 指这台夹爪的那份。）")
        return path

    try:
        # 库层同样的两档，只是它拿不到上面那个 factory 参数——它自己按包目录找。
        return resolve_calibration_path(None)
    except CalibrationRequiredError as exc:
        raise SystemExit(f"{exc}") from exc


def calibration_summary(data) -> str:
    """一行摘要：这份标定的关键值（文件里有什么就打什么）。

    打出来是为了让操作员在下发前能认出「这就是我刚在这台机器上标出来的那份」。
    SDK 自己**不记录**用了哪个文件（``load_calibration`` 只往日志写一行），
    所以来源这件事只能由调用方说清楚。

    ID 按十六进制——命令行的 ``--mst-id 0x18`` 和日志里的 ``0x18`` 都是这么写的，
    十进制 24 只会让人多换算一次（而把 ``0x18`` 写成 ``18`` 正是这批文件里真出过
    的事故）。
    """
    shown: list[str] = []
    for key, template in (
        ("zero_position_rad", "closed {v:+.4f}"),
        ("max_position_rad", "open {v:+.4f}"),
        ("rad_to_mm", "rad_to_mm {v:g}"),
        ("kp", "kp {v:g}"),
        ("kd", "kd {v:g}"),
        ("mst_id", "mst_id {v:#04x}"),
        ("can_id", "can_id {v:#04x}"),
    ):
        if key not in data:
            continue
        try:
            value = float(data[key])
        except (TypeError, ValueError):
            shown.append(f"{key}={data[key]!r}")
            continue
        shown.append(template.format(
            v=int(value) if "#04x" in template else value))
    return " · ".join(shown)


def calibration_config(calibration):
    """把一份 :class:`~litegrip_mujoco.Calibration` 装成 ``gripper.config`` 的形状。

    真机路径上用的永远是 SDK 自己那份 ``gripper.config``；这个只用于「不碰真机、
    但要算同一套数」的场合（``--dry-run`` 打印、以及测试注入）。

    字段名取自 ``GripperConfig``：``zero_position_rad`` → ``pos_closed_rad``、
    ``max_position_rad`` → ``pos_open_rad``，再加上 ``rad_to_mm`` / ``kp`` / ``kd``。
    ``max_stroke_mm`` 不在标定文件里，用名义值——它只进 :func:`check_calibration`
    的自洽性检查，不进开度换算。
    """
    raw = dict(calibration.raw or {})
    values = {
        "kp": float(raw.get("kp", 100.0)),
        "kd": float(raw.get("kd", 2.0)),
        "max_stroke_mm": NOMINAL_STROKE_MM,
        "pos_closed_rad": calibration.pos_closed_rad,
        "pos_open_rad": calibration.pos_open_rad,
        "rad_to_mm": calibration.rad_to_mm,
    }
    for key, attr in (("can_id", "can_id"), ("mst_id", "mst_id"),
                      ("channel", "can_channel")):
        if key in raw:
            values[attr] = raw[key]
    return SimpleNamespace(**values)


def read_calibration_file(path):
    """读一份标定文件，返回库层的 :class:`~litegrip_mujoco.Calibration`。

    只做校验；**载入设备**走 :func:`apply_calibration`（它会多核一道「载入后的端点
    确实是这份文件里的」——SDK 的 ``load_calibration`` 在文件读不出来时会静默改用
    打包的出厂标定并且照样返回 ``True``，光看返回值分不出来）。

    Raises:
        SystemExit: 文件不存在、坏了、或端点不自洽。
    """
    from litegrip_mujoco import CalibrationError, load_calibration_file

    try:
        return load_calibration_file(path)
    except CalibrationError as exc:
        raise SystemExit(f"{exc}") from exc


def check_calibration_matches_args(calibration, *, channel=None, can_id=None,
                                   mst_id=None) -> list[str]:
    """文件里记的 ID 和这次命令行给的对不对得上；对不上就退出。

    在**连总线之前**调用：选错文件（另一台夹爪）是能在发帧前就发现的错误，没有理由
    带着它往下走。只比对文件里确实记了的键——旧标定文件可能没有
    ``can_id``/``mst_id``。

    为什么值得拦：这批文件是真出过事的——``mst_id`` 被写成十进制 ``18``（应为
    ``0x18``）时，SDK 会把它当 ``0x12`` 用，RX 过滤器就绑到了没人应答的 ID 上，
    主控变成「聋子但一直发」。

    ``channel`` **不**在这里拦：文件里记的是上次上位机连的那条总线，换口是很正常
    的事（``--channel can1``），拦下来只会挡路。不同的话以一行提示返回，让调用方
    打出来——返回值是这些提示，可能为空。

    Returns:
        要提醒的话（``channel`` 对不上之类），没有就是空列表。

    Raises:
        SystemExit: 文件里记的 ``can_id``/``mst_id`` 与命令行给的不一致。
    """
    raw = dict(calibration.raw or {})
    wrong: list[str] = []
    for key, given in (("can_id", can_id), ("mst_id", mst_id)):
        if key not in raw or given is None:
            continue
        try:
            same = int(raw[key]) == int(given)
        except (TypeError, ValueError):
            same = False
        if not same:
            wrong.append(f"   {key}: 文件里是 {raw[key]!r}，这次命令行给的是 "
                         f"{int(given):#04x}")
    if wrong:
        raise SystemExit(
            "选中的标定文件不是这台夹爪的：\n"
            + "\n".join(wrong) + "\n"
            "   要么文件拿错了（另一台机器的标定），要么 ID 参数变了。\n"
            "   确认是同一台夹爪的话，在上位机里重新标定并保存一份，"
            "或用 --calib 指另一份文件。"
        )
    notes: list[str] = []
    if channel is not None and "channel" in raw and \
            str(raw["channel"]) != str(channel):
        notes.append(f"   注意：标定文件里记的接口是 {raw['channel']}，"
                     f"这次用的是 {channel}——同一台夹爪换口没问题，"
                     "别是另一台。")
    return notes


def check_calibration_values(pos_closed_rad: float, pos_open_rad: float,
                             rad_to_mm: float, max_stroke_mm: float) -> None:
    """确认一组标定值自洽，不自洽就带着原因退出。

    库层的 :func:`load_calibration_file` 已经查过这一条，所以文件路径上不会走到
    这里；留一个纯值版本，是给「拿一份还没落盘的数」的调用方用的，报错口径保持
    一致。

    不变量：``pos_closed_rad`` 必须比 ``pos_open_rad`` 更**正**。SDK 的
    ``goto_rad`` clamp 只有在它成立时才正确；SDK 自己的 ``GripperConfig``
    **默认值**恰好违反它，说明这台机器还没跑过 ``calibrate()``／没载入标定文件
    ——此时任何目标角都是瞎猜的，直接停下比发出去让手指撞限位好。
    """
    travel = pos_closed_rad - pos_open_rad
    if travel <= 0.0 or rad_to_mm <= 0.0 or max_stroke_mm <= 0.0:
        raise SystemExit(
            "夹爪的标定值不合法，先做标定再跑：\n"
            f"   pos_closed_rad={pos_closed_rad:+.4f} "
            f"pos_open_rad={pos_open_rad:+.4f} "
            f"rad_to_mm={rad_to_mm:.2f} max_stroke_mm={max_stroke_mm:.1f}\n"
            "   闭合位应当比张开位角度更大（pos_closed_rad > pos_open_rad）。\n"
            "   常见原因：没载入标定文件，还在用 SDK 的出厂默认值。\n"
            "   试：gripper.calibrate() 生成标定，或用 --calib 指定标定文件。"
        )


def check_calibration(gripper) -> None:
    """:func:`check_calibration_values` 在 ``gripper.config`` 上的包装。"""
    cfg = gripper.config
    check_calibration_values(cfg.pos_closed_rad, cfg.pos_open_rad,
                             cfg.rad_to_mm, getattr(cfg, "max_stroke_mm",
                                                    NOMINAL_STROKE_MM))


# ═════════════════════════════════════════════════════════════════════════
# 开度 ⇄ 目标角
# ═════════════════════════════════════════════════════════════════════════


def fraction_to_target_rad(gripper, fraction: float) -> float:
    """归一化开度 → 真机的电机目标角 [rad]。

    **归一化开度就是标定行程的百分比**：``0`` 是标定出来的闭合位，``1`` 是标定
    出来的张开位，中间线性。

        position_rad = pos_closed_rad − fraction × (pos_closed_rad − pos_open_rad)

    这里**不经过毫米**，也不碰 ``cfg.max_stroke_mm``。这正是它与 ``goto(mm)`` 的
    唯一区别，理由见 :func:`rad_to_fraction`：``position_mm`` 那把尺子是按**标定
    时那个** ``max_stroke_mm`` 定的，而 ``load_calibration()`` 从不写这个字段，它
    一直是 SDK 的默认值 120——两者对不上的时候，走毫米的换算会在中途饱和。按行程
    归一没有这个前提。

    前提是标定自洽（``pos_closed_rad`` 比 ``pos_open_rad`` 更**正**）。SDK 的
    出厂默认配置把 ``pos_open_rad`` 写成 ``+1.14``，与 ``goto`` 的符号约定相矛盾，
    此时这个函数的结果没有意义——所以真机样例在使能前会用 :func:`check_calibration`
    挡掉这种配置，而不是硬发一条越界的角度。
    """
    cfg = gripper.config
    travel = cfg.pos_closed_rad - cfg.pos_open_rad
    fraction = max(0.0, min(1.0, float(fraction)))
    return cfg.pos_closed_rad - fraction * travel


def rad_to_fraction(gripper, position_rad: float) -> float:
    """真机的电机角 [rad] → 归一化开度（:func:`fraction_to_target_rad` 的逆）。

    按**标定行程**归一，不按毫米——理由在下面，值得读完再改成「除以
    ``max_stroke_mm``」的写法。

    为什么不走毫米。SDK 的毫米刻度由 ``rad_to_mm`` 定，而 ``rad_to_mm`` 是标定
    那一刻用**当时那个** ``max_stroke_mm`` 算出来的。可是
    ``load_calibration()`` 从不写 ``config.max_stroke_mm``，它一直是
    ``GripperConfig`` 的默认值 120.0。两者只要对不上，
    ``position_mm / max_stroke_mm`` 这条映射就会在中途**饱和**：本机标定文件里的
    ``rad_to_mm`` 对应 86 mm 刻度（出厂那份是 1.409552 rad × 61.012 = 86.0 mm），
    于是行程走到 ``86 / 120 = 72%`` 就顶住了——再往上推目标角不再变化，窗口里的
    仿真手指也张不到底，而且拖到 50% 实际给的是全行程的 70%。按行程归一没有这个
    前提：它只用 ``pos_closed_rad`` / ``pos_open_rad``，而这两个角每次标定都实测。

    （标定自洽、即 ``rad_to_mm × travel == max_stroke_mm`` 时，两种写法结果相同；
    差别只在它们对不上的时候。）

    行程非正时返回 ``0.0``：标定本身不自洽，真机路径上 :func:`check_calibration`
    会先把它挡掉，这里只是不做除法。
    """
    cfg = gripper.config
    travel = cfg.pos_closed_rad - cfg.pos_open_rad
    if travel <= 0.0:
        return 0.0
    return max(0.0, min(1.0, (cfg.pos_closed_rad - float(position_rad)) / travel))


def fraction_to_gap_mm(fraction: float) -> float:
    """归一化开度 → 物理钳口间隙 [mm]。

    本仓的毫米口径是**真实毫米**（``constants.MM_SCALE`` = 85.452 mm 全行程，
    闭合时 1.548 mm），不是 SDK 名义的 120 mm 刻度——所以
    :func:`status_line` 里的毫米数由它给，而不是从一个 ``sdk_mm`` 换算过来。
    """
    from litegrip_mujoco import constants as C

    return C.q_to_gap_mm(C.q_from_frac_open(fraction))


# ═════════════════════════════════════════════════════════════════════════
# 真机状态
# ═════════════════════════════════════════════════════════════════════════

#: 读真机状态时最多等一帧状态帧的时间 [s]。SDK 的 ``get_state(wait=True)`` 内部
#: 也是等 50 ms，这里对齐它。
FRESH_WAIT_S = 0.05

#: 只读路径（``--status``）等一帧的时间 [s]。那条路径上电机通常**没使能**，
#: 不会主动发帧，所以等不到是常态而不是故障——等久一点只是给「别的程序刚放过帧」
#: 留点余地，不是指望它一定回话。
STATUS_WAIT_S = 0.5


def fresh_state(gripper, timeout_s: float = FRESH_WAIT_S):
    """等到一帧**新**的状态帧再读快照；等不到返回 ``None``。

    这是本仓库读真机位置的正确入口（03/04/05 都用它）。判据只有一条：

    :meth:`LiteGrip.poll` 为真 ⟹ **这次调用里**解出了一帧本电机的状态帧
    （SDK 自己会把读寄存器的参数应答帧排除掉），于是紧随其后的
    ``get_state(wait=False)`` 读到的就是刚才那一帧。为假就是没有新帧，返回
    ``None``。

    为什么非要问这一句：缓存里可能是 ``MotorState._position`` 的初值 ``0.0``，
    或者一个冻结的旧值。拿它当「现在的位置」去算目标角和斜坡时长，算出来的是一
    条指向别处的**阶跃**指令——电机按标定里的 ``kp`` 去追一个不存在的误差，就是
    「一开夹爪就起飞」的形态。所以拿不到新鲜读数时，调用方应当**拒绝下发**，而
    不是猜一个值。

    电机**没使能**时它不会主动发帧，这里就会一直返回 ``None``。所以未使能的只读
    路径必须把「读不到位置」当成正常结果处理，不要报错。

    两种 ``poll()`` 都认：真机 SDK 的那个返回 ``bool``；本仓
    :class:`~litegrip_mujoco.DryRunGripper` 顶替真机时返回 ``None``（它内部是一台
    仿真，快照永远是刚算出来的，「新鲜度」这个概念对它不成立），返回 ``None``
    就当作「没有新鲜度这一说」，直接读快照。

    Args:
        gripper: 夹爪。
        timeout_s: 最多等多久 [s]。

    Returns:
        ``GripperState``；``timeout_s`` 内没有新的状态帧则 ``None``。
    """
    poll = getattr(gripper, "poll", None)
    if poll is not None:
        try:
            fresh = poll(timeout_s=timeout_s)
        except TypeError:      # 老签名不带关键字参数
            fresh = poll(timeout_s)
        if fresh is not None and not fresh:
            return None
    return gripper.get_state(wait=False)


# ═════════════════════════════════════════════════════════════════════════
# CAN 接口：先探测，只在真的不对时才拉起
# ═════════════════════════════════════════════════════════════════════════
#
# 这一段照上位机 litegrip-studio 的 ``can_link.py`` 做，三条规则一样：
#
#   1. **便利，不是闸门。** 探测或拉起失败绝不拦住后面的连接尝试——只有那条路
#      才知道链路到底通不通。
#   2. **尽量少改。** 先探测；接口已经是 SDK 需要的样子就一条特权命令都不跑，
#      也就不弹密码。只有真的不对的状态才修。
#   3. 修不成就**说清楚**，并给出可粘贴的手工命令。
#
# 为什么非要读 ``can state``、不能只看标志位：``ENETDOWN``（errno 100，
# "Network is down"）有**两个**来源。一是接口没 up，看 ``<...UP...>`` 就知道；
# 二是控制器 **BUS-OFF**——这时 ``ip`` 照样印 ``UP,LOWER_UP``、比特率也正确，
# **每个标志位都是对的**，但任何一帧都发不出去。真机上撞到的就是这个形状：
#
#     enable() 第 1/3 次抛错：HardwareError: 使能失败: [Errno 100] Network is down
#
# SDK 是在 ``enable()`` 发**第一帧**时才撞上它的（``connect()`` 只开 socket 和
# bind，而这两步在 down 的接口上照样成功），于是样例把它译成「夹爪可能处于错误
# 状态或未上电」——把一个主机侧的链路问题算到了夹爪头上。这里把它翻回来。

#: 真机样例要的总线速率（达妙电机 1 Mbit/s）。
CAN_BITRATE = 1_000_000

#: 默认接口名（``--channel`` 的默认值）。
CAN_CHANNEL = "can0"

#: ``restart-ms``：控制器进入 BUS-OFF 后，内核隔多久自动把它拉回总线。
#: **0 就是「不自动恢复」**，bus-off 会一直锁着直到有人重新配置接口；100 是
#: SocketCAN 文档推荐值。内核自己的默认值就是 0（本机 can0 现在也印着 0），所以
#: 一帧坏帧就能把总线锁到下一次有人拉接口为止。
CAN_RESTART_MS = 100

#: 控制器状态的健康值。``ERROR-PASSIVE`` 只报告不修（总线边际时会出现，它会自己
#: 恢复）；``BUS-OFF`` 就是上面那个「标志位全对却发不出帧」。
CAN_ERROR_ACTIVE = "ERROR-ACTIVE"
CAN_BUS_OFF = "BUS-OFF"

#: 内核会印出来的控制器状态（``drivers/net/can/dev/dev.c`` 里那张 switch 的
#: 取值）。列全了才好把 ``can state X`` 和上面那行 operstate 分开——operstate 的
#: 值是 ``UP``/``DOWN``/``UNKNOWN``，一个都不在这里面。
_CAN_STATES = (
    CAN_ERROR_ACTIVE, "ERROR-WARNING", "ERROR-PASSIVE", CAN_BUS_OFF,
    "STOPPED", "SLEEPING",
)

#: 接口名允许长什么样。它是外部输入里唯一会进命令的一段，所以先卡一道窄的：
#: 名字只当**参数**交给 `ip`（从不拼进 shell 字符串），但提前拒掉不像接口名的
#: 东西，能让这条保证不依赖命令是怎么拼出来的。
CAN_DEVICE_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,14}$")

#: `ip` 的超时。探测是只读的，5 s 足够；配置在慢机器（或适配器重枚举）上久一点。
CAN_PROBE_TIMEOUT_S = 5.0
CAN_SETUP_TIMEOUT_S = 120.0

#: 发不出去帧时内核回的 errno。这三个都指向**主机侧**的链路，与夹爪无关：
#: ENETDOWN=接口没 up 或控制器 bus-off，ENXIO=适配器没了，ENODEV=接口没了。
_LINK_ERRNOS = frozenset({errno.ENETDOWN, errno.ENXIO, errno.ENODEV})
_ERRNO_TEXT = re.compile(r"\[Errno (\d+)\]")


@dataclass(frozen=True)
class CanLinkState:
    """一次 ``ip -details link show <dev>`` 读出来的接口状态。"""

    exists: bool
    up: bool = False
    bitrate: int | None = None
    fd: bool = False
    can_state: str = ""
    is_can: bool = False

    @property
    def deaf(self) -> bool:
        """控制器在 BUS-OFF：标志位看不出问题，但一帧都发不出去。"""
        return self.can_state == CAN_BUS_OFF

    def ready(self, bitrate: int) -> bool:
        """这就是 SDK 需要的状态吗？是的话什么都不用做。"""
        return (
            self.exists
            and self.is_can
            and self.up
            and self.bitrate == bitrate
            and not self.fd
            and not self.deaf
        )

    def describe(self) -> str:
        if not self.exists:
            return "不存在"
        if not self.is_can:
            return "不是 CAN 接口"
        if self.bitrate is None:
            return "已 up，但没配比特率" if self.up else "存在，没配比特率，也没 up"
        mode = "CAN FD" if self.fd else "经典 CAN"
        # 非健康状态一直印出来：ERROR-PASSIVE 我们不修，不印就等于没看见。
        trouble = (
            f"，控制器 {self.can_state}"
            if self.can_state and self.can_state != CAN_ERROR_ACTIVE
            else ""
        )
        up = "已 up" if self.up else "未 up"
        return f"{up}，{mode}，比特率 {self.bitrate}{trouble}"


def parse_can_link(text: str, returncode: int = 0) -> CanLinkState:
    """把 ``ip -details link show`` 的输出读成状态。

    纯函数：``ip`` 的输出长什么样只有这里知道，测试直接喂真机的原文。
    """
    if returncode != 0 or "does not exist" in text:
        return CanLinkState(exists=False)
    flags = ""
    start = text.find("<")
    end = text.find(">", start + 1)
    if start != -1 and end != -1:
        flags = text[start + 1:end]
    bitrate = re.search(r"\bbitrate (\d+)", text)
    # 按**取值**匹配、不照搬上位机的 ``\bcan state (\S+)``：开了 BERR-REPORTING 的
    # 接口印的是 ``can <BERR-REPORTING> state BUS-OFF``，中间插了一截，上位机那条
    # 正则在这台接口上会一个状态都读不到——包括最要命的 BUS-OFF。上面那行
    # operstate（``state UP``/``DOWN``/``UNKNOWN``）的取值都不在 _CAN_STATES 里，
    # 所以这样放开也不会读到它。
    can_state = re.search(r"\bstate (%s)\b" % "|".join(_CAN_STATES), text)
    return CanLinkState(
        exists=True,
        # 按逗号切开再比，**不要**用子串判断：``LOWER_UP`` 里也有 "UP"，一个
        # ``<NOARP,LOWER_UP>``（管理上没起来）会被子串判成「已 up」。
        # 也不看后面的 ``state UP``／``state UNKNOWN``：那是 operstate，`lo` 就是
        # 「管理上 up、operstate 却 UNKNOWN」的那种。
        up="UP" in flags.split(","),
        # ``\b`` 是为了不被 CAN FD 那一行 ``dbitrate 2000000`` 骗到（它的标称
        # 比特率仍是 bitrate 那行的值）。
        bitrate=int(bitrate.group(1)) if bitrate else None,
        fd=re.search(r"\bfd on\b", text) is not None,
        # 读不到就留空——空值不触发任何动作：没印不等于有病。
        can_state=can_state.group(1) if can_state else "",
        # ``link/can`` 是 `ip -details` 给 CAN 接口加的那一行，接口还没配过也在。
        # 有它才说明 --channel 指的是个 CAN 口；不然 ``ensure_can_link`` 会为了一个
        # 打错的参数对着 eth0 之流 down/up，把网卡停一下。
        is_can="link/can" in text,
    )


def probe_can_link(channel: str = CAN_CHANNEL, *, run=subprocess.run):
    """读一次接口状态（只读，不提权）。

    Returns:
        :class:`CanLinkState`；**探测不出来就返回 ``None``**（没有 ``ip``、命令
        超时、接口名不像接口名）。``None`` 不是「有病」，是「没结论」，调用方
        不该据此去动系统——沉默不是故障的证据。
    """
    if not CAN_DEVICE_RE.match(channel):
        return None
    try:
        result = run(
            ["ip", "-details", "link", "show", channel],
            capture_output=True, text=True, timeout=CAN_PROBE_TIMEOUT_S,
            # 下面要按英文串分支（``does not exist``），别让操作员的 locale 改掉它。
            env={**os.environ, "LC_ALL": "C"},
        )
    except (OSError, subprocess.SubprocessError):
        return None
    text = (result.stdout or "") + (result.stderr or "")
    return parse_can_link(text, result.returncode)


def manual_can_hint(channel: str = CAN_CHANNEL, *, bitrate: int = CAN_BITRATE,
                    restart_ms: int = CAN_RESTART_MS) -> str:
    """能直接粘进终端的三条命令（加一条查看结果的）。"""
    configure = f"sudo ip link set {channel} type can bitrate {bitrate}"
    if restart_ms:
        configure += f" restart-ms {restart_ms}"
    configure += " fd off"
    return "\n     ".join([
        f"sudo ip link set {channel} down",
        configure,
        f"sudo ip link set {channel} up",
        f"ip -details link show {channel}",
    ])


def _run_ip(argv: list, *, run) -> tuple:
    """跑一条 ``sudo ip ...``；返回 ``(成功?, 合并后的输出)``。"""
    try:
        result = run(["sudo", *argv], capture_output=True, text=True,
                     timeout=CAN_SETUP_TIMEOUT_S)
    except (OSError, subprocess.SubprocessError) as exc:
        return False, str(exc)
    text = ((result.stdout or "") + (result.stderr or "")).strip()
    return result.returncode == 0, text


def ensure_can_link(channel: str = CAN_CHANNEL, *, bitrate: int = CAN_BITRATE,
                    restart_ms: int = CAN_RESTART_MS, repair: bool = True,
                    run=subprocess.run, which=shutil.which, out=print) -> bool:
    """连接之前把 CAN 接口准备到 SDK 能用的状态。

    ``repair=False`` 时只探测、只报告，不动系统——05 的 ``--status`` 和 04 的
    ``--passive`` 走这条：它们的定义就是「只看不动」，而且 ``--status`` 要靠**真实**
    的接口状态来诊断，替它改掉就把证据抹了。

    Args:
        channel: 接口名。
        bitrate: 期望的比特率。
        restart_ms: ``restart-ms``，0 表示不写这一项。
        repair: 允许用 sudo 拉起（False = 只报告）。
        run / which / out: 测试注入用（默认 ``subprocess.run`` / ``shutil.which``
            / ``print``）。

    Returns:
        True = 接口现在可用（含「压根探测不出来」，那由后面的连接去判）；
        False = 明确看到状态不对、而且没能修好（原因和手工命令已经打印过了）。
    """
    state = probe_can_link(channel, run=run)
    if state is None:
        return True
    if state.ready(bitrate):
        return True
    out(f"   CAN 接口 {channel}：{state.describe()}")

    if not state.exists:
        # ``exists=False`` 来自 ``ip`` 退出码非 0 或那句 does not exist，所以话只说
        # 到「读不到」：读不到多半是没有这个接口，但也可能是 `ip` 自己出错了。
        out(f"   读不到 {channel}（多半是这台机器上没有这个接口）：适配器插好了吗"
            f"（lsusb 里应看到 gs_usb）？或者用 --channel 指到真正的接口。")
        return False
    if not state.is_can:
        # 这一条是「参数打错了」，不是「接口不对」：对 eth0 做 down/up 只会把网卡
        # 停一下，而它本来就不是 SDK 要找的东西。
        out(f"   {channel} 不是 CAN 接口（`ip -details link show {channel}` 里没有 "
            f"link/can 那一行）。--channel 该给的是 SocketCAN 接口，像 can0。")
        return False
    if state.fd:
        # 不替你改：SDK 自己会按 MTU 适配 FD，而「把一条别的节点也在用的总线
        # 按猜测改写」不是这里该做的事。
        out("   接口是 CAN FD。SDK 会自己按 MTU 适配，所以这里不动它——"
            "要改成经典 CAN 请自己来。")
        return False
    if state.deaf:
        out(f"   控制器在 {CAN_BUS_OFF}：标志位看着都好，但一帧都发不出去。"
            "先 down 再 up 能把它清掉。")
    if not repair:
        out(f"   只探测不拉起（这条路径只看不动）。手动配置：\n     "
            f"{manual_can_hint(channel, bitrate=bitrate, restart_ms=restart_ms)}")
        return False
    if not sys.stdin.isatty():
        # 管道/CI 里 sudo 要不到密码，会一直挂着。只打印，让它去连。
        out("   非交互环境（stdin 不是终端），不跑特权命令。手动配置：\n     "
            f"{manual_can_hint(channel, bitrate=bitrate, restart_ms=restart_ms)}")
        return False
    if which("sudo") is None:
        out("   没有 sudo，没法配置接口。用 root 手动配置：\n     "
            f"{manual_can_hint(channel, bitrate=bitrate, restart_ms=restart_ms)}")
        return False

    out(f"   要配置 {channel}（sudo 可能会问密码）。")
    # 先 down：CAN 的比特率不能在上着的接口上改。
    ok, text = _run_ip(["ip", "link", "set", channel, "down"], run=run)
    if not ok:
        out(f"   down 失败：{text}")
        return False

    base = ["ip", "link", "set", channel, "type", "can", "bitrate", str(bitrate)]
    extra = ["restart-ms", str(restart_ms)] if restart_ms else []
    ok, text = _run_ip([*base, *extra, "fd off"], run=run)
    if not ok and extra and "restart" in text.lower():
        # 有些适配器（台架这块 gs_usb 克隆就是）不认 restart-ms。**只重试这一条**、
        # 且只在错误信息提到 restart 时重试：别的失败一并重试会把「偶发错误」
        # 变成「静默降级」——接口是起来了，可再也不自动从 bus-off 恢复。
        out(f"   适配器不认 restart-ms（{text}），去掉它重试一次。")
        ok, text = _run_ip([*base, "fd off"], run=run)
    if not ok:
        out(f"   配置失败：{text}")
        out(f"   手动配置：\n     "
            f"{manual_can_hint(channel, bitrate=bitrate, restart_ms=restart_ms)}")
        return False

    ok, text = _run_ip(["ip", "link", "set", channel, "up"], run=run)
    if not ok:
        out(f"   up 失败：{text}")
        # 适配器丢掉 USB endpoint 表时（up 报 No such file or directory），上位机
        # 会 modprobe -r/modprobe 重载驱动再配一次。从一个样例脚本里去卸载内核
        # 模块不成比例，这一条留给上位机；这里只说清楚是什么。
        out("   如果这条错误里提到 No such file or directory，那是适配器丢了 USB "
            "端点表：拔插一次适配器，或用上位机 litegrip-studio 连接一次。")
        return False

    after = probe_can_link(channel, run=run)
    if after is not None and after.ready(bitrate):
        out(f"   {channel} 已就绪（{after.describe()}）")
        return True
    out(f"   配置命令都成功了，但复查仍然不对："
        f"{after.describe() if after else '探测不出来'}")
    out(f"   手动看一下：\n     "
        f"{manual_can_hint(channel, bitrate=bitrate, restart_ms=restart_ms)}")
    return False


def can_link_failure(exc, channel: str = CAN_CHANNEL, *, run=subprocess.run):
    """把 SDK 那句「使能失败」翻成主机的链路问题；不是这类错就返回 ``None``。

    ``ENETDOWN``/``ENXIO``/``ENODEV`` 都只说明**帧发不出去**，与夹爪无关——而 SDK
    是在 ``enable()`` 发第一帧时才撞上它们的，所以看上去像是夹爪没上电。
    """
    match = _ERRNO_TEXT.search(str(exc))
    if match is None or int(match.group(1)) not in _LINK_ERRNOS:
        return None
    state = probe_can_link(channel, run=run)
    seen = (f"现在读到 {channel}：{state.describe()}" if state is not None
            else f"{channel} 探测不出来（没有 ip？）")
    return (
        f"这是主机的 CAN 链路问题，不是夹爪：{exc}\n"
        f"   {seen}\n"
        f"   发不出帧（ENETDOWN）只有两种成因：接口没 up，或者控制器 BUS-OFF。\n"
        f"   后者 ``ip`` 照样印 UP,LOWER_UP、比特率也对，只有 ``can state`` 分得开。\n"
        f"   手工配置：\n     {manual_can_hint(channel)}"
    )


def connect_failure_message(channel: str = CAN_CHANNEL) -> str:
    """``connect()`` 没成的时候该说的话（03/04/05 共用这一份）。

    别把它和 :func:`can_link_failure` 混了：``connect()`` 只建 socket 和 bind，
    而这两步在**没 up 的接口上照样成功**，所以卡在这里几乎只剩两种情况——接口
    根本不存在，或者 socket 建不出来。接口没起来、控制器 BUS-OFF 都不是这里的
    事，它们是 ``enable()`` 发第一帧时才现形的。
    """
    return (
        f"连不上 {channel}。connect() 只建 socket 和 bind，这两步在没 up 的接口\n"
        f"   上也会成功，所以停在这里多半是：接口不存在，或者 socket 建不出来。\n"
        f"   1) ip -details link show {channel} —— 不存在就查适配器（lsusb 里应看到\n"
        f"      gs_usb）和 --channel 的名字对不对\n"
        f"   2) 接口在、还是建不出来：看内核有没有编 CAN（modprobe can_raw）\n"
        f"   3) 夹爪是否已上电、CAN_H/CAN_L 是否接对、终端电阻（120Ω）是否装了"
    )


def enable_failure_message(channel: str = CAN_CHANNEL, *, run=subprocess.run) -> str:
    """``enable()`` 返回 False 时的说明。

    使能是**第一次真的把帧发出去**，所以主机侧的链路问题也在这时候才现形。先探测
    一遍：接口要是根本不对，就直说是链路，别把锅甩给夹爪。
    """
    state = probe_can_link(channel, run=run)
    if state is not None and not state.ready(CAN_BITRATE):
        return (
            f"使能失败，而 {channel} 现在不是 SDK 需要的状态（{state.describe()}）：\n"
            f"   先修链路再谈夹爪。手工配置：\n     {manual_can_hint(channel)}"
        )
    seen = state.describe() if state is not None else "探测不出来"
    return (f"使能失败：夹爪可能处于错误状态或未上电（{channel} 这时的状态：{seen}）")


# ═════════════════════════════════════════════════════════════════════════
# 真机连接
# ═════════════════════════════════════════════════════════════════════════


def open_real_gripper(args, enable: bool = True, *, dry_run: bool = False,
                      noise: bool = True):
    """连接真机夹爪：导入 SDK → 选标定 → 探测 CAN 口 → connect → 载入并核实标定
    → enable。

    标定在**连接之前**就定下来（:func:`choose_calibration_file`）：``--calib``
    给的优先，没给就用 SDK 包里那份出厂标定；出厂文件也读不出来就直接退出，让你
    显式给 ``--calib``。载入走库层的
    :func:`~litegrip_mujoco.apply_calibration`，它会逐个字段核实「生效的确实是
    这一份」——SDK 在文件读不出来时会**静默**改用出厂标定并照样返回 ``True``，
    光看返回值不够。

    ``--dry-run`` **不导入 SDK**：它不碰 CAN，而 ``import litegrip`` 在没装
    SocketCAN 依赖的机器上会失败。它也因此不依赖任何标定——替身的刻度是它自己
    带的，`--calib` 在这里只用来显示摘要，一份都找不到就跳过这一步，不退出。

    任一步失败都打印可读的原因并 ``SystemExit(1)``，不会抛裸异常。

    Args:
        args: 命令行参数（``--channel`` / ``--can-id`` / ``--mst-id`` / ``--calib``）。
        enable: 是否使能。``False`` 时只连接并载入标定，**一个运动指令都不发**，
            电机保持原状——用来在不动电机的前提下先看看状态。
        dry_run: 用 :class:`~litegrip_mujoco.DryRunGripper` 顶替真机：不碰 CAN。
            它对外声称的是**真机口径**的 θ 端点，所以开度换算那段路径与接真机时
            完全一致。
        noise: ``dry_run`` 时是否给读数加传感器噪声。

    Returns:
        ``litegrip.LiteGrip``（``enable=True`` 时已使能），``dry_run`` 时是
        :class:`~litegrip_mujoco.DryRunGripper`。
    """
    from litegrip_mujoco import (
        CalibrationError,
        DryRunGripper,
        apply_calibration,
    )

    factory = factory_calibration_path()
    try:
        calib_path = choose_calibration_file(args.calib, factory=factory)
    except SystemExit:
        if not dry_run or args.calib:
            # 显式点名的那份有问题就是错，--dry-run 也不吞：那多半是路径打错了字，
            # 而信错的路径比看不见路径更糟。
            raise
        # --dry-run 没碰 CAN，也就不依赖任何标定：替身的刻度是它自己带的
        # （真机口径的 θ 端点）。这里只放弃「显示这份文件」这一步，不放弃整场演示。
        print("   [真机] --dry-run：没有显示标定文件（没给 --calib，"
              "也没找到 SDK 出厂标定）")
        print("          替身不受影响：它的刻度是自己的，不读标定文件。"
              "要让这里显示某一份，用 --calib <路径>")
        calib_path = None

    if calib_path is not None:
        calibration = read_calibration_file(calib_path)
        notes = check_calibration_matches_args(
            calibration, channel=args.channel, can_id=args.can_id,
            mst_id=args.mst_id)
        # 「是不是出厂那份」按**本层**resolve 出来的那个路径比，而不问库层的
        # ``calibration.is_sdk_factory_path``：库层只从 sys.path 上找 SDK，而这里
        # 的 factory 还会看 $LITEGRIP_SDK_DIR 与同级检出——同一个文件，两边不一定
        # 都认得。标签要跟这次真正用的那份文件走。
        print(f"   [真机] 标定 {calib_path}"
              + ("（SDK 出厂标定）" if is_sdk_factory_calibration(calib_path, factory)
                 else ""))
        print(f"          {calibration_summary(calibration.raw)}")
        for note in notes:
            print(note)

    if dry_run:
        print("   [真机] DryRunGripper（--dry-run：不碰 CAN，内部是一台独立仿真；")
        print("          它对外声称真机口径的 θ 端点，换算路径与接真机时一致）")
        gripper = DryRunGripper(realtime=True, noise=noise)
        gripper.connect()
        check_calibration(gripper)
        if enable:
            gripper.enable()
        return gripper

    litegrip = import_litegrip()
    check_sdk_api(litegrip)   # 缺公开接口就别连——宁可现在停，也别在循环里才发现

    print(f"   [真机] 连接 {args.channel} · can_id={args.can_id:#04x} · "
          f"mst_id={args.mst_id:#04x}")
    if not args.no_can_setup:
        # 放在建 LiteGrip 之前：接口不对就别先开 socket。repair=enable——05 的
        # --status 和 04 的 --passive 走的是 enable=False，那两条的定义就是「只看
        # 不动」，替它们改掉接口，恰好把 --status 要诊断的东西抹了。
        ensure_can_link(args.channel, repair=enable)
    gripper = litegrip.LiteGrip(channel=args.channel, can_id=args.can_id,
                                mst_id=args.mst_id)
    try:
        if not gripper.connect():
            raise SystemExit(connect_failure_message(args.channel))
        # 标定必须在 enable 之前载入：SDK 的毫米刻度依赖它，轨迹的归一化开度也是。
        # 出厂标定在这条路径上就是一份普通文件——库层照常校验端点，上面也已经
        # 把「这是台架夹具的参数」那句话打出来了。
        try:
            applied = apply_calibration(gripper, calib_path)
        except CalibrationError as exc:
            gripper.disconnect()
            raise SystemExit(f"标定没载入成功：\n{exc}") from exc
        print(f"   [真机] 已载入并核实标定：closed={applied.pos_closed_rad:+.6f} "
              f"open={applied.pos_open_rad:+.6f}")
        if not enable:
            # 只说「不发送运动指令」：--status 走这条路（未使能），但读寄存器仍要
            # 发读请求帧，说「一帧都不发」就把话说大了。04 的 --passive 才是真的
            # 一帧不发，它自己会这么说。
            print("   [真机] 已连接、已载入并核实标定（未使能，不发送运动指令）")
            return gripper
        if not gripper.enable():
            raise SystemExit(enable_failure_message(args.channel))
    except SystemExit:
        gripper.disconnect()
        raise
    except Exception as exc:      # SDK 的各种 *Error
        gripper.disconnect()
        # 使能那条路上撞到的 [Errno 100] 会走到这里（SDK 把 enable 的 OSError 包成
        # HardwareError 抛出）：翻成主机链路问题，别再让操作员去查夹爪上电没有。
        raise SystemExit(can_link_failure(exc, args.channel)
                         or f"初始化真机失败：{exc}") from exc

    cfg = gripper.config
    # kp/kd 一起打出来：它们是标定文件里的值（也是保持帧的刚度），改了标定之后
    # 「手感怎么变了」这个问题，第一件要看的就是这两个数。
    #
    # 行程打的是**实测的那两个角之差**，不是 ``max_stroke_mm``：后者是 SDK 的名义
    # 默认值，``load_calibration`` 不写它，所以它和这份标定对不对得上完全看标定是
    # 怎么做的（见 :func:`rad_to_fraction`）。
    print(f"   [真机] 已使能 · 行程 {cfg.pos_closed_rad - cfg.pos_open_rad:.4f} rad"
          f"（{cfg.pos_closed_rad:+.4f} → {cfg.pos_open_rad:+.4f}）"
          f" · rad_to_mm={cfg.rad_to_mm:.2f}"
          f" · kp={cfg.kp:g} kd={cfg.kd:g}")
    return gripper


# ═════════════════════════════════════════════════════════════════════════
# 输出
# ═════════════════════════════════════════════════════════════════════════


def status_line(label: str, *, fraction: float, aperture_mm: float,
                force_n: float | None = None,
                moving: bool | None = None) -> str:
    """一行状态文本，五个样例共用，保证口径一致。

    Args:
        label: 行首标签（``仿真`` / ``真机``）。
        fraction: 归一化开度（0 闭合 … 1 张开）——两侧唯一可比的量。
        aperture_mm: 物理钳口间隙 [mm]（本仓是**真实毫米**，见
            :func:`fraction_to_gap_mm`）。
        force_n: 夹持力 [N]。
        moving: 是否在动，真机才有。
    """
    bar_width = 20
    filled = int(round(bar_width * max(0.0, min(1.0, fraction))))
    bar = "█" * filled + "·" * (bar_width - filled)
    parts = [f"[{label}] {bar} {fraction * 100:5.1f}%",
             f"开口 {_num(aperture_mm, 2):>5} mm"]
    if force_n is not None:
        parts.append(f"力 {_num(force_n, 2):>5} N")
    if moving is not None:
        parts.append("运动中" if moving else "已停住")
    return " · ".join(parts)


def _num(value: float, decimals: int) -> str:
    """按 ``decimals`` 格式化，并把「四舍五入后是零」的负零收成 ``0.00``。

    为什么要这一步：空载时 ``get_force()`` 返回的不是正零而是 ``-1e-9`` 一类的
    残差，它 **truthy**，所以 ``f"{-1e-9:.2f}"`` 印出来是 ``-0.00``——看着像个
    真的负力，而这一栏是要给人眼睁睁读的。判据用「格式化之后是不是零」而不是
    「绝对值小于某个 epsilon」，这样显示精度和收敛判据永远是同一个。
    """
    text = f"{value:.{decimals}f}"
    return text if float(text) != 0.0 else f"{0.0:.{decimals}f}"


# 导入本模块时就准备好环境，这样样例只要一句 `from _common import ...` 即可，
# 不需要记住「必须先 import _common 再 import litegrip_mujoco」这个顺序。
bootstrap_src()
ensure_deps()
