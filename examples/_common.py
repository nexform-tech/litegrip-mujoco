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
                      --calibration）/ --list-calibrations
  choose_calibration_file() 定下这次用**哪一份**标定：--calib 指定 → SDK 出厂
                      标定 → 两个都没有才当场从候选里选
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
而标定的角度/毫米刻度本该是每台夹爪单独量的。拿不准就用 ``--calib`` 指这台夹爪
自己的那份：上位机 ``litegrip-studio`` / ``litegrip-console`` 标定后保存，或
SDK 自带的 ``tools/gui/litegrip_gui.py``。

真机跑之前确认 CAN 已配置好：

    sudo ip link set can0 up type can bitrate 1000000

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
  那版多一道「载入后核对端点」），出厂标定靠显式的 ``allow_factory=True`` 打开。
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from types import SimpleNamespace

__all__ = [
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
    "check_calibration",
    "check_calibration_matches_args",
    "check_calibration_values",
    "check_sdk_api",
    "choose_calibration_file",
    "ensure_deps",
    "factory_calibration_path",
    "fraction_to_gap_mm",
    "fraction_to_target_rad",
    "fresh_state",
    "import_litegrip",
    "list_calibrations",
    "make_sim",
    "missing_sdk_api",
    "open_real_gripper",
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
    """加真机连接参数 ``--channel`` / ``--can-id`` / ``--mst-id`` / ``--calib``。"""
    parser.add_argument(
        "--channel", default="can0",
        help="SocketCAN 接口名（默认 can0）",
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
             "文件也读不出来时才在终端里列候选让你选。标定文件由 "
             "litegrip-studio / litegrip-console 对着真机标定后保存得到",
    )
    parser.add_argument(
        "--list-calibrations", action="store_true",
        help="列出候选标定文件及其端点后退出（不连真机、不发帧）",
    )


def list_calibrations(out=print) -> int:
    """``--list-calibrations``：打印候选标定文件。返回退出码。"""
    from litegrip_mujoco import describe_candidate, discover_calibrations

    candidates = discover_calibrations()
    if not candidates:
        out("没有找到候选标定文件。标定文件由上位机（GUI）标定后生成，")
        out("默认写在 ~/.litegrip/，也可以放在当前目录。")
        return 0
    out(f"候选标定文件（{len(candidates)} 个；SDK 自带的出厂标定不在其中）:")
    for index, path in enumerate(candidates, start=1):
        out(f"  {index}) {path}")
        out(f"     {describe_candidate(path)}")
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
    # 这里就会漏掉，于是退回「列出候选让人选」——非交互环境里直接失败，是响的，
    # 不是错的。反过来找**对**了而库层不认识它（同一份文件、两条路径），库层的
    # 拒绝会照常触发，也是一条明确的报错。
    directory = sdk_dir()
    if directory is not None:
        candidate = directory / "litegrip" / "factory_calibration.json"
        if candidate.is_file():
            return candidate
    return None


def choose_calibration_file(requested=None, *, factory=None, ask=None, out=print):
    """定下这次用哪份标定文件。三档，**顺序就是优先级**。

    * ``--calib <路径>``（``requested``）：直接用，不提问；只做校验。
    * ``factory`` 给了且读得出来：用 SDK 包里那份出厂标定，**不提问**，只打一行
      说明。这是默认路径——``factory`` 由调用方用
      :func:`factory_calibration_path` 算出来，所以它跟着包走，换电脑也一样。
    * 出厂标定也读不出来（SDK 装得残缺、文件被删）：才回到选择器——列出候选让
      操作员当场选；非交互（stdin 不是 tty、EOF）或没有候选就直接退出。

    出厂标定是**台架夹具的实测参数**，不是每台夹爪各自量的：它是一份能用的默认
    值，不是「这台夹爪的标定」。要按这台夹爪自己的尺寸驱动，用 ``--calib`` 指
    上位机保存的那份——所以第 2 档那行提示必须把这句话说出来。

    返回的第二个值说明「这是出厂标定」——真机路径要把它转成
    :func:`apply_calibration` 的 ``allow_factory=True``，否则库层会按设计拒掉它
    （它属于任何一台夹爪，也就不属于这一台）。

    Args:
        requested: ``--calib`` 的值（``None`` = 没给）。
        factory: 出厂标定文件的路径；``None`` 表示调用方拿不到 SDK，直接进选择器。
        ask: 取输入的函数（默认 ``input``）——测试注入用。
        out: 打印函数（默认 ``print``）——测试注入用。

    Returns:
        ``(路径, 是不是出厂标定)``。路径已校验存在、可解析、字段齐、端点自洽。

    Raises:
        SystemExit: 没得选、或者选不出来。
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
        return path, False

    if factory is not None:
        path = os.path.abspath(os.path.expanduser(str(factory)))
        try:
            load_calibration_file(path)
        except CalibrationError as exc:
            out(f"（SDK 自带的出厂标定读不出来：{exc}）")
        else:
            out(f"未指定 --calib：使用 SDK 自带的出厂标定 {path}")
            out("   （台架夹具的实测参数，不是这台夹爪自己量的。"
                "换 --calib <路径> 指这台夹爪的那份。）")
            return path, True

    try:
        # 库层的选择器：扫默认目录 + 当前目录，按新旧排，非交互直接报错退出。
        # 它**不会**替你挑一份默认的——这正是这一档存在的意义。
        return resolve_calibration_path(None, input_fn=ask), False
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
# 真机连接
# ═════════════════════════════════════════════════════════════════════════


def open_real_gripper(args, enable: bool = True, *, dry_run: bool = False,
                      noise: bool = True):
    """连接真机夹爪：导入 SDK → 选标定 → connect → 载入并核实标定 → enable。

    标定在**连接之前**就定下来（:func:`choose_calibration_file`）：``--calib``
    给的优先，没给就用 SDK 包里那份出厂标定，出厂文件也读不出来才在终端里选。
    载入走库层的 :func:`~litegrip_mujoco.apply_calibration`，它会逐个字段核实
    「生效的确实是这一份」——SDK 在文件读不出来时会**静默**改用出厂标定并照样返回
    ``True``，光看返回值不够。

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

    if args.list_calibrations:
        raise SystemExit(list_calibrations())

    try:
        calib_path, is_factory = choose_calibration_file(
            args.calib, factory=factory_calibration_path())
    except SystemExit:
        if not dry_run:
            raise
        # --dry-run 没碰 CAN，也就不依赖任何标定：替身的刻度是它自己带的
        # （真机口径的 θ 端点）。这里只放弃「显示这份文件」这一步，不放弃整场演示。
        print("   [真机] --dry-run：没有显示标定文件（没给 --calib，"
              "也没找到 SDK 出厂标定）")
        print("          替身不受影响：它的刻度是自己的，不读标定文件。"
              "要让这里显示某一份，用 --calib <路径>")
        calib_path, is_factory = None, False

    if calib_path is not None:
        calibration = read_calibration_file(calib_path)
        notes = check_calibration_matches_args(
            calibration, channel=args.channel, can_id=args.can_id,
            mst_id=args.mst_id)
        print(f"   [真机] 标定 {calib_path}"
              + ("（SDK 出厂标定）" if is_factory else ""))
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
    gripper = litegrip.LiteGrip(channel=args.channel, can_id=args.can_id,
                                mst_id=args.mst_id)
    try:
        if not gripper.connect():
            raise SystemExit(
                f"连不上 {args.channel}。检查：\n"
                f"   1) 接口是否存在且已起来 —— "
                f"sudo ip link set {args.channel} up type can bitrate 1000000\n"
                f"   2) ip -details link show {args.channel}\n"
                f"   3) 夹爪是否已上电、CAN_H/CAN_L 是否接对、"
                f"终端电阻（120Ω）是否装了"
            )
        # 标定必须在 enable 之前载入：SDK 的毫米刻度依赖它，轨迹的归一化开度也是。
        # allow_factory 只在选中的是 SDK 出厂那份时为真——库层默认拒它（它不属于
        # 任何一台具体夹爪），例程这一层把它打开是因为「没给 --calib」的默认行为
        # 就是用它，而上面已经把这句话打出来了。
        try:
            applied = apply_calibration(gripper, calib_path,
                                        allow_factory=is_factory)
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
            raise SystemExit("使能失败：夹爪可能处于错误状态或未上电")
    except SystemExit:
        gripper.disconnect()
        raise
    except Exception as exc:      # SDK 的各种 *Error
        gripper.disconnect()
        raise SystemExit(f"初始化真机失败：{exc}") from exc

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
