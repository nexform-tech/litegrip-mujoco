"""对内的一套数据模型解析器：优先用真的 `litegrip` SDK，否则用本地兜底。

为什么要这一层，而不是直接 ``import litegrip``：

1. `litegrip/__init__.py` 在**模块级**执行 ``from . import can``，而 `can` 子包
   会 import SocketCAN 相关依赖。纯仿真用户没装这些，直接 import 会炸。
2. 但 ``MujocoGripper.get_state()`` 必须返回接口正确的对象，`state.is_grasped`、
   `state.aperture_mm` 这些属性都得在。所以不能干脆不提供类型。

于是：**惰性**尝试真 SDK，失败就退回 `_fallback` 的逐字段镜像。装不装 SDK，
`MujocoGripper` 的公开行为都一样；装了的话 ``isinstance(state, litegrip.GripperState)``
也成立。

**刻意不 vendor SDK 源码**：那会让本包背上 CAN 依赖，还要跟着上游改。

用法::

    from ._litegrip import GripperState, GripperConfig, HAS_SDK

    # 需要真的 LiteGrip 类时（镜像/双控）
    from ._litegrip import load_litegrip
    sdk = load_litegrip()          # 未安装则抛 ImportError，附带安装提示
"""
from __future__ import annotations

import sys
from typing import Any, Optional

from . import _fallback as _fb

#: 真 SDK 是否可用。由下面的惰性探测决定。
HAS_SDK: bool = False

#: SDK 的异常族。仿真只会抛 NotInitializedError，但其余几个也必须能 import ——
#: 否则共用代码里的 `except CommError:` 在纯仿真环境会 ImportError。
_EXC_NAMES = (
    "LiteGripError",
    "NotInitializedError",
    "ConnectError",
    "CommError",
    "CANTimeoutError",
    "HardwareError",
    "CommandError",
)

# 兜底绑定：SDK 缺席时用本地镜像。
for _n in _EXC_NAMES:
    globals()[_n] = getattr(_fb, _n)
del _n

_sdk_module: Optional[Any] = None
_sdk_error: Optional[BaseException] = None


def _adopt(module: Any) -> Any:
    """认下这个 ``litegrip`` 模块：换上它的异常族，置 ``HAS_SDK``。"""
    global _sdk_module, _sdk_error, HAS_SDK
    _sdk_module = module
    _sdk_error = None
    HAS_SDK = True
    # 抛出的异常必须是 SDK 的类对象，不能是本地同名类：真机侧写
    # `except litegrip.LiteGripError` 要能接住仿真抛出的异常。
    for name in _EXC_NAMES:
        if hasattr(module, name):
            globals()[name] = getattr(module, name)
    return _sdk_module


def _probe() -> Optional[Any]:
    """尝试 import 真 SDK；以 ``sys.modules`` 里的那一份为准。

    **本进程的 ``litegrip`` 就是 ``sys.modules["litegrip"]``**，而不是「第一次
    探测时 ``sys.path`` 上是什么」。这两者会不一样，而且必须让后者让路：

    * 例程层的 ``import_litegrip()`` 按 ``$LITEGRIP_SDK_DIR`` / 同级检出定位到
      目录后再用 importlib 显式加载，不靠 ``sys.path``——所以它完全可能在本模块
      第一次探测**失败之后**才把 SDK 装进来。当初那次失败只是说明「那会儿
      ``sys.path`` 上没有」，把它当终局的话，SDK 明明能用而 ``HAS_SDK`` 一直是
      ``False``，于是 :func:`~litegrip_mujoco.sdk_factory_calibration_path` 返回
      ``None``，出厂标定那条「必须显式 ``allow_factory=True`` 才放行」的拒绝就
      静默失效了。
    * 反过来，它也会**换掉** ``sys.modules["litegrip"]``（同名检出与已安装的包
      并存时，正是靠这一手点名用哪一份）。库里攥着旧对象不放，两边的
      ``GripperState`` 就成了两个类，``isinstance`` 会莫名其妙地假。

    失败仍然缓存：缺 CAN 依赖时每次 ``import`` 都要重跑一遍 SocketCAN 的探测，
    代价不小，而 ``sys.modules`` 里没有就是没有。
    """
    global _sdk_error
    live = sys.modules.get("litegrip")
    if live is not None:
        return live if live is _sdk_module else _adopt(live)
    if _sdk_module is not None:
        return _sdk_module
    if _sdk_error is not None:
        return None
    try:
        import litegrip  # type: ignore[import-not-found]
    except BaseException as exc:  # 缺依赖、缺 CAN 库都可能
        _sdk_error = exc
        return None
    return _adopt(litegrip)


_probe()

# 解析每个名字：SDK 有就用 SDK 的，否则用兜底。
_SDK_NAMES = (
    "GripperState",
    "GripperConfig",
    "GripperInfo",
    "GripperStatus",
    "GripperMode",
    "CalibrationData",
    "GripperParams",
    "UnitConversion",
    "ErrorCode",
    "DM_Motor_Type",
    "Control_Mode",
    "ERROR_DESCRIPTIONS",
    "describe_error",
)

for _name in _SDK_NAMES:
    if _sdk_module is not None and hasattr(_sdk_module, _name):
        globals()[_name] = getattr(_sdk_module, _name)
    else:
        globals()[_name] = getattr(_fb, _name)
del _name

# STALE_AFTER_S 不在 SDK 的包级命名空间里，它在 `litegrip.models`。
# 单独取一次，这样 SDK 改了阈值仿真会跟着改，而不是各写各的 0.5。
STALE_AFTER_S: float = _fb.STALE_AFTER_S
if _sdk_module is not None:
    _models = getattr(_sdk_module, "models", None)
    if _models is not None and hasattr(_models, "STALE_AFTER_S"):
        STALE_AFTER_S = _models.STALE_AFTER_S

__all__ = list(_SDK_NAMES) + list(_EXC_NAMES) + [
    "STALE_AFTER_S",
    "HAS_SDK",
    "load_litegrip",
    "sdk_unavailable_reason",
]


def sdk_unavailable_reason() -> Optional[BaseException]:
    """真 SDK 不可用的原因；可用时返回 None。

    先问一次 :func:`_probe`：SDK 可能是**探测失败之后**才被
    ``examples/_common.py`` 那种显式加载装进来的，那时缓存的失败原因已经过期了，
    照它报会让人以为 SDK 不可用。探测本身很便宜（``sys.modules`` 查一次），
    失败路径也仍然是缓存命中。
    """
    return None if _probe() is not None else _sdk_error


def load_litegrip() -> Any:
    """返回真的 `litegrip` 模块，不可用则抛 ImportError。

    镜像/双控模式需要真的 ``litegrip.LiteGrip`` 类，这里给出一个带安装提示的
    失败路径，而不是让调用方看到一个裸 ImportError。
    """
    mod = _probe()
    if mod is None:
        raise ImportError(
            "镜像/双控模式需要 litegrip SDK。安装：pip install -e /path/to/lite-grip\n"
            f"（原始错误：{_sdk_error!r}）"
        )
    return mod
