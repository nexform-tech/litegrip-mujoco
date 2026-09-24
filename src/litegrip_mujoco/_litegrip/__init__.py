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

from typing import Any, Optional

from . import _fallback as _fb

#: 真 SDK 是否可用。由下面的惰性探测决定。
HAS_SDK: bool = False

#: 真 SDK 的异常基类（未安装时用兜底版本）。
NotInitializedError: type = _fb.NotInitializedError

_sdk_module: Optional[Any] = None
_sdk_error: Optional[BaseException] = None


def _probe() -> Optional[Any]:
    """尝试 import 真 SDK；只试一次，结果缓存。"""
    global _sdk_module, _sdk_error, HAS_SDK, NotInitializedError
    if _sdk_module is not None or _sdk_error is not None:
        return _sdk_module
    try:
        import litegrip  # type: ignore[import-not-found]
    except BaseException as exc:  # 缺依赖、缺 CAN 库都可能
        _sdk_error = exc
        return None
    _sdk_module = litegrip
    HAS_SDK = True
    if hasattr(litegrip, "NotInitializedError"):
        NotInitializedError = litegrip.NotInitializedError  # type: ignore[assignment]
    return _sdk_module


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
)

for _name in _SDK_NAMES:
    if _sdk_module is not None and hasattr(_sdk_module, _name):
        globals()[_name] = getattr(_sdk_module, _name)
    else:
        globals()[_name] = getattr(_fb, _name)
del _name

__all__ = list(_SDK_NAMES) + [
    "HAS_SDK",
    "NotInitializedError",
    "load_litegrip",
    "sdk_unavailable_reason",
]


def sdk_unavailable_reason() -> Optional[BaseException]:
    """真 SDK 不可用的原因；可用时返回 None。"""
    return _sdk_error


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
