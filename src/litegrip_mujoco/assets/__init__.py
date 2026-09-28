"""打包进 wheel 的 MuJoCo 模型与网格。

- ``litegrip.xml``  —— 纯夹爪模型（2 自由度，无外部物体）：动力学与单位换算的基准
- ``scene.xml``     —— 演示场景：``<include>`` 上面那个，再加地板、灯光与被夹具托住的工件
- ``meshes/*.STL``  —— 从 `litegrip-urdf` 逐字节拷入的三个网格（注意扩展名是大写 ``.STL``）

用 :func:`model_path` 取绝对路径，不要手拼 —— 安装后本包可能位于 zip 之外的
任何位置。
"""
from __future__ import annotations

import os

ASSETS_DIR = os.path.dirname(os.path.abspath(__file__))


def model_path(name: str = "litegrip.xml") -> str:
    """返回 assets 目录下某个模型的绝对路径。

    Args:
        name: ``"litegrip.xml"``（默认）或 ``"scene.xml"``。

    Raises:
        FileNotFoundError: 该模型不存在。
    """
    path = os.path.join(ASSETS_DIR, name)
    if not os.path.isfile(path):
        available = sorted(
            f for f in os.listdir(ASSETS_DIR)
            if f.endswith(".xml")
        )
        raise FileNotFoundError(
            f"assets 里没有 {name!r}；现有：{available}"
        )
    return path


__all__ = ["ASSETS_DIR", "model_path"]
