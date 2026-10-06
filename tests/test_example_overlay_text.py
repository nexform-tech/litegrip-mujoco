#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""例程写进查看器窗口的文字必须是 ASCII。

MuJoCo 的查看器用内置位图字体画叠字（``Handle.set_texts`` → ``mjr_overlay``），
那套字体没有中文字形：一个汉字画出来是一个实心方块，几个字连起来就是一片乱码。
实测（``mjr_overlay`` + ``mjFONTSCALE_150``，与 ``status_text()`` 同一条路径）：
``'A'`` 70 个笔画像素、字形可辨，``'真'`` 是 12×10 全黑、``'开'`` 是 24×15 全黑外
加一条溢出横杠。

终端不受影响——终端有中文字体。所以规矩是「窗口 ASCII、终端中文」，两份都要有。

这条规则只有靠扫描才守得住：漏一个汉字在界面上是花屏，在测试里什么都不报。
"""
from __future__ import annotations

import ast
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"


def overlay_calls():
    """把所有样例里 ``*.status_text(...)`` 的调用点找出来。"""
    for path in sorted(EXAMPLES.glob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) \
                    and getattr(node.func, "attr", None) == "status_text":
                yield path, node


def test_the_scan_actually_finds_the_overlay_calls():
    """先证明扫描有效：找不到调用点就说明这个文件在空转（改个名字就悄悄失效）。"""
    found = list(overlay_calls())
    assert len(found) >= 5, f"只找到 {len(found)} 处 status_text 调用"


@pytest.mark.parametrize("path,node", list(overlay_calls()),
                         ids=[f"{p.name}:{n.lineno}" for p, n in overlay_calls()])
def test_overlay_text_is_ascii(path, node):
    """窗口里的每一段字符串都得是 ASCII。"""
    offenders = [
        arg.value for arg in ast.walk(node)
        if isinstance(arg, ast.Constant) and isinstance(arg.value, str)
        and not arg.value.isascii()
    ]
    assert not offenders, (
        f"{path.name}:{node.lineno} 往窗口里写了非 ASCII 文字 {offenders!r}；"
        "查看器字体没有中文字形，界面上会显示成实心方块。"
        "窗口用 ASCII，中文留给终端。")
