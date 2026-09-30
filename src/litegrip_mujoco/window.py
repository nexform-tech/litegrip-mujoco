"""Keyboard input for the MuJoCo viewer, and the key tables the examples use.

This module exists so that a program can ask "was Esc pressed?" without caring
whether a window is open at all. It imports nothing from MuJoCo at module level
and nothing from the rest of this package, which keeps it usable from a
``--help`` path, from a headless run, and from a test that has replaced
``mujoco.viewer`` with a stub.

What the viewer can and cannot give us
--------------------------------------

``mujoco.viewer.launch_passive`` accepts exactly one input callback::

    launch_passive(model, data, key_callback=...)

That callback receives the GLFW key code on **press**. There is no release
event, no auto-repeat, and no mouse callback. So:

* :func:`pressed` is exact.
* :func:`held` is an approximation -- see its docstring. Do not build a control
  loop that assumes it can tell "still down" from "tapped once".
* :func:`clicked` always returns ``False``. It exists so that example code can
  keep the shape ``clicked(events) or pressed(keys, CONFIRM_KEYS)`` without
  pretending the mouse works.

Key codes
---------

:data:`_KEY_LITERALS` holds the GLFW codes as plain integers. They are literals
rather than a module-level ``from mujoco.viewer import glfw`` on purpose: the
test suite swaps in a fake ``mujoco.viewer`` that has no ``glfw`` attribute, and
an eager import would send that test down a misleading error path.
:func:`key_codes` prefers the real GLFW table when it is importable and falls
back to the literals, and ``tests/test_sim_extras.py`` asserts the two agree so
the literals cannot drift unnoticed.

Threading
---------

The key callback runs on the viewer's own thread. :class:`KeyQueue` is therefore
the only thing the callback is allowed to touch: it appends an integer to a
``deque``, which is atomic under the GIL. Everything else -- converting codes to
events, deciding what a key means -- happens on the caller's thread inside
:meth:`MujocoGripper.keyboard_events`. Never call into MuJoCo from the callback;
the physics thread holds ``mjData`` and the viewer's copy of it is not yours.
"""
from __future__ import annotations

from collections import deque
from functools import lru_cache
from typing import Any, Deque, Dict, Iterable, Mapping, Sequence, Tuple

__all__ = [
    "CONFIRM_KEYS",
    "KEY_PRESSED",
    "QUIT_KEYS",
    "TELEOP_KEYS",
    "ZERO_GRAVITY_KEYS",
    "KeyQueue",
    "clicked",
    "held",
    "key_codes",
    "key_label",
    "pressed",
]

#: Event value used in the mapping returned by ``MujocoGripper.keyboard_events``.
#: Only presses are ever reported; there is no release event to report.
KEY_PRESSED: int = 1

#: GLFW key codes. Names match ``glfw.KEY_*`` so :func:`key_codes` can look them
#: up by attribute; the values match GLFW 3.x on every platform we run on.
_KEY_LITERALS: Mapping[str, int] = {
    "SPACE": 32,
    "APOSTROPHE": 39,
    "COMMA": 44,
    "MINUS": 45,
    "PERIOD": 46,
    "SLASH": 47,
    "0": 48,
    "1": 49,
    "2": 50,
    "3": 51,
    "4": 52,
    "5": 53,
    "6": 54,
    "7": 55,
    "8": 56,
    "9": 57,
    "SEMICOLON": 59,
    "EQUAL": 61,
    "A": 65,
    "C": 67,
    "D": 68,
    "F": 70,
    "H": 72,
    "O": 79,
    "Q": 81,
    "R": 82,
    "S": 83,
    "W": 87,
    "Z": 90,
    "LEFT_BRACKET": 91,
    "RIGHT_BRACKET": 93,
    "ESCAPE": 256,
    "ENTER": 257,
    "TAB": 258,
    "BACKSPACE": 259,
    "INSERT": 260,
    "DELETE": 261,
    "RIGHT": 262,
    "LEFT": 263,
    "DOWN": 264,
    "UP": 265,
    "PAGE_UP": 266,
    "PAGE_DOWN": 267,
    "HOME": 268,
    "END": 269,
    "KP_0": 320,
    "KP_1": 321,
    "KP_2": 322,
    "KP_3": 323,
    "KP_4": 324,
    "KP_5": 325,
    "KP_6": 326,
    "KP_7": 327,
    "KP_8": 328,
    "KP_9": 329,
    "KP_DECIMAL": 330,
    "KP_DIVIDE": 331,
    "KP_MULTIPLY": 332,
    "KP_SUBTRACT": 333,
    "KP_ADD": 334,
    "KP_ENTER": 335,
    "KP_EQUAL": 336,
    "LEFT_SHIFT": 340,
    "LEFT_CONTROL": 341,
    "LEFT_ALT": 342,
    "LEFT_SUPER": 343,
    "RIGHT_SHIFT": 344,
    "RIGHT_CONTROL": 345,
    "RIGHT_ALT": 346,
    "RIGHT_SUPER": 347,
}

#: Keys that mean "stop and leave". Esc is the viewer's own quit key as well, so
#: in a windowed run either one ends the loop.
QUIT_KEYS: Tuple[int, ...] = (
    _KEY_LITERALS["ESCAPE"],
    _KEY_LITERALS["Q"],
)

#: Keys that mean "yes, go ahead" -- used where an example waits for the
#: operator to acknowledge something before it moves hardware.
CONFIRM_KEYS: Tuple[int, ...] = (
    _KEY_LITERALS["ENTER"],
    _KEY_LITERALS["KP_ENTER"],
    _KEY_LITERALS["SPACE"],
)

#: Keys that toggle the real gripper's zero-gravity mode in example 04.
ZERO_GRAVITY_KEYS: Tuple[int, ...] = (_KEY_LITERALS["Z"],)

#: Teleoperation keys, used where a GUI slider would otherwise be. The viewer
#: offers no slider widget, so example 05 steps a value once per key press
#: instead of reading a continuously draggable control.
#:
#: ``LEFT``/``RIGHT``  target opening (one step per press)
#: ``DOWN``/``UP``     speed limit
#: ``MINUS``/``EQUAL`` grip force (plus the keypad equivalents)
#: ``SPACE``           stop, leaving the target where it is
#: ``H``               back to fully open
TELEOP_KEYS: Mapping[str, Tuple[int, ...]] = {
    "aperture_down": (_KEY_LITERALS["LEFT"],),
    "aperture_up": (_KEY_LITERALS["RIGHT"],),
    "slower": (_KEY_LITERALS["DOWN"],),
    "faster": (_KEY_LITERALS["UP"],),
    "force_down": (_KEY_LITERALS["MINUS"], _KEY_LITERALS["KP_SUBTRACT"]),
    "force_up": (_KEY_LITERALS["EQUAL"], _KEY_LITERALS["KP_ADD"]),
    "stop": (_KEY_LITERALS["SPACE"],),
    "home": (_KEY_LITERALS["H"],),
}

#: Human-readable name for a code, for the hint lines examples print. Falls back
#: to ``key 0x1F`` rather than raising: an unknown key is not worth a traceback.
_LABELS: Mapping[int, str] = {
    _KEY_LITERALS["ESCAPE"]: "Esc",
    _KEY_LITERALS["ENTER"]: "Enter",
    _KEY_LITERALS["KP_ENTER"]: "KP_Enter",
    _KEY_LITERALS["SPACE"]: "Space",
    _KEY_LITERALS["LEFT"]: "←",
    _KEY_LITERALS["RIGHT"]: "→",
    _KEY_LITERALS["UP"]: "↑",
    _KEY_LITERALS["DOWN"]: "↓",
    _KEY_LITERALS["MINUS"]: "-",
    _KEY_LITERALS["EQUAL"]: "=",
    _KEY_LITERALS["KP_SUBTRACT"]: "KP_-",
    _KEY_LITERALS["KP_ADD"]: "KP_+",
    _KEY_LITERALS["Z"]: "Z",
    _KEY_LITERALS["H"]: "H",
}


@lru_cache(maxsize=1)
def key_codes() -> Mapping[str, int]:
    """The GLFW key table, resolved once.

    Prefers the real ``glfw`` module that ``mujoco.viewer`` re-exports, so the
    codes are whatever GLFW actually reports. Falls back to
    :data:`_KEY_LITERALS` when GLFW is not importable -- a headless machine
    without the viewer, or a test that has stubbed ``mujoco.viewer``.

    The import is inside the function on purpose. ``mujoco.viewer`` is replaced
    with a ``glfw``-less stub by ``tests/test_mujoco_gripper.py``, and a
    module-level import would break that test.
    """
    try:
        from mujoco.viewer import glfw  # type: ignore[attr-defined]
    except Exception:
        return dict(_KEY_LITERALS)
    return {
        name: int(getattr(glfw, f"KEY_{name}", value))
        for name, value in _KEY_LITERALS.items()
    }


def key_label(code: int) -> str:
    """Readable name for a key code, e.g. ``"Esc"`` or ``"\\u2190"``."""
    return _LABELS.get(int(code), f"key 0x{int(code):X}")


def pressed(events: Mapping[int, int], keys: Sequence[int]) -> bool:
    """True if any of ``keys`` was pressed in this batch of ``events``.

    Same name and meaning as the PyBullet examples' ``pressed()``, so example
    code reads the same on both sides.
    """
    return any(events.get(int(k), 0) & KEY_PRESSED for k in keys)


def held(events: Mapping[int, int], keys: Sequence[int]) -> bool:
    """Approximation of "still held down". **Identical to** :func:`pressed`.

    ``launch_passive``'s ``key_callback`` fires once per press and never reports
    a release, so there is no way to tell a tap from a hold. This function exists
    so callers can say what they mean, and so that the day MuJoCo exposes a
    release event there is one place to change. Until then, a control loop built
    on it will step once per press -- which is exactly what
    :data:`TELEOP_KEYS` is designed around.
    """
    return pressed(events, keys)


def clicked(events: Iterable[Any]) -> bool:
    """Always ``False``. MuJoCo's passive viewer reports no mouse events.

    ``launch_passive`` takes a keyboard callback and nothing else; the mouse is
    consumed inside the viewer for camera control and perturbation dragging.
    This function is a documented stub so that example code can keep the shape
    ``clicked(events) or pressed(events, CONFIRM_KEYS)`` -- the truth always
    comes from the keyboard branch, and nothing deadlocks waiting for a click
    that can never arrive.
    """
    return False


class KeyQueue:
    """Thread-safe ledger of key presses, fed by the viewer's callback.

    The callback thread only ever calls :meth:`feed`, which appends to a
    ``deque`` -- an operation that is atomic under the GIL and touches no MuJoCo
    state. The control loop calls :meth:`drain` on its own thread.

    Codes are not deduplicated: two presses of the same key between two drains
    are two events, which is what a stepping teleoperation loop wants.
    """

    def __init__(self) -> None:
        self._codes: Deque[int] = deque()

    def feed(self, code: int) -> None:
        """Record one key press. Called from the viewer's thread."""
        self._codes.append(int(code))

    def drain(self) -> Dict[int, int]:
        """Take and clear everything recorded since the last call.

        Returns a mapping of key code to event value, which is the shape
        :func:`pressed` expects. A code that arrived several times appears once:
        the mapping cannot express multiplicity, and callers that need the count
        use :meth:`count`.
        """
        events: Dict[int, int] = {}
        while self._codes:
            events[self._codes.popleft()] = KEY_PRESSED
        return events

    def peek(self) -> Dict[int, int]:
        """Like :meth:`drain` but leaves the queue intact."""
        return {code: KEY_PRESSED for code in self._codes}

    def count(self, code: int) -> int:
        """How many times ``code`` was pressed since the last drain."""
        return sum(1 for c in self._codes if c == int(code))

    def clear(self) -> None:
        """Discard everything recorded so far."""
        self._codes.clear()

    def __len__(self) -> int:
        return len(self._codes)
