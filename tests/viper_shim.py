# SPDX-FileCopyrightText: Copyright (c) 2026 Mikey Sklar for Adafruit Industries
# SPDX-License-Identifier: MIT
"""Run src/ogg_turbo.py under CPython: viper pointers become checked views and `uint` a
32-bit mask, so a store that leaves the 32-bit range a viper int has raises here."""

import pathlib
import sys
import types

_SRC = pathlib.Path(__file__).resolve().parent.parent / "src" / "ogg_turbo.py"


class _Ptr:
    def __init__(self, buf, lo, hi):
        self.b = buf
        self.lo = lo
        self.hi = hi

    def __getitem__(self, i):
        return self.b[i]

    def __setitem__(self, i, v):
        if not self.lo <= v <= self.hi:
            raise OverflowError("store of %d at %d" % (v, i))
        self.b[i] = v


def _ptr8(b):
    return _Ptr(b, 0, 255)


def _ptr16(b):
    return _Ptr(b, -32768, 65535)


def _ptr32(b):
    return _Ptr(b, -(1 << 31), (1 << 31) - 1)


def _uint(x):
    return x & 0xFFFFFFFF


def _int(x):
    # viper int() of a uint reinterprets the 32 bits as signed
    x &= 0xFFFFFFFF
    return x - (1 << 32) if x >= 1 << 31 else x


def load():
    """Import the kernel source as adafruit_ogg.ogg_kernel."""
    mp = types.ModuleType("micropython")
    mp.viper = lambda f: f
    mp.native = lambda f: f
    sys.modules["micropython"] = mp
    mod = types.ModuleType("adafruit_ogg.ogg_kernel")
    mod.__dict__.update(ptr8=_ptr8, ptr16=_ptr16, ptr32=_ptr32, uint=_uint, int=_int)
    exec(compile(_SRC.read_text(), str(_SRC), "exec"), mod.__dict__)  # noqa: S102
    sys.modules["adafruit_ogg.ogg_kernel"] = mod
    return mod
