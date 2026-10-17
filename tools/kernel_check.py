# SPDX-FileCopyrightText: Copyright (c) 2026 Mikey Sklar for Adafruit Industries
# SPDX-License-Identifier: MIT
"""Check the kernels on a board: the C and turbo kernels agree, and how fast they decode.

For every .ogg in /ogg: when both ogg_native (C) and ogg_kernel (turbo) load, decodes the
first seconds with each and reports the largest difference; then times a decode with the
kernel the library picks. load% = decode time / audio time, under 100% plays in real time.
"""

import gc
import os
import time
from array import array

import adafruit_ogg

DIR = "/ogg"
CHECK_SECONDS = 3
TIME_SECONDS = 10

try:
    from adafruit_ogg import ogg_native
except (ImportError, ValueError):
    ogg_native = None
try:
    from adafruit_ogg import ogg_kernel
except (ImportError, ValueError):
    ogg_kernel = None
default = adafruit_ogg._kernel
print("kernel:", "C (ogg_native)" if default is ogg_native else "turbo (ogg_kernel)")
buf_a = array("h", [0]) * 2048
buf_b = array("h", [0]) * 2048


def compare(path):
    adafruit_ogg._kernel = ogg_native
    a = adafruit_ogg.OggDecoder(path)
    adafruit_ogg._kernel = ogg_kernel
    b = adafruit_ogg.OggDecoder(path)
    adafruit_ogg._kernel = default
    worst = 0
    left = a.sample_rate * a.channel_count * CHECK_SECONDS
    while left > 0:
        na = a.readinto(buf_a)
        nb = b.readinto(buf_b)
        if na != nb:
            return -1
        for i in range(na):
            d = buf_a[i] - buf_b[i]
            worst = max(worst, d if d > 0 else -d)
        if na < len(buf_a):
            break
        left -= na
    a.deinit()
    b.deinit()
    return worst


def speed(path):
    gc.collect()
    t = time.monotonic_ns()
    d = adafruit_ogg.OggDecoder(path)
    setup = (time.monotonic_ns() - t) // 1_000_000
    want = d.sample_rate * TIME_SECONDS
    frames = 0
    t = time.monotonic_ns()
    while frames < want:
        n = d.readinto(buf_a)
        frames += n // d.channel_count
        if n < len(buf_a):
            break
    dt = time.monotonic_ns() - t
    tb = d._turbo
    sram = default.address(tb.v) >= 0x20000000
    print(
        "%-34s %5d Hz %d ch  load %5.1f%%  setup %4d ms  buffers in %s"
        % (
            path,
            d.sample_rate,
            d.channel_count,
            dt / 1e7 / (frames / d.sample_rate),
            setup,
            "SRAM" if sram else "PSRAM",
        )
    )
    d.deinit()


for name in sorted(os.listdir(DIR)):
    if not name.endswith(".ogg") or name.startswith("."):
        continue
    path = DIR + "/" + name
    if ogg_native is not None and ogg_kernel is not None:
        print("%-34s C vs turbo: max difference %d LSB" % (path, compare(path)))
    speed(path)
print("done")
