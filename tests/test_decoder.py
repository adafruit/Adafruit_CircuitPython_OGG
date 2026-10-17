# SPDX-FileCopyrightText: Copyright (c) 2026 Mikey Sklar for Adafruit Industries
# SPDX-License-Identifier: MIT
"""Desktop test: decode every file in media/ and compare with ffmpeg's decode.

The kernel path runs src/ogg_turbo.py under CPython (tests/viper_shim.py, which fails on any
32-bit overflow). The float path is the plain Python decoder. Both must stay within a few LSB
of ffmpeg (which decodes in float32 and rounds on its own), and the kernel must give exactly
as many frames as ffmpeg (end trimming).

usage: python tests/test_decoder.py   (after tools/make_test_media.py)
"""

import os
import sys
from array import array

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

import viper_shim  # noqa: E402

viper_shim.load()

import adafruit_ogg  # noqa: E402

MEDIA = os.path.join(HERE, "..", "media")
KERNEL_SECONDS = 15  # the CPython model of the kernel is slow; full length for short files
FLOAT_SECONDS = 4
FULL_UNDER = 1_000_000  # bytes of .ogg decoded to the end, to check the frame count
MAX_LSB = 4
MAX_RMS = 1.0  # RMS difference, in LSB


def decode(path, turbo, seconds):
    dec = adafruit_ogg.OggDecoder(path, turbo=turbo)
    assert dec.turbo == turbo
    n = dec.sample_rate * dec.channel_count * seconds if seconds else 1 << 26
    out = array("h")
    buf = array("h", [0]) * 4096
    while len(out) < n:
        got = dec.readinto(buf)
        out.extend(buf[:got])
        if got < len(buf):
            break
    return dec, out[:n]


def compare(out, ref):
    """Largest and RMS difference, in LSB."""
    k = min(len(out), len(ref))
    worst = 0
    total = 0
    for i in range(k):
        e = out[i] - ref[i]
        total += e * e
        worst = max(worst, abs(e))
    return worst, (total / k) ** 0.5 if k else 0.0


def main():
    """Returns the number of failures."""
    failures = 0
    names = sorted(n for n in os.listdir(MEDIA) if n.endswith(".ogg"))
    for name in names:
        path = os.path.join(MEDIA, name)
        ref = array("h", open(path[:-4] + ".raw", "rb").read())
        full = os.path.getsize(path) < FULL_UNDER
        dec, out = decode(path, True, 0 if full else KERNEL_SECONDS)
        worst, rms = compare(out, ref)
        ok = worst <= MAX_LSB and rms <= MAX_RMS
        ok = ok and (len(out) == len(ref) if full else len(out) <= len(ref))
        print(
            "%-28s %5d Hz %d ch  kernel max %d rms %.2f LSB, %d of %d values%s"
            % (
                name,
                dec.sample_rate,
                dec.channel_count,
                worst,
                rms,
                len(out),
                len(ref),
                "" if ok else "  FAIL",
            )
        )
        failures += not ok
        _, out = decode(path, False, FLOAT_SECONDS)
        worst, rms = compare(out, ref)
        ok = worst <= MAX_LSB and rms <= MAX_RMS
        print("%-28s float  max %d rms %.2f LSB%s" % ("", worst, rms, "" if ok else "  FAIL"))
        failures += not ok
    return failures


def test_decoder():
    """pytest entry (skipped when tools/make_test_media.py has not run)."""
    import pytest  # noqa: PLC0415

    if not os.path.isdir(MEDIA) or not any(n.endswith(".raw") for n in os.listdir(MEDIA)):
        pytest.skip("no test media; run tools/make_test_media.py")
    assert main() == 0


if __name__ == "__main__":
    f = main()
    print("FAIL" if f else "PASS")
    sys.exit(1 if f else 0)
