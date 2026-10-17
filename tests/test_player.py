# SPDX-FileCopyrightText: Copyright (c) 2026 Mikey Sklar for Adafruit Industries
# SPDX-License-Identifier: MIT
"""Desktop test of OggPlayer buffer timing against a simulated output.

The fake output copies RawSample halves the way the rp2 audio DMA does: both
halves at play(), then one half every half-buffer period. A simulated clock
advances by a fixed cost per update() call, standing in for CPU time.

usage: python tests/test_player.py
"""

import os
import sys
import types
from array import array

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

import viper_shim  # noqa: E402

viper_shim.load()

# fake audiocore.RawSample that just keeps its arguments
audiocore = types.ModuleType("audiocore")


class RawSample:
    def __init__(self, buf, *, channel_count, sample_rate, single_buffer):
        assert not single_buffer
        self.buf = buf
        self.channel_count = channel_count
        self.sample_rate = sample_rate


audiocore.RawSample = RawSample
sys.modules["audiocore"] = audiocore

import adafruit_ogg  # noqa: E402

CLOCK = [0]
adafruit_ogg._now = lambda: CLOCK[0]


class FakeOut:
    def __init__(self):
        self.out = []
        self.sample = None

    def play(self, sample, loop=False):
        assert loop
        self.sample = sample
        self.t0 = CLOCK[0]
        half = len(sample.buf) // 2
        self.half = half
        self.hd = half // sample.channel_count * 1_000_000_000 // sample.sample_rate
        self.fetched = 0
        self._fetch()
        self._fetch()

    def _fetch(self):
        k = self.fetched
        start = (k & 1) * self.half
        self.out.extend(self.sample.buf[start : start + self.half])
        self.fetched += 1

    def tick(self):
        if self.sample is None:
            return
        while CLOCK[0] >= self.t0 + (self.fetched - 1) * self.hd:
            self._fetch()

    def stop(self):
        self.sample = None


def reference(path):
    dec = adafruit_ogg.OggDecoder(path)
    buf = array("h", bytes(2 * dec.sample_rate * dec.channel_count * 600))
    n = dec.readinto(buf)
    return list(buf[:n])


def run(path, cost_us):
    ref = reference(path)
    out = FakeOut()
    player = adafruit_ogg.OggPlayer(out, buffer_ms=200)
    CLOCK[0] = 0
    player.play(adafruit_ogg.OggDecoder(path))
    while player.update():
        CLOCK[0] += cost_us * 1000
        out.tick()
    # halves that reached the speaker: up to the silent one after the last
    # real half (the output copies one more ahead, which stop() discards)
    played = out.out[: (player._last_real + 2) * out.half]
    # the real stream must appear in order, then only silence
    ok = played[: len(ref)] == ref and not any(played[len(ref) :])
    return ok, player.underruns, len(played) - len(ref)


MEDIA = os.path.join(HERE, "..", "media")


def main():
    """Run the player checks; returns the number of failures."""
    failures = 0
    media = MEDIA
    for name in ("bach_8k_mono_q0.ogg", "mendelssohn_11k_mono.ogg"):
        path = os.path.join(media, name)
        ok, under, tail = run(path, 2000)  # 2 ms per update: keeps up
        print(f"{name} fast: exact={ok} underruns={under} silent tail={tail}")
        if not ok or under:
            failures += 1
        ok, under, _ = run(path, 300000)  # 300 ms per update: cannot keep up
        print(f"{name} slow: underruns={under} (expected > 0)")
        if not under:
            failures += 1
    # loop=True must repeat the file back to back with no gap
    path = os.path.join(media, "bach_8k_mono_q0.ogg")
    ref = reference(path)
    out = FakeOut()
    player = adafruit_ogg.OggPlayer(out, buffer_ms=200)
    CLOCK[0] = 0
    player.play(adafruit_ogg.OggDecoder(path), loop=True)
    while len(out.out) < 3 * len(ref) + out.half:
        player.update()
        CLOCK[0] += 2_000_000
        out.tick()
    player.stop()
    ok = out.out[: 3 * len(ref)] == ref * 3
    print(f"loop x3 gapless: {ok} underruns={player.underruns}")
    if not ok or player.underruns:
        failures += 1

    return failures


def test_player():
    """pytest entry (skipped when tools/make_test_media.py has not run)."""
    import pytest  # noqa: PLC0415

    if not os.path.exists(os.path.join(MEDIA, "bach_8k_mono_q0.ogg")):
        pytest.skip("no test media; run tools/make_test_media.py")
    assert main() == 0


if __name__ == "__main__":
    f = main()
    print("FAIL" if f else "PASS")
    sys.exit(1 if f else 0)
