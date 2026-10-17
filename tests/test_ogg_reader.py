# SPDX-FileCopyrightText: Copyright (c) 2026 Mikey Sklar for Adafruit Industries
# SPDX-License-Identifier: MIT
"""Desktop test of the Ogg page reader (RFC 3533) on synthetic streams. Needs no media.

Packets of awkward sizes (0, 255, 510, ones spanning several pages) must come back intact,
pages of another logical stream must be skipped, and the last packet must carry the end of
stream flag and its page's granule position.
"""

import io
import os
import random
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from adafruit_ogg import _Ogg  # noqa: E402, PLC2701


def pages(serial, packets, granules, max_segs=255, first_flags=2):
    """Ogg pages for packets (bytes). Packet i ends with granule granules[i]."""
    out = bytearray()
    segs = []  # (lacing value, data, packet index ending here or -1)
    for i, p in enumerate(packets):
        n = len(p)
        k = 0
        while n - k >= 255:
            segs.append((255, p[k : k + 255], -1))
            k += 255
        segs.append((n - k, p[k:], i))
    seq = 0
    cont = False
    while segs:
        take = segs[:max_segs]
        segs = segs[max_segs:]
        last_end = [s[2] for s in take if s[2] >= 0]
        granule = granules[last_end[-1]] if last_end else -1
        flags = (first_flags if seq == 0 else 0) | (1 if cont else 0) | (0 if segs else 4)
        lacing = bytes(s[0] for s in take)
        body = b"".join(s[1] for s in take)
        out += b"OggS" + struct.pack("<BBqIII", 0, flags, granule, serial, seq, 0)
        out += bytes([len(take)]) + lacing + body
        cont = take[-1][2] < 0
        seq += 1
    return out


def read_all(data):
    ogg = _Ogg(io.BytesIO(bytes(data)))
    got = []
    while True:
        n = ogg.packet()
        if n < 0:
            return got, ogg
        assert ogg.pkt[n : n + 8] == bytes(8)  # guard bytes for the bit reader
        got.append((bytes(ogg.pkt[:n]), ogg.pkt_granule, ogg.pkt_eos))


def test_packets():
    rnd = random.Random(1)
    sizes = [30, 0, 255, 510, 1, 254, 256, 9000, 70000, 3, 4095, 4096, 4097]
    packets = [b"\x01vorbis" + bytes(23)] + [rnd.randbytes(n) for n in sizes]
    granules = list(range(0, 1000 * len(packets), 1000))
    for max_segs in (255, 7, 1):
        got, _ = read_all(pages(77, packets, granules, max_segs=max_segs))
        assert [g[0] for g in got] == packets, max_segs
        assert got[-1][2] and not any(g[2] for g in got[:-1])
        assert got[-1][1] == granules[-1]


def test_other_stream_skipped():
    vorbis = [b"\x01vorbis" + bytes(23), b"abc" * 100, b"xyz"]
    other = [b"\x80theora" + bytes(40), b"q" * 600]
    a = pages(1, vorbis, [0, 0, 10])
    b = pages(2, other, [0, 0])
    # interleave: other stream's first page before ours, then the rest
    data = b + a
    got, _ = read_all(data)
    assert [g[0] for g in got] == vorbis


def test_truncated():
    packets = [b"\x01vorbis" + bytes(23), bytes(5000)]
    data = pages(5, packets, [0, 100])
    got, _ = read_all(data[:-100])
    assert [g[0] for g in got] == packets[:1]


if __name__ == "__main__":
    test_packets()
    test_other_stream_skipped()
    test_truncated()
    print("PASS")
