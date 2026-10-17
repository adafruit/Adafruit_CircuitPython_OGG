# SPDX-FileCopyrightText: Copyright (c) 2026 Mikey Sklar for Adafruit Industries
# SPDX-License-Identifier: MIT
"""Desktop test of codebook setup and codeword decode in the turbo kernel. Needs no media.

Runs src/ogg_turbo.py under CPython (tests/viper_shim.py). Codeword assignment follows the
Vorbis I specification, section 3.2.1, whose worked example is checked first; random
codebooks are then encoded and decoded through the fast table and the long code search.
"""

import os
import random
import sys
from array import array

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import viper_shim  # noqa: E402

K = viper_shim.load()


def reference(lengths):
    """Section 3.2.1 by the book: MSB-first codeword strings for each used entry."""
    avail = [0] * 33
    out = {}
    first = True
    for e, length in enumerate(lengths):
        if not length:
            continue
        if first:
            first = False
            for i in range(1, length + 1):
                avail[i] = 1 << (32 - i)
            res = 0
        else:
            z = length
            while z > 0 and not avail[z]:
                z -= 1
            assert z, "overspecified"
            res = avail[z]
            avail[z] = 0
            for y in range(length, z, -1):
                avail[y] = res + (1 << (32 - y))
        out[e] = format(res >> (32 - length), "0%db" % length)
    return out


def tables(lengths, fb):
    n = len(lengths)
    ln = bytearray(lengths)
    nlong = K.book_count(ln, n, fb)
    ft = array("H", [0]) * ((1 << fb) + 1)
    lc = array("i", [0]) * (nlong + 1)
    lv = array("i", [0]) * (nlong + 1)
    got = K.book_tables(ln, n, fb, ft, 0, lc, lv, 0, array("i", [0]) * 33)
    assert got == nlong
    bk = array("i", [0]) * 12
    bk[1] = fb
    bk[3] = nlong
    return bk, ft, lc, lv


def decode_all(lengths, fb, entries):
    codes = reference(lengths)
    bits = "".join(codes[e] for e in entries)
    # the bitstream is LSB first within each byte; codewords are read MSB first
    data = bytearray((len(bits) + 7) // 8 + 8)
    for i, c in enumerate(bits):
        if c == "1":
            data[i >> 3] |= 1 << (i & 7)
    bk, ft, lc, lv = tables(lengths, fb)
    st = array("i", [0]) * 4
    pos = 0
    out = []
    for _ in entries:
        e = K._cw(data, pos, bk, ft, lc, lv, 0, st)
        pos = st[0]
        out.append(e)
    assert pos == len(bits)
    return out


def test_spec_example():
    # Vorbis I spec 3.2.1: lengths 2 4 4 4 4 2 3 3
    lengths = [2, 4, 4, 4, 4, 2, 3, 3]
    assert reference(lengths) == {
        0: "00",
        1: "0100",
        2: "0101",
        3: "0110",
        4: "0111",
        5: "10",
        6: "110",
        7: "111",
    }
    for fb in (0, 2, 3, 4):
        assert decode_all(lengths, fb, [0, 1, 2, 3, 4, 5, 6, 7, 7, 0]) == [
            0,
            1,
            2,
            3,
            4,
            5,
            6,
            7,
            7,
            0,
        ]


def random_lengths(rnd, n):
    # a complete prefix code from random splits (Kraft sum exactly 1), in random order
    lens = [0]
    while len(lens) < n:
        i = rnd.randrange(len(lens))
        if lens[i] >= 24:
            continue
        lens[i] += 1
        lens.insert(i, lens[i])
    rnd.shuffle(lens)
    return lens


def test_random_books():
    rnd = random.Random(7)
    for _ in range(40):
        n = rnd.choice((2, 3, 17, 81, 300, 1000))
        lengths = random_lengths(rnd, n)
        if rnd.random() < 0.3:  # sparse: some unused entries
            lengths = [x if rnd.random() < 0.8 else 0 for x in lengths]
            if sum(1 for x in lengths if x) < 2:
                continue
        used = [e for e, x in enumerate(lengths) if x]
        entries = [rnd.choice(used) for _ in range(200)]
        for fb in (0, 4, 8):
            assert decode_all(lengths, fb, entries) == entries


def test_single_entry():
    lengths = [0, 0, 1, 0]
    for fb in (0, 1, 8):
        bk, ft, lc, lv = tables(lengths, fb)
        st = array("i", [0]) * 4
        assert K._cw(bytes(16), 0, bk, ft, lc, lv, 0, st) == 2


if __name__ == "__main__":
    test_spec_example()
    test_random_books()
    test_single_entry()
    print("PASS")
