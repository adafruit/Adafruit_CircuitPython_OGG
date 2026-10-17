# SPDX-FileCopyrightText: Copyright (c) 2026 Mikey Sklar for Adafruit Industries
# SPDX-License-Identifier: MIT
"""
`adafruit_ogg`
================================================================================

Ogg Vorbis decoding and playback for CircuitPython.

`OggDecoder` reads an Ogg Vorbis file and returns 16-bit signed PCM. `OggPlayer` streams it
to an audio output (``I2SOut``, ``PWMAudioOut`` or ``AudioOut``) through a double-buffered
``audiocore.RawSample``; call `OggPlayer.update` often from your main loop.

The audio decode (codebooks, floor, residue, channel coupling, the inverse MDCT and
overlap-add) runs in a prebuilt kernel: ``ogg_native.<arch>.mpy``, built in C from
``src/native/ogg_native.c``, on Arm chips, and otherwise ``ogg_kernel.<arch>.mpy``, built from
the turbo (viper) code in ``src/ogg_turbo.py`` with ``mpy-cross -march``. Both are fixed
point, so the output can differ from a floating point decoder by a few least significant bits.
Without a kernel for the board, a plain Python floating point decoder runs instead (correct,
but many times slower than real time).

Written from the Xiph.Org "Vorbis I specification" (section numbers in comments refer to
it). No third-party decoder code is included.

* Author(s): Mikey Sklar

Implementation Notes
--------------------

**Hardware:**

* Tested on the Adafruit Fruit Jam (RP2350B).

**Software and Dependencies:**

* Adafruit CircuitPython firmware 11 or later, with native ``.mpy`` loading:
  https://circuitpython.org/downloads
"""

import gc
import math
import struct
from array import array

try:
    from time import monotonic_ns as _now
except ImportError:  # unix port
    from time import time_ns as _now

# The audio decode kernel, prebuilt per chip: ogg_native.<arch>.mpy (C, src/native) on Arm
# chips, else ogg_kernel.<arch>.mpy (turbo, src/ogg_turbo.py). Same functions and tables.
# Without either (CPython, or a chip with no build) the plain Python decoder runs.
try:
    from adafruit_ogg import ogg_native as _kernel
except (ImportError, ValueError):
    try:
        from adafruit_ogg import ogg_kernel as _kernel
    except (ImportError, ValueError):
        _kernel = None

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_OGG.git"

# Plain Python decoder: codewords up to this many bits decode with one table lookup.
FAST_BITS = 10
# Turbo kernel: codewords up to this many bits decode with one 16-bit table lookup, longer
# ones by a binary search. 8 bits is 512 bytes of table per codebook.
_TURBO_FAST = 8

_PI = math.pi

# Section 10.1, floor1_inverse_dB_table.
# fmt: off
_INV_DB = (
    1.0649863e-07, 1.1341951e-07, 1.2079015e-07, 1.2863978e-07,
    1.3699951e-07, 1.4590251e-07, 1.5538408e-07, 1.6548181e-07,
    1.7623575e-07, 1.8768855e-07, 1.9988561e-07, 2.1287530e-07,
    2.2670913e-07, 2.4144197e-07, 2.5713223e-07, 2.7384213e-07,
    2.9163793e-07, 3.1059021e-07, 3.3077411e-07, 3.5226968e-07,
    3.7516214e-07, 3.9954229e-07, 4.2550680e-07, 4.5315863e-07,
    4.8260743e-07, 5.1396998e-07, 5.4737065e-07, 5.8294187e-07,
    6.2082472e-07, 6.6116941e-07, 7.0413592e-07, 7.4989464e-07,
    7.9862701e-07, 8.5052630e-07, 9.0579828e-07, 9.6466216e-07,
    1.0273513e-06, 1.0941144e-06, 1.1652161e-06, 1.2409384e-06,
    1.3215816e-06, 1.4074654e-06, 1.4989305e-06, 1.5963394e-06,
    1.7000785e-06, 1.8105592e-06, 1.9282195e-06, 2.0535261e-06,
    2.1869758e-06, 2.3290978e-06, 2.4804557e-06, 2.6416497e-06,
    2.8133190e-06, 2.9961443e-06, 3.1908506e-06, 3.3982101e-06,
    3.6190449e-06, 3.8542308e-06, 4.1047004e-06, 4.3714470e-06,
    4.6555282e-06, 4.9580707e-06, 5.2802740e-06, 5.6234160e-06,
    5.9888572e-06, 6.3780469e-06, 6.7925283e-06, 7.2339451e-06,
    7.7040476e-06, 8.2047000e-06, 8.7378876e-06, 9.3057248e-06,
    9.9104632e-06, 1.0554501e-05, 1.1240392e-05, 1.1970856e-05,
    1.2748789e-05, 1.3577278e-05, 1.4459606e-05, 1.5399272e-05,
    1.6400004e-05, 1.7465768e-05, 1.8600792e-05, 1.9809576e-05,
    2.1096914e-05, 2.2467911e-05, 2.3928002e-05, 2.5482978e-05,
    2.7139006e-05, 2.8902651e-05, 3.0780908e-05, 3.2781225e-05,
    3.4911534e-05, 3.7180282e-05, 3.9596466e-05, 4.2169667e-05,
    4.4910090e-05, 4.7828601e-05, 5.0936773e-05, 5.4246931e-05,
    5.7772202e-05, 6.1526565e-05, 6.5524908e-05, 6.9783085e-05,
    7.4317983e-05, 7.9147585e-05, 8.4291040e-05, 8.9768747e-05,
    9.5602426e-05, 0.0001018, 0.0001084, 0.0001154,
    0.0001229, 0.0001309, 0.0001394, 0.0001485,
    0.0001582, 0.0001684, 0.0001794, 0.0001910,
    0.0002035, 0.0002167, 0.0002308, 0.0002458,
    0.0002617, 0.0002788, 0.0002969, 0.0003162,
    0.0003367, 0.0003586, 0.0003819, 0.0004067,
    0.0004332, 0.0004613, 0.0004913, 0.0005232,
    0.0005573, 0.0005935, 0.0006320, 0.0006731,
    0.0007169, 0.0007635, 0.0008131, 0.0008659,
    0.0009222, 0.0009821, 0.0010459, 0.0011139,
    0.0011863, 0.0012634, 0.0013455, 0.0014330,
    0.0015261, 0.0016253, 0.0017309, 0.0018434,
    0.0019632, 0.0020908, 0.0022266, 0.0023713,
    0.0025254, 0.0026895, 0.0028643, 0.0030505,
    0.0032487, 0.0034598, 0.0036847, 0.0039241,
    0.0041792, 0.0044507, 0.0047400, 0.0050480,
    0.0053761, 0.0057254, 0.0060975, 0.0064938,
    0.0069158, 0.0073652, 0.0078438, 0.0083536,
    0.0088964, 0.0094746, 0.0100903, 0.0107460,
    0.0114444, 0.0121881, 0.0129801, 0.0138237,
    0.0147220, 0.0156787, 0.0166976, 0.0177827,
    0.0189384, 0.0201691, 0.0214798, 0.0228757,
    0.0243623, 0.0259455, 0.0276316, 0.0294272,
    0.0313396, 0.0333762, 0.0355452, 0.0378551,
    0.0403151, 0.0429351, 0.0457252, 0.0486967,
    0.0518613, 0.0552315, 0.0588208, 0.0626433,
    0.0667142, 0.0710497, 0.0756669, 0.0805842,
    0.0858210, 0.0913981, 0.0973377, 0.1036633,
    0.1103999, 0.1175743, 0.1252149, 0.1333521,
    0.1420181, 0.1512472, 0.1610761, 0.1715438,
    0.1826916, 0.1945640, 0.2072078, 0.2206734,
    0.2350140, 0.2502865, 0.2665515, 0.2838736,
    0.3023213, 0.3219678, 0.3428911, 0.3651741,
    0.3889052, 0.4141784, 0.4410941, 0.4697589,
    0.5002864, 0.5327979, 0.5674221, 0.6042964,
    0.6435669, 0.6853895, 0.7299300, 0.7773650,
    0.8278826, 0.8816830, 0.9389798, 1.0,
)
# fmt: on


class OggError(ValueError):
    """Raised for files that are not Ogg Vorbis or use unsupported features."""


def _ilog(x):
    # Section 9.2.1: bits needed to hold x (0 for x <= 0).
    n = 0
    while x > 0:
        n += 1
        x >>= 1
    return n


def _float32(x):
    # Section 9.2.2: Vorbis packed float.
    mant = x & 0x1FFFFF
    exp = (x >> 21) & 0x3FF
    if x & 0x80000000:
        mant = -mant
    return mant * math.pow(2.0, exp - 788)


def _lookup1_values(entries, dim):
    # Section 9.2.3: largest r with r ** dim <= entries.
    r = int(math.pow(entries, 1.0 / dim))
    while math.pow(r + 1, dim) <= entries:
        r += 1
    while r > 0 and math.pow(r, dim) > entries:
        r -= 1
    return r


class _Bits:
    """LSB-first bit reader over one packet (section 2.1.4)."""

    def __init__(self, data):
        self.data = data
        self.pos = 0
        self.end = len(data)
        self.acc = 0
        self.nb = 0
        self.pad = 0  # zero bits appended past the end of the packet
        self.eop = False

    def read(self, n):
        acc = self.acc
        nb = self.nb
        if nb < n:
            data = self.data
            pos = self.pos
            end = self.end
            while nb < n:
                if pos < end:
                    acc |= data[pos] << nb
                    pos += 1
                else:
                    self.pad += 8
                nb += 8
            self.pos = pos
        nb -= n
        if nb < self.pad:
            self.eop = True
        self.acc = acc >> n
        self.nb = nb
        return acc & ((1 << n) - 1)


class _Codebook:
    """Section 3: Huffman codebook plus optional VQ lookup."""

    def __init__(self, br, float_vq):
        if br.read(24) != 0x564342:
            raise OggError("bad codebook sync")
        self.dim = dim = br.read(16)
        self.entries = entries = br.read(24)
        lengths = bytearray(entries)
        if br.read(1):  # ordered
            cur = 0
            length = br.read(5) + 1
            while cur < entries:
                num = br.read(_ilog(entries - cur))
                for i in range(cur, cur + num):
                    lengths[i] = length
                cur += num
                length += 1
            if cur > entries:
                raise OggError("bad ordered codebook")
        else:
            sparse = br.read(1)
            for i in range(entries):
                if not sparse or br.read(1):
                    lengths[i] = br.read(5) + 1
        self.lengths = lengths
        if float_vq:
            self._build(lengths)
        else:
            # the turbo kernel builds its own tables from the lengths (_pack_books)
            self.maxlen = max(lengths) if entries else 0

        self.vq = None
        self.lookup = lookup = br.read(4)
        if lookup in (1, 2):
            minimum = _float32(br.read(32))
            delta = _float32(br.read(32))
            bits = br.read(4) + 1
            seq = br.read(1)
            if lookup == 1:
                nvals = _lookup1_values(entries, dim)
            else:
                nvals = entries * dim
            mults = [br.read(bits) for _ in range(nvals)]
            self.minimum = minimum
            self.delta = delta
            self.seq = seq
            self.nvals = nvals
            self.mults = mults
        if lookup in (1, 2) and float_vq:
            # flat array: entry e's vector is vq[e * dim : e * dim + dim]
            nvals = self.nvals
            minimum = self.minimum
            delta = self.delta
            seq = self.seq
            mults = self.mults
            vq = array("f", bytearray(4 * entries * dim))
            k = 0
            for e in range(entries):
                last = 0.0
                if lookup == 1:
                    div = 1
                    for _ in range(dim):
                        v = mults[(e // div) % nvals] * delta + minimum + last
                        if seq:
                            last = v
                        vq[k] = v
                        k += 1
                        div *= nvals
                else:
                    for _ in range(dim):
                        v = mults[k] * delta + minimum + last
                        if seq:
                            last = v
                        vq[k] = v
                        k += 1
            self.vq = vq
        if lookup > 2:
            raise OggError("bad codebook lookup type")
        if br.eop:
            raise OggError("truncated codebook")

    def _build(self, lengths):
        # Section 3.2.1 codeword assignment, then a table indexed by the
        # next FAST_BITS stream bits. Entries pack (entry << 5) | length.
        used = [i for i in range(len(lengths)) if lengths[i]]
        maxlen = max(lengths) if used else 0
        self.maxlen = maxlen
        tbits = min(maxlen, FAST_BITS)
        self.tbits = tbits
        self.tmask = (1 << tbits) - 1
        self.table = table = [-1] * (1 << tbits)
        self.long = {}
        if not used:
            return
        if len(used) == 1:
            e = used[0]
            for i in range(len(table)):
                table[i] = (e << 5) | min(lengths[e], tbits)
            return
        avail = [0] * 33
        first = used[0]
        for i in range(1, lengths[first] + 1):
            avail[i] = 1 << (32 - i)
        self._add(first, 0, lengths[first])
        for e in used[1:]:
            length = lengths[e]
            z = length
            while z > 0 and not avail[z]:
                z -= 1
            if z == 0:
                raise OggError("overspecified codebook")
            res = avail[z]
            avail[z] = 0
            # MSB-first codeword -> LSB-first stream order
            v = res >> (32 - length)
            code = 0
            for _ in range(length):
                code = (code << 1) | (v & 1)
                v >>= 1
            self._add(e, code, length)
            for y in range(length, z, -1):
                avail[y] = res + (1 << (32 - y))

    def _add(self, entry, code, length):
        tbits = self.tbits
        if length <= tbits:
            packed = (entry << 5) | length
            table = self.table
            for i in range(code, 1 << tbits, 1 << length):
                table[i] = packed
        else:
            self.long[code | (1 << length)] = entry

    def decode(self, br):
        """Read one codeword; returns the entry number or -1 at end of packet."""
        tb = self.tbits
        acc = br.acc
        nb = br.nb
        if nb < tb:
            data = br.data
            pos = br.pos
            end = br.end
            while nb < tb:
                if pos < end:
                    acc |= data[pos] << nb
                    pos += 1
                else:
                    br.pad += 8
                nb += 8
            br.pos = pos
        e = self.table[acc & self.tmask]
        if e < 0:
            br.acc = acc
            br.nb = nb
            return self._decode_long(br)
        n = e & 31
        nb -= n
        br.acc = acc >> n
        br.nb = nb
        if nb < br.pad:
            br.eop = True
            return -1
        return e >> 5

    def _decode_long(self, br):
        long = self.long
        for n in range(self.tbits + 1, self.maxlen + 1):
            while br.nb < n:
                if br.pos < br.end:
                    br.acc |= br.data[br.pos] << br.nb
                    br.pos += 1
                else:
                    br.pad += 8
                br.nb += 8
            e = long.get((br.acc & ((1 << n) - 1)) | (1 << n))
            if e is not None:
                br.acc >>= n
                br.nb -= n
                if br.nb < br.pad:
                    break
                return e
        br.eop = True
        return -1


class _Floor1:
    """Section 7: floor type 1."""

    def __init__(self, br):
        parts = br.read(5)
        self.part_class = pc = [br.read(4) for _ in range(parts)]
        nclass = max(pc) + 1 if pc else 0
        self.cdim = []
        self.csub = []
        self.cmaster = []
        self.cbooks = []
        for _ in range(nclass):
            self.cdim.append(br.read(3) + 1)
            sub = br.read(2)
            self.csub.append(sub)
            self.cmaster.append(br.read(8) if sub else -1)
            self.cbooks.append([br.read(8) - 1 for _ in range(1 << sub)])
        self.mult = br.read(2) + 1
        rbits = br.read(4)
        xs = [0, 1 << rbits]
        for c in pc:
            for _ in range(self.cdim[c]):
                xs.append(br.read(rbits))
        self.xs = xs
        nx = len(xs)
        if len(set(xs)) != nx:
            raise OggError("duplicate floor1 X")
        self.range = (256, 128, 86, 64)[self.mult - 1]
        self.ybits = _ilog(self.range - 1)
        # Sections 9.2.4 and 9.2.5
        low = [0] * nx
        high = [0] * nx
        for i in range(2, nx):
            lo = -1
            hi = -1
            for j in range(i):
                if xs[j] < xs[i] and (lo < 0 or xs[j] > xs[lo]):
                    lo = j
                if xs[j] > xs[i] and (hi < 0 or xs[j] < xs[hi]):
                    hi = j
            low[i] = lo
            high[i] = hi
        self.low = low
        self.high = high
        self.order = sorted(range(nx), key=lambda k: xs[k])

    def decode(self, br, books):
        """Section 7.2.3. Returns the Y list, or None if the floor is unused."""
        if not br.read(1):
            return None
        ybits = self.ybits
        ys = [br.read(ybits), br.read(ybits)]
        cdim = self.cdim
        csub = self.csub
        cmaster = self.cmaster
        cbooks = self.cbooks
        for c in self.part_class:
            bits = csub[c]
            cval = 0
            if bits:
                cval = books[cmaster[c]].decode(br)
            mask = (1 << bits) - 1
            sub = cbooks[c]
            for _ in range(cdim[c]):
                book = sub[cval & mask]
                cval >>= bits
                ys.append(books[book].decode(br) if book >= 0 else 0)
        if br.eop:
            return None
        return ys

    def render(self, ys, n2):
        """Section 7.2.4: returns n2 linear floor values."""
        xs = self.xs
        low = self.low
        high = self.high
        rng = self.range
        nx = len(xs)
        fy = [0] * nx
        step2 = [False] * nx
        fy[0] = ys[0]
        fy[1] = ys[1]
        step2[0] = step2[1] = True
        for i in range(2, nx):
            lo = low[i]
            hi = high[i]
            # render_point, section 9.2.6
            x0 = xs[lo]
            y0 = fy[lo]
            dy = fy[hi] - y0
            adx = xs[hi] - x0
            off = (abs(dy) * (xs[i] - x0)) // adx
            pred = y0 - off if dy < 0 else y0 + off
            val = ys[i]
            hroom = rng - pred
            lroom = pred
            room = (hroom if hroom < lroom else lroom) * 2
            if val:
                step2[lo] = step2[hi] = step2[i] = True
                if val >= room:
                    if hroom > lroom:
                        v = val - lroom + pred
                    else:
                        v = pred - val + hroom - 1
                elif val & 1:
                    v = pred - ((val + 1) >> 1)
                else:
                    v = pred + (val >> 1)
                if v < 0:
                    v = 0
                elif v >= rng:
                    v = rng - 1
                fy[i] = v
            else:
                fy[i] = pred

        out = [0.0] * n2
        mult = self.mult
        order = self.order
        lx = 0
        ly = fy[order[0]] * mult
        hx = 0
        hy = ly
        for k in order[1:]:
            if step2[k]:
                hx = xs[k]
                hy = fy[k] * mult
                _render_line(lx, ly, hx, hy, out, n2)
                lx = hx
                ly = hy
        if hx < n2:
            _render_line(hx, hy, n2, hy, out, n2)
        return out


def _render_line(x0, y0, x1, y1, out, n2):
    # Section 9.2.7, writing inverse-dB values directly.
    if x0 >= n2:
        return
    tab = _INV_DB
    dy = y1 - y0
    adx = x1 - x0
    ady = dy if dy >= 0 else -dy
    base = ady // adx
    if dy < 0:
        base = -base
        sy = base - 1
    else:
        sy = base + 1
    ady -= (base if base >= 0 else -base) * adx
    y = y0
    err = 0
    out[x0] = tab[y]
    x1 = min(x1, n2)
    for x in range(x0 + 1, x1):
        err += ady
        if err >= adx:
            err -= adx
            y += sy
        else:
            y += base
        out[x] = tab[y]


class _Residue:
    """Section 8: residue types 0, 1 and 2."""

    def __init__(self, br, rtype):
        self.type = rtype
        self.begin = br.read(24)
        self.end = br.read(24)
        self.psize = br.read(24) + 1
        self.nclass = br.read(6) + 1
        self.classbook = br.read(8)
        cascade = []
        for _ in range(self.nclass):
            lo = br.read(3)
            hi = br.read(5) if br.read(1) else 0
            cascade.append(hi * 8 + lo)
        self.books = [[br.read(8) if c & (1 << j) else -1 for j in range(8)] for c in cascade]

    def decode(self, br, books, vecs, dnd, n2):
        ch = len(vecs)
        if self.type == 2:
            if all(dnd):
                return
            big = [0.0] * (n2 * ch)
            self._decode(br, books, [big], [False], n2 * ch)
            if ch == 1:
                vecs[0][:] = big
            else:
                for c in range(ch):
                    vecs[c][:] = big[c::ch]
        else:
            self._decode(br, books, vecs, dnd, n2)

    def _decode(self, br, books, vecs, dnd, size):
        # Section 8.6.2
        begin = self.begin if self.begin < size else size
        end = self.end if self.end < size else size
        psize = self.psize
        nparts = (end - begin) // psize
        if nparts <= 0:
            return
        cbook = books[self.classbook]
        cpw = cbook.dim
        nclass = self.nclass
        rbooks = self.books
        fmt0 = self.type == 0
        ch = len(vecs)
        classes = [[0] * (nparts + cpw) for _ in range(ch)]
        for p in range(8):
            part = 0
            while part < nparts:
                if p == 0:
                    for c in range(ch):
                        if dnd[c]:
                            continue
                        t = cbook.decode(br)
                        if t < 0:
                            return
                        cl = classes[c]
                        for i in range(cpw - 1, -1, -1):
                            cl[part + i] = t % nclass
                            t //= nclass
                for _ in range(cpw):
                    if part >= nparts:
                        break
                    for c in range(ch):
                        if dnd[c]:
                            continue
                        bk = rbooks[classes[c][part]][p]
                        if bk < 0:
                            continue
                        book = books[bk]
                        vq = book.vq
                        if vq is None:
                            raise OggError("residue book has no VQ")
                        v = vecs[c]
                        off = begin + part * psize
                        dim = book.dim
                        if fmt0:
                            step = psize // dim
                            for i in range(step):
                                e = book.decode(br)
                                if e < 0:
                                    return
                                k = off + i
                                e *= dim
                                for j in range(e, e + dim):
                                    v[k] += vq[j]
                                    k += step
                        elif not _part1(br, book, v, off, off + psize):
                            return
                    part += 1


def _part1(br, book, v, k, stop):
    # Residue format 1 partition (8.6.4) with the codeword lookup inlined:
    # this loop is where most of the decode time goes. False at end of packet.
    table = book.table
    mask = book.tmask
    tb = book.tbits
    vq = book.vq
    dim = book.dim
    data = br.data
    pos = br.pos
    end = br.end
    acc = br.acc
    nb = br.nb
    pad = br.pad
    while k < stop:
        while nb < tb:
            if pos < end:
                acc |= data[pos] << nb
                pos += 1
            else:
                pad += 8
            nb += 8
        e = table[acc & mask]
        if e < 0:
            br.acc = acc
            br.nb = nb
            br.pos = pos
            br.pad = pad
            e = book._decode_long(br)
            if e < 0:
                return False
            acc = br.acc
            nb = br.nb
            pos = br.pos
        else:
            n = e & 31
            acc >>= n
            nb -= n
            if nb < pad:
                br.eop = True
                br.pad = pad
                return False
            e >>= 5
        e *= dim
        if dim == 2:
            v[k] += vq[e]
            v[k + 1] += vq[e + 1]
            k += 2
        else:
            for j in range(e, e + dim):
                v[k] += vq[j]
                k += 1
    br.acc = acc
    br.nb = nb
    br.pos = pos
    br.pad = pad
    return True


class _Mapping:
    """Section 4.2.4: mapping type 0."""

    def __init__(self, br, channels):
        self.submaps = br.read(4) + 1 if br.read(1) else 1
        self.coupling = []
        if br.read(1):
            bits = _ilog(channels - 1)
            for _ in range(br.read(8) + 1):
                m = br.read(bits)
                a = br.read(bits)
                if m == a or m >= channels or a >= channels:
                    raise OggError("bad channel coupling")
                self.coupling.append((m, a))
        if br.read(2):
            raise OggError("bad mapping")
        if self.submaps > 1:
            self.mux = [br.read(4) for _ in range(channels)]
        else:
            self.mux = [0] * channels
        self.floor = []
        self.residue = []
        for _ in range(self.submaps):
            br.read(8)
            self.floor.append(br.read(8))
            self.residue.append(br.read(8))


class _Ogg:
    """Ogg page reader (RFC 3533) that assembles Vorbis packets in one reusable buffer.

    `packet` returns the length of the next packet in ``pkt`` (followed by 8 zero bytes for
    the bit reader), or -1 at the end of the stream. Packet data is read from the file
    straight into ``pkt``."""

    def __init__(self, f):
        self.hdr = bytearray(27)
        self.lacing = bytearray(255)
        self.lacing_mv = memoryview(self.lacing)
        self.pkt = bytearray(4096)
        self.pkt_mv = memoryview(self.pkt)
        self.reset(f)

    def reset(self, f):
        """Start reading ``f`` from its beginning."""
        f.seek(0)
        self.f = f
        self.serial = None
        self.nseg = 0
        self.li = 0
        self.flags = 0
        self.granule = -1
        self.pkt_granule = -1
        self.pkt_eos = False

    def _page(self):
        # Read the next page header of our stream; its body is left in the file.
        f = self.f
        hdr = self.hdr
        lac = self.lacing
        while True:
            if f.readinto(hdr) != 27:
                return False
            if hdr[0] != 0x4F or hdr[1] != 0x67 or hdr[2] != 0x67 or hdr[3] != 0x53:
                raise OggError("lost Ogg sync")
            flags = hdr[5]
            serial = struct.unpack_from("<I", hdr, 14)[0]
            nseg = hdr[26]
            if nseg and f.readinto(self.lacing_mv[:nseg]) != nseg:
                return False
            if self.serial is None and flags & 2:
                head = f.read(7)
                f.seek(-len(head), 1)
                if head == b"\x01vorbis":
                    self.serial = serial
            if serial != self.serial:
                size = 0
                for i in range(nseg):
                    size += lac[i]
                f.seek(size, 1)
                continue
            self.nseg = nseg
            self.li = 0
            self.flags = flags
            self.granule = struct.unpack_from("<q", hdr, 6)[0]
            return True

    def packet(self):
        """Assemble the next packet into ``pkt``; returns its length or -1."""
        n = 0
        started = False
        f = self.f
        while True:
            if self.li >= self.nseg:
                if self.flags & 4 or not self._page():
                    return -1
                if not started and self.flags & 1:
                    # continuation of a packet we never saw the start of: skip it
                    lac = self.lacing
                    skip = 0
                    while self.li < self.nseg:
                        s = lac[self.li]
                        self.li += 1
                        skip += s
                        if s < 255:
                            break
                    f.seek(skip, 1)
                    continue
            lac = self.lacing
            li = self.li
            nseg = self.nseg
            size = 0
            done = False
            while li < nseg:
                s = lac[li]
                li += 1
                size += s
                if s < 255:
                    done = True
                    break
            self.li = li
            if n + size + 8 > len(self.pkt):
                grown = bytearray(2 * (n + size + 8))
                grown[:n] = self.pkt[:n]
                self.pkt = grown
                self.pkt_mv = memoryview(grown)
            if size and f.readinto(self.pkt_mv[n : n + size]) != size:
                return -1
            n += size
            started = True
            if done:
                last = li == nseg
                self.pkt_granule = self.granule if last else -1
                self.pkt_eos = bool(last and self.flags & 4)
                pkt = self.pkt
                for i in range(n, n + 8):
                    pkt[i] = 0
                return n


def _bits(d, pos, n):
    # n bits at bit position pos, LSB first (only used for the few packet header bits)
    v = 0
    for k in range(n):
        p = pos + k
        v |= ((d[p >> 3] >> (p & 7)) & 1) << k
    return v


def _rev(v, n):
    r = 0
    for _ in range(n):
        r = (r << 1) | (v & 1)
        v >>= 1
    return r


def _as_i32(u):
    return u - (1 << 32) if u >= 1 << 31 else u


def _split30(x):
    # Q30 value split into high and low 15-bit halves for the kernel's 32-bit multiplies
    v = int(round(x * (1 << 30)))
    return v >> 15, v & 0x7FFF


def _slope30(n):
    # rising half of the Vorbis power-sine window (4.3.1), as Q30 split pairs
    hp = _PI / 2
    out = array("i", [0]) * (2 * n)
    for i in range(n):
        h, lo = _split30(math.sin(hp * math.pow(math.sin((i + 0.5) / n * hp), 2)))
        out[2 * i] = h
        out[2 * i + 1] = lo
    return out


def _twiddles(n):
    # tables for kernel.imdct, see its docstring
    h = n >> 2
    m = n >> 1
    tw = array("i", [0]) * (11 * h)
    for j in range(h):
        a = -_PI * (4 * j + 1) / (4 * m)
        tw[4 * j], tw[4 * j + 1] = _split30(math.cos(a))
        tw[4 * j + 2], tw[4 * j + 3] = _split30(math.sin(a))
        a = -_PI * j / m
        t = 4 * h + 4 * j
        tw[t], tw[t + 1] = _split30(math.cos(a))
        tw[t + 2], tw[t + 3] = _split30(math.sin(a))
    for k in range(h >> 1):
        a = 2 * _PI * k / h
        t = 8 * h + 4 * k
        tw[t], tw[t + 1] = _split30(math.cos(a))
        tw[t + 2], tw[t + 3] = _split30(-math.sin(a))
    bits = _ilog(h) - 1
    for j in range(h):
        tw[10 * h + j] = _rev(j, bits)
    return tw


# Twiddle and window tables depend only on the block size: keep them for the next file.
_CACHE = {}


def _cached(make, n):
    key = (make, n)
    t = _CACHE.get(key)
    if t is None:
        t = _CACHE[key] = make(n)
    return t


def _arr(old, name, code, n):
    # The previous file's buffer when it is big enough: it is likely in internal RAM, where
    # a new one, allocated after the program has started, often isn't. New tables get some
    # headroom so the next file is likely to fit.
    a = getattr(old, name, None)
    if a is None or len(a) < n:
        a = array(code, [0]) * (n + (n >> 2) if name in _TABLES else n)
    return a


_TABLES = ("bk", "ft", "lc", "lv", "vq")


class _Turbo:
    """Flat tables and buffers for the kernel, built from the parsed headers. ``old`` is the
    previous file's, whose buffers are reused where they are big enough."""

    def __init__(self, dec, old=None):
        books = dec._books
        ch = dec._channels
        bs0, bs1 = dec._bs
        n2max = bs1 >> 1
        # Buffers first, while internal RAM is free: they are touched for every sample.
        self.v = _arr(old, "v", "i", ch * n2max)  # residue, then spectrum (Q24)
        self.prev = _arr(old, "prev", "i", ch * n2max)  # saved second halves (Q24)
        self.y = _arr(old, "y", "i", bs1)  # one channel's windowed IMDCT output
        self.re = _arr(old, "re", "i", bs1 >> 2)
        self.im = _arr(old, "im", "i", bs1 >> 2)
        self.pcm = _arr(old, "pcm", "h", ch * n2max)
        yb = getattr(old, "yb", [])
        self.yb = [
            yb[c] if c < len(yb) and len(yb[c]) >= n2max else bytearray(n2max) for c in range(ch)
        ]
        self.wk = _arr(old, "wk", "i", 768)
        self.st = _arr(old, "st", "i", 4)
        self.tw = (_cached(_twiddles, bs0), _cached(_twiddles, bs1))
        self.slope = (_cached(_slope30, bs0 >> 1), _cached(_slope30, bs1 >> 1))
        # window placement (4.3.1): (left start, length, right start, length) and slopes
        q0 = bs0 >> 2
        self.win_short = (array("i", (0, bs0 >> 1, bs0 >> 1, bs0 >> 1)), 0, 0)
        self.win_long = {}
        for prev in (0, 1):
            for nxt in (0, 1):
                if prev:
                    ls, ll, wl = 0, n2max, 1
                else:
                    ls, ll, wl = (bs1 >> 2) - q0, bs0 >> 1, 0
                if nxt:
                    rs, rl, wr = n2max, n2max, 1
                else:
                    rs, rl, wr = (bs1 * 3 >> 2) - q0, bs0 >> 1, 0
                self.win_long[(prev << 1) | nxt] = (array("i", (ls, ll, rs, rl)), wl, wr)
        self._pack_books(books, old)
        self.floors = [self._pack_floor(f) for f in dec._floors]
        self.residues = [self._pack_residue(r, books) for r in dec._residues]
        maxparts = 0
        for r in dec._residues:
            size = n2max * ch if r.type == 2 else n2max
            end = r.end if r.end < size else size
            begin = r.begin if r.begin < size else size
            parts = (end - begin) // r.psize + books[r.classbook].dim
            maxparts = max(maxparts, parts)
        cls = getattr(old, "cls", None)
        self.cls = (
            cls if cls is not None and len(cls) > ch * maxparts else bytearray(ch * maxparts + 1)
        )
        # floor curve: value for inverse-dB index y is tm[y] / 2**ts[y] (Q24 from the
        # residue scale 2**rq)
        rq = self.rq
        self.tm = array("i", [0]) * 256
        self.ts = array("i", [0]) * 256
        for y in range(256):
            m = _INV_DB[y]
            sh = 0
            while m < 16384:
                m *= 2
                sh += 1
            mi = int(m + 0.5)
            if mi >= 32768:
                mi >>= 1
                sh -= 1
            self.tm[y] = mi
            self.ts[y] = sh - 24 + rq

    def _pack_books(self, books, old):
        # all-integer VQ values (libvorbis files) stay integers, otherwise 1/16 steps
        rq = 0
        for b in books:
            if b.lookup and (b.minimum != int(b.minimum) or b.delta != int(b.delta)):
                rq = 4
        self.rq = rq
        scale = 1 << rq
        k = _kernel
        # sizes first, so the tables can be allocated in one go after the header garbage
        # is collected: the residue decode reads them for every codeword
        plans = []
        ft_len = long_len = vq_len = 0
        for b in books:
            fb = b.maxlen if b.maxlen < _TURBO_FAST else _TURBO_FAST
            if b.entries > 4095:
                fb = 0
            nlong = k.book_count(b.lengths, b.entries, fb)
            if b.lookup == 2 or (b.lookup == 1 and b.entries * b.dim <= 4096):
                lt, nv = 2, b.entries * b.dim
            elif b.lookup == 1:
                lt, nv = 1, b.nvals
            else:
                lt, nv = 0, 0
            plans.append((fb, nlong, lt, nv))
            ft_len += (1 << fb) if fb else 0
            long_len += nlong
            vq_len += nv
        gc.collect()
        self.bk = bk = _arr(old, "bk", "i", 12 * len(books))
        self.ft = ft = _arr(old, "ft", "H", ft_len + 1)  # book_tables fills its own range
        self.lc = lc = _arr(old, "lc", "i", long_len + 1)
        self.lv = lv = _arr(old, "lv", "i", long_len + 1)
        self.vq = vq = _arr(old, "vq", "i", vq_len + 1)
        av = array("i", [0]) * 33
        fo = lo = vo = 0
        for i, b in enumerate(books):
            fb, nlong, lt, nv = plans[i]
            if k.book_tables(b.lengths, b.entries, fb, ft, fo, lc, lv, lo, av) < 0:
                raise OggError("overspecified codebook")
            if lt:
                iv = array("i", [int(round((m * b.delta + b.minimum) * scale)) for m in b.mults])
                if lt == 2:
                    k.vq_fill(iv, b.nvals, b.entries, b.dim, b.seq, b.lookup == 1, vq, vo)
                else:
                    vq[vo : vo + nv] = iv
            h = 12 * i
            bk[h] = fo
            bk[h + 1] = fb
            bk[h + 2] = lo
            bk[h + 3] = nlong
            bk[h + 4] = b.dim
            bk[h + 5] = b.entries
            bk[h + 6] = lt
            bk[h + 7] = vo
            bk[h + 8] = nv
            bk[h + 9] = b.seq if b.lookup else 0
            fo += (1 << fb) if fb else 0
            lo += nlong
            vo += nv
            b.lengths = None  # no longer needed
            b.mults = None

    @staticmethod
    def _pack_floor(f):
        nx = len(f.xs)
        parts = len(f.part_class)
        nclass = len(f.cdim)
        opc = 16
        ocl = opc + parts
        oxs = ocl + 12 * nclass
        olo = oxs + nx
        ohi = olo + nx
        oord = ohi + nx
        fl = array("i", [0]) * (oord + nx)
        fl[0] = nx
        fl[1] = f.mult
        fl[2] = f.range
        fl[3] = f.ybits
        fl[4] = parts
        fl[5] = nclass
        fl[6] = opc
        fl[7] = ocl
        fl[8] = oxs
        fl[9] = olo
        fl[10] = ohi
        fl[11] = oord
        for p in range(parts):
            fl[opc + p] = f.part_class[p]
        for c in range(nclass):
            b = ocl + 12 * c
            fl[b] = f.cdim[c]
            fl[b + 1] = f.csub[c]
            fl[b + 2] = f.cmaster[c]
            for j, bk in enumerate(f.cbooks[c]):
                fl[b + 4 + j] = bk
        for i in range(nx):
            fl[oxs + i] = f.xs[i]
            fl[olo + i] = f.low[i]
            fl[ohi + i] = f.high[i]
            fl[oord + i] = f.order[i]
        return fl

    @staticmethod
    def _pack_residue(r, books):
        if books[r.classbook].dim < 1:
            raise OggError("bad residue classbook")
        rs = array("i", [0]) * (6 + 8 * r.nclass)
        rs[0] = r.type
        rs[1] = r.begin
        rs[2] = r.end
        rs[3] = r.psize
        rs[4] = r.nclass
        rs[5] = r.classbook
        for c in range(r.nclass):
            for p in range(8):
                bk = r.books[c][p]
                if bk >= 0 and not books[bk].lookup:
                    raise OggError("residue book has no VQ")
                rs[6 + 8 * c + p] = bk
        return rs

    def decode(self, dec, ogg, nbytes):
        """Decode one audio packet (4.3); returns the frames it completes, or -1."""
        k = _kernel
        d = ogg.pkt
        nbits = nbytes * 8
        if d[0] & 1:
            return -1
        mb = dec._mode_bits
        mi = _bits(d, 1, mb)
        pos = 1 + mb
        modes = dec._modes
        if mi >= len(modes) or pos > nbits:
            return -1
        blockflag, mapnum = modes[mi]
        n = dec._bs[blockflag]
        n2 = n >> 1
        if blockflag:
            win, wl, wr = self.win_long[(_bits(d, pos, 1) << 1) | _bits(d, pos + 1, 1)]
            pos += 2
        else:
            win, wl, wr = self.win_short
        mp = dec._mappings[mapnum]
        if mp.submaps != 1:
            raise OggError("multiple submaps are not supported by the turbo kernel")
        ch = dec._channels
        bk = self.bk
        ft = self.ft
        lc = self.lc
        lv = self.lv
        st = self.st
        wk = self.wk
        yb = self.yb
        floors = self.floors
        mux = mp.mux
        used = 0
        for c in range(ch):
            pos = k.floor1(
                d, pos, nbits, floors[mp.floor[mux[c]]], bk, ft, lc, lv, wk, yb[c], n2, st
            )
            if st[1]:
                used |= 1 << c
        # nonzero vector propagation (4.3.3)
        decode = used
        for m, a in mp.coupling:
            if decode & ((1 << m) | (1 << a)):
                decode |= (1 << m) | (1 << a)
        v = self.v
        pos = k.residue(
            d,
            pos,
            nbits,
            self.residues[mp.residue[0]],
            bk,
            ft,
            lc,
            lv,
            self.vq,
            v,
            ch,
            ((1 << ch) - 1) & ~decode,
            n2,
            self.cls,
            st,
        )
        for m, a in reversed(mp.coupling):
            k.couple(v, m * n2, a * n2, n2)
        pn = dec._prev_n
        if pn:
            off = (pn >> 2) - (n >> 2)
            length = (pn >> 2) + (n >> 2)
        else:
            off = 0
            length = 0
        dec._prev_n = n
        tw = self.tw[blockflag]
        slope = self.slope
        n2max = dec._bs[1] >> 1
        y = self.y
        pcm = self.pcm
        prev = self.prev
        pn2 = pn >> 1
        for c in range(ch):
            hasy = (used >> c) & 1
            if hasy:
                k.floor_apply(v, c * n2, yb[c], 0, self.tm, self.ts, n2)
                k.imdct(v, c * n2, tw, n, self.re, self.im, y, win, slope[wl], slope[wr])
            k.overlap(prev, c * n2max, y, hasy, pcm, ch, c, length, off, n, pn2)
        return length


class OggDecoder:
    """Decode an Ogg Vorbis file to 16-bit signed PCM.

    :param file: path string or a file opened in ``"rb"`` mode
    :param bool turbo: use the prebuilt kernel when one loads (default)

    Playing an Ogg file::

        import audiobusio
        import board
        import adafruit_ogg

        audio = audiobusio.I2SOut(board.I2S_BCLK, board.I2S_WS, board.I2S_DIN)
        decoder = adafruit_ogg.OggDecoder("song.ogg")
        player = adafruit_ogg.OggPlayer(audio)
        player.play(decoder)
        while player.playing:
            player.update()
    """

    def __init__(self, file, *, turbo=True):
        self._owns = False
        self._file = None
        self._ogg = None
        self._turbo = None
        self._kernel = _kernel if turbo else None
        self._open(file)

    # -- public API --

    @property
    def file(self):
        """The open file object. Assign a new file or path to switch songs."""
        return self._file

    @file.setter
    def file(self, file):
        self._open(file)

    def open(self, file):
        """Switch to a different file (same as assigning `file`)."""
        self._open(file)

    @property
    def sample_rate(self):
        """Sample rate in Hz."""
        return self._rate

    @property
    def channel_count(self):
        """1 for mono, 2 for stereo."""
        return self._channels

    @property
    def bits_per_sample(self):
        """Always 16."""
        return 16

    @property
    def samples_decoded(self):
        """Frames decoded since the file was opened or rewound."""
        return self._decoded

    @property
    def comments(self):
        """Vorbis comment tags as a dict of upper-case name to value."""
        return self._comments

    @property
    def vendor(self):
        """Encoder vendor string."""
        return self._vendor

    @property
    def nominal_bitrate(self):
        """Encoder's nominal bitrate in bits per second (0 if unknown)."""
        return self._bitrate

    @property
    def duration(self):
        """Length in seconds, read from the last Ogg page (None if unknown)."""
        if self._duration is None:
            self._duration = self._scan_duration()
        return self._duration

    @property
    def turbo(self):
        """True when a prebuilt kernel (C or turbo) decodes this file."""
        return self._turbo is not None

    @property
    def rms_level(self):
        """RMS level of the most recently decoded packet, 0.0 to 1.0."""
        pcm = self._pcm
        if pcm is None or self._pcm_len == 0:
            return 0.0
        total = 0
        count = self._pcm_len * self._channels
        # every 8th value is plenty for a level meter and keeps this cheap
        for i in range(0, count, 8):
            s = int(pcm[i])
            total += s * s
        return math.sqrt(total / ((count + 7) // 8)) / 32768

    def rewind(self):
        """Go back to the start of the audio."""
        self._start()

    def readinto(self, buf):
        """Fill ``buf`` (an ``array('h')`` or memoryview of one) with interleaved
        samples. Returns the number of values written; less than ``len(buf)``
        only at the end of the stream."""
        mv = memoryview(buf)
        total = len(mv)
        got = 0
        while got < total:
            n = self.fill(mv[got:])
            if n == 0:
                break
            got += n
        return got

    def fill(self, mv):
        """Copy decoded samples into memoryview ``mv``, decoding at most one
        packet. Returns the number of values written, 0 at end of stream."""
        while self._pcm_pos >= self._pcm_len * self._channels:
            if not self._decode_next():
                return 0
        avail = self._pcm_len * self._channels - self._pcm_pos
        n = len(mv)
        n = min(n, avail)
        p = self._pcm_pos
        mv[:n] = self._pcm_mv[p : p + n]
        self._pcm_pos = p + n
        return n

    def deinit(self):
        """Release the file (if this decoder opened it) and buffers."""
        if self._owns and self._file is not None:
            self._file.close()
        self._file = None
        self._books = None
        self._turbo = None
        self._ogg = None
        self._prev = None
        self._pcm = None
        self._pcm_mv = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.deinit()

    # -- internals --

    def _open(self, file):
        if self._owns and self._file is not None:
            self._file.close()
        if isinstance(file, str):
            file = open(file, "rb")
            self._owns = True
        else:
            self._owns = False
        self._file = file
        self._duration = None
        self._old = self._turbo  # its buffers are reused for this file
        self._turbo = None
        self._books = None
        self._pcm = None
        self._pcm_mv = None
        gc.collect()
        # the packet buffer first, while internal RAM is free: the decode reads it bit by bit
        if self._ogg is None:
            self._ogg = _Ogg(file)
        else:
            self._ogg.reset(file)
        self._read_headers(self._ogg)
        self._old = None
        self._start()

    def _start(self):
        ogg = self._ogg
        ogg.reset(self._file)
        for _ in range(3):
            if ogg.packet() < 0:
                raise OggError("missing Vorbis headers")
        self._prev = None
        self._prev_n = 0
        self._decoded = 0
        self._pcm = None
        self._pcm_mv = None
        self._pcm_len = 0
        self._pcm_pos = 0
        self._done = False
        if self._turbo is not None:
            self._pcm = self._turbo.pcm
            self._pcm_mv = memoryview(self._pcm)
            t = self._turbo.prev
            for i in range(len(t)):
                t[i] = 0

    def _read_headers(self, ogg):
        n = ogg.packet()
        p = bytes(ogg.pkt[: max(n, 0)])
        if n < 0 or p[:7] != b"\x01vorbis":
            raise OggError("not an Ogg Vorbis file")
        version, channels, rate, bmax, bnom, bmin, bs = struct.unpack("<IBIiiiB", p[7:29])
        if version != 0 or channels == 0 or rate == 0:
            raise OggError("bad Vorbis identification header")
        bs0 = 1 << (bs & 15)
        bs1 = 1 << (bs >> 4)
        if bs0 < 64 or bs1 > 8192 or bs0 > bs1:
            raise OggError("bad Vorbis block sizes")
        self._channels = channels
        self._rate = rate
        self._bitrate = bnom if bnom > 0 else 0
        self._bs = (bs0, bs1)

        n = ogg.packet()
        p = bytes(ogg.pkt[: max(n, 0)])
        if n < 0 or p[:7] != b"\x03vorbis":
            raise OggError("missing Vorbis comment header")
        self._comments = {}
        self._vendor = ""
        try:
            pos = 7
            n = struct.unpack("<I", p[pos : pos + 4])[0]
            self._vendor = str(p[pos + 4 : pos + 4 + n], "utf-8")
            pos += 4 + n
            count = struct.unpack("<I", p[pos : pos + 4])[0]
            pos += 4
            for _ in range(count):
                n = struct.unpack("<I", p[pos : pos + 4])[0]
                tag = str(p[pos + 4 : pos + 4 + n], "utf-8")
                pos += 4 + n
                k = tag.find("=")
                if k > 0:
                    self._comments[tag[:k].upper()] = tag[k + 1 :]
        except Exception:  # noqa: BLE001, tags are informational; a bad block is not fatal
            pass

        n = ogg.packet()
        p = bytes(ogg.pkt[: max(n, 0)])
        if n < 0 or p[:7] != b"\x05vorbis":
            raise OggError("missing Vorbis setup header")
        float_vq = self._kernel is None
        br = _Bits(p)
        for _ in range(7):  # skip the packet type and "vorbis"
            br.read(8)
        self._books = [_Codebook(br, float_vq) for _ in range(br.read(8) + 1)]
        for _ in range(br.read(6) + 1):
            if br.read(16):
                raise OggError("bad time domain")
        self._floors = []
        for _ in range(br.read(6) + 1):
            ftype = br.read(16)
            if ftype != 1:
                raise OggError("floor type 0 is not supported")
            self._floors.append(_Floor1(br))
        self._residues = []
        for _ in range(br.read(6) + 1):
            rtype = br.read(16)
            if rtype > 2:
                raise OggError("bad residue type")
            self._residues.append(_Residue(br, rtype))
        self._mappings = []
        for _ in range(br.read(6) + 1):
            if br.read(16):
                raise OggError("bad mapping type")
            self._mappings.append(_Mapping(br, channels))
        self._modes = []
        for _ in range(br.read(6) + 1):
            blockflag = br.read(1)
            if br.read(16) or br.read(16):
                raise OggError("bad mode")
            self._modes.append((blockflag, br.read(8)))
        if not br.read(1) or br.eop:
            raise OggError("bad setup header framing")
        self._mode_bits = _ilog(len(self._modes) - 1)
        if self._kernel is not None and all(m.submaps == 1 for m in self._mappings):
            self._turbo = _Turbo(self, self._old)
            self._books = None
        else:
            if self._kernel is not None:
                # multiple submaps: rebuild the codebooks for the Python decoder
                self._kernel = None
                return self._read_headers(_reopen(ogg))
            from adafruit_ogg import _float  # noqa: PLC0415, only without the kernel

            self._float = _float
            self._accel = _float._np is not None
            self._xf = (
                _float._Transform(bs0, bs0, self._accel),
                _float._Transform(bs1, bs0, self._accel),
            )
        return None

    def _scan_duration(self):
        f = self._file
        here = f.tell()
        try:
            size = f.seek(0, 2)
            if size is None:
                size = f.tell()
            start = size - 65536 if size > 65536 else 0
            f.seek(start)
            tail = f.read(size - start)
            k = tail.rfind(b"OggS") if hasattr(tail, "rfind") else -1
            while k >= 0:
                if k + 14 <= len(tail):
                    g = struct.unpack("<q", tail[k + 6 : k + 14])[0]
                    if g >= 0:
                        return g / self._rate
                k = tail.rfind(b"OggS", 0, k)
            return None
        finally:
            f.seek(here)

    def _decode_next(self):
        # Decode packets until one yields audio. False at end of stream.
        if self._done:
            return False
        ogg = self._ogg
        turbo = self._turbo
        while True:
            nbytes = ogg.packet()
            if nbytes < 0:
                self._done = True
                return False
            if turbo is not None:
                n = turbo.decode(self, ogg, nbytes)
                pcm = turbo.pcm
            else:
                frames = self._decode_packet(bytes(ogg.pkt[:nbytes]))
                if frames is None:
                    n = -1
                else:
                    pcm, n = frames
            if n < 0:
                continue
            if ogg.pkt_eos and ogg.pkt_granule >= 0:
                # Ogg end trimming: the last granule says how many frames exist
                keep = ogg.pkt_granule - self._decoded
                if 0 <= keep < n:
                    n = keep
            if n <= 0:
                if ogg.pkt_eos:
                    self._done = True
                    return False
                continue
            if pcm is not self._pcm:
                self._pcm = pcm
                self._pcm_mv = memoryview(pcm)
            self._pcm_len = n
            self._pcm_pos = 0
            self._decoded += n
            return True

    def _decode_packet(self, pkt):
        """Plain Python decode (4.3). Returns (interleaved int16 pcm, frames) or None."""
        br = _Bits(pkt)
        if br.read(1):
            return None
        mi = br.read(self._mode_bits)
        if mi >= len(self._modes) or br.eop:
            return None
        mode = self._modes[mi]
        blockflag, mapnum = mode
        n = self._bs[blockflag]
        n2 = n >> 1
        if blockflag:
            prev = br.read(1)
            nxt = br.read(1)
        else:
            prev = nxt = 0
        mp = self._mappings[mapnum]
        books = self._books
        ch = self._channels

        # floors (4.3.2)
        floors = [None] * ch
        for c in range(ch):
            fl = self._floors[mp.floor[mp.mux[c]]]
            ys = fl.decode(br, books)
            if ys is not None:
                floors[c] = fl.render(ys, n2)
        # nonzero vector propagation (4.3.3)
        nores = [f is None for f in floors]
        for m, a in mp.coupling:
            if not nores[m] or not nores[a]:
                nores[m] = nores[a] = False
        # residues (4.3.4)
        spec = [None] * ch
        for s in range(mp.submaps):
            chans = [c for c in range(ch) if mp.mux[c] == s]
            vecs = [[0.0] * n2 for _ in chans]
            self._residues[mp.residue[s]].decode(br, books, vecs, [nores[c] for c in chans], n2)
            for i, c in enumerate(chans):
                spec[c] = vecs[i]

        xf = self._xf[blockflag]
        win = xf.window(blockflag, prev, nxt)
        if self._accel:
            return self._float.synth_np(self, spec, floors, mp, xf, win, n)
        return self._float.synth_py(self, spec, floors, mp, xf, win, n)


def _reopen(ogg):
    ogg.reset(ogg.f)
    return ogg


class OggPlayer:
    """Stream an `OggDecoder` to an audio output from Python.

    Decoded PCM goes straight into a double-buffered `audiocore.RawSample` that loops on the
    output, so playback keeps going while your code runs. Call `update` often (several times
    per ``buffer_ms``); each call decodes at most one Vorbis packet.

    The output must copy each half when it starts playing it, as ``I2SOut``,
    ``PWMAudioOut`` and ``AudioOut`` do (into their DMA buffers): a half is rewritten as soon
    as the output has taken it. Not for ``audiomixer``, which reads the sample in place.

    Keep the buffer in internal RAM where there is a choice; pass ``buffer`` to allocate it
    early, before other code takes the internal RAM.

    :param audio: an ``audiobusio.I2SOut``, ``audiopwmio.PWMAudioOut`` or
        ``audioio.AudioOut``-like object with ``play(sample, loop=True)``, ``stop()`` and
        optionally ``pause()``
    :param int buffer_ms: length of each half of the output buffer. The output's DMA needs
        internal RAM for twice the whole buffer.
    :param buffer: optional ``array('h')`` to use for both halves (its length sets the half
        size; ``buffer_ms`` is then ignored)
    """

    def __init__(self, audio, *, buffer_ms=64, buffer=None):
        self._audio = audio
        self._buffer_ms = buffer_ms
        self._fixed = buffer is not None
        self._buf = buffer
        self._mv = memoryview(buffer) if buffer is not None else None
        self._zero = array("h", [0]) * 256
        self._decoder = None
        self._playing = False
        self._paused = False
        self.underruns = 0
        """Count of buffer halves that were not ready in time."""

    @property
    def playing(self):
        """True while a file is playing (including while paused)."""
        return self._playing

    @property
    def paused(self):
        """True while paused."""
        return self._paused

    @property
    def decoder(self):
        """The `OggDecoder` being played."""
        return self._decoder

    def play(self, decoder, *, loop=False):
        """Start playing ``decoder`` from its current position."""
        import audiocore  # noqa: PLC0415, so the decoder also imports on desktop Python

        self.stop()
        self._decoder = decoder
        self._loop = loop
        ch = decoder.channel_count
        if self._fixed:
            half = len(self._buf) // 2 // ch * ch
        else:
            half = max(256, decoder.sample_rate * self._buffer_ms // 1000) * ch
            if self._buf is None or len(self._buf) != 2 * half:
                self._buf = array("h", [0]) * (2 * half)
                self._mv = memoryview(self._buf)
        self._half = half
        self._eof = False
        self._last_real = -1
        # prime both halves before starting the output
        for w in range(2):
            self._w = w
            self._filled = 0
            while self._filled < half:
                self._fill_once()
            if self._eof and self._last_real < 0:
                self._last_real = w
        self._w = 2
        self._filled = 0
        self._half_ns = half // ch * 1_000_000_000 // decoder.sample_rate
        self._sample = audiocore.RawSample(
            self._mv[: 2 * half],
            channel_count=ch,
            sample_rate=decoder.sample_rate,
            single_buffer=False,
        )
        self._audio.play(self._sample, loop=True)
        self._t0 = _now()
        self._playing = True
        self._paused = False

    def _fill_once(self):
        # Decode up to one packet into the half being written; pad with silence at the end.
        half = self._half
        a = (self._w & 1) * half + self._filled
        room = half - self._filled
        if not self._eof:
            n = self._decoder.fill(self._mv[a : a + room])
            if not n and self._loop:
                self._decoder.rewind()
                n = self._decoder.fill(self._mv[a : a + room])
            if n:
                self._filled += n
                return
            self._eof = True
        z = self._zero
        while room > 0:
            k = room if room < len(z) else len(z)
            self._mv[a : a + k] = z[:k]
            a += k
            room -= k
        self._filled = half

    def update(self):
        """Do a slice of decoding work. Returns True while still playing."""
        if not self._playing:
            return False
        if self._paused:
            return True
        elapsed = _now() - self._t0
        hd = self._half_ns
        # halves the output has taken so far: two at start, then one per half
        fetched = 2 + elapsed // hd
        if self._last_real >= 0 and fetched >= self._last_real + 3:
            # the half after the last real one (silence) is now playing
            self.stop()
            return False
        w = self._w
        if w < fetched:
            # fell behind: half w was taken before it was ready; skip ahead
            self.underruns += 1
            w = self._w = fetched + 1
            self._filled = 0
        # Half w reuses the slot of half w - 2, taken at (w - 3) * hd. Leave a quarter
        # half of margin for the output's background task to copy it.
        if elapsed < (w - 3) * hd + (hd >> 2):
            return True
        self._fill_once()
        if self._filled >= self._half:
            if self._eof and self._last_real < 0:
                self._last_real = w
            self._w = w + 1
            self._filled = 0
        return True

    def pause(self):
        """Pause output, if the audio object supports it."""
        if self._playing and not self._paused:
            self._audio.pause()
            self._pause_t = _now()
            self._paused = True

    def resume(self):
        """Resume after `pause`."""
        if self._paused:
            self._t0 += _now() - self._pause_t
            self._audio.resume()
            self._paused = False

    def stop(self):
        """Stop playback."""
        if self._playing:
            self._audio.stop()
        self._playing = False
        self._paused = False

    def deinit(self):
        """Stop and release buffers."""
        self.stop()
        self._buf = None
        self._mv = None
        self._fixed = False
        self._sample = None
