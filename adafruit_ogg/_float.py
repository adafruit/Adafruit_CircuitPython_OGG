# SPDX-FileCopyrightText: Copyright (c) 2026 Mikey Sklar for Adafruit Industries
# SPDX-License-Identifier: MIT
"""Plain Python (float) synthesis for adafruit_ogg, used only when no turbo kernel loads.

Correct but several times slower than real time on a microcontroller. With ``ulab`` the
transform, windowing and PCM conversion are vectorized.
"""

import math
from array import array

try:
    from ulab import numpy as _np

    _ULAB = True
except ImportError:
    try:
        import numpy as _np  # desktop testing with CPython
    except ImportError:
        _np = None
    _ULAB = False

_PI = math.pi


def _ilog(x):
    n = 0
    while x > 0:
        n += 1
        x >>= 1
    return n


class _Transform:
    """IMDCT of one block size via an N/4 point complex FFT, plus windows."""

    def __init__(self, n, bs0, accel):
        self.n = n
        m = n >> 1
        h = n >> 2
        self.m = m
        self.h = h
        # pre-twiddle exp(-i*pi*(4j+1)/(4m)), post-twiddle exp(-i*pi*j/m)
        c1 = [math.cos(-_PI * (4 * j + 1) / (4 * m)) for j in range(h)]
        s1 = [math.sin(-_PI * (4 * j + 1) / (4 * m)) for j in range(h)]
        c2 = [math.cos(-_PI * j / m) for j in range(h)]
        s2 = [math.sin(-_PI * j / m) for j in range(h)]
        self.slope = _slope(n >> 1)
        self.short_slope = _slope(bs0 >> 1)
        self.bs0 = bs0
        self.windows = {}
        if accel:
            self.c1 = _np.array(c1)
            self.s1 = _np.array(s1)
            self.c2 = _np.array(c2)
            self.s2 = _np.array(s2)
        else:
            self.c1 = c1
            self.s1 = s1
            self.c2 = c2
            self.s2 = s2
            bits = _ilog(h) - 1
            rev = []
            for i in range(h):
                r = 0
                v = i
                for _ in range(bits):
                    r = (r << 1) | (v & 1)
                    v >>= 1
                rev.append(r)
            self.rev = rev
            self.fc = [math.cos(2 * _PI * k / h) for k in range(h >> 1)]
            self.fs = [-math.sin(2 * _PI * k / h) for k in range(h >> 1)]
        self.accel = accel

    def window(self, long_block, prev, nxt):
        """Section 4.3.1 window for this block size and neighbor flags."""
        key = (prev, nxt) if long_block else (1, 1)
        w = self.windows.get(key)
        if w is not None:
            return w
        n = self.n
        bs0 = self.bs0
        w = [0.0] * n
        if long_block and not prev:
            ls = n // 4 - bs0 // 4
            lslope = self.short_slope
        else:
            ls = 0
            lslope = self.slope
        if long_block and not nxt:
            rs = n * 3 // 4 - bs0 // 4
            rslope = self.short_slope
        else:
            rs = n // 2
            rslope = self.slope
        le = ls + len(lslope)
        for i in range(len(lslope)):
            w[ls + i] = lslope[i]
        for i in range(le, rs):
            w[i] = 1.0
        rn = len(rslope)
        for i in range(rn):
            w[rs + i] = rslope[rn - 1 - i]
        if self.accel:
            w = _np.array(w)
        self.windows[key] = w
        return w

    def imdct(self, x, win):
        """x: n/2 coefficients. Returns n windowed samples."""
        if self.accel:
            return self._imdct_np(x, win)
        m = self.m
        h = self.h
        c1 = self.c1
        s1 = self.s1
        re = [0.0] * h
        im = [0.0] * h
        rev = self.rev
        for j in range(h):
            a = x[2 * j]
            b = x[m - 1 - 2 * j]
            r = rev[j]
            re[r] = a * c1[j] - b * s1[j]
            im[r] = a * s1[j] + b * c1[j]
        fc = self.fc
        fs = self.fs
        size = 2
        while size <= h:
            half = size >> 1
            step = h // size
            for start in range(0, h, size):
                k = 0
                for j in range(start, start + half):
                    l = j + half
                    wr = fc[k]
                    wi = fs[k]
                    rl = re[l]
                    il = im[l]
                    tr = wr * rl - wi * il
                    ti = wr * il + wi * rl
                    rj = re[j]
                    ij = im[j]
                    re[l] = rj - tr
                    im[l] = ij - ti
                    re[j] = rj + tr
                    im[j] = ij + ti
                    k += step
            size <<= 1
        c2 = self.c2
        s2 = self.s2
        u = [0.0] * m
        for j in range(h):
            a = re[j]
            b = im[j]
            u[2 * j] = a * c2[j] - b * s2[j]
            u[m - 1 - 2 * j] = -(a * s2[j] + b * c2[j])
        # unfold the DCT-IV into the n point IMDCT output and window it
        n = self.n
        y = [0.0] * n
        q = m >> 1
        for i in range(q):
            y[i] = u[i + q] * win[i]
        for i in range(q, q + m):
            y[i] = -u[m + q - 1 - i] * win[i]
        for i in range(q + m, n):
            y[i] = -u[i - q - m] * win[i]
        return y

    def _imdct_np(self, x, win):
        np = _np
        m = self.m
        a = x[0::2]
        b = x[m - 1 :: -2]
        re = a * self.c1 - b * self.s1
        im = a * self.s1 + b * self.c1
        if _ULAB:
            fr, fi = np.fft.fft(re, im)
        else:
            z = np.fft.fft(re + 1j * im)
            fr = z.real
            fi = z.imag
        u = np.zeros(m)
        u[0::2] = fr * self.c2 - fi * self.s2
        u[m - 1 :: -2] = -(fr * self.s2 + fi * self.c2)
        q = m >> 1
        y = np.concatenate((u[q:], -u[::-1], -u[:q]))
        return y * win


def _slope(n):
    # rising half of the Vorbis power-sine window, section 4.3.1
    hp = _PI / 2
    return [math.sin(hp * math.pow(math.sin((i + 0.5) / n * hp), 2)) for i in range(n)]


def synth_py(self, spec, floors, mp, xf, win, n):
    ch = self._channels
    n2 = n >> 1
    # inverse coupling (4.3.5)
    for m, a in reversed(mp.coupling):
        mv = spec[m]
        av = spec[a]
        for i in range(n2):
            mm = mv[i]
            aa = av[i]
            if mm > 0:
                if aa > 0:
                    av[i] = mm - aa
                else:
                    av[i] = mm
                    mv[i] = mm + aa
            elif aa > 0:
                av[i] = mm + aa
            else:
                av[i] = mm
                mv[i] = mm - aa
    prevs = self._prev
    pn = self._prev_n
    if pn:
        off = pn // 4 - n // 4
        length = pn // 4 + n // 4
        pcm = array("h", bytearray(2 * length * ch))
    else:
        pcm = None
        length = 0
    newprev = []
    for c in range(ch):
        f = floors[c]
        if f is None:
            y = None
        else:
            x = spec[c]
            for i in range(n2):
                x[i] *= f[i]
            y = xf.imdct(x, win)
        if pcm is not None:
            pr = prevs[c]
            k = c
            for q in range(length):
                v = pr[q] if (pr is not None and q < len(pr)) else 0.0
                j = q - off
                if y is not None and 0 <= j < n:
                    v += y[j]
                s = int(v * 32768.0 + (0.5 if v > 0 else -0.5))
                if s > 32767:
                    s = 32767
                elif s < -32768:
                    s = -32768
                pcm[k] = s
                k += ch
        newprev.append(y[n2:] if y is not None else None)
    self._prev = newprev
    self._prev_n = n
    if pcm is None:
        return None
    return pcm, length


def synth_np(self, spec, floors, mp, xf, win, n):
    np = _np
    ch = self._channels
    n2 = n >> 1
    arrs = [np.array(s) if s is not None else None for s in spec]
    for m, a in reversed(mp.coupling):
        mm = arrs[m]
        aa = arrs[a]
        # branch-free form of the 4.3.5 table
        pa = aa > 0
        sm = (mm > 0) * 2.0 - 1.0
        t = aa * sm
        arrs[m] = mm + t - pa * t
        arrs[a] = mm - pa * t
    prevs = self._prev
    pn = self._prev_n
    if pn:
        off = pn // 4 - n // 4
        length = pn // 4 + n // 4
        pcm = np.zeros(length * ch, dtype=np.int16)
    else:
        pcm = None
        length = 0
    newprev = []
    for c in range(ch):
        f = floors[c]
        y = None
        if f is not None:
            y = xf.imdct(arrs[c] * np.array(f), win)
        if pcm is not None:
            acc = np.zeros(length)
            pr = prevs[c]
            if pr is not None:
                k = len(pr) if len(pr) < length else length
                acc[:k] = pr[:k]
            if y is not None:
                # out[q] += y[q - off] where 0 <= q - off < n
                q0 = off if off > 0 else 0
                j0 = q0 - off
                k = length - q0
                acc[q0:] = acc[q0:] + y[j0 : j0 + k]  # noqa: PLR6104, ulab slices have no +=
            acc = np.clip(np.around(acc * 32768.0), -32768.0, 32767.0)
            if ch == 1:
                pcm = np.array(acc, dtype=np.int16)
            else:
                pcm[c::ch] = np.array(acc, dtype=np.int16)
        newprev.append(y[n2:] if y is not None else None)
    self._prev = newprev
    self._prev_n = n
    if pcm is None:
        return None
    return pcm, length
