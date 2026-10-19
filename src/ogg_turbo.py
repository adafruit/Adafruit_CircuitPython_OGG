# SPDX-FileCopyrightText: Copyright (c) 2026 Mikey Sklar for Adafruit Industries
# SPDX-License-Identifier: MIT
"""Turbo (@micropython.viper) kernel for adafruit_ogg: the per-packet Vorbis audio decode.

Compile with: mpy-cross -march=armv7emsp ogg_turbo.py (see Makefile). The Python side
(adafruit_ogg/__init__.py) parses the headers once and packs everything this kernel reads
into flat int arrays; section numbers refer to the Vorbis I specification.

Turbo only speeds up integer code, so the audio path is fixed point:

- residue values: integers scaled by 2**RQ (RQ chosen per file, 0 for libvorbis files)
- spectrum and time samples: Q24 (1.0 = 2**24), PCM = Q24 >> 9
- floor curve: 15-bit mantissa and a shift per inverse-dB step
- twiddles and window slopes: Q30, stored split into high and low 15-bit halves so a
  product with a Q24 value needs only 32-bit multiplies:
  (x * w) >> 30 == xh * wh + ((xh * wl) >> 15) + ((xl * wh) >> 15), x = xh << 15 | xl

Codebook decode reads 32 stream bits at the bit position, looks the low FAST bits up in a
16-bit table ((entry << 4) | length, 0xFFFF for longer codes), and binary searches the
bit-reversed window among the sorted long codewords otherwise (as stb_vorbis does).

Big constants are built in two steps: a literal of 2**30 or more becomes a big-int object.
"""

import micropython

# Book header layout in `bk` (12 ints per book), shared with adafruit_ogg/__init__.py:
# 0 fast table offset, 1 fast bits, 2 long table offset, 3 long count, 4 dim, 5 entries,
# 6 lookup (0 none, 1 lattice computed per entry, 2 expanded table), 7 vq offset,
# 8 lookup values (lattice), 9 sequence_p


@micropython.viper
def _cw(d_obj, pos: int, bk_obj, ft_obj, lc_obj, lv_obj, b: int, st_obj) -> int:
    """Decode one codeword of book b at bit pos. Returns the entry (-1 if invalid) and
    stores the new bit position in st[0]."""
    d = ptr8(d_obj)
    bk = ptr32(bk_obj)
    ft = ptr16(ft_obj)
    lc = ptr32(lc_obj)
    lv = ptr32(lv_obj)
    st = ptr32(st_obj)
    h = b * 12
    fb = bk[h + 1]
    i = pos >> 3
    sh = pos & 7
    w = uint(d[i] | (d[i + 1] << 8) | (d[i + 2] << 16) | (d[i + 3] << 24)) >> uint(sh)
    if fb:
        e = ft[bk[h] + (int(w) & ((1 << fb) - 1))]
        if e != 0xFFFF:
            st[0] = pos + (e & 15)
            return e >> 4
    n = bk[h + 3]
    if n == 0:
        st[0] = pos
        return -1
    if sh:
        w = w | (uint(d[i + 4] & ((1 << sh) - 1)) << uint(32 - sh))
    # reverse the 32 stream bits so codewords compare MSB first
    mk = uint(0x5555)
    mk = (mk << 16) | uint(0x5555)
    w = ((w >> 1) & mk) | ((w & mk) << 1)
    mk = uint(0x3333)
    mk = (mk << 16) | uint(0x3333)
    w = ((w >> 2) & mk) | ((w & mk) << 2)
    mk = uint(0x0F0F)
    mk = (mk << 16) | uint(0x0F0F)
    w = ((w >> 4) & mk) | ((w & mk) << 4)
    mk = uint(0x00FF)
    mk = (mk << 16) | uint(0x00FF)
    w = ((w >> 8) & mk) | ((w & mk) << 8)
    w = (w >> 16) | ((w & uint(0xFFFF)) << 16)
    lo = bk[h + 2]
    x = 0
    while n > 1:
        half = n >> 1
        if uint(lc[lo + x + half]) <= w:
            x += half
            n -= half
        else:
            n = half
    ev = lv[lo + x]
    ln = ev & 31
    sh = 32 - ln
    if (w >> uint(sh)) != (uint(lc[lo + x]) >> uint(sh)):
        st[0] = pos
        return -1
    st[0] = pos + ln
    return ev >> 5


@micropython.viper
def floor1(
    d_obj,
    pos: int,
    nbits: int,
    fl_obj,
    bk_obj,
    ft_obj,
    lc_obj,
    lv_obj,
    wk_obj,
    yb_obj,
    n2: int,
    st_obj,
) -> int:
    """Floor type 1 decode and render (7.2.3, 7.2.4). Writes the inverse-dB index of each of
    the n2 bins to yb and sets st[1] to 1, or st[1] to 0 for an unused floor. Returns the
    new bit position (past nbits at the end of the packet)."""
    d = ptr8(d_obj)
    fl = ptr32(fl_obj)
    wk = ptr32(wk_obj)
    yb = ptr8(yb_obj)
    st = ptr32(st_obj)
    st[1] = 0
    if pos >= nbits:
        return pos
    if not ((d[pos >> 3] >> (pos & 7)) & 1):
        return pos + 1
    pos += 1
    nx = fl[0]
    mult = fl[1]
    rng = fl[2]
    ybits = fl[3]
    parts = fl[4]
    opc = fl[6]
    ocl = fl[7]
    oxs = fl[8]
    olo = fl[9]
    ohi = fl[10]
    oord = fl[11]
    ymask = (1 << ybits) - 1
    # ys in wk[0:nx], fy in wk[256:], step2 in wk[512:]
    i = pos >> 3
    w = int(uint(d[i] | (d[i + 1] << 8) | (d[i + 2] << 16) | (d[i + 3] << 24)) >> uint(pos & 7))
    wk[0] = w & ymask
    pos += ybits
    i = pos >> 3
    w = int(uint(d[i] | (d[i + 1] << 8) | (d[i + 2] << 16) | (d[i + 3] << 24)) >> uint(pos & 7))
    wk[1] = w & ymask
    pos += ybits
    k = 2
    for p in range(parts):
        c = fl[opc + p]
        cb = ocl + c * 12
        cdim = fl[cb]
        csub = fl[cb + 1]
        cval = 0
        if csub:
            cval = int(_cw(d_obj, pos, bk_obj, ft_obj, lc_obj, lv_obj, fl[cb + 2], st_obj))
            pos = st[0]
            if cval < 0:
                st[1] = 0
                return nbits + 1
        cmask = (1 << csub) - 1
        for j in range(cdim):
            book = fl[cb + 4 + (cval & cmask)]
            cval = cval >> csub
            y = 0
            if book >= 0:
                y = int(_cw(d_obj, pos, bk_obj, ft_obj, lc_obj, lv_obj, book, st_obj))
                pos = st[0]
                if y < 0:
                    st[1] = 0
                    return nbits + 1
            wk[k] = y
            k += 1
    if pos > nbits:
        st[1] = 0
        return pos
    # 7.2.4 step 1: amplitude values
    wk[256] = wk[0]
    wk[257] = wk[1]
    wk[512] = 1
    wk[513] = 1
    for i in range(2, nx):
        wk[512 + i] = 0
    for i in range(2, nx):
        lo = fl[olo + i]
        hi = fl[ohi + i]
        x0 = fl[oxs + lo]
        y0 = wk[256 + lo]
        dy = wk[256 + hi] - y0
        adx = fl[oxs + hi] - x0
        ady = dy
        if dy < 0:
            ady = 0 - dy
        off = (ady * (fl[oxs + i] - x0)) // adx
        pred = y0 + off
        if dy < 0:
            pred = y0 - off
        val = wk[i]
        hroom = rng - pred
        lroom = pred
        room = lroom
        if hroom < lroom:
            room = hroom
        room = room * 2
        if val:
            wk[512 + lo] = 1
            wk[512 + hi] = 1
            wk[512 + i] = 1
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
            wk[256 + i] = v
        else:
            wk[256 + i] = pred
    # step 2: line segments between the used points, in x order (9.2.7)
    lx = 0
    ly = wk[256 + fl[oord]] * mult
    hx = 0
    hy = ly
    for kk in range(1, nx + 1):
        if kk < nx:
            q = fl[oord + kk]
            if not wk[512 + q]:
                continue
            hx = fl[oxs + q]
            hy = wk[256 + q] * mult
            x1 = hx
        else:
            if hx >= n2:
                break
            lx = hx
            ly = hy
            x1 = n2
        # render_line(lx, ly, x1, hy)
        if lx < n2:
            dy = hy - ly
            adx = x1 - lx
            ady = dy
            if dy < 0:
                ady = 0 - dy
            base = ady // adx
            sy = base + 1
            if dy < 0:
                base = 0 - base
                sy = base - 1
                ady -= (0 - base) * adx
            else:
                ady -= base * adx
            y = ly
            err = 0
            yb[lx] = y
            xe = x1
            if xe > n2:
                xe = n2
            for x in range(lx + 1, xe):
                err += ady
                if err >= adx:
                    err -= adx
                    y += sy
                else:
                    y += base
                yb[x] = y
        lx = hx
        ly = hy
    st[1] = 1
    return pos


@micropython.viper
def residue(
    d_obj,
    pos: int,
    nbits: int,
    rs_obj,
    bk_obj,
    ft_obj,
    lc_obj,
    lv_obj,
    vq_obj,
    v_obj,
    ch: int,
    dnd: int,
    n2: int,
    cls_obj,
    st_obj,
) -> int:
    """Residue decode (8.6.2, types 0, 1 and 2) into v: channel c's n2 values at c * n2,
    zeroed first. dnd has bit c set for channels with no residue to decode. Returns the new
    bit position."""
    d = ptr8(d_obj)
    rs = ptr32(rs_obj)
    bk = ptr32(bk_obj)
    ft = ptr16(ft_obj)
    vq = ptr32(vq_obj)
    v = ptr32(v_obj)
    cls = ptr8(cls_obj)
    st = ptr32(st_obj)
    total = ch * n2
    for i in range(total):
        v[i] = 0
    rtype = rs[0]
    size = n2
    nch = ch
    dmask = dnd
    if rtype == 2:
        if dnd == (1 << ch) - 1:
            return pos
        size = total
        nch = 1
        dmask = 0
    begin = rs[1]
    if begin > size:
        begin = size
    end = rs[2]
    if end > size:
        end = size
    psize = rs[3]
    nclass = rs[4]
    cbook = rs[5]
    nparts = (end - begin) // psize
    if nparts <= 0:
        return pos
    cpw = bk[cbook * 12 + 4]
    stride = nparts + cpw
    for p in range(8):
        part = 0
        while part < nparts:
            if p == 0:
                for c in range(nch):
                    if (dmask >> c) & 1:
                        continue
                    t = int(_cw(d_obj, pos, bk_obj, ft_obj, lc_obj, lv_obj, cbook, st_obj))
                    pos = st[0]
                    if t < 0 or pos > nbits:
                        return nbits + 1
                    i = cpw - 1
                    while i >= 0:
                        cls[c * stride + part + i] = t % nclass
                        t = t // nclass
                        i -= 1
            jj = 0
            while jj < cpw and part < nparts:
                for c in range(nch):
                    if (dmask >> c) & 1:
                        continue
                    bkn = rs[6 + cls[c * stride + part] * 8 + p]
                    if bkn < 0:
                        continue
                    h = bkn * 12
                    fo = bk[h]
                    fb = bk[h + 1]
                    fmask = (1 << fb) - 1
                    dim = bk[h + 4]
                    lt = bk[h + 6]
                    vo = bk[h + 7]
                    nv = bk[h + 8]
                    seq = bk[h + 9]
                    off = begin + part * psize
                    stop = off + psize
                    step = psize // dim
                    k = off
                    i = 0
                    while k < stop:
                        # codeword
                        bi = pos >> 3
                        w = int(
                            uint(d[bi] | (d[bi + 1] << 8) | (d[bi + 2] << 16) | (d[bi + 3] << 24))
                            >> uint(pos & 7)
                        )
                        e = 0xFFFF
                        if fb:
                            e = ft[fo + (w & fmask)]
                        if e != 0xFFFF:
                            pos += e & 15
                            e = e >> 4
                        else:
                            e = int(_cw(d_obj, pos, bk_obj, ft_obj, lc_obj, lv_obj, bkn, st_obj))
                            pos = st[0]
                            if e < 0:
                                return nbits + 1
                        if pos > nbits:
                            return pos
                        # add the entry's vector (8.6.3, 8.6.4)
                        last = 0
                        q = vo + e * dim
                        rem = e
                        for j in range(dim):
                            if lt == 2:
                                val = vq[q + j]
                            else:
                                val = vq[vo + rem % nv] + last
                                rem = rem // nv
                                if seq:
                                    last = val
                            if rtype == 0:
                                idx = c * n2 + off + i + j * step
                            elif rtype == 1:
                                idx = c * n2 + k
                                k += 1
                            elif ch == 2:
                                idx = (k & 1) * n2 + (k >> 1)
                                k += 1
                            elif ch == 1:
                                idx = k
                                k += 1
                            else:
                                idx = (k % ch) * n2 + k // ch
                                k += 1
                            v[idx] += val
                        if rtype == 0:
                            i += 1
                            k = off + i * dim
                part += 1
                jj += 1
    return pos


@micropython.viper
def couple(v_obj, mo: int, ao: int, n2: int):
    """Inverse channel coupling (4.3.5) of the magnitude vector at mo and angle at ao."""
    v = ptr32(v_obj)
    for i in range(n2):
        mm = v[mo + i]
        aa = v[ao + i]
        if mm > 0:
            if aa > 0:
                v[ao + i] = mm - aa
            else:
                v[ao + i] = mm
                v[mo + i] = mm + aa
        elif aa > 0:
            v[ao + i] = mm + aa
        else:
            v[ao + i] = mm
            v[mo + i] = mm - aa


@micropython.viper
def floor_apply(v_obj, off: int, yb_obj, yoff: int, tm_obj, ts_obj, n2: int):
    """Spectrum = floor * residue (4.3.6) for n2 bins at v[off:], in place, giving Q24.
    The floor value for index y is tm[y] / 2**ts[y] in the residue's scale."""
    v = ptr32(v_obj)
    yb = ptr8(yb_obj)
    tm = ptr32(tm_obj)
    ts = ptr32(ts_obj)
    for i in range(n2):
        r = v[off + i]
        if r == 0:
            continue
        y = yb[yoff + i]
        m = tm[y]
        s = ts[y]
        ar = r
        if r < 0:
            ar = 0 - r
        if ar < 65536:
            pr = r * m
            if s >= 0:
                v[off + i] = pr >> s
            else:
                v[off + i] = pr << (0 - s)
        else:
            rh = r >> 8
            rl = r & 255
            if s >= 8:
                v[off + i] = ((rh * m) >> (s - 8)) + ((rl * m) >> s)
            elif s >= 0:
                v[off + i] = ((rh * m) << (8 - s)) + ((rl * m) >> s)
            else:
                v[off + i] = ((rh * m) << (8 - s)) + ((rl * m) << (0 - s))


@micropython.viper
def imdct(v_obj, off: int, tw_obj, n: int, re_obj, im_obj):
    """DCT-IV half of the IMDCT (4.3.7) of the n/2 Q24 coefficients at v[off:], in place;
    overlap() unfolds and windows it.

    tw holds, for h = n/4: pre-twiddles (4h ints: cos high, cos low, sin high, sin low),
    post-twiddles (4h), FFT twiddles for k < h/2 (2h) and the bit-reverse permutation (h)."""
    v = ptr32(v_obj)
    tw = ptr32(tw_obj)
    re = ptr32(re_obj)
    im = ptr32(im_obj)
    m = n >> 1
    h = n >> 2
    lm = 0x7FFF
    o_post = 4 * h
    o_fft = 8 * h
    o_rev = 10 * h
    # pre-twiddle into bit-reversed order
    for j in range(h):
        a = v[off + 2 * j]
        b = v[off + m - 1 - 2 * j]
        t = 4 * j
        ch_ = tw[t]
        cl = tw[t + 1]
        sh_ = tw[t + 2]
        sl = tw[t + 3]
        ah = a >> 15
        al = a & lm
        bh = b >> 15
        bl = b & lm
        ac = ah * ch_ + ((ah * cl) >> 15) + ((al * ch_) >> 15)
        bs = bh * sh_ + ((bh * sl) >> 15) + ((bl * sh_) >> 15)
        as_ = ah * sh_ + ((ah * sl) >> 15) + ((al * sh_) >> 15)
        bc = bh * ch_ + ((bh * cl) >> 15) + ((bl * ch_) >> 15)
        r = tw[o_rev + j]
        re[r] = ac - bs
        im[r] = as_ + bc
    # radix-2 FFT, size h
    size = 2
    sstep = h >> 1  # twiddle index step for this stage, h // size
    while size <= h:
        half = size >> 1
        # Twiddle 1 first, with no multiplies; then each twiddle is loaded once and used
        # for every group of the stage.
        j = 0
        while j < h:
            lj = j + half
            tr = re[lj]
            ti = im[lj]
            rj = re[j]
            ij = im[j]
            re[lj] = rj - tr
            im[lj] = ij - ti
            re[j] = rj + tr
            im[j] = ij + ti
            j += size
        tk = o_fft
        for k in range(1, half):
            tk += 4 * sstep
            # the twiddle rounded to Q15: two multiplies per product, not three
            qr = tw[tk] + (tw[tk + 1] >> 14)
            qi = tw[tk + 2] + (tw[tk + 3] >> 14)
            j = k
            while j < h:
                lj = j + half
                rl = re[lj]
                il = im[lj]
                rh = rl >> 15
                rlo = rl & lm
                ih = il >> 15
                ilo = il & lm
                # tr = qr * rl - qi * il, ti = qr * il + qi * rl
                tr = rh * qr + ((rlo * qr) >> 15) - ih * qi - ((ilo * qi) >> 15)
                ti = ih * qr + ((ilo * qr) >> 15) + rh * qi + ((rlo * qi) >> 15)
                rj = re[j]
                ij = im[j]
                re[lj] = rj - tr
                im[lj] = ij - ti
                re[j] = rj + tr
                im[j] = ij + ti
                j += size
        size = size << 1
        sstep = sstep >> 1
    # post-twiddle into u = v[off:off + m]
    for j in range(h):
        a = re[j]
        b = im[j]
        t = o_post + 4 * j
        ch_ = tw[t]
        cl = tw[t + 1]
        sh_ = tw[t + 2]
        sl = tw[t + 3]
        ah = a >> 15
        al = a & lm
        bh = b >> 15
        bl = b & lm
        ac = ah * ch_ + ((ah * cl) >> 15) + ((al * ch_) >> 15)
        bs = bh * sh_ + ((bh * sl) >> 15) + ((bl * sh_) >> 15)
        as_ = ah * sh_ + ((ah * sl) >> 15) + ((al * sh_) >> 15)
        bc = bh * ch_ + ((bh * cl) >> 15) + ((bl * ch_) >> 15)
        v[off + 2 * j] = ac - bs
        v[off + m - 1 - 2 * j] = 0 - (as_ + bc)


@micropython.viper
def overlap(
    prev_obj,
    poff: int,
    v_obj,
    voff: int,
    hasy: int,
    pcm_obj,
    ch: int,
    c: int,
    length: int,
    off: int,
    n: int,
    pn2: int,
    wp_obj,
    wl_obj,
    wr_obj,
):
    """Unfold, window and overlap-add (4.3.1, 4.3.8) in one pass: PCM for channel c,
    interleaved by ch, is the saved half of the previous block (pn2 values at prev[poff:])
    plus this block's windowed y[q - off]; then the second half of y (zeros if hasy is 0) is
    saved for the next block.

    y is read straight from the DCT-IV output u = v[voff:voff + n/2] left by imdct():
    y[j] = u[j + n/4] for j < n/4, -u[3n/4 - 1 - j] up to 3n/4, then -u[j - 3n/4]. The left
    window slope lies in the first half of y and the right one in the second (4.3.1), so each
    half is a few straight runs with no per-sample tests. wp = (left start, left length, right
    start, right length) of the slopes wl and wr (Q30 split pairs, rising)."""
    prev = ptr32(prev_obj)
    v = ptr32(v_obj)
    pcm = ptr16(pcm_obj)
    wp = ptr32(wp_obj)
    wl = ptr32(wl_obj)
    wr = ptr32(wr_obj)
    lm = 0x7FFF
    m = n >> 1
    h = n >> 2
    # PCM from prev alone, before y starts (or for a silent channel)
    qy = length
    if hasy:
        qy = 0
        if off > 0:
            qy = off
        if qy > length:
            qy = length
    e = qy
    if e > pn2:
        e = pn2
    q = 0
    k = c
    while q < e:
        s = (prev[poff + q] + 256) >> 9
        if s > 32767:
            s = 32767
        elif s < -32768:
            s = -32768
        pcm[k] = s
        k += ch
        q += 1
    while q < qy:
        pcm[k] = 0
        k += ch
        q += 1
    if hasy:
        # the rest of the PCM, in runs of one unfold direction, window part and prev part
        ls = wp[0]
        le = ls + wp[1]
        pv = pn2 - off  # y index where prev ends
        j = q - off
        jend = length - off
        while j < jend:
            e = jend
            if j < h:
                if h < e:
                    e = h
                u = voff + j + h
                du = 1
                sg = 1
            else:
                u = voff + m + h - 1 - j
                du = -1
                sg = -1
            kind = 2  # window 1
            if j < ls:
                kind = 0  # window 0
                if ls < e:
                    e = ls
            elif j < le:
                kind = 1  # rising slope
                if le < e:
                    e = le
            pm = 0
            if j < pv:
                pm = -1
                if pv < e:
                    e = pv
            p = poff + j + off
            if kind == 1:
                wi = 2 * (j - ls)
                while j < e:
                    x = v[u] * sg
                    xh = x >> 15
                    xl = x & lm
                    wh = wl[wi]
                    wlo = wl[wi + 1]
                    x = xh * wh + ((xh * wlo) >> 15) + ((xl * wh) >> 15)
                    s = ((prev[p] & pm) + x + 256) >> 9
                    if s > 32767:
                        s = 32767
                    elif s < -32768:
                        s = -32768
                    pcm[k] = s
                    k += ch
                    p += 1
                    u += du
                    wi += 2
                    j += 1
            elif kind == 2:
                while j < e:
                    s = ((prev[p] & pm) + v[u] * sg + 256) >> 9
                    if s > 32767:
                        s = 32767
                    elif s < -32768:
                        s = -32768
                    pcm[k] = s
                    k += ch
                    p += 1
                    u += du
                    j += 1
            else:
                while j < e:
                    s = ((prev[p] & pm) + 256) >> 9
                    if s > 32767:
                        s = 32767
                    elif s < -32768:
                        s = -32768
                    pcm[k] = s
                    k += ch
                    p += 1
                    j += 1
    # save the windowed second half y[m:n] for the next block
    p = poff
    if not hasy:
        pe = poff + m
        while p < pe:
            prev[p] = 0
            p += 1
        return
    rs = wp[2]
    rl = wp[3]
    re_ = rs + rl
    h3 = m + h
    j = m
    while j < n:
        e = n
        if j < h3:
            e = h3
            u = voff + h3 - 1 - j
            du = -1
        else:
            u = voff + j - h3
            du = 1
        if j < rs:
            if rs < e:
                e = rs
            while j < e:
                prev[p] = 0 - v[u]
                p += 1
                u += du
                j += 1
        elif j < re_:
            if re_ < e:
                e = re_
            wi = 2 * (re_ - 1 - j)  # falling: the rising slope read backwards
            while j < e:
                x = 0 - v[u]
                xh = x >> 15
                xl = x & lm
                wh = wr[wi]
                wlo = wr[wi + 1]
                prev[p] = xh * wh + ((xh * wlo) >> 15) + ((xl * wh) >> 15)
                p += 1
                u += du
                wi -= 2
                j += 1
        else:
            while j < e:
                prev[p] = 0
                p += 1
                j += 1


@micropython.viper
def book_count(lengths_obj, n: int, fb: int) -> int:
    """Number of used entries of a codebook with codewords longer than fb bits."""
    ln = ptr8(lengths_obj)
    c = 0
    for e in range(n):
        if ln[e] > fb:
            c += 1
    return c


@micropython.viper
def book_tables(
    lengths_obj, n: int, fb: int, ft_obj, fo: int, lc_obj, lv_obj, lo: int, av_obj
) -> int:
    """Codeword assignment (3.2.1) for the n entry lengths, into the fast table at ft[fo:]
    (fb bits) and the sorted long codeword table at lc[lo:], lv[lo:]. av is scratch for 33
    ints. Returns the number of long codewords, or -1 for an overspecified codebook."""
    ln = ptr8(lengths_obj)
    ft = ptr16(ft_obj)
    lc = ptr32(lc_obj)
    lv = ptr32(lv_obj)
    av = ptr32(av_obj)
    for i in range(33):
        av[i] = 0
    used = 0
    last = 0
    for e in range(n):
        if ln[e]:
            used += 1
            last = e
    size = 1 << fb
    if fb:
        for k in range(size):
            ft[fo + k] = 0xFFFF
    if used == 1:
        # one entry: every bit pattern decodes to it (stb_vorbis and libvorbis agree)
        length = ln[last]
        if fb:
            if length > fb:
                length = fb
            for k in range(size):
                ft[fo + k] = (last << 4) | length
            return 0
        lc[lo] = 0
        lv[lo] = (last << 5) | length
        return 1
    cnt = 0
    first = 1
    for e in range(n):
        length = ln[e]
        if not length:
            continue
        if first:
            first = 0
            for i in range(1, length + 1):
                av[i] = int(uint(1) << uint(32 - i))
            res = uint(0)
        else:
            z = length
            while z > 0 and av[z] == 0:
                z -= 1
            if z == 0:
                return -1
            res = uint(av[z])
            av[z] = 0
            y = length
            while y > z:
                av[y] = int(res + (uint(1) << uint(32 - y)))
                y -= 1
        if length <= fb:
            # LSB-first code: the bit reversal of the left-aligned codeword
            w = res
            mk = uint(0x5555)
            mk = (mk << 16) | uint(0x5555)
            w = ((w >> 1) & mk) | ((w & mk) << 1)
            mk = uint(0x3333)
            mk = (mk << 16) | uint(0x3333)
            w = ((w >> 2) & mk) | ((w & mk) << 2)
            mk = uint(0x0F0F)
            mk = (mk << 16) | uint(0x0F0F)
            w = ((w >> 4) & mk) | ((w & mk) << 4)
            mk = uint(0x00FF)
            mk = (mk << 16) | uint(0x00FF)
            w = ((w >> 8) & mk) | ((w & mk) << 8)
            w = (w >> 16) | ((w & uint(0xFFFF)) << 16)
            packed = (e << 4) | length
            k = int(w)
            st = 1 << length
            while k < size:
                ft[fo + k] = packed
                k += st
        else:
            # insertion sort as unsigned, for the binary search in _cw
            j = cnt
            while j > 0 and uint(lc[lo + j - 1]) > res:
                lc[lo + j] = lc[lo + j - 1]
                lv[lo + j] = lv[lo + j - 1]
                j -= 1
            lc[lo + j] = int(res)
            lv[lo + j] = (e << 5) | length
            cnt += 1
    return cnt


@micropython.viper
def vq_fill(iv_obj, nv: int, entries: int, dim: int, seq: int, lattice: int, vq_obj, vo: int):
    """Expand a VQ table (3.2.1, lookup types 1 and 2) of integer values iv into
    vq[vo:vo + entries * dim]."""
    iv = ptr32(iv_obj)
    vq = ptr32(vq_obj)
    k = vo
    q = 0
    for e in range(entries):
        last = 0
        rem = e
        for j in range(dim):
            if lattice:
                val = iv[rem % nv] + last
                rem = rem // nv
            else:
                val = iv[q] + last
                q += 1
            if seq:
                last = val
            vq[k] = val
            k += 1


@micropython.viper
def gain(pcm: ptr16, n: int, q: int):
    """Scale n 16-bit samples in place by q / 32768, for 0 <= q <= 32768."""
    i = 0
    while i < n:
        v = int(pcm[i])  # ptr16 loads are unsigned
        if v & 0x8000:
            v -= 0x10000
        pcm[i] = (v * q) >> 15
        i += 1


@micropython.viper
def address(buf) -> uint:
    """Address of a buffer, for checking whether it is in internal SRAM or PSRAM."""
    return uint(ptr8(buf))
