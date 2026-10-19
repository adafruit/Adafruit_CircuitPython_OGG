// SPDX-FileCopyrightText: Copyright (c) 2026 Mikey Sklar for Adafruit Industries
// SPDX-License-Identifier: MIT
//
// ogg_native: the adafruit_ogg kernel (src/ogg_turbo.py) as a native C module, for chips
// where turbo code is not fast enough for 44.1 kHz stereo (Arm and xtensawin). Same functions, arguments,
// tables and fixed point formats as the turbo kernel; see ogg_turbo.py for the layouts.
// Products are 32 x 32 -> 64 bit, so results can differ from the turbo kernel by an LSB.
//
// Build: make -C src/native MPY_DIR=path/to/circuitpython ARCH=armv7emsp

#include "py/dynruntime.h"

#define BK 12

static inline uint32_t peek32(const uint8_t *d, int pos) {
    const uint8_t *p = d + (pos >> 3);
    int sh = pos & 7;
    uint32_t w = ((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
        ((uint32_t)p[3] << 24)) >> sh;
    if (sh) {
        w |= (uint32_t)p[4] << (32 - sh);
    }
    return w;
}

static inline uint32_t rev32(uint32_t w) {
    w = ((w >> 1) & 0x55555555u) | ((w & 0x55555555u) << 1);
    w = ((w >> 2) & 0x33333333u) | ((w & 0x33333333u) << 2);
    w = ((w >> 4) & 0x0F0F0F0Fu) | ((w & 0x0F0F0F0Fu) << 4);
    w = ((w >> 8) & 0x00FF00FFu) | ((w & 0x00FF00FFu) << 8);
    return (w >> 16) | (w << 16);
}

typedef struct {
    const int32_t *bk;
    const uint16_t *ft;
    const int32_t *lc;
    const int32_t *lv;
} books_t;

// One codeword of book b at *pos; -1 if invalid.
static int cw(const uint8_t *d, int *pos, const books_t *B, int b) {
    const int32_t *h = B->bk + b * BK;
    uint32_t w = peek32(d, *pos);
    int fb = h[1];
    if (fb) {
        unsigned e = B->ft[h[0] + (w & ((1u << fb) - 1))];
        if (e != 0xFFFF) {
            *pos += e & 15;
            return e >> 4;
        }
    }
    int n = h[3];
    if (n == 0) {
        return -1;
    }
    w = rev32(w);
    const uint32_t *lc = (const uint32_t *)B->lc + h[2];
    int x = 0;
    while (n > 1) {
        int half = n >> 1;
        if (lc[x + half] <= w) {
            x += half;
            n -= half;
        } else {
            n = half;
        }
    }
    int32_t ev = B->lv[h[2] + x];
    int ln = ev & 31;
    if ((w >> (32 - ln)) != (lc[x] >> (32 - ln))) {
        return -1;
    }
    *pos += ln;
    return ev >> 5;
}

static void *buf_w(mp_obj_t o) {
    mp_buffer_info_t bi;
    mp_get_buffer_raise(o, &bi, MP_BUFFER_RW);
    return bi.buf;
}

static const void *buf_r(mp_obj_t o) {
    mp_buffer_info_t bi;
    mp_get_buffer_raise(o, &bi, MP_BUFFER_READ);
    return bi.buf;
}

static inline int bits_at(const uint8_t *d, int pos, int n) {
    return (int)(peek32(d, pos) & ((1u << n) - 1));
}

// floor1(d, pos, nbits, fl, bk, ft, lc, lv, wk, yb, n2, st) -> pos
static mp_obj_t floor1(size_t n_args, const mp_obj_t *args) {
    const uint8_t *d = buf_r(args[0]);
    int pos = mp_obj_get_int(args[1]);
    int nbits = mp_obj_get_int(args[2]);
    const int32_t *fl = buf_r(args[3]);
    books_t B = {buf_r(args[4]), buf_r(args[5]), buf_r(args[6]), buf_r(args[7])};
    int32_t *wk = buf_w(args[8]);
    uint8_t *yb = buf_w(args[9]);
    int n2 = mp_obj_get_int(args[10]);
    int32_t *st = buf_w(args[11]);
    st[1] = 0;
    if (pos >= nbits) {
        return mp_obj_new_int(pos);
    }
    if (!((d[pos >> 3] >> (pos & 7)) & 1)) {
        return mp_obj_new_int(pos + 1);
    }
    pos += 1;
    int nx = fl[0], mult = fl[1], rng = fl[2], ybits = fl[3], parts = fl[4];
    const int32_t *pc = fl + fl[6], *cl = fl + fl[7], *xs = fl + fl[8];
    const int32_t *lo = fl + fl[9], *hi = fl + fl[10], *ord = fl + fl[11];
    int32_t *ys = wk, *fy = wk + 256, *step2 = wk + 512;
    ys[0] = bits_at(d, pos, ybits);
    pos += ybits;
    ys[1] = bits_at(d, pos, ybits);
    pos += ybits;
    int k = 2;
    for (int p = 0; p < parts; p++) {
        const int32_t *c = cl + pc[p] * 12;
        int cdim = c[0], csub = c[1];
        int cval = 0;
        if (csub) {
            cval = cw(d, &pos, &B, c[2]);
            if (cval < 0) {
                return mp_obj_new_int(nbits + 1);
            }
        }
        int cmask = (1 << csub) - 1;
        for (int j = 0; j < cdim; j++) {
            int book = c[4 + (cval & cmask)];
            cval >>= csub;
            int y = 0;
            if (book >= 0) {
                y = cw(d, &pos, &B, book);
                if (y < 0) {
                    return mp_obj_new_int(nbits + 1);
                }
            }
            ys[k++] = y;
        }
    }
    if (pos > nbits) {
        return mp_obj_new_int(pos);
    }
    // 7.2.4 step 1: amplitude values
    fy[0] = ys[0];
    fy[1] = ys[1];
    step2[0] = step2[1] = 1;
    for (int i = 2; i < nx; i++) {
        step2[i] = 0;
    }
    for (int i = 2; i < nx; i++) {
        int l = lo[i], hh = hi[i];
        int x0 = xs[l], y0 = fy[l];
        int dy = fy[hh] - y0;
        int adx = xs[hh] - x0;
        int ady = dy < 0 ? -dy : dy;
        int off = (ady * (xs[i] - x0)) / adx;
        int pred = dy < 0 ? y0 - off : y0 + off;
        int val = ys[i];
        int hroom = rng - pred, lroom = pred;
        int room = (hroom < lroom ? hroom : lroom) * 2;
        if (val) {
            step2[l] = step2[hh] = step2[i] = 1;
            int v;
            if (val >= room) {
                v = hroom > lroom ? val - lroom + pred : pred - val + hroom - 1;
            } else if (val & 1) {
                v = pred - ((val + 1) >> 1);
            } else {
                v = pred + (val >> 1);
            }
            if (v < 0) {
                v = 0;
            } else if (v >= rng) {
                v = rng - 1;
            }
            fy[i] = v;
        } else {
            fy[i] = pred;
        }
    }
    // step 2: line segments between the used points, in x order (9.2.7)
    int lx = 0, ly = fy[ord[0]] * mult, hx = 0, hy = ly;
    for (int kk = 1; kk <= nx; kk++) {
        int x1;
        if (kk < nx) {
            int q = ord[kk];
            if (!step2[q]) {
                continue;
            }
            hx = xs[q];
            hy = fy[q] * mult;
            x1 = hx;
        } else {
            if (hx >= n2) {
                break;
            }
            lx = hx;
            ly = hy;
            x1 = n2;
        }
        if (lx < n2) {
            int dy = hy - ly, adx = x1 - lx;
            int ady = dy < 0 ? -dy : dy;
            int base = ady / adx, sy;
            if (dy < 0) {
                base = -base;
                sy = base - 1;
            } else {
                sy = base + 1;
            }
            ady -= (base < 0 ? -base : base) * adx;
            int y = ly, err = 0;
            yb[lx] = y;
            int xe = x1 > n2 ? n2 : x1;
            for (int x = lx + 1; x < xe; x++) {
                err += ady;
                if (err >= adx) {
                    err -= adx;
                    y += sy;
                } else {
                    y += base;
                }
                yb[x] = y;
            }
        }
        lx = hx;
        ly = hy;
    }
    st[1] = 1;
    return mp_obj_new_int(pos);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(floor1_obj, 12, 12, floor1);

// residue(d, pos, nbits, rs, bk, ft, lc, lv, vq, v, ch, dnd, n2, cls, st) -> pos
static mp_obj_t residue(size_t n_args, const mp_obj_t *args) {
    const uint8_t *d = buf_r(args[0]);
    int pos = mp_obj_get_int(args[1]);
    int nbits = mp_obj_get_int(args[2]);
    const int32_t *rs = buf_r(args[3]);
    books_t B = {buf_r(args[4]), buf_r(args[5]), buf_r(args[6]), buf_r(args[7])};
    const int32_t *vq = buf_r(args[8]);
    int32_t *v = buf_w(args[9]);
    int ch = mp_obj_get_int(args[10]);
    int dnd = mp_obj_get_int(args[11]);
    int n2 = mp_obj_get_int(args[12]);
    uint8_t *cls = buf_w(args[13]);
    int total = ch * n2;
    for (int i = 0; i < total; i++) {
        v[i] = 0;
    }
    int rtype = rs[0], size = n2, nch = ch, dmask = dnd;
    if (rtype == 2) {
        if (dnd == (1 << ch) - 1) {
            return mp_obj_new_int(pos);
        }
        size = total;
        nch = 1;
        dmask = 0;
    }
    int begin = rs[1] > size ? size : rs[1];
    int end = rs[2] > size ? size : rs[2];
    int psize = rs[3], nclass = rs[4], cbook = rs[5];
    int nparts = (end - begin) / psize;
    if (nparts <= 0) {
        return mp_obj_new_int(pos);
    }
    int cpw = B.bk[cbook * BK + 4];
    int stride = nparts + cpw;
    for (int p = 0; p < 8; p++) {
        int part = 0;
        while (part < nparts) {
            if (p == 0) {
                for (int c = 0; c < nch; c++) {
                    if ((dmask >> c) & 1) {
                        continue;
                    }
                    int t = cw(d, &pos, &B, cbook);
                    if (t < 0 || pos > nbits) {
                        return mp_obj_new_int(nbits + 1);
                    }
                    for (int i = cpw - 1; i >= 0; i--) {
                        cls[c * stride + part + i] = t % nclass;
                        t /= nclass;
                    }
                }
            }
            for (int jj = 0; jj < cpw && part < nparts; jj++, part++) {
                for (int c = 0; c < nch; c++) {
                    if ((dmask >> c) & 1) {
                        continue;
                    }
                    int bkn = rs[6 + cls[c * stride + part] * 8 + p];
                    if (bkn < 0) {
                        continue;
                    }
                    const int32_t *h = B.bk + bkn * BK;
                    int dim = h[4], lt = h[6], nv = h[8], seq = h[9];
                    const int32_t *vb = vq + h[7];
                    int off = begin + part * psize;
                    int stop = off + psize;
                    int step = psize / dim;
                    int k = off, i = 0;
                    while (k < stop) {
                        int e = cw(d, &pos, &B, bkn);
                        if (e < 0) {
                            return mp_obj_new_int(nbits + 1);
                        }
                        if (pos > nbits) {
                            return mp_obj_new_int(pos);
                        }
                        int32_t last = 0;
                        const int32_t *q = vb + e * dim;
                        int rem = e;
                        for (int j = 0; j < dim; j++) {
                            int32_t val;
                            if (lt == 2) {
                                val = q[j];
                            } else {
                                val = vb[rem % nv] + last;
                                rem /= nv;
                                if (seq) {
                                    last = val;
                                }
                            }
                            int idx;
                            if (rtype == 0) {
                                idx = c * n2 + off + i + j * step;
                            } else if (rtype == 1) {
                                idx = c * n2 + k++;
                            } else if (ch == 2) {
                                idx = (k & 1) * n2 + (k >> 1);
                                k++;
                            } else if (ch == 1) {
                                idx = k++;
                            } else {
                                idx = (k % ch) * n2 + k / ch;
                                k++;
                            }
                            v[idx] += val;
                        }
                        if (rtype == 0) {
                            i++;
                            k = off + i * dim;
                        }
                    }
                }
            }
        }
    }
    return mp_obj_new_int(pos);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(residue_obj, 15, 15, residue);

// couple(v, mo, ao, n2)
static mp_obj_t couple(size_t n_args, const mp_obj_t *args) {
    int32_t *v = buf_w(args[0]);
    int32_t *mg = v + mp_obj_get_int(args[1]);
    int32_t *an = v + mp_obj_get_int(args[2]);
    int n2 = mp_obj_get_int(args[3]);
    for (int i = 0; i < n2; i++) {
        int32_t m = mg[i], a = an[i];
        if (m > 0) {
            if (a > 0) {
                an[i] = m - a;
            } else {
                an[i] = m;
                mg[i] = m + a;
            }
        } else if (a > 0) {
            an[i] = m + a;
        } else {
            an[i] = m;
            mg[i] = m - a;
        }
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(couple_obj, 4, 4, couple);

// floor_apply(v, off, yb, yoff, tm, ts, n2)
static mp_obj_t floor_apply(size_t n_args, const mp_obj_t *args) {
    int32_t *v = (int32_t *)buf_w(args[0]) + mp_obj_get_int(args[1]);
    const uint8_t *yb = (const uint8_t *)buf_r(args[2]) + mp_obj_get_int(args[3]);
    const int32_t *tm = buf_r(args[4]);
    const int32_t *ts = buf_r(args[5]);
    int n2 = mp_obj_get_int(args[6]);
    for (int i = 0; i < n2; i++) {
        int32_t r = v[i];
        if (r) {
            int y = yb[i];
            int64_t p = (int64_t)r * tm[y];
            int s = ts[y];
            v[i] = (int32_t)(s >= 0 ? p >> s : p << -s);
        }
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(floor_apply_obj, 7, 7, floor_apply);

static inline int32_t mul30(int32_t x, int32_t wh, int32_t wl) {
    return (int32_t)(((int64_t)x * (wh * 32768 + wl)) >> 30);
}

// imdct(v, off, tw, n, re, im): DCT-IV half, in place; overlap() unfolds and windows it
static mp_obj_t imdct(size_t n_args, const mp_obj_t *args) {
    int32_t *v = (int32_t *)buf_w(args[0]) + mp_obj_get_int(args[1]);
    const int32_t *tw = buf_r(args[2]);
    int n = mp_obj_get_int(args[3]);
    int32_t *re = buf_w(args[4]);
    int32_t *im = buf_w(args[5]);
    int m = n >> 1, h = n >> 2;
    const int32_t *pre = tw, *post = tw + 4 * h, *ftw = tw + 8 * h, *rev = tw + 10 * h;
    // pre-twiddle into bit-reversed order
    for (int j = 0; j < h; j++) {
        int32_t a = v[2 * j], b = v[m - 1 - 2 * j];
        const int32_t *t = pre + 4 * j;
        int64_t c = t[0] * 32768 + t[1], s = t[2] * 32768 + t[3];
        int r = rev[j];
        re[r] = (int32_t)((a * c - b * s) >> 30);
        im[r] = (int32_t)((a * s + b * c) >> 30);
    }
    // radix-2 FFT, size h
    for (int size = 2, sstep = h >> 1; size <= h; size <<= 1, sstep >>= 1) {
        int half = size >> 1;
        for (int k = 0; k < half; k++) {
            const int32_t *t = ftw + 4 * k * sstep;
            int64_t wre = t[0] * 32768 + t[1], wim = t[2] * 32768 + t[3];
            for (int j = k; j < h; j += size) {
                int lj = j + half;
                int32_t tr, ti;
                if (k == 0) {
                    tr = re[lj];
                    ti = im[lj];
                } else {
                    int64_t rl = re[lj], il = im[lj];
                    tr = (int32_t)((rl * wre - il * wim) >> 30);
                    ti = (int32_t)((il * wre + rl * wim) >> 30);
                }
                int32_t rj = re[j], ij = im[j];
                re[lj] = rj - tr;
                im[lj] = ij - ti;
                re[j] = rj + tr;
                im[j] = ij + ti;
            }
        }
    }
    // post-twiddle into u = v[0:m]
    for (int j = 0; j < h; j++) {
        int64_t a = re[j], b = im[j];
        const int32_t *t = post + 4 * j;
        int64_t c = t[0] * 32768 + t[1], s = t[2] * 32768 + t[3];
        v[2 * j] = (int32_t)((a * c - b * s) >> 30);
        v[m - 1 - 2 * j] = -(int32_t)((a * s + b * c) >> 30);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(imdct_obj, 6, 6, imdct);

// Windowed y[j] of a block of n, unfolded from the DCT-IV output u (4.3.1)
static inline int32_t unfold(const int32_t *u, int j, int n, const int32_t *wp,
    const int32_t *wl, const int32_t *wr) {
    int m = n >> 1, h = n >> 2;
    int32_t x = j < h ? u[j + h] : j < m + h ? -u[m + h - 1 - j] : -u[j - m - h];
    int ls = wp[0], ll = wp[1], rs = wp[2], rl = wp[3];
    if (j < ls || j >= rs + rl) {
        return 0;
    }
    if (j < ls + ll) {
        const int32_t *w = wl + 2 * (j - ls);
        return mul30(x, w[0], w[1]);
    }
    if (j >= rs) {
        const int32_t *w = wr + 2 * (rs + rl - 1 - j);
        return mul30(x, w[0], w[1]);
    }
    return x;
}

// overlap(prev, poff, v, voff, hasy, pcm, ch, c, length, off, n, pn2, wp, wl, wr)
static mp_obj_t overlap(size_t n_args, const mp_obj_t *args) {
    int32_t *prev = (int32_t *)buf_w(args[0]) + mp_obj_get_int(args[1]);
    const int32_t *u = (const int32_t *)buf_r(args[2]) + mp_obj_get_int(args[3]);
    int hasy = mp_obj_get_int(args[4]);
    int16_t *pcm = buf_w(args[5]);
    int ch = mp_obj_get_int(args[6]);
    int c = mp_obj_get_int(args[7]);
    int length = mp_obj_get_int(args[8]);
    int off = mp_obj_get_int(args[9]);
    int n = mp_obj_get_int(args[10]);
    int pn2 = mp_obj_get_int(args[11]);
    const int32_t *wp = buf_r(args[12]);
    const int32_t *wl = buf_r(args[13]);
    const int32_t *wr = buf_r(args[14]);
    int16_t *o = pcm + c;
    for (int q = 0; q < length; q++, o += ch) {
        int32_t val = q < pn2 ? prev[q] : 0;
        int j = q - off;
        if (hasy && j >= 0 && j < n) {
            val += unfold(u, j, n, wp, wl, wr);
        }
        int32_t s = (val + 256) >> 9;
        *o = s > 32767 ? 32767 : s < -32768 ? -32768 : s;
    }
    int n2 = n >> 1;
    for (int i = 0; i < n2; i++) {
        prev[i] = hasy ? unfold(u, n2 + i, n, wp, wl, wr) : 0;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(overlap_obj, 15, 15, overlap);

// book_count(lengths, n, fb) -> number of codewords longer than fb bits
static mp_obj_t book_count(mp_obj_t l_in, mp_obj_t n_in, mp_obj_t fb_in) {
    const uint8_t *ln = buf_r(l_in);
    int n = mp_obj_get_int(n_in), fb = mp_obj_get_int(fb_in), c = 0;
    for (int e = 0; e < n; e++) {
        c += ln[e] > fb;
    }
    return mp_obj_new_int(c);
}
static MP_DEFINE_CONST_FUN_OBJ_3(book_count_obj, book_count);

// book_tables(lengths, n, fb, ft, fo, lc, lv, lo, av) -> long codewords, -1 if overspecified
static mp_obj_t book_tables(size_t n_args, const mp_obj_t *args) {
    const uint8_t *ln = buf_r(args[0]);
    int n = mp_obj_get_int(args[1]), fb = mp_obj_get_int(args[2]);
    uint16_t *ft = (uint16_t *)buf_w(args[3]) + mp_obj_get_int(args[4]);
    int lo = mp_obj_get_int(args[7]);
    uint32_t *lc = (uint32_t *)buf_w(args[5]) + lo;
    int32_t *lv = (int32_t *)buf_w(args[6]) + lo;
    uint32_t av[33] = {0};
    int used = 0, last = 0, size = 1 << fb;
    for (int e = 0; e < n; e++) {
        if (ln[e]) {
            used++;
            last = e;
        }
    }
    if (fb) {
        for (int k = 0; k < size; k++) {
            ft[k] = 0xFFFF;
        }
    }
    if (used == 1) {
        int length = ln[last];
        if (fb) {
            for (int k = 0; k < size; k++) {
                ft[k] = (last << 4) | (length > fb ? fb : length);
            }
            return mp_obj_new_int(0);
        }
        lc[0] = 0;
        lv[0] = (last << 5) | length;
        return mp_obj_new_int(1);
    }
    int cnt = 0;
    bool first = true;
    for (int e = 0; e < n; e++) {
        int length = ln[e];
        if (!length) {
            continue;
        }
        uint32_t res;
        if (first) {
            first = false;
            for (int i = 1; i <= length; i++) {
                av[i] = 1u << (32 - i);
            }
            res = 0;
        } else {
            int z = length;
            while (z > 0 && !av[z]) {
                z--;
            }
            if (z == 0) {
                return mp_obj_new_int(-1);
            }
            res = av[z];
            av[z] = 0;
            for (int y = length; y > z; y--) {
                av[y] = res + (1u << (32 - y));
            }
        }
        if (length <= fb) {
            int packed = (e << 4) | length;
            for (int k = rev32(res); k < size; k += 1 << length) {
                ft[k] = packed;
            }
        } else {
            int j = cnt++;
            while (j > 0 && lc[j - 1] > res) {
                lc[j] = lc[j - 1];
                lv[j] = lv[j - 1];
                j--;
            }
            lc[j] = res;
            lv[j] = (e << 5) | length;
        }
    }
    return mp_obj_new_int(cnt);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(book_tables_obj, 9, 9, book_tables);

// vq_fill(iv, nv, entries, dim, seq, lattice, vq, vo)
static mp_obj_t vq_fill(size_t n_args, const mp_obj_t *args) {
    const int32_t *iv = buf_r(args[0]);
    int nv = mp_obj_get_int(args[1]), entries = mp_obj_get_int(args[2]);
    int dim = mp_obj_get_int(args[3]), seq = mp_obj_get_int(args[4]);
    int lattice = mp_obj_get_int(args[5]);
    int32_t *vq = (int32_t *)buf_w(args[6]) + mp_obj_get_int(args[7]);
    int q = 0;
    for (int e = 0; e < entries; e++) {
        int32_t last = 0;
        int rem = e;
        for (int j = 0; j < dim; j++) {
            int32_t val;
            if (lattice) {
                val = iv[rem % nv] + last;
                rem /= nv;
            } else {
                val = iv[q++] + last;
            }
            if (seq) {
                last = val;
            }
            *vq++ = val;
        }
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(vq_fill_obj, 8, 8, vq_fill);

// gain(pcm, n, q): scale n 16-bit samples in place by q / 32768, for 0 <= q <= 32768
static mp_obj_t gain(mp_obj_t pcm_in, mp_obj_t n_in, mp_obj_t q_in) {
    int16_t *pcm = (int16_t *)buf_w(pcm_in);
    int n = mp_obj_get_int(n_in), q = mp_obj_get_int(q_in);
    for (int i = 0; i < n; i++) {
        pcm[i] = (int16_t)((pcm[i] * q) >> 15);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(gain_obj, gain);

static mp_obj_t address(mp_obj_t buf) {
    return mp_obj_new_int_from_uint((uintptr_t)buf_r(buf));
}
static MP_DEFINE_CONST_FUN_OBJ_1(address_obj, address);

mp_obj_t mpy_init(mp_obj_fun_bc_t *self, size_t n_args, size_t n_kw, mp_obj_t *args) {
    MP_DYNRUNTIME_INIT_ENTRY
    mp_store_global(MP_QSTR_floor1, MP_OBJ_FROM_PTR(&floor1_obj));
    mp_store_global(MP_QSTR_residue, MP_OBJ_FROM_PTR(&residue_obj));
    mp_store_global(MP_QSTR_couple, MP_OBJ_FROM_PTR(&couple_obj));
    mp_store_global(MP_QSTR_floor_apply, MP_OBJ_FROM_PTR(&floor_apply_obj));
    mp_store_global(MP_QSTR_imdct, MP_OBJ_FROM_PTR(&imdct_obj));
    mp_store_global(MP_QSTR_overlap, MP_OBJ_FROM_PTR(&overlap_obj));
    mp_store_global(MP_QSTR_book_count, MP_OBJ_FROM_PTR(&book_count_obj));
    mp_store_global(MP_QSTR_book_tables, MP_OBJ_FROM_PTR(&book_tables_obj));
    mp_store_global(MP_QSTR_vq_fill, MP_OBJ_FROM_PTR(&vq_fill_obj));
    mp_store_global(MP_QSTR_gain, MP_OBJ_FROM_PTR(&gain_obj));
    mp_store_global(MP_QSTR_address, MP_OBJ_FROM_PTR(&address_obj));
    MP_DYNRUNTIME_INIT_EXIT
}
