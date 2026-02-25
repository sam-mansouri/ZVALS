/*
 * fft.h — Self-contained radix-2 DIT (decimation-in-time) FFT
 *
 * No external dependencies beyond <math.h>.
 * Complex samples stored as interleaved float pairs: buf[2k]=real, buf[2k+1]=imag.
 *
 * Usage:
 *   1. Call fft_init(N) once with your transform size (must be power of 2).
 *      Returns a heap-allocated fft_state_t * — check for NULL.
 *   2. Fill state->buf[0..2N-1] with windowed IQ samples (re, im interleaved).
 *   3. Call fft_execute(state) — result is in-place in state->buf.
 *   4. bin k holds frequency (k * Fs / N) Hz, or (k - N) * Fs / N for k > N/2.
 *   5. Call fft_free(state) when done.
 *
 * Twiddle table: N/2 complex exponentials W_N^k = e^{-j2πk/N}, precomputed
 * once so the hot path is pure multiply-add with no trig calls.
 *
 * Bit-reversal permutation is also precomputed into state->bitrev[].
 */

#pragma once

#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    int      N;          /* transform size (power of 2)                    */
    int      log2N;      /* number of butterfly stages                     */
    float   *buf;        /* interleaved IQ working buffer [2N floats]      */
    float   *twiddle;    /* precomputed twiddles [N floats]: re0,im0,re1,… */
    int     *bitrev;     /* bit-reversal permutation table [N ints]        */
} fft_state_t;

/* ── fft_init ───────────────────────────────────────────────────────────── */
static inline fft_state_t *fft_init(int N) {
    /* Verify power of 2 */
    if (N < 2 || (N & (N - 1)) != 0) return NULL;

    fft_state_t *s = (fft_state_t *)malloc(sizeof(fft_state_t));
    if (!s) return NULL;

    s->N       = N;
    s->log2N   = 0;
    s->buf     = NULL;
    s->twiddle = NULL;
    s->bitrev  = NULL;

    /* log2(N) */
    for (int tmp = N; tmp > 1; tmp >>= 1) s->log2N++;

    s->buf     = (float *)malloc(2 * N * sizeof(float));
    s->twiddle = (float *)malloc(N * sizeof(float));   /* N/2 complex = N floats */
    s->bitrev  = (int   *)malloc(N * sizeof(int));
    if (!s->buf || !s->twiddle || !s->bitrev) {
        free(s->buf); free(s->twiddle); free(s->bitrev); free(s);
        return NULL;
    }

    /*
     * Twiddle factors W_N^k = e^{-j2πk/N} for k = 0 … N/2-1.
     * Stored as: twiddle[2k] = cos(-2πk/N), twiddle[2k+1] = sin(-2πk/N).
     * Only N/2 unique twiddles are needed (symmetry), but we allocate N
     * floats so indexing is twiddle[2k] / twiddle[2k+1].
     */
    for (int k = 0; k < N / 2; k++) {
        double angle       = -2.0 * M_PI * k / N;
        s->twiddle[2*k]    = (float)cos(angle);
        s->twiddle[2*k+1]  = (float)sin(angle);
    }

    /*
     * Bit-reversal permutation.
     * bitrev[i] = bit-reversal of i with log2N bits.
     */
    for (int i = 0; i < N; i++) {
        int x = i, r = 0;
        for (int b = 0; b < s->log2N; b++) {
            r = (r << 1) | (x & 1);
            x >>= 1;
        }
        s->bitrev[i] = r;
    }

    return s;
}

/* ── fft_free ───────────────────────────────────────────────────────────── */
static inline void fft_free(fft_state_t *s) {
    if (!s) return;
    free(s->buf);
    free(s->twiddle);
    free(s->bitrev);
    free(s);
}

/* ── fft_execute ────────────────────────────────────────────────────────── */
/*
 * In-place radix-2 DIT FFT on s->buf[0..2N-1].
 * Caller fills s->buf before calling; result is in s->buf on return.
 *
 * Algorithm: Cooley-Tukey, decimation-in-time.
 *   1. Bit-reversal shuffle.
 *   2. log2(N) butterfly stages, each doubling the DFT size.
 *      At stage p (butterfly span = 2^p):
 *        for each group of 2^p pairs:
 *          twiddle index = k * (N / span)
 *          butterfly: t = W * odd_half
 *                     out_even = even_half + t
 *                     out_odd  = even_half - t
 */
static inline void fft_execute(fft_state_t *s) {
    const int N = s->N;
    float    *x = s->buf;

    /* ── Step 1: bit-reversal permutation ── */
    for (int i = 0; i < N; i++) {
        int j = s->bitrev[i];
        if (j > i) {
            /* Swap complex sample i ↔ j */
            float tr = x[2*i], ti = x[2*i+1];
            x[2*i]   = x[2*j];   x[2*i+1] = x[2*j+1];
            x[2*j]   = tr;        x[2*j+1] = ti;
        }
    }

    /* ── Step 2: butterfly stages ── */
    for (int stage = 1; stage <= s->log2N; stage++) {
        int span      = 1 << stage;       /* size of each butterfly group  */
        int half_span = span >> 1;        /* N/2 pairs per group           */
        int twiddle_stride = N / span;    /* step through twiddle table    */

        for (int k = 0; k < N; k += span) {
            for (int j = 0; j < half_span; j++) {
                /* Twiddle W_N^(j * twiddle_stride) */
                int   ti  = j * twiddle_stride;
                float wr  = s->twiddle[2*ti];
                float wi  = s->twiddle[2*ti+1];

                /* Even and odd element indices */
                int e = k + j;
                int o = k + j + half_span;

                float er = x[2*e],   ei = x[2*e+1];
                float or_ = x[2*o],  oi = x[2*o+1];

                /* t = W * odd */
                float tr = wr*or_ - wi*oi;
                float ti2 = wr*oi  + wi*or_;

                /* Butterfly output */
                x[2*e]   = er + tr;
                x[2*e+1] = ei + ti2;
                x[2*o]   = er - tr;
                x[2*o+1] = ei - ti2;
            }
        }
    }
}
