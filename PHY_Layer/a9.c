/*
 * node_a.c  —  ToF Ranging Initiator
 *
 * Two-phase detection strategy
 * ──────────────────────────────
 * Phase 1 (coarse): N_ACCUM-window incoherent accumulator, stepped every
 *   SCAN_STEP_MS.  Used only to confirm the pong is present.  No timing
 *   is taken from this phase.
 *
 * Phase 2 (fine): once Phase 1 triggers, walk BACKWARD through the ring
 *   buffer one FFT_SIZE sub-window at a time, finding the EARLIEST
 *   sub-window whose single-window SNR exceeds FINE_SNR_THRESH.
 *   The START sample of that sub-window is the pong arrival estimate.
 *
 * Timing model
 * ─────────────
 * t=0  is stamped immediately after hackrf_start_rx() returns (rx_arm_time).
 * Arrival time = rx_arm_time + arrival_sample / Fs
 * prop RTT     = arrival_time - rx_arm_time - NODE_B_DELAY_S
 * distance     = prop_rtt * c / 2 - TURNAROUND_CALIBRATION_M
 *
 * The remaining fixed offset (USB RX startup latency, HackRF FIFO) is
 * constant and absorbed by TURNAROUND_CALIBRATION_M.
 *
 * Calibration procedure
 * ──────────────────────
 * 1. Set TURNAROUND_CALIBRATION_M = 0, recompile.
 * 2. Place nodes touching. Run several shots. Note mean reported distance.
 * 3. Set TURNAROUND_CALIBRATION_M to that mean value. Recompile.
 * 4. Verify touching now reads ~0m.
 *
 * Build:
 *   gcc node_a.c -o node_a -lhackrf -lm
 * Requires fft.h in the same directory.
 */

#include "FFT.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <libhackrf/hackrf.h>

/* ── Configuration ──────────────────────────────────────────────────────── */
#define SAMPLE_RATE               20000000
#define CARRIER_FREQ              915000000ULL
#define TONE_OFFSET_HZ           -300000.0f
#define PULSE_MS                  500
#define TXVGA_GAIN                47
#define RXLNA_GAIN                16
#define RXVGA_GAIN                20

#define NODE_B_DELAY_MS           5000.0

/* Coarse scan (Phase 1) - confident detection, needs good freq resolution */
#define FFT_SIZE                  131072    /* 2^17, ~6.5ms/sub-win, 152Hz/bin */
#define N_ACCUM                   15        /* ~98ms integrated                 */
#define SCAN_STEP_MS              50
#define COARSE_SNR_THRESH         350.0f

/*
 * Fine scan (Phase 2) - presence/absence only, needs fine time resolution.
 * Smaller FFT = shorter sub-window = finer leading-edge quantization.
 * 16384 (2^14) @ 20MS/s = 0.82ms per sub-window = ~123km quantization.
 * Freq resolution = 20e6/16384 = 1221Hz -- still plenty to see the tone.
 * SNR per window is lower (less averaging) but 600x+ burst SNR has
 * enormous headroom. FINE_SNR_THRESH needs retuning -- run once and
 * observe single-window SNR values, set threshold between noise and burst.
 */
#define FINE_FFT_SIZE             16384     /* 2^14, ~0.82ms/sub-win           */
#define FINE_SNR_THRESH           200.0f     /* start here, retune from output  */

#define SCAN_CENTER_HZ           -334000.0f
#define SCAN_WINDOW_HZ            100000.0f
#define DC_REJECT_HZ              20000.0f

#define SPEED_OF_LIGHT            299792458.0
#define TURNAROUND_CALIBRATION_M  0.0      /* set after calibration run        */

#define RING_SECONDS              8        /* deeper ring to back-solve into   */
/* ──────────────────────────────────────────────────────────────────────── */

#define FREQ_RES        ((float)SAMPLE_RATE / FFT_SIZE)
#define FINE_FREQ_RES   ((float)SAMPLE_RATE / FINE_FFT_SIZE)
#define SUB_WIN_SEC     ((double)FINE_FFT_SIZE / SAMPLE_RATE)   /* ~0.82ms */

static int8_t  *tx_buf      = NULL;
static int      tx_total    = 0;
static volatile int  tx_done = 0;
static          int  tx_sent = 0;

static long     ring_samples;
static int8_t  *ring_buf    = NULL;
static volatile long ring_write = 0;
static volatile long samples_rx = 0;

static int      step_samples;
static double   rx_arm_time = 0.0;

static volatile int running = 1;
static void sigint_handler(int s) { (void)s; running = 0; }

static fft_state_t *fft      = NULL;   /* coarse: FFT_SIZE */
static fft_state_t *fine_fft = NULL;   /* fine:   FINE_FFT_SIZE */
static float       *hann     = NULL;   /* coarse window */
static float       *fine_hann= NULL;   /* fine window */
static float       *accum    = NULL;
static int          scan_bin_lo, scan_bin_hi;

/* ── Helpers ────────────────────────────────────────────────────────────── */
static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}
static inline int freq_to_bin(float hz) {
    int b = (int)roundf(hz / FREQ_RES);
    if (b < 0) b += FFT_SIZE;
    return b & (FFT_SIZE - 1);
}
static inline float bin_to_freq(int b) {
    if (b > FFT_SIZE / 2) b -= FFT_SIZE;
    return (float)b * FREQ_RES;
}

/* ── Callbacks ──────────────────────────────────────────────────────────── */
static int tx_callback(hackrf_transfer *t) {
    int8_t *out = (int8_t *)t->buffer;
    int     req = (int)t->buffer_length;
    if (tx_done) { memset(out, 0, req); return 0; }
    int rem = tx_total - tx_sent;
    if (rem >= req) { memcpy(out, tx_buf + tx_sent, req); tx_sent += req; }
    else {
        memcpy(out, tx_buf + tx_sent, rem);
        memset(out + rem, 0, req - rem);
        tx_sent += rem; tx_done = 1;
    }
    return 0;
}
static int rx_callback(hackrf_transfer *t) {
    if (!running) return 0;
    int8_t *in = (int8_t *)t->buffer;
    int     n  = (int)t->valid_length / 2;
    for (int i = 0; i < n; i++) {
        long pos          = ring_write % ring_samples;
        ring_buf[2*pos]   = in[2*i];
        ring_buf[2*pos+1] = in[2*i+1];
        ring_write++;
    }
    samples_rx += n;
    return 0;
}

/*
 * ── single_subwin_snr ────────────────────────────────────────────────────
 * Run one FFT on the sub-window starting at ring position `start`.
 * Returns SNR within the search band; sets *peak_freq_out.
 */
static float single_subwin_snr(long start, float *peak_freq_out) {
    for (int k = 0; k < FINE_FFT_SIZE; k++) {
        long pos             = (start + k) % ring_samples;
        float w              = fine_hann[k];
        fine_fft->buf[2*k]   = (float)ring_buf[2*pos]   * w;
        fine_fft->buf[2*k+1] = (float)ring_buf[2*pos+1] * w;
    }
    fft_execute(fine_fft);

    /* Search using fine freq resolution */
    float peak_p = 0.0f, total_p = 0.0f;
    int   n_valid = 0;
    int   fine_peak_bin = (int)roundf(SCAN_CENTER_HZ / FINE_FREQ_RES);
    if (fine_peak_bin < 0) fine_peak_bin += FINE_FFT_SIZE;
    fine_peak_bin &= (FINE_FFT_SIZE - 1);

    int fine_lo = (int)roundf((SCAN_CENTER_HZ - SCAN_WINDOW_HZ) / FINE_FREQ_RES);
    int fine_hi = (int)roundf((SCAN_CENTER_HZ + SCAN_WINDOW_HZ) / FINE_FREQ_RES);
    if (fine_lo < 0) fine_lo += FINE_FFT_SIZE;
    if (fine_hi < 0) fine_hi += FINE_FFT_SIZE;
    fine_lo &= (FINE_FFT_SIZE - 1);
    fine_hi &= (FINE_FFT_SIZE - 1);

    for (int b = fine_lo; b != (fine_hi + 1) % FINE_FFT_SIZE;
         b = (b + 1) % FINE_FFT_SIZE) {
        int bi = b & (FINE_FFT_SIZE - 1);
        float re = fine_fft->buf[2*bi], im = fine_fft->buf[2*bi+1];
        float p  = re*re + im*im;
        /* DC reject */
        float f = (bi > FINE_FFT_SIZE/2 ? bi - FINE_FFT_SIZE : bi) * FINE_FREQ_RES;
        if (fabsf(f) < DC_REJECT_HZ) continue;
        total_p += p; n_valid++;
        if (p > peak_p) { peak_p = p; fine_peak_bin = bi; }
    }
    if (peak_freq_out) {
        int pb = fine_peak_bin > FINE_FFT_SIZE/2 ? fine_peak_bin - FINE_FFT_SIZE : fine_peak_bin;
        *peak_freq_out = (float)pb * FINE_FREQ_RES;
    }
    if (n_valid == 0) return 0.0f;
    float mean = total_p / (float)n_valid;
    return (mean > 0.0f) ? peak_p / mean : 0.0f;
}

/*
 * ── coarse_scan ──────────────────────────────────────────────────────────
 * Phase 1: incoherent accumulation of N_ACCUM sub-windows.
 * Used only for confident detection — timing not taken from here.
 */
static float coarse_scan(float *peak_freq_out, int win_num) {
    long needed = (long)FFT_SIZE * N_ACCUM;
    if (ring_write < needed) { *peak_freq_out = SCAN_CENTER_HZ; return 0.0f; }

    memset(accum, 0, FFT_SIZE * sizeof(float));
    long base = ring_write - needed;

    for (int a = 0; a < N_ACCUM; a++) {
        long sub = base + (long)a * FFT_SIZE;
        for (int k = 0; k < FFT_SIZE; k++) {
            long pos        = (sub + k) % ring_samples;
            float w         = hann[k];
            fft->buf[2*k]   = (float)ring_buf[2*pos]   * w;
            fft->buf[2*k+1] = (float)ring_buf[2*pos+1] * w;
        }
        fft_execute(fft);
        for (int k = 0; k < FFT_SIZE; k++) {
            float re = fft->buf[2*k], im = fft->buf[2*k+1];
            accum[k] += re*re + im*im;
        }
    }

    float peak_p = 0.0f, total_p = 0.0f;
    int   peak_bin = freq_to_bin(SCAN_CENTER_HZ), n_valid = 0;
    for (int b = scan_bin_lo; b != (scan_bin_hi + 1) % FFT_SIZE;
         b = (b + 1) % FFT_SIZE) {
        float f = bin_to_freq(b);
        if (fabsf(f) < DC_REJECT_HZ) continue;
        float p = accum[b]; total_p += p; n_valid++;
        if (p > peak_p) { peak_p = p; peak_bin = b; }
    }

    float peak_f = bin_to_freq(peak_bin);
    *peak_freq_out = peak_f;
    if (n_valid == 0) return 0.0f;
    float mean = total_p / (float)n_valid;
    float snr  = (mean > 0.0f) ? peak_p / mean : 0.0f;

    if (snr > COARSE_SNR_THRESH * 0.5f || win_num % 50 == 0) {
        const int n_bar = 40;
        printf("  [win %4d]  peak: %+.0fkHz  SNR: %7.1fx  |",
               win_num, peak_f / 1e3, snr);
        float bar_step = (2.0f * SCAN_WINDOW_HZ) / n_bar;
        for (int i = 0; i < n_bar; i++) {
            float f = (SCAN_CENTER_HZ - SCAN_WINDOW_HZ) + (i + 0.5f) * bar_step;
            if (fabsf(f) < DC_REJECT_HZ) { putchar('_'); continue; }
            int b = freq_to_bin(f);
            float r = (peak_p > 0.0f) ? accum[b] / peak_p : 0.0f;
            putchar(r > 0.75f ? '#' : r > 0.40f ? '+' : r > 0.15f ? '.' : ' ');
        }
        printf("|\n"); fflush(stdout);
    }
    return snr;
}

/*
 * ── fine_backsolve ───────────────────────────────────────────────────────
 * Phase 2: find pong arrival sample via leading edge search.
 *
 * With FINE_SNR_THRESH calibrated above the noise floor (73x) and well
 * below burst SNR (600x+), we can reliably find the leading edge:
 *
 *   Walk newest→oldest. Collect all hot windows.
 *   The OLDEST hot window is the leading edge of the burst.
 *
 * The trailing edge approach failed because hackrf_stop_tx() drains
 * the USB FIFO non-deterministically, making burst *end* vary by 200ms.
 * The burst *start* is controlled by node B's clock_nanosleep deadline,
 * which is stable to ~100µs — so the leading edge is the right anchor.
 *
 * The leading edge sub-window will have partial-burst SNR (somewhere
 * between the noise floor and peak burst SNR). As long as FINE_SNR_THRESH
 * sits cleanly in the noise-floor→burst gap, this window is found
 * consistently.
 *
 * Returns estimated arrival sample (start of first hot sub-window).
 */
static long fine_backsolve(long trigger_ring_write, float *fine_freq_out) {
    long search_end = (trigger_ring_write / FINE_FFT_SIZE) * FINE_FFT_SIZE;
    long search_beg = search_end - (long)SAMPLE_RATE * RING_SECONDS + FINE_FFT_SIZE;
    if (search_beg < 0) search_beg = 0;
    search_beg = (search_beg / FINE_FFT_SIZE) * FINE_FFT_SIZE;

    long  leading_edge = search_end;   /* fallback */
    float best_freq    = SCAN_CENTER_HZ;
    int   found_edge   = 0;

    /* Walk newest→oldest, track the oldest hot window seen */
    for (long s = search_end; s >= search_beg; s -= FFT_SIZE) {
        float pf;
        float snr = single_subwin_snr(s, &pf);
        if (snr >= FINE_SNR_THRESH) {
            leading_edge = s;   /* keep updating — oldest hot window wins */
            best_freq    = pf;
            found_edge   = 1;
        } else if (found_edge) {
            /* Was hot, now cold — leading edge is one step forward in time.
             * Stop here; going further back is just noise. */
            break;
        }
        /* If not yet found_edge and cold, keep walking back (still in
         * post-burst noise before the burst arrived). */
    }

    if (!found_edge) {
        fprintf(stderr, "  [fine] WARNING: no hot window found\n");
    }

    /*
     * Phase interpolation for sub-window timing.
     *
     * Energy detection gives us leading_edge to +-FINE_FFT_SIZE samples
     * (~0.82ms). We can do much better using the phase of the tone.
     *
     * A pure tone at frequency f arriving at sample t_arr has phase:
     *   phi(n) = 2*pi*f*(n - t_arr)/Fs  for n >= t_arr, 0 before
     *
     * Running a Goertzel on the leading-edge window gives us the
     * accumulated phasor. For a full-window tone the phase at bin center
     * is phi(N/2). For a partial window starting at offset d samples in:
     *   measured_phase = 2*pi*f*(N/2 - d + t_arr_frac)/Fs  (approx)
     *
     * In practice we use the phase difference between two consecutive
     * windows: the leading-edge window (partial) and the next window
     * (full). The full window gives us a clean reference phase; the
     * partial window's phase deviation tells us how much of the window
     * was filled, i.e. the sub-window arrival offset.
     *
     * delta_phi = phi_partial - phi_full + pi*f*FINE_FFT_SIZE/Fs
     * fill_fraction = delta_phi / (2*pi*f*FINE_FFT_SIZE/Fs)
     * sub_sample_offset = fill_fraction * FINE_FFT_SIZE
     * arrival = leading_edge + FINE_FFT_SIZE - sub_sample_offset
     *         = leading_edge + (1 - fill_fraction) * FINE_FFT_SIZE
     *
     * This gives arrival timing to ~1/100th cycle of the tone frequency.
     * At 300kHz: 1 cycle = 1/300000 s = 3.33us = ~500m round trip = ~250m.
     * 1/100th cycle = ~2.5m one-way accuracy (vs ~120km from energy alone).
     */

    /* Goertzel on leading-edge window (partial burst) */
    float tone_hz = SCAN_CENTER_HZ;   /* the pong tone frequency */
    /* Use absolute value since freq is negative (below carrier) */
    /* Phase relationship works the same for negative offsets */

    double omega = 2.0 * M_PI * fabsf(tone_hz) / SAMPLE_RATE;
    double coeff = 2.0 * cos(omega);

    /* Goertzel on leading-edge window */
    double s0r=0,s1r=0,s2r=0, s0i=0,s1i=0,s2i=0;
    for (int k = 0; k < FINE_FFT_SIZE; k++) {
        long pos = (leading_edge + k) % ring_samples;
        double xr = ring_buf[2*pos],   xi = ring_buf[2*pos+1];
        s0r = xr + coeff*s1r - s2r; s2r = s1r; s1r = s0r;
        s0i = xi + coeff*s1i - s2i; s2i = s1i; s1i = s0i;
    }
    double re_part = s1r - s2r*cos(omega);
    double im_part = s1i - s2i*cos(omega);
    /* For complex IQ: combine I and Q phasors */
    double phi_partial = atan2(im_part - re_part*sin(omega),
                               re_part - s2r*sin(omega) +
                               im_part*cos(omega));

    /* Goertzel on the NEXT window (should be fully in burst) */
    long next_win = leading_edge + FINE_FFT_SIZE;
    s0r=0;s1r=0;s2r=0; s0i=0;s1i=0;s2i=0;
    for (int k = 0; k < FINE_FFT_SIZE; k++) {
        long pos = (next_win + k) % ring_samples;
        double xr = ring_buf[2*pos],   xi = ring_buf[2*pos+1];
        s0r = xr + coeff*s1r - s2r; s2r = s1r; s1r = s0r;
        s0i = xi + coeff*s1i - s2i; s2i = s1i; s1i = s0i;
    }
    double phi_full = atan2(im_part - s2i*sin(omega),
                            s1r - s2r*cos(omega));
    /* recompute cleanly for full window */
    {
        double fr = s1r - s2r*cos(omega);
        double fi = s1i - s2i*cos(omega);
        phi_full = atan2(fi, fr);
    }

    /*
     * Phase advance per full window = omega * FINE_FFT_SIZE (mod 2pi).
     * delta = phi_full - phi_partial - omega*FINE_FFT_SIZE
     * Normalise to [-pi, pi].
     * fill = 1 + delta / (omega * FINE_FFT_SIZE)
     * sub_offset = (1 - fill) * FINE_FFT_SIZE  samples from window start
     */
    double window_phase = omega * FINE_FFT_SIZE;
    double delta = phi_full - phi_partial - window_phase;
    /* wrap to [-pi, pi] */
    while (delta >  M_PI) delta -= 2*M_PI;
    while (delta < -M_PI) delta += 2*M_PI;

    double fill         = 1.0 + delta / window_phase;
    /* clamp fill to [0,1] — noise can push it slightly outside */
    if (fill < 0.0) fill = 0.0;
    if (fill > 1.0) fill = 1.0;

    /* sub-sample arrival: leading_edge window started (1-fill)*N samples
     * before the tone actually arrived */
    long phase_offset_samples = (long)((1.0 - fill) * FINE_FFT_SIZE);
    long phase_arrival = leading_edge + phase_offset_samples;

    printf("  [fine] energy edge @ %ldms  fill=%.3f  phase arrival @ %ldms",
           (long)((double)leading_edge / SAMPLE_RATE * 1e3),
           fill,
           (long)((double)phase_arrival / SAMPLE_RATE * 1e3));

    *fine_freq_out = best_freq;
    return phase_arrival;
}

/* ── TX buffer ──────────────────────────────────────────────────────────── */
static int build_tx_buffer(void) {
    int n = (int)((double)SAMPLE_RATE * PULSE_MS / 1e3);
    tx_total = n * 2;
    tx_buf   = (int8_t *)malloc(tx_total);
    if (!tx_buf) return -1;
    for (int i = 0; i < n; i++) {
        float ph     = 2.0f * (float)M_PI * TONE_OFFSET_HZ * i / SAMPLE_RATE;
        tx_buf[2*i]  = (int8_t)(cosf(ph) * 120.0f);
        tx_buf[2*i+1]= (int8_t)(sinf(ph) * 120.0f);
    }
    return 0;
}

/* ── main ───────────────────────────────────────────────────────────────── */
int main(void) {
    hackrf_device *device = NULL;
    int res;

    signal(SIGINT,  sigint_handler);
    signal(SIGTERM, sigint_handler);

    step_samples = (int)((double)SAMPLE_RATE * SCAN_STEP_MS / 1e3);
    ring_samples = (long)SAMPLE_RATE * RING_SECONDS;

    ring_buf = (int8_t *)calloc(ring_samples * 2, 1);
    if (!ring_buf) { fprintf(stderr, "malloc ring\n"); return 1; }
    if (build_tx_buffer()) { fprintf(stderr, "malloc tx\n"); return 1; }

    fft      = fft_init(FFT_SIZE);
    fine_fft = fft_init(FINE_FFT_SIZE);
    accum    = (float *)malloc(FFT_SIZE * sizeof(float));
    hann     = (float *)malloc(FFT_SIZE * sizeof(float));
    fine_hann= (float *)malloc(FINE_FFT_SIZE * sizeof(float));
    if (!fft || !fine_fft || !accum || !hann || !fine_hann) { fprintf(stderr, "malloc fft\n"); return 1; }

    for (int k = 0; k < FFT_SIZE; k++)
        hann[k] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * k / (FFT_SIZE - 1)));
    for (int k = 0; k < FINE_FFT_SIZE; k++)
        fine_hann[k] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * k / (FINE_FFT_SIZE - 1)));

    scan_bin_lo = freq_to_bin(SCAN_CENTER_HZ - SCAN_WINDOW_HZ);
    scan_bin_hi = freq_to_bin(SCAN_CENTER_HZ + SCAN_WINDOW_HZ);

    if (hackrf_init() != HACKRF_SUCCESS) { fprintf(stderr, "hackrf_init\n"); return 1; }
    if (hackrf_open(&device) != HACKRF_SUCCESS) { fprintf(stderr, "hackrf_open\n"); hackrf_exit(); return 1; }
    hackrf_set_sample_rate(device, SAMPLE_RATE);
    hackrf_set_freq(device, CARRIER_FREQ);

    printf("Node A — ranging initiator\n");
    printf("Coarse: %d×%d pts (%.0fms)  Step: %dms  Threshold: %.0fx\n",
           N_ACCUM, FFT_SIZE, (double)FFT_SIZE*N_ACCUM/SAMPLE_RATE*1e3,
           SCAN_STEP_MS, (double)COARSE_SNR_THRESH);
    printf("Fine: single sub-win %.1fms  Threshold: %.1fx\n",
           SUB_WIN_SEC*1e3, (double)FINE_SNR_THRESH);
    printf("TURNAROUND_CALIBRATION_M = %.3f\n\n", TURNAROUND_CALIBRATION_M);
    printf("Press Enter to range (Ctrl-C to quit)...\n");

    long first_scan_at = (long)FFT_SIZE * N_ACCUM;

    while (running) {
        if (getchar() == EOF || !running) break;
        printf("\n[ranging...]\n");

        /* ── TX ── */
        tx_done = 0; tx_sent = 0;
        hackrf_set_txvga_gain(device, TXVGA_GAIN);
        hackrf_set_antenna_enable(device, 1);
        res = hackrf_start_tx(device, tx_callback, NULL);
        if (res != HACKRF_SUCCESS) { fprintf(stderr, "start_tx\n"); break; }
        while (!tx_done && running) usleep(200);
        usleep(3000);
        hackrf_stop_tx(device);
        usleep(5000);   /* let HackRF settle before RX arm */
        printf("  ping sent\n");
        if (!running) break;

        /* ── RX arm ── */
        hackrf_set_lna_gain(device, RXLNA_GAIN);
        hackrf_set_vga_gain(device, RXVGA_GAIN);
        hackrf_set_antenna_enable(device, 0);
        ring_write = 0; samples_rx = 0;
        memset(ring_buf, 0, ring_samples * 2);
        res = hackrf_start_rx(device, rx_callback, NULL);
        if (res != HACKRF_SUCCESS) { fprintf(stderr, "start_rx\n"); break; }

        rx_arm_time = now_sec();   /* t=0: after stop_tx drained, RX running */
        printf("  RX armed (t=0) — pong in ~%.0fs\n\n", NODE_B_DELAY_MS/1e3);

        /* Dead zone */
        double scan_open = rx_arm_time + (NODE_B_DELAY_MS/1e3) - 0.5;
        while (now_sec() < scan_open && running) usleep(10000);
        if (!running) { hackrf_stop_rx(device); break; }
        printf("  scan window open\n");

        /* ── Phase 1: coarse scan ── */
        long next_scan_at = samples_rx + first_scan_at;
        int  window = 0, detected = 0;

        while (!detected && running) {
            while (samples_rx < next_scan_at && running) usleep(500);
            if (!running) break;
            next_scan_at += step_samples;

            float peak_freq;
            float snr = coarse_scan(&peak_freq, ++window);

            if (snr >= COARSE_SNR_THRESH) {
                /*
                 * Snapshot ring_write immediately — before stopping RX,
                 * before any further samples arrive.
                 */
                long trigger_rw = ring_write;
                hackrf_stop_rx(device);

                printf("\n  [coarse] PONG  win=%d  SNR=%.0fx  peak=%+.0fkHz\n",
                       window, snr, peak_freq/1e3);

                /* ── Phase 2: fine back-solve ── */
                float fine_freq;
                long arrival_sample = fine_backsolve(trigger_rw, &fine_freq);

                double prop_rtt = (double)arrival_sample / SAMPLE_RATE
                                  - (NODE_B_DELAY_MS / 1e3);
                double distance = prop_rtt * SPEED_OF_LIGHT / 2.0
                                  - TURNAROUND_CALIBRATION_M;

                printf("  arrival sample:  %ld  (%.3f ms after RX arm)\n",
                       arrival_sample, (double)arrival_sample/SAMPLE_RATE*1e3);
                printf("  prop RTT:        %.3f ms\n",  prop_rtt*1e3);
                printf("  Distance:        %.2f m\n\n", distance);
                detected = 1;
            }
        }

        if (!detected && !running) hackrf_stop_rx(device);
        usleep(3000);
        if (running) printf("Press Enter to range again (Ctrl-C to quit)...\n");
    }

    hackrf_close(device);
    hackrf_exit();
    fft_free(fft); fft_free(fine_fft);
    free(accum); free(hann); free(fine_hann); free(tx_buf); free(ring_buf);
    printf("Node A stopped.\n");
    return EXIT_SUCCESS;
}
