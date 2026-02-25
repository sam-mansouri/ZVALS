/*
 * node_b.c  —  ToF Ranging Responder  (radix-2 FFT, back-solved deadline)
 *
 * Key fix over previous version:
 *   The turnaround delay deadline is now anchored to the ESTIMATED PING
 *   ARRIVAL TIME, not to the wall clock at scan trigger.
 *
 *   Old approach:
 *     clock_gettime(&deadline);           // "now" = whenever scan triggered
 *     deadline += NODE_B_DELAY_MS;        // varies by up to one scan window
 *
 *   New approach:
 *     ping_arrival = back_solve_ping();   // estimated leading edge sample
 *     deadline = rx_arm_time + ping_arrival/Fs + NODE_B_DELAY_MS
 *
 *   This removes the scan-window jitter from the turnaround delay, making
 *   the pong fire at a consistent time relative to ping arrival regardless
 *   of when in the scan window the trigger happened.
 *
 * Build:
 *   gcc node_b.c -o node_b -lhackrf -lm -lpthread
 * Requires fft.h in the same directory.
 */

#include "FFT.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <libhackrf/hackrf.h>

/* ── Configuration ──────────────────────────────────────────────────────── */
#define SAMPLE_RATE          20000000
#define CARRIER_FREQ         915000000ULL
#define PULSE_MS             500
#define TXVGA_GAIN           47
#define RXLNA_GAIN           16
#define RXVGA_GAIN           20
#define NODE_B_DELAY_MS      5000

#define FFT_SIZE             131072
#define N_ACCUM              15
#define SCAN_STEP_MS         50

/* Ping tone from node A */
#define SCAN_CENTER_HZ      -300000.0f
#define SCAN_WINDOW_HZ       100000.0f
#define DC_REJECT_HZ         20000.0f
#define COARSE_SNR_THRESH    350.0f

/*
 * FINE_SNR_THRESH: single sub-window threshold for ping back-solve.
 * Must sit above noise floor and below in-burst SNR.
 * Tune the same way as on node A: noise floor observed ~73x → use 90x.
 */
#define FINE_FFT_SIZE        16384     /* 2^14, ~0.82ms/sub-win */
#define FINE_SNR_THRESH      100.0f     /* retune from observed noise floor */

#define RING_SECONDS         8
/* ──────────────────────────────────────────────────────────────────────── */

#define FREQ_RES  ((float)SAMPLE_RATE / FFT_SIZE)

static int      step_samples;
static long     ring_samples;
static int8_t  *ring_buf    = NULL;
static volatile long ring_write  = 0;
static volatile long samples_rx  = 0;

static int8_t  *tx_buf      = NULL;
static int      tx_total    = 0;
static volatile int  tx_done = 0;
static          int  tx_sent = 0;

static float    detected_offset_hz = SCAN_CENTER_HZ;
static volatile int running = 1;
static void sigint_handler(int s) { (void)s; running = 0; }

static fft_state_t *fft      = NULL;
static fft_state_t *fine_fft = NULL;
static float       *hann     = NULL;
static float       *fine_hann= NULL;
static float       *accum    = NULL;
static int          scan_bin_lo, scan_bin_hi;

/* Wall clock at the moment hackrf_start_rx() returns each cycle */
static double rx_arm_time = 0.0;

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
static void ts_add_ms(struct timespec *ts, long ms) {
    ts->tv_nsec += ms * 1000000L;
    ts->tv_sec  += ts->tv_nsec / 1000000000L;
    ts->tv_nsec %= 1000000000L;
}
static void ts_add_us(struct timespec *ts, long us) {
    ts->tv_nsec += us * 1000L;
    ts->tv_sec  += ts->tv_nsec / 1000000000L;
    ts->tv_nsec %= 1000000000L;
}

/* ── Callbacks ──────────────────────────────────────────────────────────── */
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

/* ── Single sub-window SNR ──────────────────────────────────────────────── */
static float single_subwin_snr(long start, float *peak_freq_out) {
    float fine_freq_res = (float)SAMPLE_RATE / FINE_FFT_SIZE;
    for (int k = 0; k < FINE_FFT_SIZE; k++) {
        long pos             = (start + k) % ring_samples;
        float w              = fine_hann[k];
        fine_fft->buf[2*k]   = (float)ring_buf[2*pos]   * w;
        fine_fft->buf[2*k+1] = (float)ring_buf[2*pos+1] * w;
    }
    fft_execute(fine_fft);

    float peak_p = 0.0f, total_p = 0.0f;
    int   fine_peak_bin = 0, n_valid = 0;
    for (int b = 0; b < FINE_FFT_SIZE; b++) {
        float f = (b > FINE_FFT_SIZE/2 ? b - FINE_FFT_SIZE : b) * fine_freq_res;
        if (fabsf(f) < DC_REJECT_HZ) continue;
        if (fabsf(f - SCAN_CENTER_HZ) > SCAN_WINDOW_HZ) continue;
        float re = fine_fft->buf[2*b], im = fine_fft->buf[2*b+1];
        float p  = re*re + im*im;
        total_p += p; n_valid++;
        if (p > peak_p) { peak_p = p; fine_peak_bin = b; }
    }
    if (peak_freq_out) {
        int pb = fine_peak_bin > FINE_FFT_SIZE/2 ? fine_peak_bin - FINE_FFT_SIZE : fine_peak_bin;
        *peak_freq_out = (float)pb * fine_freq_res;
    }
    if (n_valid == 0) return 0.0f;
    float mean = total_p / (float)n_valid;
    return (mean > 0.0f) ? peak_p / mean : 0.0f;
}

/* ── Coarse scan (Phase 1) ──────────────────────────────────────────────── */
static float coarse_scan(float *peak_freq_out) {
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
    *peak_freq_out = bin_to_freq(peak_bin);
    if (n_valid == 0) return 0.0f;
    float mean = total_p / (float)n_valid;
    return (mean > 0.0f) ? peak_p / mean : 0.0f;
}

/*
 * ── back_solve_ping ──────────────────────────────────────────────────────
 * Find the leading edge of the ping burst in the ring buffer.
 * Same algorithm as node A's fine_backsolve:
 *   Walk newest→oldest, track oldest hot sub-window.
 *   Stop when hot→cold transition is found.
 * Returns leading edge sample index.
 */
static long back_solve_ping(long trigger_rw) {
    long search_end = (trigger_rw / FINE_FFT_SIZE) * FINE_FFT_SIZE;
    long search_beg = search_end - (long)SAMPLE_RATE * RING_SECONDS + FINE_FFT_SIZE;
    if (search_beg < 0) search_beg = 0;
    search_beg = (search_beg / FINE_FFT_SIZE) * FINE_FFT_SIZE;

    long leading_edge = search_end;
    int  found_edge   = 0;

    for (long s = search_end; s >= search_beg; s -= FINE_FFT_SIZE) {
        float pf;
        float snr = single_subwin_snr(s, &pf);
        if (snr >= FINE_SNR_THRESH) {
            leading_edge = s;
            found_edge   = 1;
        } else if (found_edge) {
            break;   /* cold after hot — leading edge found */
        }
    }

    printf("  [back-solve] ping leading edge @ %ldms after RX arm\n",
           (long)((double)leading_edge / SAMPLE_RATE * 1e3));
    return leading_edge;
}

/* ── Fire pong ──────────────────────────────────────────────────────────── */
static int fire_pong(hackrf_device *device) {
    int pulse_samples = (int)((double)SAMPLE_RATE * PULSE_MS / 1e3);
    int buf_len = pulse_samples * 2;
    if (!tx_buf) tx_buf = (int8_t *)malloc(buf_len);
    if (!tx_buf) return -1;
    tx_total = buf_len;

    for (int n = 0; n < pulse_samples; n++) {
        float ph      = 2.0f * (float)M_PI * detected_offset_hz * n / SAMPLE_RATE;
        tx_buf[2*n]   = (int8_t)(cosf(ph) * 120.0f);
        tx_buf[2*n+1] = (int8_t)(sinf(ph) * 120.0f);
    }
    tx_done = 0; tx_sent = 0;

    hackrf_set_txvga_gain(device, TXVGA_GAIN);
    hackrf_set_antenna_enable(device, 1);
    int res = hackrf_start_tx(device, tx_callback, NULL);
    if (res != HACKRF_SUCCESS) { fprintf(stderr, "start_tx: %s\n", hackrf_error_name(res)); return -1; }
    while (!tx_done && running) usleep(200);
    usleep(5000);
    hackrf_stop_tx(device);
    usleep(3000);
    return 0;
}

/* ── Arm RX ─────────────────────────────────────────────────────────────── */
static int arm_rx(hackrf_device *device) {
    hackrf_set_lna_gain(device, RXLNA_GAIN);
    hackrf_set_vga_gain(device, RXVGA_GAIN);
    hackrf_set_antenna_enable(device, 0);
    ring_write = 0; samples_rx = 0;
    memset(ring_buf, 0, ring_samples * 2);
    int res = hackrf_start_rx(device, rx_callback, NULL);
    rx_arm_time = now_sec();
    return res;
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

    printf("Node B — responder (back-solved deadline)\n");
    printf("FFT: %d pts  Res: %.0f Hz  Accum: %d  Step: %dms\n",
           FFT_SIZE, (double)FREQ_RES, N_ACCUM, SCAN_STEP_MS);
    printf("Coarse thresh: %.0fx  Fine thresh: %.0fx  Delay: %dms\n\n",
           (double)COARSE_SNR_THRESH, (double)FINE_SNR_THRESH, NODE_B_DELAY_MS);

    res = arm_rx(device);
    if (res != HACKRF_SUCCESS) { fprintf(stderr, "start_rx: %s\n", hackrf_error_name(res)); goto cleanup; }

    long first_scan_at = (long)FFT_SIZE * N_ACCUM;
    long next_scan_at  = first_scan_at;
    int  window = 0, cycle = 0;

    while (running) {
        while (samples_rx < next_scan_at && running) usleep(100);
        if (!running) break;
        next_scan_at += step_samples;

        float peak_freq;
        float snr = coarse_scan(&peak_freq);
        window++;

        if (window % 20 == 0 || snr > COARSE_SNR_THRESH * 0.5f) {
            printf("  [win %6d]  SNR: %6.1fx  peak: %+.0fkHz\n",
                   window, snr, peak_freq / 1e3);
            fflush(stdout);
        }

        if (snr >= COARSE_SNR_THRESH) {
            detected_offset_hz = peak_freq;
            long trigger_rw = ring_write;
            hackrf_stop_rx(device);
            ++cycle;

            printf("\n[cycle %d] PING detected  SNR=%.0fx  peak=%+.0fkHz\n",
                   cycle, snr, peak_freq / 1e3);

            /*
             * Back-solve to find when the ping actually arrived.
             * Build an absolute deadline from that estimated arrival time
             * rather than from "now", eliminating scan-window jitter.
             *
             *   ping_arrival_wall = rx_arm_time + leading_edge_sample / Fs
             *   deadline          = ping_arrival_wall + NODE_B_DELAY_MS
             */
            long ping_sample = back_solve_ping(trigger_rw);
            double ping_arrival_wall = rx_arm_time + (double)ping_sample / SAMPLE_RATE;

            struct timespec deadline;
            deadline.tv_sec  = (time_t)ping_arrival_wall;
            deadline.tv_nsec = (long)((ping_arrival_wall - deadline.tv_sec) * 1e9);
            ts_add_ms(&deadline, NODE_B_DELAY_MS);

            /* Sanity: if deadline is already past (shouldn't happen with
             * a 5s delay and sub-second back-solve), fire immediately. */
            struct timespec now_ts;
            clock_gettime(CLOCK_MONOTONIC, &now_ts);
            double deadline_sec = deadline.tv_sec + deadline.tv_nsec * 1e-9;
            double now_wall     = now_ts.tv_sec   + now_ts.tv_nsec   * 1e-9;
            printf("  deadline in %.1f ms\n", (deadline_sec - now_wall) * 1e3);
            fflush(stdout);

            if (deadline_sec > now_wall) {
                while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                                       &deadline, NULL) != 0 && running)
                    ;
            }
            if (!running) break;

            printf("  firing pong at %+.0fkHz\n", detected_offset_hz / 1e3);
            fflush(stdout);
            fire_pong(device);
            printf("  pong sent. back to RX.\n\n");
            fflush(stdout);

            res = arm_rx(device);
            if (res != HACKRF_SUCCESS) { fprintf(stderr, "start_rx: %s\n", hackrf_error_name(res)); break; }
            while (samples_rx < 1000 && running) usleep(1000);
            next_scan_at = samples_rx + first_scan_at;
            window = 0;
        }
    }

cleanup:
    hackrf_stop_rx(device);
    hackrf_close(device);
    hackrf_exit();
    fft_free(fft); fft_free(fine_fft);
    free(accum); free(hann); free(fine_hann); free(tx_buf); free(ring_buf);
    printf("Node B stopped.\n");
    return EXIT_SUCCESS;
}

