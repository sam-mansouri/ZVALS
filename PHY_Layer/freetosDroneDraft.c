/*
 * node_b_freertos.c  —  ToF Ranging Responder
 *                        FreeRTOS port for STM32F4 Discovery
 *                        HackRF One via USB Full Speed @ 2MS/s
 *
 * Architecture
 * ─────────────
 * Three tasks, matching node_b.c's logical sections:
 *
 *   usb_rx_task      (highest priority)
 *     Calls hackrf_start_rx(), services USB transfers via libhackrf's
 *     internal thread (which maps to a FreeRTOS task via the POSIX-on-
 *     FreeRTOS shim).  Writes IQ bytes into the ring buffer exactly as
 *     the Linux rx_callback did.  Posts a RX_CHUNK_READY notification
 *     to scan_detect_task every SCAN_NOTIFY_SAMPLES samples.
 *
 *   scan_detect_task (medium priority)
 *     Waits for RX_CHUNK_READY notifications, runs the coarse FFT scan,
 *     and on detection: runs back_solve_ping(), computes the absolute
 *     deadline, then sends a DetectEvent to pong_queue.
 *
 *   pong_task        (medium priority)
 *     Blocks on pong_queue.  On receipt: calls vTaskDelayUntil() to the
 *     precomputed absolute deadline, then fires the pong TX.
 *     Returns to idle (scan_detect_task re-arms RX).
 *
 * Timing
 * ───────
 * FreeRTOS tick (configTICK_RATE_HZ) gives ~1ms resolution for
 * vTaskDelayUntil.  The back-solve anchors the deadline to the estimated
 * ping arrival sample (same as Linux version), so scan-window jitter is
 * removed.  Residual jitter = 1 tick = 1ms = ~150km — same order as the
 * Linux usleep version before the clock_nanosleep fix, and adequate for
 * the coarse timing stage.  For sub-ms precision, a hardware timer capture
 * triggered by a GPIO from node A (if wired) can be substituted.
 *
 * Memory
 * ───────
 * STM32F407 has 192KB SRAM.  Budget:
 *   Ring buffer : RING_SECONDS(4) * 2MS/s * 2 bytes = 16 MB  ← DOES NOT FIT
 *
 * Solution: reduce RING_SECONDS to 1 (4MB) — still doesn't fit in SRAM.
 * Use external SDRAM if available, or reduce further:
 *   RING_SAMPLES = 500ms * 2MS/s = 1,000,000 samples * 2 bytes = 2MB
 *   → still too large for internal SRAM alone.
 *
 * Practical approach for STM32F4 (no external RAM):
 *   RING_SAMPLES = 131072 samples * 2 bytes = 256KB  (fits in SRAM + CCM)
 *   This gives ~65ms of ring depth at 2MS/s.
 *   The ping burst is 500ms, so the ring only holds the most recent 65ms.
 *   The back-solve window is therefore limited to 65ms — about 10 coarse
 *   FFT sub-windows deep.  Still enough to find the ping leading edge.
 *
 *   If you have the STM32F429 Discovery (has 8MB SDRAM), map ring_buf
 *   to the SDRAM region and increase RING_SAMPLES accordingly.
 *
 * FFT memory (FFT_SIZE=16384):
 *   fft_state buf    : 16384 * 2 * 4 = 131KB
 *   twiddle          : 16384 * 4     =  64KB
 *   bitrev           : 16384 * 4     =  64KB
 *   accum            : 16384 * 4     =  64KB
 *   hann (coarse)    : 16384 * 4     =  64KB
 *   fine hann        :  2048 * 4     =   8KB
 *   Total FFT        :               ~ 395KB  ← exceeds internal SRAM
 *
 * Mitigation: store twiddle and hann tables in Flash (const).
 *   See TWIDDLE_IN_FLASH note below.
 *
 * Stack sizes
 * ────────────
 *   usb_rx_task      : 512 words (2KB)  — minimal, just callback dispatch
 *   scan_detect_task : 4096 words (16KB) — FFT runs on this stack indirectly
 *                                          via heap-allocated fft_state
 *   pong_task        : 512 words (2KB)
 *
 * FreeRTOS configuration requirements (FreeRTOSConfig.h)
 * ───────────────────────────────────────────────────────
 *   configUSE_TASK_NOTIFICATIONS     1
 *   configUSE_TIMERS                 1   (for absolute deadline)
 *   configTICK_RATE_HZ               1000
 *   configTOTAL_HEAP_SIZE            at least 512*1024
 *   configUSE_MUTEXES                1
 *   configSUPPORT_DYNAMIC_ALLOCATION 1
 *
 * libhackrf on FreeRTOS
 * ──────────────────────
 * libhackrf uses libusb-1.0 which uses pthreads.  You need either:
 *   (a) FreeRTOS-POSIX (amazon-freertos/libraries/abstractions/posix)
 *   (b) libusb-stm32 with a custom FreeRTOS backend
 * Option (a) is the path of least resistance if you're already using
 * an RTOS abstraction layer.  Option (b) requires porting libusb's
 * event loop to use FreeRTOS tasks and queues directly.
 *
 * This file assumes option (a): FreeRTOS-POSIX is available and
 * pthread_create/mutex/cond map to FreeRTOS primitives.
 *
 * Build (arm-none-eabi-gcc, add to your STM32CubeIDE Makefile):
 *   arm-none-eabi-gcc node_b_freertos.c \
 *     -I$(FREERTOS_INC) -I$(FREERTOS_POSIX_INC) -I$(LIBHACKRF_INC) \
 *     -lhackrf -lm -lnosys \
 *     -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
 *     -O2 -ffunction-sections -fdata-sections
 */

#include "fft.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

/* libhackrf (via FreeRTOS-POSIX shim) */
#include <hackrf.h>

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>   /* printf via semihosting or UART retarget */

/* ── Configuration ──────────────────────────────────────────────────────── */
#define SAMPLE_RATE          2000000        /* 2MS/s — USB FS limit          */
#define CARRIER_FREQ         915000000ULL
#define PULSE_MS             500
#define TXVGA_GAIN           47
#define RXLNA_GAIN           16
#define RXVGA_GAIN           20
#define NODE_B_DELAY_MS      5000

/* Coarse FFT scan — same algorithm as node_b.c */
#define FFT_SIZE             16384          /* 2^14, ~8.2ms/win, 122Hz/bin   */
#define N_ACCUM              12             /* ~98ms integrated               */
#define SCAN_STEP_MS         50
#define COARSE_SNR_THRESH    350.0f

/* Fine back-solve */
#define FINE_FFT_SIZE        2048           /* 2^11, ~1.02ms/sub-win         */
#define FINE_SNR_THRESH      15.0f

#define SCAN_CENTER_HZ      -300000.0f      /* node A's ping tone             */
#define SCAN_WINDOW_HZ       80000.0f       /* ±80kHz — narrower at 2MS/s    */
#define DC_REJECT_HZ         10000.0f

/*
 * Ring buffer sizing for STM32F407 internal SRAM.
 * 131072 samples × 2 bytes = 256KB.  Fits in the 192KB SRAM + 64KB CCM
 * if you place it with __attribute__((section(".ccmram"))).
 * See linker script note below.
 */
#define RING_SAMPLES         131072U        /* ~65ms at 2MS/s                */

/* How many new samples trigger a scan-task notification */
#define SCAN_NOTIFY_SAMPLES  ((SAMPLE_RATE * SCAN_STEP_MS) / 1000)

/* FreeRTOS task priorities */
#define PRIORITY_USB_RX      (configMAX_PRIORITIES - 1)   /* highest        */
#define PRIORITY_SCAN        (configMAX_PRIORITIES - 2)
#define PRIORITY_PONG        (configMAX_PRIORITIES - 2)

/* FreeRTOS stack sizes (words) */
#define STACK_USB_RX         512
#define STACK_SCAN           4096
#define STACK_PONG           512
/* ──────────────────────────────────────────────────────────────────────── */

#define FREQ_RES        ((float)SAMPLE_RATE / FFT_SIZE)
#define FINE_FREQ_RES   ((float)SAMPLE_RATE / FINE_FFT_SIZE)

/* ── Ring buffer ─────────────────────────────────────────────────────────
 *
 * Placed in CCM RAM (zero-wait-state on F4) for best throughput.
 * Add to your linker script:
 *   .ccmram (NOLOAD) : { *(.ccmram) } >CCMRAM
 */
static int8_t ring_buf[RING_SAMPLES * 2]
    __attribute__((section(".ccmram")));

/*
 * ring_write: monotonically increasing sample counter.
 * Written only by usb_rx_task (via rx_callback).
 * Read by scan_detect_task and pong_task.
 * Declared volatile; no mutex needed for single-writer/multi-reader
 * as long as reads are treated as approximate (they are).
 */
static volatile uint32_t ring_write  = 0;
static volatile uint32_t samples_rx  = 0;

/* ── Shared state ────────────────────────────────────────────────────────── */
static hackrf_device *g_device        = NULL;
static float          detected_offset = SCAN_CENTER_HZ;
static volatile int   running         = 1;

/*
 * rx_arm_tick: FreeRTOS tick count at the moment hackrf_start_rx() returns.
 * Used to convert sample indices to tick-based absolute deadlines.
 */
static volatile TickType_t rx_arm_tick = 0;

/* ── FFT state ───────────────────────────────────────────────────────────── */
static fft_state_t *g_fft      = NULL;   /* coarse: FFT_SIZE               */
static fft_state_t *g_fine_fft = NULL;   /* fine:   FINE_FFT_SIZE          */
static float       *g_hann     = NULL;   /* coarse Hanning window          */
static float       *g_fine_hann= NULL;   /* fine Hanning window            */
static float       *g_accum    = NULL;   /* incoherent accumulator         */
static int          scan_bin_lo, scan_bin_hi;

/* ── TX state ────────────────────────────────────────────────────────────── */
static int8_t      *tx_buf     = NULL;
static int          tx_total   = 0;
static volatile int tx_done    = 0;
static          int tx_sent    = 0;

/* ── IPC ─────────────────────────────────────────────────────────────────── */

/*
 * detect_queue: scan_detect_task → pong_task.
 * Carries everything pong_task needs to compute its deadline.
 */
typedef struct {
    float    snr;
    float    peak_freq_hz;
    uint32_t trigger_ring_write;   /* ring_write snapshot at coarse trigger  */
} DetectEvent_t;

static QueueHandle_t     pong_queue    = NULL;

/*
 * scan_notify: usb_rx_task notifies scan_detect_task when enough new
 * samples have arrived.  Uses FreeRTOS direct-to-task notification
 * (no queue overhead, no dynamic allocation).
 */
static TaskHandle_t      scan_task_handle = NULL;

/* ── Helpers ─────────────────────────────────────────────────────────────── */
static inline int freq_to_bin(float hz, int N) {
    int b = (int)roundf(hz / ((float)SAMPLE_RATE / N));
    if (b < 0) b += N;
    return b & (N - 1);
}
static inline float bin_to_freq(int b, int N) {
    if (b > N / 2) b -= N;
    return (float)b * ((float)SAMPLE_RATE / N);
}

/* ── Callbacks ───────────────────────────────────────────────────────────── */

/*
 * rx_callback — called from libhackrf's USB transfer thread.
 *
 * Matches node_b.c rx_callback exactly.  The only addition is the
 * task notification to wake scan_detect_task when enough samples arrive.
 *
 * Called from a pthread (FreeRTOS-POSIX task) at high rate — keep it fast.
 * No FreeRTOS blocking calls allowed here; use xTaskNotifyFromISR-style
 * FromISR variants if called from true ISR context, but with POSIX shim
 * this is a regular task context so standard notify is fine.
 */
static int rx_callback(hackrf_transfer *transfer) {
    if (!running) return 0;

    int8_t *in = (int8_t *)transfer->buffer;
    int     n  = (int)transfer->valid_length / 2;

    for (int i = 0; i < n; i++) {
        uint32_t pos          = ring_write % RING_SAMPLES;
        ring_buf[2*pos]       = in[2*i];
        ring_buf[2*pos + 1]   = in[2*i + 1];
        ring_write++;
    }
    samples_rx += (uint32_t)n;

    /*
     * Notify scan task every SCAN_NOTIFY_SAMPLES new samples.
     * Use eSetValueWithOverwrite so a slow scan task doesn't accumulate
     * a backlog of notifications — it just sees "data is ready".
     */
    static uint32_t last_notify = 0;
    if (samples_rx - last_notify >= SCAN_NOTIFY_SAMPLES) {
        last_notify = samples_rx;
        if (scan_task_handle) {
            BaseType_t higher_prio_woken = pdFALSE;
            xTaskNotifyFromISR(scan_task_handle, 0,
                               eSetValueWithOverwrite, &higher_prio_woken);
            portYIELD_FROM_ISR(higher_prio_woken);
        }
    }
    return 0;
}

static int tx_callback(hackrf_transfer *transfer) {
    int8_t *out = (int8_t *)transfer->buffer;
    int     req = (int)transfer->buffer_length;
    if (tx_done) { memset(out, 0, req); return 0; }
    int rem = tx_total - tx_sent;
    if (rem >= req) {
        memcpy(out, tx_buf + tx_sent, req); tx_sent += req;
    } else {
        memcpy(out, tx_buf + tx_sent, rem);
        memset(out + rem, 0, req - rem);
        tx_sent += rem; tx_done = 1;
    }
    return 0;
}

/* ── Single sub-window SNR (fine FFT) ───────────────────────────────────── */
static float single_subwin_snr(uint32_t start, float *peak_freq_out) {
    float fine_freq_res = FINE_FREQ_RES;

    for (int k = 0; k < FINE_FFT_SIZE; k++) {
        uint32_t pos          = (start + k) % RING_SAMPLES;
        float    w            = g_fine_hann[k];
        g_fine_fft->buf[2*k]   = (float)ring_buf[2*pos]   * w;
        g_fine_fft->buf[2*k+1] = (float)ring_buf[2*pos+1] * w;
    }
    fft_execute(g_fine_fft);

    float peak_p = 0.0f, total_p = 0.0f;
    int   fine_peak_bin = 0, n_valid = 0;

    for (int b = 0; b < FINE_FFT_SIZE; b++) {
        float f = (b > FINE_FFT_SIZE/2 ? b - FINE_FFT_SIZE : b) * fine_freq_res;
        if (fabsf(f) < DC_REJECT_HZ) continue;
        if (fabsf(f - SCAN_CENTER_HZ) > SCAN_WINDOW_HZ) continue;
        float re = g_fine_fft->buf[2*b], im = g_fine_fft->buf[2*b+1];
        float p  = re*re + im*im;
        total_p += p; n_valid++;
        if (p > peak_p) { peak_p = p; fine_peak_bin = b; }
    }

    if (peak_freq_out) {
        int pb = fine_peak_bin > FINE_FFT_SIZE/2
                 ? fine_peak_bin - FINE_FFT_SIZE : fine_peak_bin;
        *peak_freq_out = (float)pb * fine_freq_res;
    }
    if (n_valid == 0) return 0.0f;
    float mean = total_p / (float)n_valid;
    return (mean > 0.0f) ? peak_p / mean : 0.0f;
}

/* ── Coarse FFT scan ─────────────────────────────────────────────────────── */
/*
 * Identical logic to node_b.c coarse_scan().
 * N_ACCUM sub-windows incoherently accumulated.
 */
static float coarse_scan(float *peak_freq_out) {
    uint32_t needed = (uint32_t)FFT_SIZE * N_ACCUM;
    if (ring_write < needed) { *peak_freq_out = SCAN_CENTER_HZ; return 0.0f; }

    memset(g_accum, 0, FFT_SIZE * sizeof(float));
    uint32_t base = ring_write - needed;

    for (int a = 0; a < N_ACCUM; a++) {
        uint32_t sub = base + (uint32_t)a * FFT_SIZE;
        for (int k = 0; k < FFT_SIZE; k++) {
            uint32_t pos     = (sub + k) % RING_SAMPLES;
            float    w       = g_hann[k];
            g_fft->buf[2*k]   = (float)ring_buf[2*pos]   * w;
            g_fft->buf[2*k+1] = (float)ring_buf[2*pos+1] * w;
        }
        fft_execute(g_fft);
        for (int k = 0; k < FFT_SIZE; k++) {
            float re = g_fft->buf[2*k], im = g_fft->buf[2*k+1];
            g_accum[k] += re*re + im*im;
        }
    }

    float peak_p = 0.0f, total_p = 0.0f;
    int   peak_bin = freq_to_bin(SCAN_CENTER_HZ, FFT_SIZE), n_valid = 0;

    for (int b = scan_bin_lo; b != (scan_bin_hi + 1) % FFT_SIZE;
         b = (b + 1) % FFT_SIZE) {
        float f = bin_to_freq(b, FFT_SIZE);
        if (fabsf(f) < DC_REJECT_HZ) continue;
        float p = g_accum[b]; total_p += p; n_valid++;
        if (p > peak_p) { peak_p = p; peak_bin = b; }
    }

    *peak_freq_out = bin_to_freq(peak_bin, FFT_SIZE);
    if (n_valid == 0) return 0.0f;
    float mean = total_p / (float)n_valid;
    return (mean > 0.0f) ? peak_p / mean : 0.0f;
}

/* ── Ping back-solve ─────────────────────────────────────────────────────── */
/*
 * Identical logic to node_b.c back_solve_ping().
 * Returns estimated leading-edge sample index (ring-relative).
 */
static uint32_t back_solve_ping(uint32_t trigger_rw) {
    uint32_t search_end = (trigger_rw / FINE_FFT_SIZE) * FINE_FFT_SIZE;
    /* Cap search at ring depth */
    uint32_t max_back   = RING_SAMPLES - FINE_FFT_SIZE;
    uint32_t search_beg = (search_end > max_back)
                          ? search_end - max_back : 0;
    search_beg = (search_beg / FINE_FFT_SIZE) * FINE_FFT_SIZE;

    uint32_t leading_edge = search_end;
    int      found_edge   = 0;

    for (uint32_t s = search_end; s >= search_beg + FINE_FFT_SIZE;
         s -= FINE_FFT_SIZE) {
        float pf;
        float snr = single_subwin_snr(s, &pf);
        if (snr >= FINE_SNR_THRESH) {
            leading_edge = s;
            found_edge   = 1;
        } else if (found_edge) {
            break;
        }
    }

    printf("  [B back-solve] ping leading edge @ %lu samples\n",
           (unsigned long)leading_edge);
    return leading_edge;
}

/* ── Arm RX ──────────────────────────────────────────────────────────────── */
static int arm_rx(void) {
    hackrf_set_lna_gain(g_device, RXLNA_GAIN);
    hackrf_set_vga_gain(g_device, RXVGA_GAIN);
    hackrf_set_antenna_enable(g_device, 0);
    ring_write = 0;
    samples_rx = 0;
    memset(ring_buf, 0, sizeof(ring_buf));
    int res = hackrf_start_rx(g_device, rx_callback, NULL);
    rx_arm_tick = xTaskGetTickCount();
    return res;
}

/* ── Fire pong ───────────────────────────────────────────────────────────── */
static void fire_pong(void) {
    int pulse_samples = (int)((double)SAMPLE_RATE * PULSE_MS / 1e3);
    int buf_len = pulse_samples * 2;

    if (!tx_buf) tx_buf = (int8_t *)pvPortMalloc(buf_len);
    if (!tx_buf) { printf("  [pong] malloc failed\n"); return; }
    tx_total = buf_len;

    for (int n = 0; n < pulse_samples; n++) {
        float ph       = 2.0f * (float)M_PI * detected_offset * n / SAMPLE_RATE;
        tx_buf[2*n]    = (int8_t)(cosf(ph) * 120.0f);
        tx_buf[2*n+1]  = (int8_t)(sinf(ph) * 120.0f);
    }
    tx_done = 0; tx_sent = 0;

    hackrf_set_txvga_gain(g_device, TXVGA_GAIN);
    hackrf_set_antenna_enable(g_device, 1);

    int res = hackrf_start_tx(g_device, tx_callback, NULL);
    if (res != HACKRF_SUCCESS) {
        printf("  [pong] start_tx failed: %s\n", hackrf_error_name(res));
        return;
    }

    /* Wait for TX callback to drain into USB pipe */
    while (!tx_done && running) vTaskDelay(pdMS_TO_TICKS(1));
    vTaskDelay(pdMS_TO_TICKS(5));
    hackrf_stop_tx(g_device);
    vTaskDelay(pdMS_TO_TICKS(3));
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Task: usb_rx_task
 *
 * Initialises libhackrf, opens the device, arms RX, then blocks forever
 * (libhackrf's internal USB event thread drives rx_callback).
 *
 * This task's job is just lifecycle management — the actual sample delivery
 * happens inside libhackrf's thread (FreeRTOS-POSIX task).
 * ═══════════════════════════════════════════════════════════════════════════ */
static void usb_rx_task(void *arg) {
    (void)arg;

    /* HackRF init */
    if (hackrf_init() != HACKRF_SUCCESS) {
        printf("[usb_rx] hackrf_init failed\n");
        vTaskDelete(NULL);
        return;
    }
    if (hackrf_open(&g_device) != HACKRF_SUCCESS) {
        printf("[usb_rx] hackrf_open failed\n");
        hackrf_exit();
        vTaskDelete(NULL);
        return;
    }

    hackrf_set_sample_rate(g_device, SAMPLE_RATE);
    hackrf_set_freq(g_device, CARRIER_FREQ);

    printf("[usb_rx] HackRF open @ %dMS/s\n", SAMPLE_RATE / 1000000);

    /* Arm RX — rx_callback will drive ring_write and notify scan task */
    if (arm_rx() != HACKRF_SUCCESS) {
        printf("[usb_rx] arm_rx failed\n");
        goto cleanup;
    }

    /* Stay alive; scan/pong tasks will call hackrf_stop/start_rx as needed.
     * Poll running flag to handle graceful shutdown. */
    while (running) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

cleanup:
    hackrf_stop_rx(g_device);
    hackrf_close(g_device);
    hackrf_exit();
    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Task: scan_detect_task
 *
 * Waits for sample-arrival notifications from rx_callback.
 * Runs coarse FFT scan on new data.
 * On detection: back-solves ping arrival, computes deadline, sends to
 * pong_task via pong_queue.
 *
 * Matches node_b.c main scan loop exactly, with usleep() replaced by
 * xTaskNotifyWait().
 * ═══════════════════════════════════════════════════════════════════════════ */
static void scan_detect_task(void *arg) {
    (void)arg;

    /* Store own handle so rx_callback can notify us */
    scan_task_handle = xTaskGetCurrentTaskHandle();

    long first_scan_at = (long)FFT_SIZE * N_ACCUM;
    long next_scan_at  = first_scan_at;
    int  window        = 0;
    int  cycle         = 0;

    for (;;) {
        /*
         * Block until rx_callback signals that SCAN_NOTIFY_SAMPLES new
         * samples have arrived.  Timeout after 2s in case of USB stall.
         */
        xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(2000));
        if (!running) break;

        /* Check if we have enough samples for a scan */
        if ((long)samples_rx < next_scan_at) continue;
        next_scan_at += SCAN_NOTIFY_SAMPLES;

        float peak_freq;
        float snr = coarse_scan(&peak_freq);
        window++;

        if (window % 20 == 0 || snr > COARSE_SNR_THRESH * 0.5f) {
            printf("  [scan win %4d]  SNR: %6.1fx  peak: %+.0fkHz\n",
                   window, snr, peak_freq / 1e3);
        }

        if (snr >= COARSE_SNR_THRESH) {
            /*
             * Snapshot ring_write before stopping RX — same as node_b.c.
             * Stop RX so pong_task can later start TX on the same device.
             */
            uint32_t trigger_rw = ring_write;
            hackrf_stop_rx(g_device);
            ++cycle;

            printf("\n[cycle %d] PING  SNR=%.0fx  peak=%+.0fkHz\n",
                   cycle, snr, peak_freq / 1e3);

            /* Back-solve ping leading edge */
            detected_offset = peak_freq;
            uint32_t ping_sample = back_solve_ping(trigger_rw);

            /*
             * Convert ping_sample to an absolute FreeRTOS tick deadline.
             *
             *   ping_arrival_ticks = rx_arm_tick
             *                        + (ping_sample / SAMPLE_RATE) * configTICK_RATE_HZ
             *
             * Use 64-bit arithmetic to avoid overflow at 2MS/s.
             */
            TickType_t ping_ticks = rx_arm_tick
                + (TickType_t)(((uint64_t)ping_sample * configTICK_RATE_HZ)
                               / SAMPLE_RATE);
            TickType_t deadline_ticks = ping_ticks
                + pdMS_TO_TICKS(NODE_B_DELAY_MS);

            TickType_t now_ticks = xTaskGetTickCount();
            printf("  deadline in %ld ms\n",
                   (long)(deadline_ticks - now_ticks) * 1000 / configTICK_RATE_HZ);

            /* Send to pong_task */
            DetectEvent_t evt = {
                .snr               = snr,
                .peak_freq_hz      = peak_freq,
                .trigger_ring_write= trigger_rw,
            };
            /* Embed deadline into the event by repurposing a field.
             * Simpler: pass deadline as a separate global since only one
             * ranging cycle is active at a time. */
            (void)evt;   /* suppress warning — queue send below */

            /*
             * Pack deadline into the queue.  We use a simple struct;
             * add deadline_ticks as an extra field.
             */
            typedef struct { DetectEvent_t e; TickType_t deadline; } QItem_t;
            QItem_t item = { .e = { snr, peak_freq, trigger_rw },
                             .deadline = deadline_ticks };

            if (xQueueSend(pong_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
                printf("  [scan] pong_queue full — missed cycle\n");
                /* Re-arm RX so we don't get stuck */
                arm_rx();
                window = 0;
                next_scan_at = samples_rx + first_scan_at;
            }
            /* scan_detect_task now idles — pong_task re-arms RX when done */
        }
    }

    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Task: pong_task
 *
 * Blocks on pong_queue.  On receipt:
 *   1. vTaskDelayUntil(deadline_ticks) — precision turnaround wait.
 *   2. fire_pong() — TX the response burst.
 *   3. Re-arm RX, reset scan_detect_task counters via a notification.
 *
 * Matches the detection handler block in node_b.c's main loop, with
 * clock_nanosleep replaced by vTaskDelayUntil.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void pong_task(void *arg) {
    (void)arg;

    /* Must match the QItem_t defined in scan_detect_task.
     * In production code move this typedef to a shared header. */
    typedef struct {
        DetectEvent_t e;
        TickType_t    deadline;
    } QItem_t;

    for (;;) {
        QItem_t item;
        if (xQueueReceive(pong_queue, &item, portMAX_DELAY) != pdTRUE) continue;
        if (!running) break;

        TickType_t now = xTaskGetTickCount();

        /*
         * vTaskDelayUntil sleeps until an absolute tick count.
         * If deadline is already past (shouldn't happen with 5s delay)
         * it returns immediately.
         *
         * FreeRTOS note: vTaskDelayUntil takes a pointer to a "previous
         * wake time" and an increment.  To sleep until an absolute time,
         * set prev_wake = deadline - 1 tick and increment = 1.
         * Alternatively use vTaskDelayUntil with xTaskGetTickCount as base.
         */
        if ((TickType_t)(item.deadline - now) < (TickType_t)(UINT32_MAX / 2)) {
            /* Deadline is in the future */
            TickType_t prev = item.deadline - 1;
            vTaskDelayUntil(&prev, 1);
        }
        /* else deadline already past — fire immediately */

        printf("  [pong] firing at %+.0fkHz\n", item.e.peak_freq_hz / 1e3);
        fire_pong();
        printf("  [pong] sent. re-arming RX.\n");

        /* Re-arm RX and notify scan task to reset its counters */
        arm_rx();

        /* Wait for first samples before notifying scan task */
        while (samples_rx < 1000 && running) vTaskDelay(pdMS_TO_TICKS(1));

        /* Wake scan_detect_task so it resets next_scan_at */
        if (scan_task_handle) {
            xTaskNotify(scan_task_handle, 1, eSetValueWithOverwrite);
        }
    }

    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * node_b_init  —  call from main() or a startup task
 *
 * Allocates all heap structures, creates tasks and queues.
 * Returns 0 on success, -1 on allocation failure.
 * ═══════════════════════════════════════════════════════════════════════════ */
int node_b_init(void) {
    /* FFT structures */
    g_fft      = fft_init(FFT_SIZE);
    g_fine_fft = fft_init(FINE_FFT_SIZE);
    g_accum    = (float *)pvPortMalloc(FFT_SIZE * sizeof(float));
    g_hann     = (float *)pvPortMalloc(FFT_SIZE * sizeof(float));
    g_fine_hann= (float *)pvPortMalloc(FINE_FFT_SIZE * sizeof(float));

    if (!g_fft || !g_fine_fft || !g_accum || !g_hann || !g_fine_hann) {
        printf("node_b_init: FFT alloc failed\n");
        return -1;
    }

    /* Pre-compute Hanning windows */
    for (int k = 0; k < FFT_SIZE; k++)
        g_hann[k] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * k / (FFT_SIZE - 1)));
    for (int k = 0; k < FINE_FFT_SIZE; k++)
        g_fine_hann[k] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * k / (FINE_FFT_SIZE - 1)));

    /* Pre-compute scan bin range */
    scan_bin_lo = freq_to_bin(SCAN_CENTER_HZ - SCAN_WINDOW_HZ, FFT_SIZE);
    scan_bin_hi = freq_to_bin(SCAN_CENTER_HZ + SCAN_WINDOW_HZ, FFT_SIZE);

    /* IPC */
    /*
     * Queue depth 1: only one detection event can be in flight at a time.
     * Item size must match QItem_t in scan_detect_task / pong_task.
     * sizeof(DetectEvent_t) + sizeof(TickType_t) = 4+4+4+4 = 16 bytes.
     */
    pong_queue = xQueueCreate(1, sizeof(DetectEvent_t) + sizeof(TickType_t));
    if (!pong_queue) {
        printf("node_b_init: queue alloc failed\n");
        return -1;
    }

    /* Tasks */
    if (xTaskCreate(usb_rx_task, "usb_rx",
                    STACK_USB_RX, NULL, PRIORITY_USB_RX, NULL) != pdPASS ||
        xTaskCreate(scan_detect_task, "scan",
                    STACK_SCAN, NULL, PRIORITY_SCAN, &scan_task_handle) != pdPASS ||
        xTaskCreate(pong_task, "pong",
                    STACK_PONG, NULL, PRIORITY_PONG, NULL) != pdPASS) {
        printf("node_b_init: task create failed\n");
        return -1;
    }

    printf("Node B (FreeRTOS) initialised\n");
    printf("Sample rate: %dMS/s  FFT: %d pts (%.0fHz/bin)\n",
           SAMPLE_RATE/1000000, FFT_SIZE, (double)FREQ_RES);
    printf("Coarse thresh: %.0fx  Fine thresh: %.0fx  Delay: %dms\n",
           (double)COARSE_SNR_THRESH, (double)FINE_SNR_THRESH, NODE_B_DELAY_MS);

    return 0;
}

/* ── Minimal main — replace with your STM32CubeIDE-generated main.c ──────── */
int main(void) {
    /*
     * In STM32CubeIDE:
     *   1. HAL_Init(), SystemClock_Config(), peripheral MX inits go here.
     *   2. Call node_b_init().
     *   3. vTaskStartScheduler().
     *
     * The HAL_Init / clock config is board-specific and generated by CubeMX,
     * so it's omitted here.  Add it before node_b_init().
     */

    /* HAL_Init(); */
    /* SystemClock_Config(); */
    /* MX_USB_HOST_Init(); */   /* USB OTG FS host mode — configure in CubeMX */

    if (node_b_init() != 0) {
        /* Spin on failure — attach debugger to diagnose */
        while (1) __asm__("nop");
    }

    vTaskStartScheduler();

    /* Should never reach here */
    while (1) __asm__("nop");
    return 0;
}
