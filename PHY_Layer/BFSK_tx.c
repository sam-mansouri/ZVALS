#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#include <libhackrf/hackrf.h>

#define SAMPLE_RATE 2000000
#define FREQ 915500000          // 915.5 MHz
#define BIT_RATE 5000           // 5kbps
#define SAMPLES_PER_BIT (SAMPLE_RATE / BIT_RATE)
#define F_OFFSET 200000.0f      // 200kHz shift

// 3 doubles (24 bytes) + 8 bytes preamble = 32 bytes total
unsigned char tx_data[32];
float complex *tone_samples;
int total_tone_samples;

void prepare_data() {
    // 1. Preamble 0xAA (10101010) repeated 8 times
    memset(tx_data, 0xAA, 8);

    // 2. Pack 3 doubles
    double d1 = 1.234, d2 = 5.678, d3 = 9.012;
    memcpy(tx_data + 8, &d1, 8);
    memcpy(tx_data + 16, &d2, 8);
    memcpy(tx_data + 24, &d3, 8);

    total_tone_samples = 32 * 8 * SAMPLES_PER_BIT;
    tone_samples = malloc(total_tone_samples * sizeof(float complex));

    printf("Generating BFSK tones...\n");
    int sample_ptr = 0;
    for (int i = 0; i < 32; i++) {
        for (int bit = 7; bit >= 0; bit--) {
            int b = (tx_data[i] >> bit) & 1;
            float freq = b ? -F_OFFSET : F_OFFSET;
            for (int s = 0; s < SAMPLES_PER_BIT; s++) {
                float t = (float)s / SAMPLE_RATE;
                tone_samples[sample_ptr++] = cexpf(I * 2.0f * M_PI * freq * t);
            }
        }
    }
}

int tx_callback(hackrf_transfer* transfer) {
    static int current_sample = 0;
    int8_t* buffer = (int8_t*)transfer->buffer;

    for (int i = 0; i < transfer->valid_length / 2; i++) {
        float complex s = tone_samples[current_sample];
       
        buffer[2 * i]     = (int8_t)(crealf(s) * 120.0f);
        buffer[2 * i + 1] = (int8_t)(cimagf(s) * 120.0f);

        current_sample++;
        if (current_sample >= total_tone_samples) {
            current_sample = 0; // Loop the transmission
        }
    }
    return 0;
}

int main() {
    prepare_data();

    hackrf_init();
    hackrf_device* device = NULL;
    if (hackrf_open(&device) != HACKRF_SUCCESS) {
        fprintf(stderr, "HackRF not found.\n");
        return 1;
    }

    hackrf_set_sample_rate(device, SAMPLE_RATE);
    hackrf_set_freq(device, FREQ);
    hackrf_set_txvga_gain(device, 47);
    hackrf_set_amp_enable(device, 0);

    printf("BFSK TX Active: 915.5 MHz | 5kbps | 3 Doubles\n");
    printf("Press Enter to quit.\n");

    hackrf_start_tx(device, tx_callback, NULL);
    getchar();

    hackrf_stop_tx(device);
    hackrf_close(device);
    hackrf_exit();
    free(tone_samples);
    return 0;
}
