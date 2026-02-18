#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#include <libhackrf/hackrf.h>

#define SAMPLE_RATE 2000000
#define FREQ 915500000
#define BIT_RATE 5000
#define SAMPLES_PER_BIT (SAMPLE_RATE / BIT_RATE)
#define F_OFFSET 200000.0f
#define PAYLOAD_BITS (24 * 8) // 3 doubles

// Global state for the decoder
float complex filter0, filter1;
int bit_sample_count = 0;
uint64_t shift_reg = 0; // To detect preamble 0xAAAAAAAA
int decoding_active = 0;
int bit_idx = 0;
uint8_t rx_buffer[24];

void init_filters() {
    // Pre-calculate the "reference" phasor for the bit duration
    // Using a simplified single-point correlation for speed
    filter0 = cexpf(-I * 2.0f * M_PI * F_OFFSET / SAMPLE_RATE);
    filter1 = cexpf(-I * 2.0f * M_PI * -F_OFFSET / SAMPLE_RATE);
}

int rx_callback(hackrf_transfer* transfer) {
    int8_t* buffer = (int8_t*)transfer->buffer;
    int len = transfer->valid_length / 2;

    static float complex accum0 = 0, accum1 = 0;

    for (int i = 0; i < len; i++) {
        float complex sample = (float)buffer[2*i] + I * (float)buffer[2*i+1];

        // Correlate against the two target frequencies
        // In a real MCU, you'd use a Goertzel filter here
        float phase0 = 2.0f * M_PI * F_OFFSET * (bit_sample_count / (float)SAMPLE_RATE);
        float phase1 = 2.0f * M_PI * -F_OFFSET * (bit_sample_count / (float)SAMPLE_RATE);
        
        accum0 += sample * cexpf(-I * phase0);
        accum1 += sample * cexpf(-I * phase1);

        bit_sample_count++;

        // Once we have a full bit's worth of samples
        if (bit_sample_count >= SAMPLES_PER_BIT) {
            int bit = (cabsf(accum1) > cabsf(accum0)) ? 1 : 0;
            
            if (!decoding_active) {
                // Look for preamble (0xAA repeated)
                shift_reg = (shift_reg << 1) | bit;
                if (shift_reg == 0xAAAAAAAAAAAAAAAA) { // Found 8 bytes of 0xAA
                    decoding_active = 1;
                    bit_idx = 0;
                    memset(rx_buffer, 0, 24);
                    printf("\n[Frame Detected] -> ");
                }
            } else {
                // Store the data bits
                int byte_pos = bit_idx / 8;
                int bit_pos = 7 - (bit_idx % 8);
                if (bit) rx_buffer[byte_pos] |= (1 << bit_pos);

                bit_idx++;
                if (bit_idx >= PAYLOAD_BITS) {
                    double d[3];
                    memcpy(d, rx_buffer, 24);
                    printf("D1: %.3f, D2: %.3f, D3: %.3f\n", d[0], d[1], d[2]);
                    decoding_active = 0;
                    shift_reg = 0;
                }
            }

            // Reset for next bit
            bit_sample_count = 0;
            accum0 = 0;
            accum1 = 0;
        }
    }
    return 0;
}

int main() {
    init_filters();
    if (hackrf_init() != HACKRF_SUCCESS) return 1;

    hackrf_device* device = NULL;
    if (hackrf_open(&device) != HACKRF_SUCCESS) return 1;

    hackrf_set_sample_rate(device, SAMPLE_RATE);
    hackrf_set_freq(device, FREQ);
    hackrf_set_lna_gain(device, 32);
    hackrf_set_vga_gain(device, 30);

    printf("Live BFSK Receiver Active at 915.5 MHz\n");
    printf("Listening for 3 doubles...\n");

    hackrf_start_rx(device, rx_callback, NULL);
    
    while(1) {
        // Keep main thread alive
    }

    hackrf_stop_rx(device);
    hackrf_close(device);
    hackrf_exit();
    return 0;
}
