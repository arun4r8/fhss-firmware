/*
 * gen_ref_sequences.c - writes hop sequences from the real firmware code
 * (src/fhss.c) for tools/matlab/test_fhss_sequence.m to compare against.
 *
 *   cc -Iinclude tools/matlab/gen_ref_sequences.c src/fhss.c -o build/gen_ref
 *   ./build/gen_ref > tools/matlab/ref_sequences.csv
 *
 * One line per case: seed,num_channels,channel,channel,...
 */
#include <stdio.h>

#include "fhss.h"

int main(void)
{
    static const uint32_t seeds[] = {0x6A5D10u, 0u, 1u, 2u, 12345u, 0xDEADBEEFu, 0x7FFFFFFFu,
                                     0x80000000u, 0xFFFFFFFFu};
    static const uint16_t counts[] = {50, 64, 79};
    for (size_t c = 0; c < sizeof(counts) / sizeof(counts[0]); c++) {
        for (size_t i = 0; i < sizeof(seeds) / sizeof(seeds[0]); i++) {
            fhss_t f;
            fhss_config_t cfg = {
                .base_freq_hz = 902200000u, .channel_spacing_hz = 300000u,
                .num_channels = counts[c], .seed = seeds[i],
            };
            fhss_hal_t hal = {0};
            if (fhss_init(&f, &cfg, &hal) != FHSS_OK) {
                fprintf(stderr, "init failed: seed %u, %u channels\n", seeds[i], counts[c]);
                return 1;
            }
            printf("%u,%u", seeds[i], counts[c]);
            for (uint16_t k = 0; k < counts[c]; k++) {
                printf(",%u", f.sequence[k]);
            }
            printf("\n");
        }
    }
    return 0;
}
