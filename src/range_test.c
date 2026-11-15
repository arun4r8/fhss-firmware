#include "range_test.h"

void range_pattern(uint16_t hop_index, uint8_t *out, size_t n)
{
    /* seed must be non-zero; spread hop indexes over the 9-bit state */
    uint16_t lfsr = (uint16_t)(((uint32_t)hop_index * 37u + 1u) % 511u + 1u);
    for (size_t i = 0; i < n; i++) {
        uint8_t byte = 0;
        for (int b = 0; b < 8; b++) {
            unsigned bit = (((unsigned)lfsr >> 8) ^ ((unsigned)lfsr >> 4)) & 1u; /* taps 9 and 5 */
            lfsr = (uint16_t)((((unsigned)lfsr << 1) | bit) & 0x1FFu);
            byte = (uint8_t)(((unsigned)byte << 1) | bit);
        }
        out[i] = byte;
    }
}

size_t range_build(const range_info_t *info, uint16_t hop_index, uint8_t *out, size_t max)
{
    if (max < RANGE_PAYLOAD_LEN) {
        return 0;
    }
    out[0] = RANGE_MAGIC;
    out[1] = (uint8_t)(info->seq >> 8);
    out[2] = (uint8_t)info->seq;
    out[3] = (uint8_t)(info->up_ok >> 8);
    out[4] = (uint8_t)info->up_ok;
    out[5] = (uint8_t)info->up_rssi_dbm;
    out[6] = (uint8_t)info->up_snr_q4;
    range_pattern(hop_index, &out[RANGE_PATTERN_AT], RANGE_PAYLOAD_LEN - RANGE_PATTERN_AT);
    return RANGE_PAYLOAD_LEN;
}

bool range_parse(const uint8_t *payload, size_t len, range_info_t *info)
{
    if (len != RANGE_PAYLOAD_LEN || payload[0] != RANGE_MAGIC) {
        return false;
    }
    info->seq = (uint16_t)((payload[1] << 8) | payload[2]);
    info->up_ok = (uint16_t)((payload[3] << 8) | payload[4]);
    info->up_rssi_dbm = (int8_t)payload[5];
    info->up_snr_q4 = (int8_t)payload[6];
    return true;
}

static uint32_t popcount8(uint8_t x)
{
    uint32_t n = 0;
    while (x) {
        n += x & 1u;
        x >>= 1;
    }
    return n;
}

uint32_t range_bit_errors(const uint8_t *payload, size_t len, uint16_t hop_index, uint32_t *bits)
{
    uint8_t want[RANGE_PAYLOAD_LEN];
    range_pattern(hop_index, &want[RANGE_PATTERN_AT], RANGE_PAYLOAD_LEN - RANGE_PATTERN_AT);
    want[0] = RANGE_MAGIC;

    uint32_t errors = 0;
    for (size_t i = 0; i < RANGE_PAYLOAD_LEN; i++) {
        if (i > 0 && i < RANGE_PATTERN_AT) {
            continue; /* counters: not known in advance */
        }
        errors += i < len ? popcount8((uint8_t)(payload[i] ^ want[i])) : 8u;
    }
    *bits = RANGE_CHECKED_BITS;
    return errors;
}
