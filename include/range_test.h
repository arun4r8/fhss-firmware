/*
 * range_test.h - Payload for range testing: lets the ground measure packet
 * loss, bit errors, RSSI and SNR in both directions while the pod is out of
 * reach (powered from a battery, no console).
 *
 * Pod -> ground payload (RANGE_PAYLOAD_LEN bytes, the size of a telemetry
 * frame so the airtime matches real traffic):
 *   byte 0        RANGE_MAGIC
 *   bytes 1-2     pod transmit counter (big-endian), for counting lost packets
 *   bytes 3-4     ground -> pod commands the pod has received so far
 *   byte 5        RSSI of the last command the pod received, dBm (signed)
 *   byte 6        SNR of that command, quarter dB (signed)
 *   bytes 7..     known pseudo-random pattern (PRBS-9), seeded by the hop
 *                 index the packet is sent on
 *
 * The ground knows the hop index too (it is locked to the pod), so it can
 * regenerate the pattern and count bit errors even in packets whose CRC
 * failed. Bits checked per packet: the magic byte and the pattern
 * (RANGE_CHECKED_BITS); the counters can't be checked and are only used
 * from packets that passed the CRC.
 */
#ifndef RANGE_TEST_H
#define RANGE_TEST_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RANGE_PAYLOAD_LEN  77u
#define RANGE_MAGIC        0xA5u
#define RANGE_PATTERN_AT   7u
#define RANGE_CHECKED_BITS (8u * (1u + RANGE_PAYLOAD_LEN - RANGE_PATTERN_AT)) /* 568 */

typedef struct {
    uint16_t seq;        /* pod transmit counter */
    uint16_t up_ok;      /* commands the pod has received */
    int8_t up_rssi_dbm;  /* last command's RSSI */
    int8_t up_snr_q4;    /* last command's SNR, quarter dB */
} range_info_t;

/* PRBS-9 (x^9 + x^5 + 1) bytes for a hop index. */
void range_pattern(uint16_t hop_index, uint8_t *out, size_t n);

/* Build a payload (RANGE_PAYLOAD_LEN bytes) into out. Returns the length,
 * 0 if max is too small. */
size_t range_build(const range_info_t *info, uint16_t hop_index, uint8_t *out, size_t max);

/* Read the counters from a payload that passed the CRC. false if it isn't a
 * range-test payload (wrong length or magic). */
bool range_parse(const uint8_t *payload, size_t len, range_info_t *info);

/* Bit errors in the checked part of a received payload, against the
 * pattern for hop_index. Bytes missing from a short payload count as all
 * wrong. Returns the error count; *bits gets the number of bits checked. */
uint32_t range_bit_errors(const uint8_t *payload, size_t len, uint16_t hop_index, uint32_t *bits);

#endif /* RANGE_TEST_H */
