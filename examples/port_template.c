/*
 * port_template.c - how to wire fhss into a board. Not compiled by default.
 *
 * Replace the four radio_* / millis() stubs with your MCU + transceiver
 * driver calls (e.g. SX1276 setFrequency, CC1101 channel register, HAL_GetTick).
 * Packet layout used here:  [4-byte fhss sync][payload...]
 */
#include "fhss.h"

#include <stddef.h>
#include <stdint.h>

/* ---- board-specific stubs: implement these ----------------------------- */
extern uint32_t millis(void);
extern void radio_set_frequency(uint32_t hz);
extern void radio_send(const uint8_t *buf, size_t len);
extern int radio_receive(uint8_t *buf, size_t max); /* bytes read, 0 if none */
extern void app_handle_payload(const uint8_t *buf, size_t len);
extern size_t app_next_payload(uint8_t *buf, size_t max);

#define IS_LEADER 1 /* one node is the leader, the rest followers */

static void hal_set_frequency(void *user, uint32_t hz)
{
    (void)user;
    radio_set_frequency(hz);
}

static fhss_t fhss;

void comms_init(void)
{
    fhss_config_t cfg = {
        .base_freq_hz = 902200000u,
        .channel_spacing_hz = 500000u,
        .num_channels = 50,
        .dwell_ms = 400,
        .seed = 0x6A5D10u, /* pick one per team/network, same on every node */
        .role = IS_LEADER ? FHSS_ROLE_LEADER : FHSS_ROLE_FOLLOWER,
        .guard_ms = 20,     /* >= longest packet airtime + radio retune time */
        .rx_latency_ms = 5, /* measure: airtime of the sync bytes + ISR delay */
        .max_missed_hops = 3,
    };
    fhss_hal_t hal = {hal_set_frequency, NULL};

    if (fhss_init(&fhss, &cfg, &hal) != FHSS_OK) {
        for (;;) { } /* bad config: fix before flashing */
    }
    fhss_start(&fhss, millis());
}

void comms_loop(void)
{
    uint8_t buf[64];
    uint32_t now = millis();

    fhss_tick(&fhss, now);

    int n = radio_receive(buf, sizeof(buf));
    if (n >= FHSS_SYNC_WIRE_SIZE) {
        fhss_sync_t sync;
        fhss_sync_decode(buf, &sync);
        fhss_on_sync(&fhss, millis(), &sync);
        app_handle_payload(buf + FHSS_SYNC_WIRE_SIZE, (size_t)n - FHSS_SYNC_WIRE_SIZE);
    }

    /* Leader should send at least one packet early in every dwell (even an
     * empty one) so followers can acquire and stay locked. */
    if (fhss_can_transmit(&fhss, now)) {
        size_t len = app_next_payload(buf + FHSS_SYNC_WIRE_SIZE,
                                      sizeof(buf) - FHSS_SYNC_WIRE_SIZE);
        if (len > 0 || IS_LEADER) {
            fhss_sync_t sync;
            fhss_get_sync(&fhss, now, &sync);
            fhss_sync_encode(&sync, buf);
            radio_send(buf, FHSS_SYNC_WIRE_SIZE + len);
        }
    }
}
