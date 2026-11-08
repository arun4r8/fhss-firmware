/*
 * fhss_link.h - Frequency-hopping packet link: fhss core + SX1262 radio.
 *
 * Each 400 ms dwell is split in two slots:
 *   first leader_slot_ms   leader transmits (always at least one beacon per dwell)
 *   the rest               followers transmit (only once locked)
 * Default is half and half. Every packet must start and end inside its
 * node's slot with SLOT_MARGIN_MS to spare, so none straddles a hop. The
 * split must be the same on every node.
 *
 * On-air packet: [net_id][flags][fhss sync x4][payload 0..FHSS_LINK_MAX_PAYLOAD]
 */
#ifndef FHSS_LINK_H
#define FHSS_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "fhss.h"
#include "sx1262.h"

#define FHSS_LINK_MAX_PAYLOAD 77 /* one 38-sensor telemetry frame (telem_frame.h) */
#define FHSS_LINK_QUEUE_LEN   8
#define FHSS_LINK_HEADER_LEN  (2 + FHSS_SYNC_WIRE_SIZE)

typedef void (*fhss_link_rx_cb)(void *user, const uint8_t *data, uint8_t len, int16_t rssi_dbm);

typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *nss_port;
    uint16_t nss_pin;
    GPIO_TypeDef *rst_port;
    uint16_t rst_pin;
    GPIO_TypeDef *busy_port;
    uint16_t busy_pin;
    GPIO_TypeDef *ant_pwr_port; /* shield antenna-switch supply (D8), or NULL */
    uint16_t ant_pwr_pin;
    sx1262_clock_t clock;       /* SX1262_CLOCK_AUTO unless you know the shield */

    bool leader;         /* exactly one leader per network */
    uint32_t seed;       /* hop sequence seed, same on every node */
    uint8_t net_id;      /* drops packets from other networks on our channels */
    int8_t tx_power_dbm; /* -9..22 */
    uint16_t leader_slot_ms; /* leader's share of each dwell; 0 = half */
    uint32_t bandwidth_hz;   /* LoRa bandwidth: 0 = 125 kHz, or 250000. Same on every
                              * node. 250 kHz has a 20 dB bandwidth of 250 kHz or
                              * more, so FCC 15.247(a)(1)(i) then asks for >= 25
                              * channels and <= 0.4 s per channel in 10 s, which
                              * 50 channels at 500 kHz / 400 ms / 20 s also meet. */

    fhss_link_rx_cb on_rx;
    void *user;
    /* optional: runs while a transmission is in progress (see
     * sx1262_t.tx_wait_hook); may call fhss_link_send() but not poll() */
    void (*tx_wait_hook)(void *user);
    /* optional: asked for a payload (up to max bytes) the moment this
     * node's slot has room for one and the queue is empty; return its
     * length, 0 for nothing. Lets time-sensitive data be built at the last
     * moment; back-to-back packets go out within one fhss_link_poll(). */
    uint8_t (*fill)(void *user, uint8_t *buf, uint8_t max);
    /* optional: called after each transmission (payload length, start ms) */
    void (*on_tx)(void *user, uint8_t len, uint32_t start_ms);
} fhss_link_config_t;

typedef struct {
    uint32_t tx_packets;
    uint32_t rx_packets;
    uint32_t rx_crc_errors;
    uint32_t rx_foreign; /* wrong net_id or malformed */
    uint32_t tx_dropped; /* queue full */
    uint32_t tx_errors;
} fhss_link_stats_t;

typedef struct {
    fhss_link_config_t cfg;
    sx1262_t radio;
    fhss_t fhss;
    uint16_t leader_slot_ms;
    uint8_t max_send_len; /* longest payload that fits in this node's slot */
    uint16_t last_hop_index;
    bool sent_this_dwell;
    struct {
        uint8_t len;
        uint8_t data[FHSS_LINK_MAX_PAYLOAD];
    } queue[FHSS_LINK_QUEUE_LEN];
    uint8_t q_head;
    uint8_t q_count;
    fhss_link_stats_t stats;
    sx1262_err_t radio_err; /* why init failed, if it did */
} fhss_link_t;

typedef enum {
    FHSS_LINK_OK = 0,
    FHSS_LINK_ERR_RADIO,
    FHSS_LINK_ERR_CONFIG
} fhss_link_err_t;

fhss_link_err_t fhss_link_init(fhss_link_t *l, const fhss_link_config_t *cfg);

/* Call from the main loop as often as possible (at least every 1-2 ms). */
void fhss_link_poll(fhss_link_t *l);

/* Queue a payload; it goes out in this node's next free slot. Refused if
 * the queue is full or it is longer than max_send_len. */
bool fhss_link_send(fhss_link_t *l, const uint8_t *data, uint8_t len);

bool fhss_link_locked(const fhss_link_t *l);

/* true if a payload of len bytes could start right now in this node's slot.
 * Lets the caller build time-sensitive data at the last moment: queue it
 * when this turns true and the next fhss_link_poll() sends it. */
bool fhss_link_slot_open(const fhss_link_t *l, uint8_t len);

#endif /* FHSS_LINK_H */
