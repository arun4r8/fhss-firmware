/*
 * fhss_link.h - Frequency-hopping packet link: fhss core + SX1262 radio.
 *
 * Each 400 ms dwell is split in two halves:
 *   first half   leader transmits (always at least one beacon per dwell)
 *   second half  followers transmit (only once locked)
 * ending guard_ms before the hop so no packet straddles a channel change.
 *
 * On-air packet: [net_id][flags][fhss sync x4][payload 0..FHSS_LINK_MAX_PAYLOAD]
 */
#ifndef FHSS_LINK_H
#define FHSS_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "fhss.h"
#include "sx1262.h"

#define FHSS_LINK_MAX_PAYLOAD 24
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

    fhss_link_rx_cb on_rx;
    void *user;
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
    uint16_t max_airtime_ms;
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

/* Queue a payload; it goes out in this node's next free slot. */
bool fhss_link_send(fhss_link_t *l, const uint8_t *data, uint8_t len);

bool fhss_link_locked(const fhss_link_t *l);

#endif /* FHSS_LINK_H */
