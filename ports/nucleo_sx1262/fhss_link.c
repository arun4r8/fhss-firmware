#include "fhss_link.h"

#include <string.h>

/* 50 channels, 902.2 - 926.7 MHz, 500 kHz apart. A 125 kHz LoRa signal
 * leaves plenty of room between channels and stays under the 250 kHz
 * 20 dB bandwidth limit that allows the 50-channel / 400 ms rule. */
#define LINK_BASE_FREQ_HZ   902200000u
#define LINK_SPACING_HZ     500000u
#define LINK_NUM_CHANNELS   50u

#define RETUNE_MARGIN_MS    5u  /* radio retune + SPI overhead */
#define SLOT_MARGIN_MS      10u /* absorbs sync error between nodes */

#define FLAG_FROM_LEADER    0x01

static const sx1262_lora_cfg_t link_lora = {
    .spreading_factor = 7,
    .bandwidth_hz = 125000,
    .coding_rate = 1, /* 4/5 */
    .preamble_len = 8,
    .sync_word = 0x12,
    .tx_power_dbm = 14, /* overwritten from config */
};

static void hal_set_frequency(void *user, uint32_t hz)
{
    sx1262_set_frequency((sx1262_t *)user, hz);
}

fhss_link_err_t fhss_link_init(fhss_link_t *l, const fhss_link_config_t *cfg)
{
    memset(l, 0, sizeof(*l));
    l->cfg = *cfg;
    l->radio.spi = cfg->spi;
    l->radio.nss_port = cfg->nss_port;
    l->radio.nss_pin = cfg->nss_pin;
    l->radio.rst_port = cfg->rst_port;
    l->radio.rst_pin = cfg->rst_pin;
    l->radio.busy_port = cfg->busy_port;
    l->radio.busy_pin = cfg->busy_pin;
    l->radio.ant_pwr_port = cfg->ant_pwr_port;
    l->radio.ant_pwr_pin = cfg->ant_pwr_pin;
    l->radio.clock = cfg->clock;
    l->radio.tx_wait_hook = cfg->tx_wait_hook;
    l->radio.tx_wait_ctx = cfg->user;

    sx1262_lora_cfg_t lora = link_lora;
    lora.tx_power_dbm = cfg->tx_power_dbm;
    l->radio_err = sx1262_init(&l->radio, &lora, LINK_BASE_FREQ_HZ);
    if (l->radio_err != SX1262_OK) {
        return FHSS_LINK_ERR_RADIO;
    }

    l->max_airtime_ms = (uint16_t)sx1262_airtime_ms(
        &lora, FHSS_LINK_HEADER_LEN + FHSS_LINK_MAX_PAYLOAD);

    fhss_config_t fc = {
        .base_freq_hz = LINK_BASE_FREQ_HZ,
        .channel_spacing_hz = LINK_SPACING_HZ,
        .num_channels = LINK_NUM_CHANNELS,
        .dwell_ms = FHSS_DWELL_MS,
        .seed = cfg->seed,
        .role = cfg->leader ? FHSS_ROLE_LEADER : FHSS_ROLE_FOLLOWER,
        .guard_ms = (uint16_t)(l->max_airtime_ms + RETUNE_MARGIN_MS),
        .rx_latency_ms = 0, /* compensated per packet from its airtime */
        .max_missed_hops = 3,
    };
    fhss_hal_t hal = {hal_set_frequency, &l->radio};
    if (fhss_init(&l->fhss, &fc, &hal) != FHSS_OK) {
        return FHSS_LINK_ERR_CONFIG;
    }

    l->radio.rx_active = true; /* so fhss_start's retune leaves us listening */
    fhss_start(&l->fhss, HAL_GetTick());
    sx1262_start_rx(&l->radio);
    l->last_hop_index = l->fhss.hop_index;
    return FHSS_LINK_OK;
}

bool fhss_link_send(fhss_link_t *l, const uint8_t *data, uint8_t len)
{
    if (len > FHSS_LINK_MAX_PAYLOAD || l->q_count == FHSS_LINK_QUEUE_LEN) {
        l->stats.tx_dropped++;
        return false;
    }
    uint8_t tail = (uint8_t)((l->q_head + l->q_count) % FHSS_LINK_QUEUE_LEN);
    l->queue[tail].len = len;
    memcpy(l->queue[tail].data, data, len);
    l->q_count++;
    return true;
}

bool fhss_link_locked(const fhss_link_t *l)
{
    return fhss_state(&l->fhss) == FHSS_STATE_HOPPING;
}

static void handle_rx(fhss_link_t *l)
{
    uint8_t buf[FHSS_LINK_HEADER_LEN + FHSS_LINK_MAX_PAYLOAD];
    int16_t rssi = 0;
    int n = sx1262_read_packet(&l->radio, buf, sizeof(buf), &rssi);
    uint32_t rx_done_ms = HAL_GetTick();

    if (n == 0) {
        return;
    }
    if (n == SX1262_ERR_CRC) {
        l->stats.rx_crc_errors++;
        return;
    }
    if (n < FHSS_LINK_HEADER_LEN || buf[0] != l->cfg.net_id) {
        l->stats.rx_foreign++;
        return;
    }

    if (buf[1] & FLAG_FROM_LEADER) {
        fhss_sync_t sync;
        fhss_sync_decode(&buf[2], &sync);
        /* The leader stamped the sync when it started sending; we see the
         * packet only once the last symbol arrives. */
        uint32_t tx_start_ms = rx_done_ms - sx1262_airtime_ms(&l->radio.cfg, (uint8_t)n);
        if (fhss_on_sync(&l->fhss, tx_start_ms, &sync) != FHSS_OK) {
            l->stats.rx_foreign++;
            return;
        }
    }

    l->stats.rx_packets++;
    if (n > FHSS_LINK_HEADER_LEN && l->cfg.on_rx) {
        l->cfg.on_rx(l->cfg.user, &buf[FHSS_LINK_HEADER_LEN],
                     (uint8_t)(n - FHSS_LINK_HEADER_LEN), rssi);
    }
}

static bool in_my_slot(const fhss_link_t *l, uint32_t now, uint32_t airtime)
{
    if (!fhss_can_transmit(&l->fhss, now)) {
        return false; /* searching, or inside the pre-hop guard */
    }
    uint32_t dwell = l->fhss.cfg.dwell_ms;
    uint32_t until_hop = fhss_ms_until_hop(&l->fhss, now);
    uint32_t into = dwell - until_hop;
    uint32_t half = dwell / 2;

    if (l->cfg.leader) {
        /* Wait a little into the dwell so followers whose clocks run slightly
         * behind have already hopped when the packet starts. */
        return into >= SLOT_MARGIN_MS && into + airtime + SLOT_MARGIN_MS <= half;
    }
    return into >= half + SLOT_MARGIN_MS && until_hop > airtime + RETUNE_MARGIN_MS;
}

static void try_transmit(fhss_link_t *l)
{
    bool have_data = l->q_count > 0;
    bool need_beacon = l->cfg.leader && !l->sent_this_dwell;
    if (!have_data && !need_beacon) {
        return;
    }

    uint8_t len = have_data ? l->queue[l->q_head].len : 0;
    uint8_t total = (uint8_t)(FHSS_LINK_HEADER_LEN + len);
    uint32_t now = HAL_GetTick();
    if (!in_my_slot(l, now, sx1262_airtime_ms(&l->radio.cfg, total))) {
        return;
    }

    uint8_t pkt[FHSS_LINK_HEADER_LEN + FHSS_LINK_MAX_PAYLOAD];
    fhss_sync_t sync;
    fhss_get_sync(&l->fhss, now, &sync);
    pkt[0] = l->cfg.net_id;
    pkt[1] = l->cfg.leader ? FLAG_FROM_LEADER : 0;
    fhss_sync_encode(&sync, &pkt[2]);
    if (have_data) {
        memcpy(&pkt[FHSS_LINK_HEADER_LEN], l->queue[l->q_head].data, len);
    }

    if (sx1262_send(&l->radio, pkt, total) == SX1262_OK) {
        l->stats.tx_packets++;
    } else {
        l->stats.tx_errors++;
    }
    l->sent_this_dwell = true;
    if (have_data) {
        l->q_head = (uint8_t)((l->q_head + 1) % FHSS_LINK_QUEUE_LEN);
        l->q_count--;
    }
}

void fhss_link_poll(fhss_link_t *l)
{
    fhss_tick(&l->fhss, HAL_GetTick());
    if (l->fhss.hop_index != l->last_hop_index) {
        l->last_hop_index = l->fhss.hop_index;
        l->sent_this_dwell = false;
    }

    handle_rx(l);
    try_transmit(l);
}
