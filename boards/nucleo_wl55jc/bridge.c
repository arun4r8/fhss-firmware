/*
 * bridge.c - CAN <-> frequency-hopping radio bridge for NUCLEO-WL55JC +
 * SX1262 shield + MCP2515 (see README.md for wiring). Logic lives in
 * src/can_bridge.c; this file wires it to the hardware.
 *
 * Build one role per board:
 *   make bridge ROLE=pod      on the pod: hub messages from the CAN bus go out
 *                             over the radio. Hopping leader, so its beacons
 *                             carry the telemetry.
 *   make bridge ROLE=ground   at the ground station: received messages are
 *                             printed on the serial console and, if an MCP2515
 *                             is connected, re-sent on CAN as ID 0x200 + hub.
 *   make bridge ROLE=pod DEMO=1
 *                             single-board test: no CAN bus needed. Fake hub
 *                             messages go through the MCP2515 in loopback, so
 *                             reassembly, chunking and transmission all run.
 *
 * Console (ST-LINK virtual COM port, 115200 8N1): a status line every 5 s;
 * the ground station also prints every message it receives.
 *
 * Machine-readable log lines for tools/matlab (HOP_LOG 0 turns them off):
 *   CFG,<role>,<seed>,<channels>,<base_hz>,<spacing_hz>,<dwell_ms>   once at start
 *   HOP,<ms>,<hop_index>,<channel>,<H|S>   every channel change (S = searching)
 *   TX,<ms>,<hop_index>,<channel>,<payload bytes>   ms = start of transmission
 *   RX,<ms>,<hop_index>,<channel>          a packet from our network arrived
 * ms is the board's uptime clock; hop_index is the position in the sequence.
 * LEDs: blue toggles on every hop, green = link up (ground: locked to the
 *       pod), red = CAN controller or radio problem.
 */
#include <string.h>

#include "board.h"
#include "can_bridge.h"
#include "fhss_link.h"
#include "mcp2515.h"

#define ROLE_POD    1
#define ROLE_GROUND 2
#ifndef BRIDGE_ROLE
#define BRIDGE_ROLE ROLE_POD
#endif
#ifndef BRIDGE_DEMO
#define BRIDGE_DEMO 0
#endif

/* Placeholders until the embedded team confirms them (README "Open
 * questions"): bus bit rate, which IDs carry hub messages, resend IDs. */
#define CAN_OSC_HZ      8000000u /* HW-184 crystal, confirmed 8.000 MHz */
#define CAN_BITRATE     500000u
#define HUB_ID_MASK     0x000u   /* 0 = forward every ID */
#define HUB_ID_MATCH    0x000u
#define GROUND_ID_BASE  0x200u

#define FHSS_SEED       0x6A5D10u /* same on both boards */
#define FHSS_NET_ID     0x47
#define TX_DBM          0         /* bench power; raise for range tests */
#define STATUS_EVERY_MS 5000u
#define DEMO_EVERY_MS   100u      /* fake hub message rate in DEMO mode */
#ifndef HOP_LOG
#define HOP_LOG 1
#endif

static fhss_link_t link;
static can_bridge_t bridge;
static mcp2515_t can = {
    .spi = &board_spi1, .cs_port = CAN_CS_PORT, .cs_pin = CAN_CS_PIN,
    .osc_hz = CAN_OSC_HZ, .bitrate = CAN_BITRATE,
    .mode = BRIDGE_DEMO ? MCP2515_MODE_LOOPBACK : MCP2515_MODE_NORMAL,
};
static int can_ok;
static uint32_t can_tx_errors;

static bool radio_idle(void *user)
{
    (void)user;
    return link.q_count == 0;
}

static bool radio_send(void *user, const uint8_t *data, uint8_t len)
{
    (void)user;
    return fhss_link_send(&link, data, len);
}

/* Pod: move every waiting CAN frame into the bridge. Runs from the main loop
 * and while the radio transmits, since the MCP2515 holds only 2 frames. */
static void drain_can(void *user)
{
    (void)user;
    can_frame_t f;
    while (can_ok && mcp2515_receive(&can, &f)) {
        can_bridge_on_frame(&bridge, f.id, f.data, f.dlc, HAL_GetTick());
    }
}

static void print_message(const can_hub_msg_t *m, int16_t rssi)
{
    uart_puts("rx hub ");
    uart_put_int(m->hub);
    uart_puts(", rssi ");
    uart_put_int(rssi);
    uart_puts(" dBm:");
    for (uint8_t i = 0; i < m->count; i++) {
        const can_sensor_t *s = &m->sensor[i];
        uart_puts(" ");
        uart_put_int(s->id);
        uart_puts(s->error ? "!" : "");
        uart_puts("=");
        uart_put_int(s->reading);
    }
    uart_puts("\n");
}

/* Ground: every payload from the pod. */
static void on_rx(void *user, const uint8_t *data, uint8_t len, int16_t rssi_dbm)
{
    (void)user;
    if (len == 0) {
        return; /* beacon with no telemetry */
    }
    static can_hub_msg_t m;
    uint8_t frames[CAN_MSG_MAX_FRAMES][8], dlc[CAN_MSG_MAX_FRAMES];
    uint16_t id;
    uint8_t n = can_bridge_from_radio(&bridge, data, len, &m, frames, dlc, &id);
    if (n == 0) {
        uart_puts("rx payload is not a hub message (");
        uart_put_int(len);
        uart_puts(" bytes)\n");
        return;
    }
    print_message(&m, rssi_dbm);
    for (uint8_t i = 0; can_ok && i < n; i++) {
        can_frame_t f = {.id = id, .dlc = dlc[i]};
        memcpy(f.data, frames[i], dlc[i]);
        if (mcp2515_send(&can, &f) != MCP2515_OK) {
            can_tx_errors++;
            break; /* no other node ACKing: don't stall the radio */
        }
    }
}

/* DEMO: a 10-sensor message from hub 3, readings counting up, sent as CAN
 * frames that the MCP2515 loops straight back to us. */
static void demo_hub_message(void)
{
    static uint16_t counter;
    can_hub_msg_t m = {.hub = 3, .count = 10};
    for (uint8_t i = 0; i < 10; i++) {
        m.sensor[i] = (can_sensor_t){.type = 1, .id = i, .reading = (uint16_t)(counter + i)};
    }
    counter = (uint16_t)(counter + 10);
    uint8_t buf[CAN_MSG_MAX_BYTES], data[CAN_MSG_MAX_FRAMES][8], dlc[CAN_MSG_MAX_FRAMES];
    uint8_t n = can_msg_to_frames(buf, can_msg_encode(&m, buf, sizeof(buf)), data, dlc);
    for (uint8_t i = 0; i < n; i++) {
        can_frame_t f = {.id = 0x103, .dlc = dlc[i]};
        memcpy(f.data, data[i], dlc[i]);
        mcp2515_send(&can, &f);
        drain_can(NULL); /* loopback holds only 2 frames too */
    }
}

static void log_fields(const char *tag, uint32_t ms)
{
    uart_puts(tag);
    uart_puts(",");
    uart_put_int((int32_t)ms);
    uart_puts(",");
    uart_put_int(link.fhss.hop_index);
    uart_puts(",");
    uart_put_int(fhss_current_channel(&link.fhss));
}

static void log_config(void)
{
    const fhss_config_t *c = &link.fhss.cfg;
    uart_puts(BRIDGE_ROLE == ROLE_POD ? "CFG,pod," : "CFG,ground,");
    uart_put_int((int32_t)c->seed);
    uart_puts(",");
    uart_put_int(c->num_channels);
    uart_puts(",");
    uart_put_int((int32_t)c->base_freq_hz);
    uart_puts(",");
    uart_put_int((int32_t)c->channel_spacing_hz);
    uart_puts(",");
    uart_put_int(c->dwell_ms);
    uart_puts("\n");
}

static void print_status(void)
{
    const can_bridge_stats_t *b = &bridge.stats;
    if (BRIDGE_ROLE == ROLE_POD) {
        uart_puts("status: CAN frames ");
        uart_put_int((int32_t)b->frames_in);
        uart_puts(", hub msgs ");
        uart_put_int((int32_t)b->messages_in);
        uart_puts(", sent ");
        uart_put_int((int32_t)b->messages_sent);
        uart_puts(" (");
        uart_put_int((int32_t)b->chunks_sent);
        uart_puts(" pkts), replaced by newer ");
        uart_put_int((int32_t)b->superseded);
        uart_puts(", errors ");
        uart_put_int((int32_t)(b->reasm_errors + b->no_slot + b->radio_refused));
        uart_puts(", radio tx ");
        uart_put_int((int32_t)link.stats.tx_packets);
    } else {
        uart_puts(fhss_link_locked(&link) ? "status: locked" : "status: searching for the pod");
        uart_puts(", hub msgs ");
        uart_put_int((int32_t)b->radio_in);
        uart_puts(", bad ");
        uart_put_int((int32_t)b->radio_bad);
        uart_puts(", crc errors ");
        uart_put_int((int32_t)link.stats.rx_crc_errors);
        if (can_ok) {
            uart_puts(", CAN resend errors ");
            uart_put_int((int32_t)can_tx_errors);
        }
    }
    if (can_ok) {
        uint8_t tec, rec;
        uint8_t eflg = mcp2515_error_flags(&can, &tec, &rec);
        if (eflg) {
            uart_puts(", CAN EFLG 0x");
            uart_put_hex(eflg, 2);
        }
    }
    uart_puts("\n");
}

int main(void)
{
    board_init();
    HAL_Delay(200); /* let the host's serial monitor attach */

    uart_puts(BRIDGE_ROLE == ROLE_POD
                  ? (BRIDGE_DEMO ? "\n=== CAN bridge: pod, DEMO (MCP2515 loopback, fake hub 3) ===\n"
                                 : "\n=== CAN bridge: pod (CAN -> radio, hopping leader) ===\n")
                  : "\n=== CAN bridge: ground (radio -> console + CAN) ===\n");

    int err = mcp2515_init(&can);
    can_ok = err == MCP2515_OK;
    if (!can_ok) {
        uart_puts("CAN controller init failed, error ");
        uart_put_int(err);
        uart_puts(BRIDGE_ROLE == ROLE_POD ? " (nothing to forward; check wiring, see selftest)\n"
                                          : " (continuing: console output only)\n");
    }

    can_bridge_config_t bc = {
        .rx_id_mask = HUB_ID_MASK, .rx_id_match = HUB_ID_MATCH, .tx_id_base = GROUND_ID_BASE,
        .max_payload = FHSS_LINK_MAX_PAYLOAD,
        .radio_idle = radio_idle, .radio_send = radio_send,
    };
    can_bridge_init(&bridge, &bc);

    fhss_link_config_t lc = {
        .spi = &board_spi1,
        .nss_port = RADIO_NSS_PORT, .nss_pin = RADIO_NSS_PIN,
        .rst_port = RADIO_RST_PORT, .rst_pin = RADIO_RST_PIN,
        .busy_port = RADIO_BUSY_PORT, .busy_pin = RADIO_BUSY_PIN,
        .ant_pwr_port = RADIO_ANT_PORT, .ant_pwr_pin = RADIO_ANT_PIN,
        .clock = SX1262_CLOCK_AUTO,
        .leader = BRIDGE_ROLE == ROLE_POD,
        .seed = FHSS_SEED,
        .net_id = FHSS_NET_ID,
        .tx_power_dbm = TX_DBM,
        .on_rx = BRIDGE_ROLE == ROLE_GROUND ? on_rx : NULL,
        .tx_wait_hook = BRIDGE_ROLE == ROLE_POD ? drain_can : NULL,
    };
    while (fhss_link_init(&link, &lc) != FHSS_LINK_OK) {
        uart_puts("radio init failed, error ");
        uart_put_int(link.radio_err);
        uart_puts(" (shield seated? run the selftest build); retrying in 3 s\n");
        led_set(LED_RED_PORT, LED_RED_PIN, 1);
        HAL_Delay(3000);
    }
    led_set(LED_RED_PORT, LED_RED_PIN, !can_ok && BRIDGE_ROLE == ROLE_POD);
    uart_puts(BRIDGE_ROLE == ROLE_POD ? "radio up, hopping\n" : "radio up, searching for the pod\n");

    if (HOP_LOG) {
        log_config();
        log_fields("HOP", HAL_GetTick());
        uart_puts(fhss_link_locked(&link) ? ",H\n" : ",S\n");
    }

    uint16_t last_hop = link.fhss.hop_index;
    uint32_t last_tx = 0, last_rx = 0;
    bool was_locked = fhss_link_locked(&link);
    uint32_t next_status = HAL_GetTick() + STATUS_EVERY_MS;
    uint32_t next_demo = HAL_GetTick() + DEMO_EVERY_MS;
    for (;;) {
        if (BRIDGE_ROLE == ROLE_POD) {
            if (BRIDGE_DEMO && can_ok && (int32_t)(HAL_GetTick() - next_demo) >= 0) {
                next_demo += DEMO_EVERY_MS;
                demo_hub_message();
            }
            drain_can(NULL);
            can_bridge_poll(&bridge);
        }
        /* what the next transmission would carry, to log its length */
        uint8_t queued_len = link.q_count ? link.queue[link.q_head].len : 0;
        fhss_link_poll(&link);

        if (HOP_LOG && link.stats.tx_packets != last_tx) {
            last_tx = link.stats.tx_packets;
            /* poll returns once the packet is sent: back-date to its start */
            uint32_t air = sx1262_airtime_ms(&link.radio.cfg, (uint8_t)(FHSS_LINK_HEADER_LEN + queued_len));
            log_fields("TX", HAL_GetTick() - air);
            uart_puts(",");
            uart_put_int(queued_len);
            uart_puts("\n");
        }
        if (HOP_LOG && link.stats.rx_packets != last_rx) {
            last_rx = link.stats.rx_packets;
            log_fields("RX", HAL_GetTick());
            uart_puts("\n");
        }
        if (link.fhss.hop_index != last_hop || (HOP_LOG && fhss_link_locked(&link) != was_locked)) {
            was_locked = fhss_link_locked(&link);
            if (HOP_LOG) {
                log_fields("HOP", HAL_GetTick());
                uart_puts(was_locked ? ",H\n" : ",S\n");
            }
        }
        if (link.fhss.hop_index != last_hop) {
            last_hop = link.fhss.hop_index;
            led_toggle(LED_BLUE_PORT, LED_BLUE_PIN);
            led_set(LED_GREEN_PORT, LED_GREEN_PIN,
                    BRIDGE_ROLE == ROLE_POD ? can_ok : fhss_link_locked(&link));
        }
        if ((int32_t)(HAL_GetTick() - next_status) >= 0) {
            next_status += STATUS_EVERY_MS;
            print_status();
        }
    }
}
