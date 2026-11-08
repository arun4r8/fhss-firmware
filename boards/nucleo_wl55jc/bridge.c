/*
 * bridge.c - CAN <-> frequency-hopping radio bridge for NUCLEO-WL55JC +
 * SX1262 shield + MCP2515 (see README.md for wiring). Logic lives in
 * src/can_bridge.c; this file wires it to the hardware.
 *
 * Telemetry goes as one frame of every sensor (src/telem_frame.c, layout
 * TELEM_SCHEMA): the pod keeps each sensor's latest reading from the hub
 * messages on CAN and sends the whole set, without per-sensor tags, each
 * time its slot opens and something changed. Each 400 ms hop gives the pod
 * the first LINK_LEADER_SLOT_MS (two 77-byte frames = 5 frames/s) and the
 * ground the rest (one payload of up to 9 bytes, for commands).
 *
 * Build one role per board:
 *   make bridge ROLE=pod      on the pod: sensor readings from the CAN bus go
 *                             out over the radio. Hopping leader, so its
 *                             frames also carry the hop timing.
 *   make bridge ROLE=ground   at the ground station: received frames are
 *                             printed on the serial console and, if an MCP2515
 *                             is connected, each hub's message is rebuilt and
 *                             re-sent on CAN as ID 0x200 + hub.
 *   make bridge ROLE=pod DEMO=1
 *                             single-board test: no CAN bus needed. Fake
 *                             messages from hubs 1-4 go through the MCP2515 in
 *                             loopback, so the whole pod path runs.
 *   make bridge ROLE=ground HUB=1
 *                             real CAN bus test with two boards: the ground
 *                             station's MCP2515 plays hub 4 on the bus (ID
 *                             0x104, every 100 ms). The pod (plain ROLE=pod)
 *                             picks the frames up through its transceiver and
 *                             radios them back, so the ground prints what it
 *                             put on the wires. Received messages are not
 *                             re-sent on CAN in this mode, or they would loop.
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
#include "telem_frame.h"

#define ROLE_POD    1
#define ROLE_GROUND 2
#ifndef BRIDGE_ROLE
#define BRIDGE_ROLE ROLE_POD
#endif
#ifndef BRIDGE_DEMO
#define BRIDGE_DEMO 0
#endif
#ifndef BRIDGE_HUB
#define BRIDGE_HUB 0
#endif

/* Placeholders until the embedded team confirms them (README "Open
 * questions"): bus bit rate, which IDs carry hub messages, resend IDs. */
#define CAN_OSC_HZ      8000000u /* HW-184 crystal, confirmed 8.000 MHz */
#define CAN_BITRATE     500000u
#define HUB_ID_MASK     0x000u   /* 0 = forward every ID */
#define HUB_ID_MATCH    0x000u
#define GROUND_ID_BASE  0x200u
/* sensor order: placeholder until the embedded team sends the real one */
#define TELEM_SCHEMA    telem_schema_placeholder
/* LoRa bandwidth and the pod's share of each 400 ms hop; both must be the
 * same on both boards. 125 kHz + 330 ms: 2 frames per hop (5/s), 9-byte
 * commands up. 250 kHz + 340 ms (make bridge ... BW250=1): 4 frames per hop
 * (10/s), 30-byte commands up; verified in host emulation only, and needs
 * the RF lead's OK on the 20 dB bandwidth first (fhss_link.h). */
#ifndef BRIDGE_BW250
#define BRIDGE_BW250 0
#endif
/* Until the embedded team defines real commands, the ground sends a dummy
 * command every hop (as long as its slot allows: 9 B at 125 kHz, 30 B at
 * 250 kHz) so the ground -> pod direction is exercised on hardware:
 * "CMD", a 16-bit counter, then a counting byte pattern. The pod checks and
 * counts them. Set to 0 once real commands exist. */
#ifndef GROUND_TEST_CMDS
#define GROUND_TEST_CMDS 1
#endif
#define LINK_BANDWIDTH_HZ   (BRIDGE_BW250 ? 250000u : 125000u)
#define LINK_LEADER_SLOT_MS (BRIDGE_BW250 ? 340u : 330u)

#define FHSS_SEED       0x6A5D10u /* same on both boards */
#define FHSS_NET_ID     0x47
#define TX_DBM          0         /* bench power; raise for range tests */
#define STATUS_EVERY_MS 5000u
#define DEMO_EVERY_MS   25u       /* fake hub messages: hubs 1-4 in turn, each at 10 Hz */
#define HUB_EVERY_MS    100u      /* HUB mode: hub 4 on the real bus at 10 Hz */
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
static uint32_t hub_msgs_sent;
static uint32_t cmds_sent, cmds_ok, cmds_bad, cmds_missed;
static uint16_t cmd_seq;

/* Ground: queue the next dummy command (fills this node's slot). */
static void send_test_cmd(void)
{
    uint8_t c[FHSS_LINK_MAX_PAYLOAD];
    uint8_t n = link.max_send_len;
    if (n < 5) {
        return; /* no room for "CMD" + counter */
    }
    c[0] = 'C';
    c[1] = 'M';
    c[2] = 'D';
    c[3] = (uint8_t)(cmd_seq >> 8);
    c[4] = (uint8_t)cmd_seq;
    for (uint8_t i = 5; i < n; i++) {
        c[i] = (uint8_t)(cmd_seq + i);
    }
    if (fhss_link_send(&link, c, n)) {
        cmd_seq++;
        cmds_sent++;
    }
}

/* Pod: payloads from the ground (only dummy commands for now). */
static void pod_on_rx(void *user, const uint8_t *data, uint8_t len, int16_t rssi_dbm)
{
    (void)user;
    (void)rssi_dbm;
    static int32_t last = -1;
    if (len == 0) {
        return;
    }
    bool ok = len >= 5 && data[0] == 'C' && data[1] == 'M' && data[2] == 'D';
    uint16_t seq = ok ? (uint16_t)((data[3] << 8) | data[4]) : 0;
    for (uint8_t i = 5; ok && i < len; i++) {
        ok = data[i] == (uint8_t)(seq + i);
    }
    if (!ok) {
        cmds_bad++;
        return;
    }
    if (last >= 0) {
        cmds_missed += (uint16_t)(seq - (uint16_t)last - 1u);
    }
    last = seq;
    cmds_ok++;
}

static bool radio_idle(void *user)
{
    (void)user;
    /* build the frame only when it can go out at once, so it is fresh */
    return link.q_count == 0 &&
           fhss_link_slot_open(&link, (uint8_t)telem_frame_length(&TELEM_SCHEMA));
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

/* Ground: one line per frame, "hub.sensor=reading", "-" = no valid reading */
static void print_frame(const uint16_t *v, int16_t rssi)
{
    const telem_schema_t *sc = &TELEM_SCHEMA;
    uart_puts("rx frame, rssi ");
    uart_put_int(rssi);
    uart_puts(" dBm:");
    for (uint8_t i = 0; i < sc->count; i++) {
        uart_puts(" ");
        uart_put_int(sc->slot[i].hub);
        uart_puts(".");
        uart_put_int(sc->slot[i].id);
        uart_puts("=");
        if (v[i] == TELEM_FAULT) {
            uart_puts("-");
        } else {
            uart_put_int(v[i]);
        }
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
    uint16_t v[TELEM_MAX_SENSORS];
    if (!can_bridge_frame_from_radio(&bridge, data, len, v)) {
        uart_puts("rx payload is not a telemetry frame for this layout (");
        uart_put_int(len);
        uart_puts(" bytes)\n");
        return;
    }
    print_frame(v, rssi_dbm);
    if (!can_ok || BRIDGE_HUB) {
        return;
    }
    /* rebuild each hub's message and re-send it on ID 0x200 + hub */
    for (uint8_t hub = 0; hub < 16; hub++) {
        can_hub_msg_t m;
        if (!telem_frame_hub_msg(&TELEM_SCHEMA, v, hub, &m)) {
            continue;
        }
        uint8_t buf[CAN_MSG_MAX_BYTES], frames[CAN_MSG_MAX_FRAMES][8], dlc[CAN_MSG_MAX_FRAMES];
        uint8_t n = can_msg_to_frames(buf, can_msg_encode(&m, buf, sizeof(buf)), frames, dlc);
        for (uint8_t i = 0; i < n; i++) {
            can_frame_t f = {.id = (uint16_t)(GROUND_ID_BASE + hub), .dlc = dlc[i]};
            memcpy(f.data, frames[i], dlc[i]);
            if (mcp2515_send(&can, &f) != MCP2515_OK) {
                can_tx_errors++;
                return; /* no other node ACKing: don't stall the radio */
            }
        }
    }
}

/* DEMO: messages from hubs 1-4 in turn (each hub every 100 ms), readings
 * counting up, sent as CAN frames that the MCP2515 loops straight back to us.
 * HUB: hub 4 only, onto the real bus for the pod to pick up. Sensors are the
 * layout's sensors for that hub. */
static void demo_hub_message(void)
{
    static uint16_t counter;
    static uint8_t turn;
    const uint8_t hub = BRIDGE_HUB ? 4 : (uint8_t)(1 + turn++ % 4);
    can_hub_msg_t m;
    telem_frame_hub_msg(&TELEM_SCHEMA, (const uint16_t[TELEM_MAX_SENSORS]){0}, hub, &m);
    for (uint8_t i = 0; i < m.count; i++) {
        m.sensor[i].reading = (uint16_t)(counter + i);
    }
    counter = (uint16_t)(counter + 10);
    uint8_t buf[CAN_MSG_MAX_BYTES], data[CAN_MSG_MAX_FRAMES][8], dlc[CAN_MSG_MAX_FRAMES];
    uint8_t n = can_msg_to_frames(buf, can_msg_encode(&m, buf, sizeof(buf)), data, dlc);
    for (uint8_t i = 0; i < n; i++) {
        if (!can_ok) {
            /* DEMO without a working MCP2515: hand the frames straight to
             * the bridge, so the radio path can still be tested */
            can_bridge_on_frame(&bridge, (uint16_t)(0x100 + hub), data[i], dlc[i], HAL_GetTick());
            continue;
        }
        can_frame_t f = {.id = (uint16_t)(0x100 + hub), .dlc = dlc[i]};
        memcpy(f.data, data[i], dlc[i]);
        if (mcp2515_send(&can, &f) != MCP2515_OK) {
            can_tx_errors++; /* on a real bus: nobody ACKed (wiring, termination) */
            return;
        }
        if (!BRIDGE_HUB) {
            drain_can(NULL); /* loopback holds only 2 frames too */
        }
    }
    hub_msgs_sent++;
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
        uart_puts(", frames sent ");
        uart_put_int((int32_t)b->messages_sent);
        uart_puts(", msgs merged before sending ");
        uart_put_int((int32_t)b->superseded);
        uart_puts(", unknown sensors ");
        uart_put_int((int32_t)bridge.snap.unmapped);
        uart_puts(", errors ");
        uart_put_int((int32_t)(b->reasm_errors + b->no_slot + b->radio_refused));
        uart_puts(", radio tx ");
        uart_put_int((int32_t)link.stats.tx_packets);
        uart_puts(", ground cmds ok ");
        uart_put_int((int32_t)cmds_ok);
        uart_puts(", missed ");
        uart_put_int((int32_t)cmds_missed);
        uart_puts(", bad ");
        uart_put_int((int32_t)cmds_bad);
    } else {
        uart_puts(fhss_link_locked(&link) ? "status: locked" : "status: searching for the pod");
        uart_puts(", frames ");
        uart_put_int((int32_t)b->radio_in);
        uart_puts(", missed ");
        uart_put_int((int32_t)b->frames_missed);
        uart_puts(", bad ");
        uart_put_int((int32_t)b->radio_bad);
        uart_puts(", crc errors ");
        uart_put_int((int32_t)link.stats.rx_crc_errors);
        if (GROUND_TEST_CMDS) {
            uart_puts(", test cmds sent ");
            uart_put_int((int32_t)cmds_sent);
            uart_puts(" (");
            uart_put_int(link.max_send_len);
            uart_puts(" B)");
        }
        if (can_ok && BRIDGE_HUB) {
            uart_puts(", hub 4 msgs put on CAN ");
            uart_put_int((int32_t)hub_msgs_sent);
            uart_puts(", CAN send failures ");
            uart_put_int((int32_t)can_tx_errors);
        } else if (can_ok) {
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

/* Link callbacks */
static void on_tx_log(void *user, uint8_t len, uint32_t start_ms)
{
    (void)user;
    if (HOP_LOG) {
        log_fields("TX", start_ms);
        uart_puts(",");
        uart_put_int(len);
        uart_puts("\n");
    }
}

/* Pod: the link asks for a frame the moment its slot has room for one */
static uint8_t pod_fill(void *user, uint8_t *buf, uint8_t max)
{
    (void)user;
    drain_can(NULL);
    return can_bridge_fill(&bridge, buf, max);
}

int main(void)
{
    board_init();
    HAL_Delay(200); /* let the host's serial monitor attach */

    uart_puts(BRIDGE_ROLE == ROLE_POD
                  ? (BRIDGE_DEMO ? "\n=== CAN bridge: pod, DEMO (MCP2515 loopback, fake hubs 1-4) ===\n"
                                 : "\n=== CAN bridge: pod (CAN -> radio, hopping leader) ===\n")
                  : (BRIDGE_HUB ? "\n=== CAN bridge: ground + fake hub 4 on the CAN bus ===\n"
                                : "\n=== CAN bridge: ground (radio -> console + CAN) ===\n"));

    int err = mcp2515_init(&can);
    can_ok = err == MCP2515_OK;
    if (!can_ok) {
        uart_puts("CAN controller init failed, error ");
        uart_put_int(err);
        uart_puts(BRIDGE_ROLE != ROLE_POD ? " (continuing: console output only)\n"
                  : BRIDGE_DEMO ? " (DEMO continues without it: fake hub frames go straight to the bridge)\n"
                                : " (nothing to forward; check wiring, see selftest)\n");
    }

    can_bridge_config_t bc = {
        .rx_id_mask = HUB_ID_MASK, .rx_id_match = HUB_ID_MATCH, .tx_id_base = GROUND_ID_BASE,
        .max_payload = FHSS_LINK_MAX_PAYLOAD, .schema = &TELEM_SCHEMA,
        .radio_idle = radio_idle, .radio_send = radio_send,
    };
    if (!can_bridge_init(&bridge, &bc)) {
        uart_puts("telemetry layout does not fit the radio payload; fix TELEM_SCHEMA\n");
        led_set(LED_RED_PORT, LED_RED_PIN, 1);
        for (;;) {
        }
    }

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
        .leader_slot_ms = LINK_LEADER_SLOT_MS,
        .bandwidth_hz = LINK_BANDWIDTH_HZ,
        .on_rx = BRIDGE_ROLE == ROLE_GROUND ? on_rx : pod_on_rx,
        .tx_wait_hook = BRIDGE_ROLE == ROLE_POD ? drain_can : NULL,
        .fill = BRIDGE_ROLE == ROLE_POD ? pod_fill : NULL,
        .on_tx = on_tx_log,
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
    uint32_t last_rx = 0;
    bool was_locked = fhss_link_locked(&link);
    uint32_t next_status = HAL_GetTick() + STATUS_EVERY_MS;
    uint32_t next_demo = HAL_GetTick() + DEMO_EVERY_MS;
    for (;;) {
        if (BRIDGE_ROLE == ROLE_POD) {
            if (BRIDGE_DEMO && (int32_t)(HAL_GetTick() - next_demo) >= 0) {
                next_demo += DEMO_EVERY_MS;
                demo_hub_message();
            }
            drain_can(NULL); /* frames go out through pod_fill() */
        } else {
            if (BRIDGE_HUB && can_ok && (int32_t)(HAL_GetTick() - next_demo) >= 0) {
                next_demo += HUB_EVERY_MS;
                demo_hub_message();
            }
            if (GROUND_TEST_CMDS && link.q_count == 0 && fhss_link_locked(&link)) {
                send_test_cmd();
            }
        }
        /* what the next transmission would carry, to log its length */
        fhss_link_poll(&link);

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
