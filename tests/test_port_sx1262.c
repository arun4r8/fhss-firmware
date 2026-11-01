/*
 * Host tests for the Nucleo + SX1262 port: real driver and link code,
 * emulated SX1262 radios on a shared virtual air (tests/mocks).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "can_bridge.h"
#include "fhss_link.h"
#include "sx1262.h"
#include "sx1262_emu.h"

static int failures;
static int checks;

#define CHECK(cond, ...)                                                   \
    do {                                                                   \
        checks++;                                                          \
        if (!(cond)) {                                                     \
            failures++;                                                    \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__);                  \
            printf(__VA_ARGS__);                                           \
            printf("\n");                                                  \
        }                                                                  \
    } while (0)

#define NO_VIOLATIONS(e) \
    CHECK((e)->violations == 0, "radio %d: %d protocol violations, last: %s", (e)->id, \
          (e)->violations, (e)->last_violation)

static const sx1262_lora_cfg_t sf7 = {
    .spreading_factor = 7, .bandwidth_hz = 125000, .coding_rate = 1,
    .preamble_len = 8, .sync_word = 0x12, .tx_power_dbm = 14,
};

static sx1262_t radio_on(sx1262_emu_t *e)
{
    sx1262_t r = {
        .spi = &e->spi, .nss_port = &e->port, .nss_pin = GPIO_PIN_NSS,
        .rst_port = &e->port, .rst_pin = GPIO_PIN_RST,
        .busy_port = &e->port, .busy_pin = GPIO_PIN_BUSY,
        .ant_pwr_port = &e->port, .ant_pwr_pin = GPIO_PIN_ANT,
    };
    return r;
}

/* ---- driver ------------------------------------------------------------ */

static void test_freq_and_airtime(void)
{
    printf("RF frequency word and LoRa airtime\n");
    CHECK(sx1262_rf_freq_from_hz(915000000u) == 0x39300000u, "915 MHz word %08X",
          sx1262_rf_freq_from_hz(915000000u));
    /* Semtech LoRa calculator: SF7/125k/4-5/8 preamble/10 B = 41.216 ms */
    CHECK(sx1262_airtime_ms(&sf7, 10) == 42, "SF7 10 B airtime %u", sx1262_airtime_ms(&sf7, 10));
    /* SF12/125k needs low data rate optimize: 991.232 ms */
    sx1262_lora_cfg_t sf12 = sf7;
    sf12.spreading_factor = 12;
    CHECK(sx1262_airtime_ms(&sf12, 10) == 992, "SF12 10 B airtime %u",
          sx1262_airtime_ms(&sf12, 10));
}

static void test_init_state(void)
{
    printf("init configures the chip in a legal order\n");
    emu_reset_world();
    static sx1262_emu_t e;
    emu_add_radio(&e, 0);
    sx1262_t r = radio_on(&e);
    CHECK(sx1262_init(&r, &sf7, 915000000u) == SX1262_OK, "init failed");
    CHECK(e.mode == EMU_STBY_RC, "mode %d (want standby)", e.mode);
    CHECK(e.calibrated, "not calibrated");
    CHECK(e.packet_type == 1, "packet type %u", e.packet_type);
    CHECK(e.rf_freq == 0x39300000u, "rf freq %08X", e.rf_freq);
    CHECK(e.mod[0] == 7 && e.mod[1] == 0x04 && e.mod[2] == 1 && e.mod[3] == 0,
          "modulation %02X %02X %02X %02X", e.mod[0], e.mod[1], e.mod[2], e.mod[3]);
    CHECK(e.pkt[0] == 0 && e.pkt[1] == 8 && e.pkt[2] == 0 && e.pkt[4] == 1 && e.pkt[5] == 0,
          "packet params");
    CHECK(e.regs[0x0740] == 0x14 && e.regs[0x0741] == 0x24, "sync word %02X%02X",
          e.regs[0x0740], e.regs[0x0741]);
    CHECK(e.dio2_rf_switch, "DIO2 RF switch not enabled");
    CHECK(e.ant_power, "antenna switch power pin not driven");
    CHECK(e.tx_power == 14, "tx power %d", e.tx_power);
    CHECK((e.regs[0x08D8] & 0x1E) == 0x1E, "errata 15.2 TX clamp not applied");
    CHECK(e.regs[0x0889] & 0x04, "errata 15.1 modulation bit not set for 125 kHz");
    CHECK(!e.tcxo_cfg, "TCXO enabled on a crystal board");
    NO_VIOLATIONS(&e);
}

static void test_init_errors(void)
{
    printf("init diagnoses missing chip, stuck BUSY, clock mismatch, auto-detects clock\n");
    static sx1262_emu_t e;
    sx1262_t r;

    emu_reset_world();
    emu_add_radio(&e, 0);
    e.present = false;
    r = radio_on(&e);
    CHECK(sx1262_init(&r, &sf7, 915000000u) == SX1262_ERR_NOT_FOUND, "missing chip not detected");

    emu_reset_world();
    emu_add_radio(&e, 0);
    e.busy_stuck = true;
    r = radio_on(&e);
    CHECK(sx1262_init(&r, &sf7, 915000000u) == SX1262_ERR_BUSY, "stuck BUSY not detected");

    /* forced clock settings that don't match the board */
    emu_reset_world();
    emu_add_radio(&e, 0);
    e.has_tcxo = true;
    r = radio_on(&e);
    r.clock = SX1262_CLOCK_XTAL;
    CHECK(sx1262_init(&r, &sf7, 915000000u) == SX1262_ERR_OSC, "TCXO board forced to crystal");

    emu_reset_world();
    emu_add_radio(&e, 0);
    r = radio_on(&e);
    r.clock = SX1262_CLOCK_TCXO;
    CHECK(sx1262_init(&r, &sf7, 915000000u) == SX1262_ERR_OSC, "crystal board forced to TCXO");

    /* auto-detect finds either clock */
    emu_reset_world();
    emu_add_radio(&e, 0);
    r = radio_on(&e);
    CHECK(sx1262_init(&r, &sf7, 915000000u) == SX1262_OK && !r.has_tcxo && !e.tcxo_cfg,
          "auto on crystal board");
    NO_VIOLATIONS(&e);

    emu_reset_world();
    emu_add_radio(&e, 0);
    e.has_tcxo = true;
    r = radio_on(&e);
    CHECK(sx1262_init(&r, &sf7, 915000000u) == SX1262_OK && r.has_tcxo && e.tcxo_cfg,
          "auto on TCXO board");
    NO_VIOLATIONS(&e);

    sx1262_lora_cfg_t bad = sf7;
    bad.bandwidth_hz = 200000;
    CHECK(sx1262_init(&r, &bad, 915000000u) == SX1262_ERR_BAD_CONFIG, "bad bandwidth accepted");
    bad = sf7;
    bad.tx_power_dbm = 23;
    CHECK(sx1262_init(&r, &bad, 915000000u) == SX1262_ERR_BAD_CONFIG, "23 dBm accepted");
    CHECK(sx1262_init(&r, &sf7, 868000000u) == SX1262_ERR_BAD_CONFIG, "868 MHz accepted");
}

static void test_send_receive_and_retune(void)
{
    printf("send / receive / retune while listening\n");
    emu_reset_world();
    static sx1262_emu_t ea, eb;
    emu_add_radio(&ea, 0);
    emu_add_radio(&eb, 1);
    sx1262_t a = radio_on(&ea), b = radio_on(&eb);
    sx1262_init(&a, &sf7, 915000000u);
    sx1262_init(&b, &sf7, 915000000u);
    sx1262_start_rx(&b);

    const uint8_t msg[] = "hello";
    CHECK(sx1262_send(&a, msg, sizeof(msg)) == SX1262_OK, "send");
    CHECK(ea.mode == EMU_RX, "sender not back in RX");
    uint8_t buf[32];
    int16_t rssi = 0;
    int n = sx1262_read_packet(&b, buf, sizeof(buf), &rssi);
    CHECK(n == (int)sizeof(msg) && memcmp(buf, msg, sizeof(msg)) == 0, "payload mismatch (n=%d)", n);
    CHECK(rssi == -57, "rssi %d", rssi);
    CHECK(sx1262_read_packet(&b, buf, sizeof(buf), NULL) == 0, "packet read twice");

    sx1262_set_frequency(&b, 903000000u);
    CHECK(eb.mode == EMU_RX, "retune dropped out of RX");
    CHECK(eb.rf_freq == sx1262_rf_freq_from_hz(903000000u), "retune freq");

    sx1262_send(&a, msg, sizeof(msg));
    CHECK(sx1262_read_packet(&b, buf, sizeof(buf), NULL) == 0, "received on wrong channel");
    NO_VIOLATIONS(&ea);
    NO_VIOLATIONS(&eb);
}


static void test_bringup_checks(void)
{
    printf("bring-up checks: PLL lock on all 50 channels, RSSI, PLL failure\n");
    emu_reset_world();
    static sx1262_emu_t e;
    emu_add_radio(&e, 0);
    sx1262_t r = radio_on(&e);
    sx1262_init(&r, &sf7, 915000000u);
    sx1262_start_rx(&r);

    int locked = 0;
    for (uint32_t ch = 0; ch < 50; ch++) {
        locked += sx1262_check_pll(&r, 902200000u + ch * 500000u) == SX1262_OK;
    }
    CHECK(locked == 50, "PLL locked on %d of 50 channels", locked);
    CHECK(e.mode == EMU_RX, "check_pll did not return to RX");
    CHECK(sx1262_rssi_inst(&r) == -110, "rssi %d", sx1262_rssi_inst(&r));
    CHECK(sx1262_device_errors(&r) == 0, "device errors %04X", sx1262_device_errors(&r));
    NO_VIOLATIONS(&e);

    e.pll_fail = true;
    CHECK(sx1262_check_pll(&r, 915000000u) == SX1262_ERR_PLL, "PLL failure not reported");
}

/* ---- link end to end --------------------------------------------------- */

typedef struct {
    int received;
    char last[32];
} rx_sink_t;

static void on_rx(void *user, const uint8_t *data, uint8_t len, int16_t rssi)
{
    (void)rssi;
    rx_sink_t *s = user;
    s->received++;
    memcpy(s->last, data, len < 31 ? len : 31);
    s->last[len < 31 ? len : 31] = 0;
}

static fhss_link_config_t link_cfg(sx1262_emu_t *e, bool leader, rx_sink_t *sink)
{
    fhss_link_config_t c = {
        .spi = &e->spi, .nss_port = &e->port, .nss_pin = GPIO_PIN_NSS,
        .rst_port = &e->port, .rst_pin = GPIO_PIN_RST,
        .busy_port = &e->port, .busy_pin = GPIO_PIN_BUSY,
        .ant_pwr_port = &e->port, .ant_pwr_pin = GPIO_PIN_ANT,
        .leader = leader, .seed = 0x6A5D10u, .net_id = 0x47, .tx_power_dbm = 14,
        .on_rx = on_rx, .user = sink,
    };
    return c;
}

static uint32_t freq_at(const sx1262_emu_t *e, uint32_t t)
{
    uint32_t f = 0;
    for (int i = 0; i < e->freq_log_n && e->freq_log[i].t_ms <= t; i++) {
        f = e->freq_log[i].rf_freq;
    }
    return f;
}

static void test_link_end_to_end(void)
{
    printf("leader + follower over emulated air (3 min, with 3 s outage)\n");
    emu_reset_world();
    static sx1262_emu_t ea, eb;
    static fhss_link_t lead, fol;
    static rx_sink_t lead_rx, fol_rx;
    memset(&lead_rx, 0, sizeof(lead_rx));
    memset(&fol_rx, 0, sizeof(fol_rx));
    emu_add_radio(&ea, 0);
    emu_add_radio(&eb, 1);

    fhss_link_config_t lc = link_cfg(&ea, true, &lead_rx);
    fhss_link_config_t fc = link_cfg(&eb, false, &fol_rx);
    CHECK(fhss_link_init(&lead, &lc) == FHSS_LINK_OK, "leader init");

    const uint32_t follower_boot = 1234, outage_start = 90000, outage_end = 93000;
    const uint32_t end = 180000;
    bool fol_up = false;
    uint32_t lock1 = 0, lock2 = 0, next_msg = 2000;
    int sent_l = 0, sent_f = 0, sent_l_locked = 0, sent_f_locked = 0;
    uint32_t mismatch_samples = 0, locked_samples = 0, locked_ms = 0, last_sample = 0;
    bool was_locked = false;

    while (emu_now_ms < end) {
        if (!fol_up && emu_now_ms >= follower_boot) {
            CHECK(fhss_link_init(&fol, &fc) == FHSS_LINK_OK, "follower init");
            fol_up = true;
        }
        emu_air_enabled = !(emu_now_ms >= outage_start && emu_now_ms < outage_end);

        if (emu_now_ms >= next_msg) {
            char m[16];
            int n = snprintf(m, sizeof(m), "L%d", sent_l);
            fhss_link_send(&lead, (const uint8_t *)m, (uint8_t)n);
            sent_l++;
            if (fol_up && fhss_link_locked(&fol)) {
                sent_l_locked++;
                n = snprintf(m, sizeof(m), "F%d", sent_f);
                fhss_link_send(&fol, (const uint8_t *)m, (uint8_t)n);
                sent_f++;
                sent_f_locked++;
            }
            next_msg += 1000;
        }

        fhss_link_poll(&lead);
        if (fol_up) {
            fhss_link_poll(&fol);
            bool locked = fhss_link_locked(&fol);
            if (locked && !was_locked) {
                if (!lock1) {
                    lock1 = emu_now_ms;
                } else if (emu_now_ms > outage_end && !lock2) {
                    lock2 = emu_now_ms;
                }
            }
            /* measure by the clock: it jumps forward while a node transmits */
            if (was_locked && locked) {
                locked_ms += emu_now_ms - last_sample;
            }
            last_sample = emu_now_ms;
            was_locked = locked;
            if (locked && emu_air_enabled) {
                locked_samples++;
                if (ea.rf_freq != eb.rf_freq && ea.mode == EMU_RX) {
                    mismatch_samples++;
                }
            }
        }
        emu_now_ms++;
    }

    printf("  follower locked at %u ms (boot %u), relocked at %u ms (outage end %u)\n",
           lock1, follower_boot, lock2, outage_end);
    printf("  leader->follower %d/%d delivered, follower->leader %d/%d delivered\n",
           fol_rx.received, sent_l, lead_rx.received, sent_f);
    printf("  locked %u ms total, out of step in %u of %u samples, %d transmissions checked\n",
           locked_ms, mismatch_samples, locked_samples, emu_tx_log_n);

    CHECK(lock1 && lock1 - follower_boot <= 20000 + 400, "first lock took %u ms",
          lock1 - follower_boot);
    CHECK(lock2 && lock2 - outage_end <= 20000 + 400 + 3 * 400, "relock took %u ms",
          lock2 - outage_end);
    CHECK(mismatch_samples * 100 <= locked_samples, "out of step %u of %u samples",
          mismatch_samples, locked_samples);
    /* Once locked, the follower may only lose lock during the outage. */
    uint32_t expected_locked = (end - lock1) - (lock2 - outage_start);
    CHECK(locked_ms + 500 >= expected_locked, "follower only locked %u of ~%u ms", locked_ms,
          expected_locked);
    CHECK(sent_l_locked >= 140, "only %d leader messages sent while locked", sent_l_locked);

    /* Everything sent while locked, outside the outage, should arrive. */
    int lost_l = sent_l_locked - fol_rx.received;
    int lost_f = sent_f_locked - lead_rx.received;
    CHECK(lost_l >= 0 && lost_l <= 4, "leader->follower lost %d of %d", lost_l, sent_l_locked);
    CHECK(lost_f >= 0 && lost_f <= 4, "follower->leader lost %d of %d", lost_f, sent_f_locked);
    CHECK(lead.stats.tx_errors == 0 && fol.stats.tx_errors == 0, "tx errors");
    CHECK(lead.stats.rx_crc_errors == 0 && fol.stats.rx_crc_errors == 0, "crc errors");

    /* Leader's radio hops every 400 ms exactly (after the init retune). */
    for (int i = 2; i < ea.freq_log_n; i++) {
        uint32_t d = ea.freq_log[i].t_ms - ea.freq_log[i - 1].t_ms;
        CHECK(d == 400, "leader radio dwell %u ms at hop %d", d, i);
    }

    /* Every transmission fits inside one leader dwell, on the leader's channel,
     * and followers only transmit once locked. */
    for (int i = 0; i < emu_tx_log_n; i++) {
        const emu_tx_event_t *t = &emu_tx_log[i];
        bool hop_inside = false;
        for (int k = 0; k < ea.freq_log_n; k++) {
            if (ea.freq_log[k].t_ms > t->start_ms && ea.freq_log[k].t_ms < t->end_ms) {
                hop_inside = true;
            }
        }
        CHECK(!hop_inside, "tx %d (radio %d) straddles a hop", i, t->radio_id);
        CHECK(t->rf_freq == freq_at(&ea, t->start_ms), "tx %d (radio %d) on wrong channel", i,
              t->radio_id);
        if (t->radio_id == 1) {
            CHECK(t->start_ms >= lock1, "follower transmitted before lock");
        }
    }
    NO_VIOLATIONS(&ea);
    NO_VIOLATIONS(&eb);
}


static void test_link_max_payload(void)
{
    printf("max-size payloads (31 B) back to back: every packet inside one dwell\n");
    emu_reset_world();
    static sx1262_emu_t ea, eb;
    static fhss_link_t lead, fol;
    static rx_sink_t lead_rx, fol_rx;
    memset(&lead_rx, 0, sizeof(lead_rx));
    memset(&fol_rx, 0, sizeof(fol_rx));
    emu_add_radio(&ea, 0);
    emu_add_radio(&eb, 1);
    fhss_link_config_t lc = link_cfg(&ea, true, &lead_rx), fc = link_cfg(&eb, false, &fol_rx);
    fhss_link_init(&lead, &lc);
    fhss_link_init(&fol, &fc);

    uint8_t big[FHSS_LINK_MAX_PAYLOAD];
    memset(big, 0x5A, sizeof(big));
    uint32_t t_lock = 0;
    while (emu_now_ms < 90000) {
        if (fhss_link_locked(&fol) && !t_lock) {
            t_lock = emu_now_ms;
        }
        /* keep both queues topped up */
        if (lead.q_count < FHSS_LINK_QUEUE_LEN) fhss_link_send(&lead, big, sizeof(big));
        if (t_lock && fol.q_count < FHSS_LINK_QUEUE_LEN) fhss_link_send(&fol, big, sizeof(big));
        fhss_link_poll(&lead);
        fhss_link_poll(&fol);
        emu_now_ms++;
    }
    int straddle = 0, wrong = 0;
    for (int i = 0; i < emu_tx_log_n; i++) {
        const emu_tx_event_t *t = &emu_tx_log[i];
        for (int k = 0; k < ea.freq_log_n; k++) {
            if (ea.freq_log[k].t_ms > t->start_ms && ea.freq_log[k].t_ms < t->end_ms) straddle++;
        }
        wrong += t->rf_freq != freq_at(&ea, t->start_ms);
    }
    uint32_t locked_s = (90000 - t_lock) / 1000;
    printf("  %d packets on air, leader->follower %d, follower->leader %d over %u s locked\n",
           emu_tx_log_n, fol_rx.received, lead_rx.received, locked_s);
    CHECK(t_lock && straddle == 0 && wrong == 0, "straddle %d, wrong channel %d", straddle, wrong);
    /* 2 full packets per node per dwell = 5 per second each way */
    CHECK(fol_rx.received >= (int)(locked_s * 5) - 5, "leader->follower only %d", fol_rx.received);
    CHECK(lead_rx.received >= (int)(locked_s * 5) - 5, "follower->leader only %d", lead_rx.received);
    NO_VIOLATIONS(&ea);
    NO_VIOLATIONS(&eb);
}

static void test_foreign_network_ignored(void)
{
    printf("packets with another net_id are ignored\n");
    emu_reset_world();
    static sx1262_emu_t ea, eb;
    static fhss_link_t lead, fol;
    static rx_sink_t sink;
    memset(&sink, 0, sizeof(sink));
    emu_add_radio(&ea, 0);
    emu_add_radio(&eb, 1);
    fhss_link_config_t lc = link_cfg(&ea, true, &sink);
    fhss_link_config_t fc = link_cfg(&eb, false, &sink);
    fc.net_id = 0x99;
    fhss_link_init(&lead, &lc);
    fhss_link_init(&fol, &fc);
    while (emu_now_ms < 45000) {
        fhss_link_poll(&lead);
        fhss_link_poll(&fol);
        emu_now_ms++;
    }
    CHECK(!fhss_link_locked(&fol), "locked onto a foreign network");
    CHECK(fol.stats.rx_foreign > 0, "never saw the foreign beacons");
}

/* ---- CAN bridge over the link ---------------------------------------- */

typedef struct {
    fhss_link_t *link;
    can_bridge_t bridge;
    int got[16];           /* messages received per hub */
    int bad;               /* payloads that failed to decode or verify */
    uint32_t max_age_ms;   /* oldest reading delivered */
    uint16_t last_reading[16];
    int out_of_order;      /* a hub's reading arrived older than the last one */
} bridge_node_t;

static bool bridge_radio_idle(void *user)
{
    return ((bridge_node_t *)user)->link->q_count == 0;
}

static bool bridge_radio_send(void *user, const uint8_t *data, uint8_t len)
{
    return fhss_link_send(((bridge_node_t *)user)->link, data, len);
}

/* every reading in a test message is the send time in units of 10 ms */
static void bridge_on_rx(void *user, const uint8_t *data, uint8_t len, int16_t rssi)
{
    (void)rssi;
    bridge_node_t *n = user;
    can_hub_msg_t m;
    uint8_t frames[CAN_MSG_MAX_FRAMES][8], dlc[CAN_MSG_MAX_FRAMES];
    uint16_t id;
    uint8_t nf = can_bridge_from_radio(&n->bridge, data, len, &m, frames, dlc, &id);
    if (nf == 0 || m.count != 10 || id != 0x200 + m.hub) {
        n->bad++;
        return;
    }
    for (uint8_t i = 1; i < m.count; i++) {
        if (m.sensor[i].reading != m.sensor[0].reading || m.sensor[i].id != i) {
            n->bad++;
            return;
        }
    }
    uint32_t age = emu_now_ms - 10u * m.sensor[0].reading;
    n->max_age_ms = age > n->max_age_ms ? age : n->max_age_ms;
    if (n->got[m.hub & 15] > 0 && m.sensor[0].reading <= n->last_reading[m.hub & 15]) {
        n->out_of_order++;
    }
    n->last_reading[m.hub & 15] = m.sensor[0].reading;
    n->got[m.hub & 15]++;
}

/* Hubs 3 and 6 each send a 10-sensor message every 50 ms, offset by 25 ms.
 * Catches up on every millisecond since the last call, so traffic keeps
 * arriving while a transmission blocks the main loop (via the tx-wait hook). */
static uint32_t hub_next_ms;
static int hub_msgs;

static void hub_traffic(void *user)
{
    bridge_node_t *pod = user;
    for (; hub_next_ms <= emu_now_ms; hub_next_ms += 25) {
        uint8_t hub = hub_next_ms % 50 == 0 ? 3 : 6;
        can_hub_msg_t m = {.hub = hub, .count = 10};
        for (uint8_t i = 0; i < 10; i++) {
            m.sensor[i] = (can_sensor_t){.type = 1, .id = i, .reading = (uint16_t)(hub_next_ms / 10)};
        }
        uint8_t buf[CAN_MSG_MAX_BYTES], data[CAN_MSG_MAX_FRAMES][8], dlc[CAN_MSG_MAX_FRAMES];
        uint8_t nf = can_msg_to_frames(buf, can_msg_encode(&m, buf, sizeof(buf)), data, dlc);
        for (uint8_t f = 0; f < nf; f++) {
            can_bridge_on_frame(&pod->bridge, (uint16_t)(0x100 + hub), data[f], dlc[f], emu_now_ms);
        }
        hub_msgs++;
    }
}

static void test_can_bridge_over_link(void)
{
    printf("CAN bridge: two hubs on the pod's bus -> hopping link -> ground (60 s)\n");
    emu_reset_world();
    static sx1262_emu_t ea, eb;
    static fhss_link_t pod_link, gnd_link;
    static bridge_node_t pod, gnd;
    memset(&pod, 0, sizeof(pod));
    memset(&gnd, 0, sizeof(gnd));
    pod.link = &pod_link;
    gnd.link = &gnd_link;
    emu_add_radio(&ea, 0);
    emu_add_radio(&eb, 1);

    /* 40 messages a second against a link that carries about 5: more than
     * any buffer can deliver, so this is the latest_only case. */
    can_bridge_config_t bc = {
        .rx_id_mask = 0x7F0, .rx_id_match = 0x100, .tx_id_base = 0x200,
        .max_payload = FHSS_LINK_MAX_PAYLOAD, .latest_only = true,
        .radio_idle = bridge_radio_idle, .radio_send = bridge_radio_send,
    };
    bc.user = &pod;
    CHECK(can_bridge_init(&pod.bridge, &bc), "pod bridge init");
    bc.user = &gnd;
    CHECK(can_bridge_init(&gnd.bridge, &bc), "ground bridge init");

    /* pod = leader (its beacons carry the telemetry), ground = follower */
    fhss_link_config_t pc = link_cfg(&ea, true, NULL), gc = link_cfg(&eb, false, NULL);
    pc.on_rx = bridge_on_rx;
    pc.user = &pod;
    pc.tx_wait_hook = hub_traffic;
    gc.on_rx = bridge_on_rx;
    gc.user = &gnd;
    CHECK(fhss_link_init(&pod_link, &pc) == FHSS_LINK_OK, "pod link init");
    CHECK(fhss_link_init(&gnd_link, &gc) == FHSS_LINK_OK, "ground link init");

    uint32_t t_lock = 0;
    hub_next_ms = 0;
    hub_msgs = 0;
    while (emu_now_ms < 60000) {
        if (!t_lock && fhss_link_locked(&gnd_link)) {
            t_lock = emu_now_ms;
        }
        hub_traffic(&pod);
        can_bridge_poll(&pod.bridge);
        fhss_link_poll(&pod_link);
        fhss_link_poll(&gnd_link);
        emu_now_ms++;
    }
    int delivered = gnd.got[3] + gnd.got[6];
    uint32_t locked_s = t_lock ? (60000 - t_lock) / 1000 : 0;
    printf("  %d hub messages on CAN, %u handed to radio, %d delivered over %u s locked "
           "(hub 3: %d, hub 6: %d), oldest reading %u ms\n",
           hub_msgs, pod.bridge.stats.messages_sent, delivered, locked_s, gnd.got[3], gnd.got[6],
           gnd.max_age_ms);
    CHECK(t_lock, "ground never locked");
    CHECK(gnd.bad == 0 && gnd.bridge.stats.radio_bad == 0, "%d bad payloads", gnd.bad);
    CHECK(pod.bridge.stats.messages_in == (uint32_t)hub_msgs && pod.bridge.stats.reasm_errors == 0,
          "pod reassembled %u of %d", pod.bridge.stats.messages_in, hub_msgs);
    CHECK(pod_link.stats.tx_dropped == 0 && pod.bridge.stats.radio_refused == 0,
          "radio queue overflowed");
    /* about two 31-byte packets per leader half-dwell = 5 per second */
    CHECK(delivered >= (int)locked_s * 4, "only %d delivered in %u s", delivered, locked_s);
    CHECK(gnd.got[3] * 10 >= delivered * 4 && gnd.got[6] * 10 >= delivered * 4,
          "hubs not sharing the link: %d vs %d", gnd.got[3], gnd.got[6]);
    CHECK(hub_msgs >= 2390, "only %d hub messages generated", hub_msgs);
    /* Fresh data. Worst case: a message completes just after the leader's
     * half-dwell closes (~290 ms wait), then waits behind the other hub's
     * packet (~75 ms) and goes out (~75 ms), plus slot margins. */
    CHECK(gnd.max_age_ms <= 550, "oldest delivered reading %u ms", gnd.max_age_ms);
    NO_VIOLATIONS(&ea);
    NO_VIOLATIONS(&eb);
}

/* Hubs 3 and 6 take turns, one 10-sensor message every SLOW_PERIOD_MS between
 * them: about 3.3 a second, inside what the link carries. Only runs between
 * slow_start_ms and slow_stop_ms, so it starts once the ground is locked and
 * stops early enough for the queue to drain. */
#define SLOW_PERIOD_MS 300u
#define BURST_MSGS 12          /* sent all at once, 10 s into the traffic */
static uint32_t slow_next_ms, slow_start_ms, slow_stop_ms;
static int slow_msgs;
static bool burst_done;

static void emit_hub_msg(bridge_node_t *pod, uint8_t hub, uint16_t reading)
{
    can_hub_msg_t m = {.hub = hub, .count = 10};
    for (uint8_t i = 0; i < 10; i++) {
        m.sensor[i] = (can_sensor_t){.type = 1, .id = i, .reading = reading};
    }
    uint8_t buf[CAN_MSG_MAX_BYTES], data[CAN_MSG_MAX_FRAMES][8], dlc[CAN_MSG_MAX_FRAMES];
    uint8_t nf = can_msg_to_frames(buf, can_msg_encode(&m, buf, sizeof(buf)), data, dlc);
    for (uint8_t f = 0; f < nf; f++) {
        can_bridge_on_frame(&pod->bridge, (uint16_t)(0x100 + hub), data[f], dlc[f], emu_now_ms);
    }
    slow_msgs++;
}

static void slow_hub_traffic(void *user)
{
    bridge_node_t *pod = user;
    if (!slow_start_ms) {
        return;
    }
    if (slow_next_ms < slow_start_ms) {
        slow_next_ms = slow_start_ms;
    }
    for (; slow_next_ms <= emu_now_ms && slow_next_ms < slow_stop_ms; slow_next_ms += SLOW_PERIOD_MS) {
        emit_hub_msg(pod, (slow_next_ms / SLOW_PERIOD_MS) % 2 ? 3 : 6, (uint16_t)(slow_next_ms / 10));
        /* A burst, as when the VCU reads every hub back to back: far more
         * than the link carries at that moment, so it must wait in the
         * queue. Readings step 10 ms apart after this message's, so each
         * hub's readings still only go up. */
        if (!burst_done && slow_next_ms >= slow_start_ms + 10000) {
            burst_done = true;
            for (uint16_t k = 1; k <= BURST_MSGS; k++) {
                emit_hub_msg(pod, k % 2 ? 3 : 6, (uint16_t)(slow_next_ms / 10 + k));
            }
        }
    }
}

static void test_can_bridge_lossless_over_link(void)
{
    printf("CAN bridge, default mode: traffic the link can carry arrives complete (60 s)\n");
    emu_reset_world();
    static sx1262_emu_t ea, eb;
    static fhss_link_t pod_link, gnd_link;
    static bridge_node_t pod, gnd;
    memset(&pod, 0, sizeof(pod));
    memset(&gnd, 0, sizeof(gnd));
    pod.link = &pod_link;
    gnd.link = &gnd_link;
    emu_add_radio(&ea, 0);
    emu_add_radio(&eb, 1);

    can_bridge_config_t bc = {
        .rx_id_mask = 0x7F0, .rx_id_match = 0x100, .tx_id_base = 0x200,
        .max_payload = FHSS_LINK_MAX_PAYLOAD,
        .radio_idle = bridge_radio_idle, .radio_send = bridge_radio_send,
    };
    bc.user = &pod;
    CHECK(can_bridge_init(&pod.bridge, &bc), "pod bridge init");
    bc.user = &gnd;
    CHECK(can_bridge_init(&gnd.bridge, &bc), "ground bridge init");

    fhss_link_config_t pc = link_cfg(&ea, true, NULL), gc = link_cfg(&eb, false, NULL);
    pc.on_rx = bridge_on_rx;
    pc.user = &pod;
    pc.tx_wait_hook = slow_hub_traffic;
    gc.on_rx = bridge_on_rx;
    gc.user = &gnd;
    CHECK(fhss_link_init(&pod_link, &pc) == FHSS_LINK_OK, "pod link init");
    CHECK(fhss_link_init(&gnd_link, &gc) == FHSS_LINK_OK, "ground link init");

    slow_next_ms = slow_start_ms = 0;
    slow_stop_ms = 50000;
    slow_msgs = 0;
    burst_done = false;
    while (emu_now_ms < 60000) {
        if (!slow_start_ms && fhss_link_locked(&gnd_link)) {
            slow_start_ms = emu_now_ms + 1000; /* traffic starts once locked */
        }
        slow_hub_traffic(&pod);
        can_bridge_poll(&pod.bridge);
        fhss_link_poll(&pod_link);
        fhss_link_poll(&gnd_link);
        emu_now_ms++;
    }
    const can_bridge_stats_t *s = &pod.bridge.stats;
    int delivered = gnd.got[3] + gnd.got[6];
    printf("  %d hub messages on CAN, %u sent, %d delivered, queue peak %u, "
           "oldest reading %u ms\n",
           slow_msgs, s->messages_sent, delivered, s->queue_peak, gnd.max_age_ms);
    CHECK(slow_start_ms, "ground never locked");
    CHECK(slow_msgs >= 100 && burst_done, "only %d messages generated", slow_msgs);
    CHECK(s->queue_peak >= BURST_MSGS / 2, "the burst never queued (peak %u)", s->queue_peak);
    CHECK(s->messages_in == (uint32_t)slow_msgs && s->reasm_errors == 0 && s->no_slot == 0,
          "pod reassembled %u of %d", s->messages_in, slow_msgs);
    CHECK(s->queue_full == 0 && s->superseded == 0, "dropped %u, replaced %u", s->queue_full,
          s->superseded);
    CHECK(delivered == slow_msgs, "every message must arrive: %d of %d", delivered, slow_msgs);
    CHECK(gnd.bad == 0 && gnd.out_of_order == 0, "%d bad, %d out of order", gnd.bad,
          gnd.out_of_order);
    CHECK(pod_link.stats.tx_dropped == 0, "radio queue overflowed");
    NO_VIOLATIONS(&ea);
    NO_VIOLATIONS(&eb);
}

int main(void)
{
    test_freq_and_airtime();
    test_init_state();
    test_init_errors();
    test_send_receive_and_retune();
    test_bringup_checks();
    test_link_end_to_end();
    test_link_max_payload();
    test_foreign_network_ignored();
    test_can_bridge_over_link();
    test_can_bridge_lossless_over_link();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
