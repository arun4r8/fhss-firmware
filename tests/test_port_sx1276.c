/*
 * Host tests for the STM32F446RE + SX1276 port: real driver and link code,
 * emulated radios on a shared virtual air (tests/mocks).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fhss_link.h"
#include "sx1276.h"
#include "sx1276_emu.h"

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

static const sx1276_lora_cfg_t sf7 = {
    .spreading_factor = 7, .bandwidth_hz = 125000, .coding_rate = 1,
    .preamble_len = 8, .sync_word = 0x12, .tx_power_dbm = 14,
};

static sx1276_t radio_on(sx1276_emu_t *e)
{
    sx1276_t r = {
        .spi = &e->spi, .nss_port = &e->port, .nss_pin = GPIO_PIN_NSS,
        .rst_port = &e->port, .rst_pin = GPIO_PIN_RST,
    };
    return r;
}

/* ---- driver ------------------------------------------------------------ */

static void test_frf_and_airtime(void)
{
    printf("FRF register math and LoRa airtime\n");
    CHECK(sx1276_frf_from_hz(915000000u) == 0xE4C000u, "915 MHz FRF %06X",
          sx1276_frf_from_hz(915000000u));
    /* Semtech LoRa calculator: SF7/125k/4-5/8 preamble/10 B = 41.216 ms */
    CHECK(sx1276_airtime_ms(&sf7, 10) == 42, "SF7 10 B airtime %u", sx1276_airtime_ms(&sf7, 10));
    /* SF12/125k needs low data rate optimize: 991.232 ms */
    sx1276_lora_cfg_t sf12 = sf7;
    sf12.spreading_factor = 12;
    CHECK(sx1276_airtime_ms(&sf12, 10) == 992, "SF12 10 B airtime %u",
          sx1276_airtime_ms(&sf12, 10));
}

static void test_init_registers(void)
{
    printf("init programs LoRa registers\n");
    emu_reset_world();
    static sx1276_emu_t e;
    emu_add_radio(&e, 0);
    sx1276_t r = radio_on(&e);
    CHECK(sx1276_init(&r, &sf7, 915000000u) == SX1276_OK, "init failed");
    CHECK(e.regs[0x01] == 0x81, "op mode %02X (want LoRa standby)", e.regs[0x01]);
    CHECK(emu_frf(&e) == 0xE4C000u, "FRF %06X", emu_frf(&e));
    CHECK(e.regs[0x1D] == 0x72, "modem cfg1 %02X", e.regs[0x1D]);
    CHECK(e.regs[0x1E] == 0x74, "modem cfg2 %02X", e.regs[0x1E]);
    CHECK(e.regs[0x26] == 0x04, "modem cfg3 %02X", e.regs[0x26]);
    CHECK(e.regs[0x39] == 0x12, "sync word %02X", e.regs[0x39]);
    CHECK(e.regs[0x09] == 0xFC, "PA config %02X (want PA_BOOST 14 dBm)", e.regs[0x09]);
}

static void test_init_errors(void)
{
    printf("init reports missing chip and bad config\n");
    emu_reset_world();
    static sx1276_emu_t e;
    emu_add_radio(&e, 0);
    e.present = false;
    sx1276_t r = radio_on(&e);
    CHECK(sx1276_init(&r, &sf7, 915000000u) == SX1276_ERR_NOT_FOUND, "missing chip not detected");

    e.present = true;
    sx1276_lora_cfg_t bad = sf7;
    bad.bandwidth_hz = 200000;
    CHECK(sx1276_init(&r, &bad, 915000000u) == SX1276_ERR_BAD_CONFIG, "bad bandwidth accepted");
    bad = sf7;
    bad.tx_power_dbm = 20;
    CHECK(sx1276_init(&r, &bad, 915000000u) == SX1276_ERR_BAD_CONFIG, "20 dBm accepted");
}

static void test_send_receive_and_retune(void)
{
    printf("send / receive / retune while listening\n");
    emu_reset_world();
    static sx1276_emu_t ea, eb;
    emu_add_radio(&ea, 0);
    emu_add_radio(&eb, 1);
    sx1276_t a = radio_on(&ea), b = radio_on(&eb);
    sx1276_init(&a, &sf7, 915000000u);
    sx1276_init(&b, &sf7, 915000000u);
    sx1276_start_rx(&b);

    const uint8_t msg[] = "hello";
    CHECK(sx1276_send(&a, msg, sizeof(msg)) == SX1276_OK, "send");
    CHECK(emu_mode(&ea) == 0x05, "sender not back in RX");
    uint8_t buf[32];
    int16_t rssi = 0;
    int n = sx1276_read_packet(&b, buf, sizeof(buf), &rssi);
    CHECK(n == (int)sizeof(msg) && memcmp(buf, msg, sizeof(msg)) == 0, "payload mismatch (n=%d)", n);
    CHECK(rssi == -57, "rssi %d", rssi);
    CHECK(sx1276_read_packet(&b, buf, sizeof(buf), NULL) == 0, "packet read twice");

    sx1276_set_frequency(&b, 903000000u);
    CHECK(emu_mode(&eb) == 0x05, "retune dropped out of RX");
    CHECK(emu_frf(&eb) == sx1276_frf_from_hz(903000000u), "retune FRF");

    sx1276_send(&a, msg, sizeof(msg));
    CHECK(sx1276_read_packet(&b, buf, sizeof(buf), NULL) == 0, "received on wrong channel");
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

static fhss_link_config_t link_cfg(sx1276_emu_t *e, bool leader, rx_sink_t *sink)
{
    fhss_link_config_t c = {
        .spi = &e->spi, .nss_port = &e->port, .nss_pin = GPIO_PIN_NSS,
        .rst_port = &e->port, .rst_pin = GPIO_PIN_RST,
        .leader = leader, .seed = 0x6A5D10u, .net_id = 0x47, .tx_power_dbm = 14,
        .on_rx = on_rx, .user = sink,
    };
    return c;
}

static uint32_t frf_at(const sx1276_emu_t *e, uint32_t t)
{
    uint32_t frf = 0;
    for (int i = 0; i < e->frf_log_n && e->frf_log[i].t_ms <= t; i++) {
        frf = e->frf_log[i].frf;
    }
    return frf;
}

static void test_link_end_to_end(void)
{
    printf("leader + follower over emulated air (3 min, with 3 s outage)\n");
    emu_reset_world();
    static sx1276_emu_t ea, eb;
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
                if (emu_frf(&ea) != emu_frf(&eb) && emu_mode(&ea) == 0x05) {
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
    for (int i = 2; i < ea.frf_log_n; i++) {
        uint32_t d = ea.frf_log[i].t_ms - ea.frf_log[i - 1].t_ms;
        CHECK(d == 400, "leader radio dwell %u ms at hop %d", d, i);
    }

    /* Every transmission fits inside one leader dwell, on the leader's channel,
     * and followers only transmit once locked. */
    for (int i = 0; i < emu_tx_log_n; i++) {
        const emu_tx_event_t *t = &emu_tx_log[i];
        bool hop_inside = false;
        for (int k = 0; k < ea.frf_log_n; k++) {
            if (ea.frf_log[k].t_ms > t->start_ms && ea.frf_log[k].t_ms < t->end_ms) {
                hop_inside = true;
            }
        }
        CHECK(!hop_inside, "tx %d (radio %d) straddles a hop", i, t->radio_id);
        CHECK(t->frf == frf_at(&ea, t->start_ms), "tx %d (radio %d) on wrong channel", i,
              t->radio_id);
        if (t->radio_id == 1) {
            CHECK(t->start_ms >= lock1, "follower transmitted before lock");
        }
    }
}

static void test_foreign_network_ignored(void)
{
    printf("packets with another net_id are ignored\n");
    emu_reset_world();
    static sx1276_emu_t ea, eb;
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

int main(void)
{
    test_frf_and_airtime();
    test_init_registers();
    test_init_errors();
    test_send_receive_and_retune();
    test_link_end_to_end();
    test_foreign_network_ignored();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
