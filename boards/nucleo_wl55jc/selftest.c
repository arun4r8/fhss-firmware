/*
 * selftest.c - Single-board bring-up for NUCLEO-WL55JC + SX1262 shield.
 *
 * Prints results on the ST-LINK virtual COM port (115200 8N1):
 *   1. Radio init     SPI link, BUSY handshake, oscillator (auto crystal/TCXO)
 *   2. PLL lock       synthesizer locks on all 50 hop channels
 *   3. Noise floor    RSSI on every channel while listening (info)
 *   4. TX timing      5 packets on different channels; TxDone vs. airtime
 *   5. Device errors  none latched after the tests
 *   6. CAN loopback   MCP2515 answers on SPI and loops a frame back
 *                     (checks the controller only; the bus needs a 2nd node)
 * Then runs the hopping link as leader forever, printing one line per
 * 20 s cycle with the measured dwell times.
 *
 * LEDs: green = all checks passed, red = a check failed (or a fault),
 *       blue toggles on every hop.
 * Transmit power is kept at 0 dBm for bench use.
 */
#include <string.h>

#include "board.h"
#include "fhss_link.h"
#include "mcp2515.h"
#include "sx1262.h"

/* HW-184 crystal: confirmed 8.000 MHz (marking on the "OSC" can).
 * Bus bit rate: placeholder until the embedded team confirms it. */
#define CAN_OSC_HZ    8000000u
#define CAN_BITRATE   500000u

#define TX_DBM        0
#define BASE_HZ       902200000u
#define SPACING_HZ    500000u
#define NUM_CHANNELS  50u
#define TEST_PKT_LEN  30u
#define TX_TOLERANCE_MS 4

static const sx1262_lora_cfg_t lora = {
    .spreading_factor = 7,
    .bandwidth_hz = 125000,
    .coding_rate = 1,
    .preamble_len = 8,
    .sync_word = 0x12,
    .tx_power_dbm = TX_DBM,
};

static int failures;

static void check(const char *name, int ok)
{
    uart_puts(ok ? "  PASS  " : "  FAIL  ");
    uart_puts(name);
    uart_puts("\n");
    failures += !ok;
}

static void check_can(void);

static const char *init_hint(int err)
{
    switch (err) {
    case SX1262_ERR_NOT_FOUND: return "no SPI reply: shield seated? NSS on D7 (PC1)? SPI on D11-D13?";
    case SX1262_ERR_BUSY:      return "BUSY stuck high: check D3 (PB3) and RESET on A0 (PB1)";
    case SX1262_ERR_OSC:       return "oscillator did not start as crystal or TCXO";
    case SX1262_ERR_CALIB:     return "calibration failed: check 3V3 supply";
    default:                   return "";
    }
}

static uint32_t channel_hz(uint32_t ch)
{
    return BASE_HZ + ch * SPACING_HZ;
}

static int run_checks(sx1262_t *r)
{
    /* 1. init */
    int err = sx1262_init(r, &lora, BASE_HZ);
    if (err != SX1262_OK) {
        uart_puts("  FAIL  radio init, error ");
        uart_put_int(err);
        uart_puts(": ");
        uart_puts(init_hint(err));
        uart_puts("\n");
        return 0;
    }
    uart_puts(r->has_tcxo ? "  PASS  radio init (clock: TCXO)\n" : "  PASS  radio init (clock: crystal)\n");

    /* 2. PLL lock on every channel */
    uint32_t locked = 0;
    for (uint32_t ch = 0; ch < NUM_CHANNELS; ch++) {
        if (sx1262_check_pll(r, channel_hz(ch)) == SX1262_OK) {
            locked++;
        } else {
            uart_puts("        no lock on channel ");
            uart_put_int((int32_t)ch);
            uart_puts("\n");
        }
    }
    uart_puts("        PLL locked on ");
    uart_put_int((int32_t)locked);
    uart_puts(" / 50 channels\n");
    check("PLL lock", locked == NUM_CHANNELS);

    /* 3. noise floor while listening */
    sx1262_start_rx(r);
    int32_t lo = 0, hi = -200, sum = 0;
    for (uint32_t ch = 0; ch < NUM_CHANNELS; ch++) {
        sx1262_set_frequency(r, channel_hz(ch));
        HAL_Delay(3);
        int32_t rssi = sx1262_rssi_inst(r);
        lo = (ch == 0 || rssi < lo) ? rssi : lo;
        hi = rssi > hi ? rssi : hi;
        sum += rssi;
    }
    uart_puts("  INFO  noise floor dBm: min ");
    uart_put_int(lo);
    uart_puts(", avg ");
    uart_put_int(sum / (int32_t)NUM_CHANNELS);
    uart_puts(", max ");
    uart_put_int(hi);
    uart_puts(hi > -80 ? "  (strong signal on some channel: nearby transmitter?)\n" : "\n");

    /* 4. transmit timing, one packet per channel so we keep hopping */
    uint8_t pkt[TEST_PKT_LEN];
    for (uint32_t i = 0; i < TEST_PKT_LEN; i++) {
        pkt[i] = (uint8_t)i;
    }
    int32_t expected = (int32_t)sx1262_airtime_ms(&lora, TEST_PKT_LEN);
    int tx_ok = 1;
    uart_puts("        TX ");
    uart_put_int((int32_t)TEST_PKT_LEN);
    uart_puts(" B, expected ");
    uart_put_int(expected);
    uart_puts(" ms, measured:");
    for (uint32_t i = 0; i < 5; i++) {
        sx1262_set_frequency(r, channel_hz(i * 11u % NUM_CHANNELS));
        uint32_t t0 = HAL_GetTick();
        int e = sx1262_send(r, pkt, TEST_PKT_LEN);
        int32_t dt = (int32_t)(HAL_GetTick() - t0);
        uart_puts(" ");
        uart_put_int(dt);
        if (e != SX1262_OK) {
            uart_puts("(timeout)");
        }
        tx_ok &= e == SX1262_OK && dt >= expected - TX_TOLERANCE_MS && dt <= expected + TX_TOLERANCE_MS;
    }
    uart_puts("\n");
    check("TX timing", tx_ok);

    /* 5. nothing latched */
    uint16_t errs = sx1262_device_errors(r);
    if (errs) {
        uart_puts("        device errors 0x");
        uart_put_hex(errs, 4);
        uart_puts("\n");
    }
    check("device errors", errs == 0 && !r->busy_timeout);

    check_can();
    return 1;
}

static void check_can(void)
{
    mcp2515_t can = {
        .spi = &board_spi1, .cs_port = CAN_CS_PORT, .cs_pin = CAN_CS_PIN,
        .osc_hz = CAN_OSC_HZ, .bitrate = CAN_BITRATE, .mode = MCP2515_MODE_LOOPBACK,
    };
    int err = mcp2515_init(&can);
    if (err != MCP2515_OK) {
        uart_puts("  FAIL  CAN controller init, error ");
        uart_put_int(err);
        uart_puts(err == MCP2515_ERR_NOT_FOUND
                      ? ": no SPI reply from MCP2515 (CS on PA8? VCC 3.3 V? SCK/SI/SO wired?)\n"
                      : "\n");
        failures++;
        return;
    }
    can_frame_t rx, tx = {.id = 0x5A3, .dlc = 8, .data = {0xDE, 0xAD, 0xBE, 0xEF, 1, 2, 3, 4}};
    int ok = mcp2515_send(&can, &tx) == MCP2515_OK;
    uint32_t t0 = HAL_GetTick();
    int got = 0;
    while (ok && !got && HAL_GetTick() - t0 < 20) {
        got = mcp2515_receive(&can, &rx);
    }
    ok = ok && got && rx.id == tx.id && rx.dlc == 8 && memcmp(rx.data, tx.data, 8) == 0;
    check("CAN controller loopback (MCP2515)", ok);
}

static void run_leader(void)
{
    static fhss_link_t link;
    fhss_link_config_t cfg = {
        .spi = &board_spi1,
        .nss_port = RADIO_NSS_PORT, .nss_pin = RADIO_NSS_PIN,
        .rst_port = RADIO_RST_PORT, .rst_pin = RADIO_RST_PIN,
        .busy_port = RADIO_BUSY_PORT, .busy_pin = RADIO_BUSY_PIN,
        .ant_pwr_port = RADIO_ANT_PORT, .ant_pwr_pin = RADIO_ANT_PIN,
        .clock = SX1262_CLOCK_AUTO,
        .leader = 1,
        .seed = 0x6A5D10u,
        .net_id = 0x47,
        .tx_power_dbm = TX_DBM,
    };
    if (fhss_link_init(&link, &cfg) != FHSS_LINK_OK) {
        uart_puts("leader start failed, radio error ");
        uart_put_int(link.radio_err);
        uart_puts("\n");
        return;
    }
    uart_puts("\nHopping as leader (0 dBm, beacon every dwell). One line per 20 s cycle:\n");

    uint16_t last = link.fhss.hop_index;
    uint32_t last_hop_ms = HAL_GetTick(), hops = 0, cycle = 0;
    int32_t dmin = 100000, dmax = 0;
    for (;;) {
        fhss_link_poll(&link);
        if (link.fhss.hop_index == last) {
            continue;
        }
        uint32_t now = HAL_GetTick();
        int32_t dwell = (int32_t)(now - last_hop_ms);
        last_hop_ms = now;
        last = link.fhss.hop_index;
        led_toggle(LED_BLUE_PORT, LED_BLUE_PIN);
        if (hops++ > 0) { /* first interval starts mid-dwell */
            dmin = dwell < dmin ? dwell : dmin;
            dmax = dwell > dmax ? dwell : dmax;
        }
        if (last == 0 && hops > 1) {
            uart_puts("  cycle ");
            uart_put_int((int32_t)++cycle);
            uart_puts(": dwell ");
            uart_put_int(dmin);
            uart_puts("-");
            uart_put_int(dmax);
            uart_puts(" ms, beacons sent ");
            uart_put_int((int32_t)link.stats.tx_packets);
            uart_puts(", tx errors ");
            uart_put_int((int32_t)link.stats.tx_errors);
            uart_puts("\n");
            dmin = 100000;
            dmax = 0;
        }
    }
}

int main(void)
{
    board_init();
    HAL_Delay(200); /* let the host's serial monitor attach */

    uart_puts("\n=== FHSS self-test: NUCLEO-WL55JC + SX1262 shield ===\n");
    sx1262_t radio = {
        .spi = &board_spi1,
        .nss_port = RADIO_NSS_PORT, .nss_pin = RADIO_NSS_PIN,
        .rst_port = RADIO_RST_PORT, .rst_pin = RADIO_RST_PIN,
        .busy_port = RADIO_BUSY_PORT, .busy_pin = RADIO_BUSY_PIN,
        .ant_pwr_port = RADIO_ANT_PORT, .ant_pwr_pin = RADIO_ANT_PIN,
        .clock = SX1262_CLOCK_AUTO,
    };

    /* Retry init every 3 s so wiring can be fixed without reflashing. */
    while (!run_checks(&radio)) {
        led_set(LED_RED_PORT, LED_RED_PIN, 1);
        failures = 0;
        HAL_Delay(3000);
        uart_puts("retrying...\n");
    }

    uart_puts(failures ? "\nRESULT: " : "\nRESULT: all checks passed\n");
    if (failures) {
        uart_put_int(failures);
        uart_puts(" check(s) failed\n");
    }
    led_set(LED_RED_PORT, LED_RED_PIN, failures != 0);
    led_set(LED_GREEN_PORT, LED_GREEN_PIN, failures == 0);

    run_leader();
    for (;;) {
    }
}
