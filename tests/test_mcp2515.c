/* Host tests for the MCP2515 driver against emulated controllers. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "can_ring.h"
#include "mcp2515.h"
#include "mcp2515_emu.h"

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

static mcp2515_t ctl(mcp2515_emu_t *e, mcp2515_mode_t mode)
{
    mcp2515_t c = {
        .spi = &e->spi, .cs_port = &e->port, .cs_pin = 1,
        .osc_hz = 8000000u, .bitrate = 500000u, .mode = mode,
    };
    return c;
}

static void test_bit_timing(void)
{
    printf("bit timing: known values and every reachable combination\n");
    uint8_t c[3];
    CHECK(mcp2515_bit_timing(8000000u, 500000u, c) && c[0] == 0x00 && c[1] == 0x91 && c[2] == 0x01,
          "8 MHz / 500k: %02X %02X %02X", c[0], c[1], c[2]);
    CHECK(mcp2515_bit_timing(16000000u, 500000u, c) && c[0] == 0x00 && c[1] == 0xB5 && c[2] == 0x01,
          "16 MHz / 500k: %02X %02X %02X", c[0], c[1], c[2]);
    CHECK(mcp2515_bit_timing(8000000u, 125000u, c) && c[0] == 0x01 && c[1] == 0xB5 && c[2] == 0x01,
          "8 MHz / 125k: %02X %02X %02X", c[0], c[1], c[2]);
    CHECK(!mcp2515_bit_timing(8000000u, 1000000u, c), "8 MHz cannot do 1 Mbit/s");
    CHECK(mcp2515_bit_timing(16000000u, 1000000u, c), "16 MHz / 1 Mbit/s");

    static const uint32_t osc[] = {8000000u, 16000000u, 20000000u};
    static const uint32_t rates[] = {10000u, 20000u, 50000u, 100000u, 125000u, 250000u, 500000u,
                                     800000u, 1000000u};
    for (unsigned i = 0; i < 3; i++) {
        for (unsigned j = 0; j < sizeof(rates) / sizeof(rates[0]); j++) {
            if (!mcp2515_bit_timing(osc[i], rates[j], c)) {
                continue;
            }
            uint32_t brp = (c[0] & 0x3Fu) + 1, ps1 = ((c[1] >> 3) & 7u) + 1, prop = (c[1] & 7u) + 1;
            uint32_t ps2 = (c[2] & 7u) + 1, n = 1 + prop + ps1 + ps2;
            CHECK(osc[i] / (2 * brp * n) == rates[j] && osc[i] % (2 * brp * n) == 0,
                  "%u Hz / %u: bit rate off", osc[i], rates[j]);
            CHECK(prop + ps1 >= ps2 && ps2 >= 2 && n >= 5 && n <= 25,
                  "%u Hz / %u: segment rules broken", osc[i], rates[j]);
        }
    }
}

static void test_init(void)
{
    printf("init: presence check, registers, error cases\n");
    static mcp2515_emu_t e;
    can_emu_reset_world();
    can_emu_add(&e);
    mcp2515_t c = ctl(&e, MCP2515_MODE_NORMAL);
    CHECK(mcp2515_init(&c) == MCP2515_OK, "init");
    CHECK((e.regs[0x0E] & 0xE0) == 0x00, "not in normal mode: CANSTAT %02X", e.regs[0x0E]);
    CHECK(e.regs[0x2A] == 0x00 && e.regs[0x29] == 0x91 && e.regs[0x28] == 0x01, "CNF registers");
    CHECK(e.regs[0x60] == 0x64 && e.regs[0x70] == 0x60, "RX buffers not accepting all");
    CHECK(e.violations == 0, "%d SPI violations", e.violations);

    e.present = false;
    CHECK(mcp2515_init(&c) == MCP2515_ERR_NOT_FOUND, "missing chip not detected");
    e.present = true;
    e.mode_stuck = true;
    CHECK(mcp2515_init(&c) == MCP2515_ERR_MODE, "stuck mode not detected");
    e.mode_stuck = false;
    c.bitrate = 1000000u;
    CHECK(mcp2515_init(&c) == MCP2515_ERR_BAD_BITRATE, "unreachable bit rate accepted");
}

static void test_loopback(void)
{
    printf("loopback: frame sent comes back intact (single-board self-test path)\n");
    static mcp2515_emu_t e;
    can_emu_reset_world();
    can_emu_add(&e);
    mcp2515_t c = ctl(&e, MCP2515_MODE_LOOPBACK);
    CHECK(mcp2515_init(&c) == MCP2515_OK, "init loopback");
    can_frame_t rx, tx = {.id = 0x5A3, .dlc = 8, .data = {1, 2, 3, 4, 5, 6, 7, 8}};
    CHECK(!mcp2515_receive(&c, &rx), "frame before sending");
    CHECK(mcp2515_send(&c, &tx) == MCP2515_OK, "send");
    CHECK(mcp2515_receive(&c, &rx) && rx.id == tx.id && rx.dlc == 8 &&
          memcmp(rx.data, tx.data, 8) == 0, "looped frame differs");
    CHECK(!mcp2515_receive(&c, &rx), "frame read twice");
    CHECK(e.violations == 0, "%d SPI violations", e.violations);
}

static void test_bus(void)
{
    printf("bus: two nodes exchange frames; lone node fails cleanly; filters; rollover\n");
    static mcp2515_emu_t ea, eb;
    can_emu_reset_world();
    can_emu_add(&ea);
    mcp2515_t a = ctl(&ea, MCP2515_MODE_NORMAL);
    mcp2515_init(&a);

    can_frame_t rx, tx = {.id = 0x123, .dlc = 3, .data = {0xAA, 0xBB, 0xCC}};
    CHECK(mcp2515_send(&a, &tx) == MCP2515_ERR_TX_FAILED, "lone node send should fail (no ACK)");
    CHECK(!(ea.regs[0x30] & 0x08), "TXREQ left set after failure");

    can_emu_add(&eb);
    mcp2515_t b = ctl(&eb, MCP2515_MODE_NORMAL);
    mcp2515_init(&b);
    CHECK(mcp2515_send(&a, &tx) == MCP2515_OK, "send with a peer");
    CHECK(mcp2515_receive(&b, &rx) && rx.id == 0x123 && rx.dlc == 3 && rx.data[2] == 0xCC,
          "peer did not receive");

    /* two frames queued before reading: RXB0 rolls over into RXB1 */
    can_frame_t t1 = {.id = 0x10, .dlc = 1, .data = {1}}, t2 = {.id = 0x11, .dlc = 1, .data = {2}};
    mcp2515_send(&a, &t1);
    mcp2515_send(&a, &t2);
    can_frame_t r1, r2;
    CHECK(mcp2515_receive(&b, &r1) && mcp2515_receive(&b, &r2) && r1.id == 0x10 && r2.id == 0x11,
          "rollover lost or reordered frames");

    /* filter: B only accepts ID 0x100 */
    b.filter_mask = 0x7FF;
    b.filter_id = 0x100;
    mcp2515_init(&b);
    can_frame_t other = {.id = 0x101, .dlc = 0}, mine = {.id = 0x100, .dlc = 0};
    mcp2515_send(&a, &other);
    CHECK(!mcp2515_receive(&b, &rx), "filtered ID got through");
    mcp2515_send(&a, &mine);
    CHECK(mcp2515_receive(&b, &rx) && rx.id == 0x100, "accepted ID missing");
    CHECK(ea.violations == 0 && eb.violations == 0, "SPI violations %d %d", ea.violations, eb.violations);
}

/* What the board's INT handler does: empty the chip into the ring while INT is low. */
static void isr(mcp2515_t *c, mcp2515_emu_t *e, can_ring_t *r)
{
    can_frame_t f;
    while (mcp2515_emu_int_low(e) && mcp2515_receive(c, &f)) {
        can_ring_frame_t x = {.id = f.id, .dlc = f.dlc, .ms = emu_now_ms};
        memcpy(x.data, f.data, f.dlc);
        can_ring_push(r, &x);
    }
}

/* Send n frames from a to b; b's main loop only gets to run every `stall`
 * frames. irq: b's INT handler runs as each frame lands (interrupt mode);
 * otherwise b is polled only when its main loop runs. Returns frames the
 * main loop received in order (ids 0..n-1), or -1 on a gap or reorder. */
static int burst(mcp2515_t *a, mcp2515_t *b, mcp2515_emu_t *eb, can_ring_t *r, bool irq,
                 int n, int stall)
{
    int got = 0;
    can_frame_t f;
    can_ring_frame_t x;
    for (int i = 0; i < n; i++) {
        can_frame_t t = {.id = (uint16_t)i, .dlc = 2, .data = {(uint8_t)i, 0x5A}};
        mcp2515_send(a, &t);
        if (irq) {
            isr(b, eb, r);
        }
        if ((i + 1) % stall == 0 || i == n - 1) { /* main loop runs */
            if (irq) {
                while (can_ring_pop(r, &x)) {
                    if (x.id != got || x.data[0] != (uint8_t)got) return -1;
                    got++;
                }
            } else {
                while (mcp2515_receive(b, &f)) {
                    got++;
                }
            }
        }
    }
    return got;
}

static void test_irq(void)
{
    printf("rx interrupt: INT pin, ring buffer rides out main-loop stalls\n");
    static mcp2515_emu_t ea, eb;
    static can_ring_t ring;
    can_emu_reset_world();
    can_emu_add(&ea);
    can_emu_add(&eb);
    mcp2515_t a = ctl(&ea, MCP2515_MODE_NORMAL), b = ctl(&eb, MCP2515_MODE_NORMAL);
    mcp2515_init(&a);
    mcp2515_init(&b);
    CHECK(eb.regs[0x2B] == 0x00, "polled build enabled interrupts: CANINTE %02X", eb.regs[0x2B]);

    b.rx_irq = true;
    mcp2515_init(&b);
    CHECK(eb.regs[0x2B] == 0x03, "rx_irq: CANINTE %02X, want RX0IE|RX1IE", eb.regs[0x2B]);
    CHECK(!mcp2515_emu_int_low(&eb), "INT low with nothing received");
    can_frame_t t = {.id = 0x101, .dlc = 1, .data = {7}}, f;
    mcp2515_send(&a, &t);
    CHECK(mcp2515_emu_int_low(&eb), "INT not asserted for a received frame");
    mcp2515_send(&a, &t);
    CHECK(mcp2515_receive(&b, &f) && mcp2515_emu_int_low(&eb), "INT released with a frame still waiting");
    CHECK(mcp2515_receive(&b, &f) && !mcp2515_emu_int_low(&eb), "INT stuck low after both buffers read");
    mcp2515_send(&b, &t); /* our own transmit must not raise INT (TX interrupts stay off) */
    CHECK(!mcp2515_emu_int_low(&eb), "INT asserted by a transmit");

    /* 200 frames while the main loop only gets a turn every 40 frames
     * (e.g. ~250 ms at 160 frames/s): polled loses nearly all of them, the
     * interrupt + ring loses none */
    b.rx_irq = false;
    mcp2515_init(&b);
    int polled = burst(&a, &b, &eb, &ring, false, 200, 40);
    CHECK(polled == 10 && (eb.regs[0x2D] & 0xC0), "polled: %d of 200 kept (2 per turn expected), EFLG %02X",
          polled, eb.regs[0x2D]);

    b.rx_irq = true;
    mcp2515_init(&b);
    can_ring_init(&ring);
    int got = burst(&a, &b, &eb, &ring, true, 200, 40);
    CHECK(got == 200, "interrupt: %d of 200 received in order", got);
    CHECK(ring.dropped == 0 && ring.max_depth == 40, "dropped %u, max depth %u", (unsigned)ring.dropped,
          (unsigned)ring.max_depth);
    CHECK(!(eb.regs[0x2D] & 0xC0), "chip overflowed in interrupt mode: EFLG %02X", eb.regs[0x2D]);

    /* a stall longer than the ring: the excess is dropped and counted, the
     * rest still arrive in order */
    can_ring_init(&ring);
    can_ring_frame_t x;
    int kept = 0;
    for (int i = 0; i < 100; i++) {
        can_frame_t u = {.id = (uint16_t)i, .dlc = 1, .data = {(uint8_t)i}};
        mcp2515_send(&a, &u);
        isr(&b, &eb, &ring);
    }
    bool in_order = true;
    while (can_ring_pop(&ring, &x)) {
        in_order = in_order && x.id == kept;
        kept++;
    }
    CHECK(kept == (int)CAN_RING_SIZE && ring.dropped == 100u - CAN_RING_SIZE && in_order,
          "overfull ring: kept %d, dropped %u", kept, (unsigned)ring.dropped);
    CHECK(ea.violations == 0 && eb.violations == 0, "SPI violations %d %d", ea.violations, eb.violations);
}

int main(void)
{
    test_bit_timing();
    test_init();
    test_loopback();
    test_bus();
    test_irq();
    printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
