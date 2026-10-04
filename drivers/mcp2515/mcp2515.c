#include "mcp2515.h"

/* SPI instructions (MCP2515 datasheet, table 12-1) */
#define INS_RESET      0xC0
#define INS_READ       0x03
#define INS_WRITE      0x02
#define INS_BIT_MODIFY 0x05
#define INS_LOAD_TX0   0x40 /* starts at TXB0SIDH */
#define INS_RTS_TX0    0x81
#define INS_READ_RX0   0x90 /* starts at RXB0SIDH; clears RX0IF on CS high */
#define INS_READ_RX1   0x94
#define INS_RX_STATUS  0xB0

/* Registers */
#define REG_RXF0SIDH   0x00
#define REG_RXF3SIDH   0x10
#define REG_TEC        0x1C
#define REG_REC        0x1D
#define REG_CANSTAT    0x0E
#define REG_CANCTRL    0x0F
#define REG_RXM0SIDH   0x20
#define REG_RXM1SIDH   0x24
#define REG_CNF3       0x28
#define REG_CNF2       0x29
#define REG_CNF1       0x2A
#define REG_CANINTE    0x2B
#define REG_CANINTF    0x2C
#define REG_EFLG       0x2D
#define REG_TXB0CTRL   0x30
#define REG_RXB0CTRL   0x60
#define REG_RXB1CTRL   0x70

#define MODE_MASK      0xE0
#define MODE_NORMAL    0x00
#define MODE_LOOPBACK  0x40
#define MODE_LISTEN    0x60
#define MODE_CONFIG    0x80

#define TXREQ          0x08
#define TXERR          0x10
#define MLOA           0x20
#define RXM_ANY        0x60 /* RXBnCTRL: filters off, accept everything */
#define RXM_FILTERED   0x00
#define BUKT           0x04 /* RXB0 rolls over into RXB1 when full */
#define SIDL_IDE       0x08

#define SPI_TIMEOUT_MS  10u
#define MODE_TIMEOUT_MS 10u
#define TX_TIMEOUT_MS   10u

/* ---- SPI --------------------------------------------------------------- */

static void cs_low(mcp2515_t *c) { HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_RESET); }
static void cs_high(mcp2515_t *c) { HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_SET); }

static void xfer_out(mcp2515_t *c, const uint8_t *b, uint8_t n)
{
    HAL_SPI_Transmit(c->spi, (uint8_t *)b, n, SPI_TIMEOUT_MS);
}

static void write_regs(mcp2515_t *c, uint8_t addr, const uint8_t *data, uint8_t n)
{
    uint8_t hdr[2] = {INS_WRITE, addr};
    cs_low(c);
    xfer_out(c, hdr, 2);
    xfer_out(c, data, n);
    cs_high(c);
}

static void write_reg(mcp2515_t *c, uint8_t addr, uint8_t v) { write_regs(c, addr, &v, 1); }

static uint8_t read_reg(mcp2515_t *c, uint8_t addr)
{
    uint8_t hdr[2] = {INS_READ, addr}, v = 0;
    cs_low(c);
    xfer_out(c, hdr, 2);
    HAL_SPI_Receive(c->spi, &v, 1, SPI_TIMEOUT_MS);
    cs_high(c);
    return v;
}

static void bit_modify(mcp2515_t *c, uint8_t addr, uint8_t mask, uint8_t v)
{
    uint8_t b[4] = {INS_BIT_MODIFY, addr, mask, v};
    cs_low(c);
    xfer_out(c, b, 4);
    cs_high(c);
}

static void instruction(mcp2515_t *c, uint8_t ins)
{
    cs_low(c);
    xfer_out(c, &ins, 1);
    cs_high(c);
}

static bool set_mode(mcp2515_t *c, uint8_t mode)
{
    bit_modify(c, REG_CANCTRL, MODE_MASK, mode);
    uint32_t start = HAL_GetTick();
    while ((read_reg(c, REG_CANSTAT) & MODE_MASK) != mode) {
        if (HAL_GetTick() - start > MODE_TIMEOUT_MS) {
            return false;
        }
    }
    return true;
}

static void put_sid(uint8_t *b, uint16_t id)
{
    b[0] = (uint8_t)(id >> 3);
    b[1] = (uint8_t)((id & 0x7) << 5);
}

/* ---- bit timing -------------------------------------------------------- */

bool mcp2515_bit_timing(uint32_t osc_hz, uint32_t bitrate, uint8_t cnf[3])
{
    if (bitrate == 0 || osc_hz % (2u * bitrate) != 0) {
        return false;
    }
    uint32_t tq_per_bit_x_brp = osc_hz / (2u * bitrate); /* TQ = 2 * BRP / Fosc */

    /* Prefer 8..16 TQ per bit (good resolution), then 5..7. */
    static const uint8_t order[] = {16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5};
    for (unsigned i = 0; i < sizeof(order); i++) {
        uint32_t n = order[i];
        if (tq_per_bit_x_brp % n != 0) {
            continue;
        }
        uint32_t brp = tq_per_bit_x_brp / n;
        if (brp < 1 || brp > 64) {
            continue;
        }
        /* sample point as close to 87.5% as the segment limits allow */
        uint32_t ps2 = n - (n * 7u + 4u) / 8u;
        if (ps2 < 2) {
            ps2 = 2;
        }
        uint32_t rest = n - 1u - ps2; /* propagation + phase 1 */
        uint32_t ps1 = (rest + 1u) / 2u;
        if (ps1 > 8) {
            ps1 = 8;
        }
        uint32_t prop = rest - ps1;
        /* datasheet rules: PropSeg + PS1 >= PS2, PS2 > SJW, each 1..8 TQ */
        if (prop < 1 || prop > 8 || prop + ps1 < ps2 || ps2 > 8) {
            continue;
        }
        cnf[0] = (uint8_t)(brp - 1u);                                   /* SJW = 1 TQ */
        cnf[1] = (uint8_t)(0x80 | ((ps1 - 1u) << 3) | (prop - 1u));     /* BTLMODE=1 */
        cnf[2] = (uint8_t)(ps2 - 1u);
        return true;
    }
    return false;
}

/* ---- public API -------------------------------------------------------- */

mcp2515_err_t mcp2515_init(mcp2515_t *c)
{
    uint8_t cnf[3];
    if (!mcp2515_bit_timing(c->osc_hz, c->bitrate, cnf)) {
        return MCP2515_ERR_BAD_BITRATE;
    }

    cs_high(c);
    instruction(c, INS_RESET);
    HAL_Delay(2); /* oscillator start-up after reset */

    /* Presence check: reset leaves it in configuration mode, and CNF1
     * must read back what we write. */
    if ((read_reg(c, REG_CANSTAT) & MODE_MASK) != MODE_CONFIG) {
        return MCP2515_ERR_NOT_FOUND;
    }
    write_reg(c, REG_CNF1, 0x55);
    if (read_reg(c, REG_CNF1) != 0x55) {
        return MCP2515_ERR_NOT_FOUND;
    }

    uint8_t cnf_regs[3] = {cnf[2], cnf[1], cnf[0]}; /* CNF3, CNF2, CNF1 are consecutive */
    write_regs(c, REG_CNF3, cnf_regs, 3);

    if (c->filter_mask) {
        uint8_t m[4], f[4] = {0, 0, 0, 0};
        put_sid(m, c->filter_mask);
        m[2] = m[3] = 0;
        put_sid(f, c->filter_id);
        write_regs(c, REG_RXM0SIDH, m, 4);
        write_regs(c, REG_RXM1SIDH, m, 4);
        for (uint8_t r = 0; r < 3; r++) { /* RXF0-2 */
            write_regs(c, (uint8_t)(REG_RXF0SIDH + 4 * r), f, 4);
            write_regs(c, (uint8_t)(REG_RXF3SIDH + 4 * r), f, 4); /* RXF3-5 */
        }
        write_reg(c, REG_RXB0CTRL, RXM_FILTERED | BUKT);
        write_reg(c, REG_RXB1CTRL, RXM_FILTERED);
    } else {
        write_reg(c, REG_RXB0CTRL, RXM_ANY | BUKT);
        write_reg(c, REG_RXB1CTRL, RXM_ANY);
    }
    write_reg(c, REG_CANINTE, 0x00); /* polled */
    write_reg(c, REG_CANINTF, 0x00);

    uint8_t mode = c->mode == MCP2515_MODE_LOOPBACK ? MODE_LOOPBACK
                 : c->mode == MCP2515_MODE_LISTEN   ? MODE_LISTEN
                                                    : MODE_NORMAL;
    return set_mode(c, mode) ? MCP2515_OK : MCP2515_ERR_MODE;
}

mcp2515_err_t mcp2515_send(mcp2515_t *c, const can_frame_t *f)
{
    if (f->dlc > 8 || f->id > 0x7FF) {
        return MCP2515_ERR_BAD_FRAME;
    }
    if (read_reg(c, REG_TXB0CTRL) & TXREQ) {
        return MCP2515_ERR_TX_BUSY;
    }

    uint8_t b[1 + 5 + 8];
    b[0] = INS_LOAD_TX0;
    put_sid(&b[1], f->id);
    b[3] = 0; /* EID8 */
    b[4] = 0; /* EID0 */
    b[5] = f->dlc;
    for (uint8_t i = 0; i < f->dlc; i++) {
        b[6 + i] = f->data[i];
    }
    cs_low(c);
    xfer_out(c, b, (uint8_t)(6 + f->dlc));
    cs_high(c);
    instruction(c, INS_RTS_TX0);

    uint32_t start = HAL_GetTick();
    for (;;) {
        uint8_t ctrl = read_reg(c, REG_TXB0CTRL);
        if (!(ctrl & TXREQ)) {
            return MCP2515_OK;
        }
        if ((ctrl & (TXERR | MLOA)) || HAL_GetTick() - start > TX_TIMEOUT_MS) {
            bit_modify(c, REG_TXB0CTRL, TXREQ, 0); /* abort so the buffer frees up */
            return MCP2515_ERR_TX_FAILED;
        }
    }
}

bool mcp2515_receive(mcp2515_t *c, can_frame_t *f)
{
    uint8_t ins = INS_RX_STATUS, st = 0;
    cs_low(c);
    xfer_out(c, &ins, 1);
    HAL_SPI_Receive(c->spi, &st, 1, SPI_TIMEOUT_MS);
    cs_high(c);

    uint8_t which = st & 0xC0; /* 0x40 = RXB0, 0x80 = RXB1, 0xC0 = both */
    if (!which) {
        return false;
    }

    uint8_t b[13];
    ins = (which & 0x40) ? INS_READ_RX0 : INS_READ_RX1;
    cs_low(c);
    xfer_out(c, &ins, 1);
    HAL_SPI_Receive(c->spi, b, 13, SPI_TIMEOUT_MS);
    cs_high(c); /* clears the buffer's RXnIF */

    if (b[1] & SIDL_IDE) {
        return false; /* extended frame: not part of the team's plan */
    }
    f->id = (uint16_t)((b[0] << 3) | (b[1] >> 5));
    f->dlc = b[4] & 0x0F;
    if (f->dlc > 8) {
        f->dlc = 8;
    }
    for (uint8_t i = 0; i < f->dlc; i++) {
        f->data[i] = b[5 + i];
    }
    return true;
}

uint8_t mcp2515_error_flags(mcp2515_t *c, uint8_t *tec, uint8_t *rec)
{
    if (tec) {
        *tec = read_reg(c, REG_TEC);
    }
    if (rec) {
        *rec = read_reg(c, REG_REC);
    }
    return read_reg(c, REG_EFLG);
}
