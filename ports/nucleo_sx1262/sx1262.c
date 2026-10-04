#include "sx1262.h"

/* Opcodes (SX1261/2 datasheet, section 11) */
#define OP_SET_STANDBY           0x80
#define OP_SET_FS                0xC1
#define OP_GET_RSSI_INST         0x15
#define OP_SET_RX                0x82
#define OP_SET_TX                0x83
#define OP_SET_RF_FREQUENCY      0x86
#define OP_CALIBRATE             0x89
#define OP_SET_PACKET_TYPE       0x8A
#define OP_SET_MODULATION_PARAMS 0x8B
#define OP_SET_PACKET_PARAMS     0x8C
#define OP_SET_TX_PARAMS         0x8E
#define OP_SET_BUFFER_BASE_ADDR  0x8F
#define OP_SET_PA_CONFIG         0x95
#define OP_SET_REGULATOR_MODE    0x96
#define OP_SET_DIO3_AS_TCXO      0x97
#define OP_CALIBRATE_IMAGE       0x98
#define OP_SET_DIO2_AS_RF_SWITCH 0x9D
#define OP_SET_DIO_IRQ_PARAMS    0x08
#define OP_CLEAR_IRQ_STATUS      0x02
#define OP_CLEAR_DEVICE_ERRORS   0x07
#define OP_WRITE_REGISTER        0x0D
#define OP_WRITE_BUFFER          0x0E
#define OP_GET_IRQ_STATUS        0x12
#define OP_GET_RX_BUFFER_STATUS  0x13
#define OP_GET_PACKET_STATUS     0x14
#define OP_GET_DEVICE_ERRORS     0x17
#define OP_READ_REGISTER         0x1D
#define OP_READ_BUFFER           0x1E
#define OP_GET_STATUS            0xC0

/* Registers */
#define REG_LORA_SYNC_WORD_MSB   0x0740
#define REG_TX_MODULATION        0x0889 /* errata 15.1 */
#define REG_TX_CLAMP_CONFIG      0x08D8 /* errata 15.2 */

#define STDBY_RC        0x00
#define PACKET_TYPE_LORA 0x01
#define REGULATOR_DCDC  0x01
#define RX_CONTINUOUS   0xFFFFFFu

#define IRQ_TX_DONE     0x0001
#define IRQ_RX_DONE     0x0002
#define IRQ_HEADER_ERR  0x0020
#define IRQ_CRC_ERR     0x0040
#define IRQ_TIMEOUT     0x0200
#define IRQ_ALL         0x03FF

#define ERR_CALIB_MASK  0x003F /* RC64K..IMG calibration failures */
#define ERR_XOSC_START  0x0020
#define ERR_PLL_LOCK    0x0040

#define CHIP_MODE_STBY_RC 0x2

#define XTAL_HZ           32000000u
#define SPI_TIMEOUT_MS    10u
#define BUSY_TIMEOUT_MS   20u
#define TX_TIMEOUT_MARGIN_MS 50u

/* ---- SPI transport ----------------------------------------------------- */

static bool wait_busy(sx1262_t *r)
{
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(r->busy_port, r->busy_pin) == GPIO_PIN_SET) {
        if (HAL_GetTick() - start > BUSY_TIMEOUT_MS) {
            r->busy_timeout = true;
            return false;
        }
    }
    return true;
}

static void cs_low(sx1262_t *r) { HAL_GPIO_WritePin(r->nss_port, r->nss_pin, GPIO_PIN_RESET); }
static void cs_high(sx1262_t *r) { HAL_GPIO_WritePin(r->nss_port, r->nss_pin, GPIO_PIN_SET); }

/* opcode + params [+ extra payload] */
static void command(sx1262_t *r, uint8_t op, const uint8_t *params, uint8_t n,
                    const uint8_t *extra, uint8_t extra_n)
{
    wait_busy(r);
    cs_low(r);
    HAL_SPI_Transmit(r->spi, &op, 1, SPI_TIMEOUT_MS);
    if (n) {
        HAL_SPI_Transmit(r->spi, (uint8_t *)params, n, SPI_TIMEOUT_MS);
    }
    if (extra_n) {
        HAL_SPI_Transmit(r->spi, (uint8_t *)extra, extra_n, SPI_TIMEOUT_MS);
    }
    cs_high(r);
}

/* opcode + params, then read n bytes (the first returned byte is status) */
static void query(sx1262_t *r, uint8_t op, const uint8_t *params, uint8_t np, uint8_t *out,
                  uint8_t n)
{
    wait_busy(r);
    cs_low(r);
    HAL_SPI_Transmit(r->spi, &op, 1, SPI_TIMEOUT_MS);
    if (np) {
        HAL_SPI_Transmit(r->spi, (uint8_t *)params, np, SPI_TIMEOUT_MS);
    }
    HAL_SPI_Receive(r->spi, out, n, SPI_TIMEOUT_MS);
    cs_high(r);
}

static void cmd0(sx1262_t *r, uint8_t op, const uint8_t *p, uint8_t n) { command(r, op, p, n, 0, 0); }

static void cmd1(sx1262_t *r, uint8_t op, uint8_t a) { cmd0(r, op, &a, 1); }

static void write_register(sx1262_t *r, uint16_t addr, const uint8_t *data, uint8_t n)
{
    uint8_t a[2] = {(uint8_t)(addr >> 8), (uint8_t)addr};
    command(r, OP_WRITE_REGISTER, a, 2, data, n);
}

static void read_register(sx1262_t *r, uint16_t addr, uint8_t *data, uint8_t n)
{
    uint8_t p[2] = {(uint8_t)(addr >> 8), (uint8_t)addr};
    uint8_t buf[1 + 8];
    query(r, OP_READ_REGISTER, p, 2, buf, (uint8_t)(1 + n));
    for (uint8_t i = 0; i < n; i++) {
        data[i] = buf[1 + i];
    }
}

static uint16_t get_irq(sx1262_t *r)
{
    uint8_t b[3];
    query(r, OP_GET_IRQ_STATUS, 0, 0, b, 3);
    return (uint16_t)((b[1] << 8) | b[2]);
}

static void clear_irq(sx1262_t *r, uint16_t mask)
{
    uint8_t p[2] = {(uint8_t)(mask >> 8), (uint8_t)mask};
    cmd0(r, OP_CLEAR_IRQ_STATUS, p, 2);
}

static uint16_t get_device_errors(sx1262_t *r)
{
    uint8_t b[3];
    query(r, OP_GET_DEVICE_ERRORS, 0, 0, b, 3);
    return (uint16_t)((b[1] << 8) | b[2]);
}

static void standby(sx1262_t *r) { cmd1(r, OP_SET_STANDBY, STDBY_RC); }

/* ---- helpers ----------------------------------------------------------- */

static int bw_code(uint32_t bw_hz)
{
    switch (bw_hz) {
    case 125000: return 0x04;
    case 250000: return 0x05;
    case 500000: return 0x06;
    default: return -1;
    }
}

static uint32_t symbol_time_us(const sx1262_lora_cfg_t *c)
{
    return (uint32_t)(((uint64_t)1000000u << c->spreading_factor) / c->bandwidth_hz);
}

static bool low_data_rate_optimize(const sx1262_lora_cfg_t *c) { return symbol_time_us(c) > 16000u; }

uint32_t sx1262_rf_freq_from_hz(uint32_t freq_hz)
{
    return (uint32_t)(((uint64_t)freq_hz << 25) / XTAL_HZ);
}

uint32_t sx1262_airtime_ms(const sx1262_lora_cfg_t *c, uint8_t payload_len)
{
    int32_t sf = c->spreading_factor;
    int32_t de = low_data_rate_optimize(c);
    int32_t num = 8 * (int32_t)payload_len - 4 * sf + 28 + 16 /* CRC on */ - 0 /* explicit hdr */;
    int32_t den = 4 * (sf - 2 * de);
    int32_t blocks = num > 0 ? (num + den - 1) / den : 0;
    uint32_t payload_syms = 8u + (uint32_t)(blocks * (c->coding_rate + 4));

    /* preamble adds 4.25 symbols: work in quarter-symbols to stay integer */
    uint32_t quarter_syms = 4u * (c->preamble_len + payload_syms) + 17u;
    uint64_t us = (uint64_t)quarter_syms * symbol_time_us(c) / 4u;
    return (uint32_t)((us + 999u) / 1000u);
}

static void set_packet_params(sx1262_t *r, uint8_t payload_len)
{
    uint8_t p[6] = {
        (uint8_t)(r->cfg.preamble_len >> 8), (uint8_t)r->cfg.preamble_len,
        0x00,        /* explicit header */
        payload_len, /* TX length; max length in RX */
        0x01,        /* CRC on */
        0x00,        /* standard IQ */
    };
    cmd0(r, OP_SET_PACKET_PARAMS, p, 6);
}

static void write_rf_frequency(sx1262_t *r, uint32_t freq_hz)
{
    uint32_t f = sx1262_rf_freq_from_hz(freq_hz);
    uint8_t p[4] = {(uint8_t)(f >> 24), (uint8_t)(f >> 16), (uint8_t)(f >> 8), (uint8_t)f};
    cmd0(r, OP_SET_RF_FREQUENCY, p, 4);
}

static void enter_rx(sx1262_t *r)
{
    uint8_t t[3] = {(uint8_t)(RX_CONTINUOUS >> 16), (uint8_t)(RX_CONTINUOUS >> 8),
                    (uint8_t)RX_CONTINUOUS};
    cmd0(r, OP_SET_RX, t, 3);
}

/* ---- public API -------------------------------------------------------- */

/* Reset, presence check, clock setup and calibration. */
static sx1262_err_t bring_up(sx1262_t *r, const sx1262_lora_cfg_t *cfg, bool tcxo)
{
    cs_high(r);
    if (r->ant_pwr_port) {
        HAL_GPIO_WritePin(r->ant_pwr_port, r->ant_pwr_pin, GPIO_PIN_SET);
    }
    HAL_GPIO_WritePin(r->rst_port, r->rst_pin, GPIO_PIN_RESET);
    HAL_Delay(1); /* >= 100 us */
    HAL_GPIO_WritePin(r->rst_port, r->rst_pin, GPIO_PIN_SET);
    HAL_Delay(5);
    if (!wait_busy(r)) {
        return SX1262_ERR_BUSY;
    }

    standby(r);

    /* Presence check: status must report STBY_RC and a register must read
     * back what we wrote (the LoRa sync word, which we set anyway). */
    uint8_t status;
    query(r, OP_GET_STATUS, 0, 0, &status, 1);
    uint8_t sw[2] = {(uint8_t)((cfg->sync_word & 0xF0) | 0x04),
                     (uint8_t)(((cfg->sync_word & 0x0F) << 4) | 0x04)};
    write_register(r, REG_LORA_SYNC_WORD_MSB, sw, 2);
    uint8_t sw_back[2] = {0, 0};
    read_register(r, REG_LORA_SYNC_WORD_MSB, sw_back, 2);
    if (((status >> 4) & 0x7) != CHIP_MODE_STBY_RC || sw_back[0] != sw[0] || sw_back[1] != sw[1]) {
        return SX1262_ERR_NOT_FOUND;
    }

    if (tcxo) {
        /* 1.7 V on DIO3 (Semtech TCXO boards), 5 ms startup in 15.625 us units */
        uint8_t p[4] = {0x01, 0x00, 0x01, 0x40};
        cmd0(r, OP_SET_DIO3_AS_TCXO, p, 4);
    }
    uint8_t clr[2] = {0, 0};
    cmd0(r, OP_CLEAR_DEVICE_ERRORS, clr, 2);
    cmd1(r, OP_CALIBRATE, 0x7F); /* all blocks */
    if (!wait_busy(r)) {
        return SX1262_ERR_BUSY;
    }
    uint16_t errs = get_device_errors(r);
    if (errs & ERR_XOSC_START) {
        return SX1262_ERR_OSC;
    }
    if (errs & ERR_CALIB_MASK) {
        return SX1262_ERR_CALIB;
    }
    r->has_tcxo = tcxo;
    return SX1262_OK;
}

sx1262_err_t sx1262_init(sx1262_t *r, const sx1262_lora_cfg_t *cfg, uint32_t freq_hz)
{
    int bw = bw_code(cfg->bandwidth_hz);
    if (bw < 0 || cfg->spreading_factor < 7 || cfg->spreading_factor > 12 ||
        cfg->coding_rate < 1 || cfg->coding_rate > 4 || cfg->tx_power_dbm < -9 ||
        cfg->tx_power_dbm > 22 || cfg->preamble_len < 6 || freq_hz < 902000000u ||
        freq_hz > 928000000u) {
        return SX1262_ERR_BAD_CONFIG;
    }
    r->cfg = *cfg;
    r->rx_active = false;
    r->busy_timeout = false;

    sx1262_err_t err;
    if (r->clock == SX1262_CLOCK_AUTO) {
        err = bring_up(r, cfg, false);
        if (err == SX1262_ERR_OSC) {
            err = bring_up(r, cfg, true); /* crystal didn't start: try TCXO */
        }
    } else {
        err = bring_up(r, cfg, r->clock == SX1262_CLOCK_TCXO);
    }
    if (err != SX1262_OK) {
        return err;
    }

    cmd1(r, OP_SET_REGULATOR_MODE, REGULATOR_DCDC);
    cmd1(r, OP_SET_DIO2_AS_RF_SWITCH, 0x01); /* shield antenna switch on DIO2 */
    cmd1(r, OP_SET_PACKET_TYPE, PACKET_TYPE_LORA);
    uint8_t img[2] = {0xE1, 0xE9}; /* image calibration for 902-928 MHz */
    cmd0(r, OP_CALIBRATE_IMAGE, img, 2);
    write_rf_frequency(r, freq_hz);

    /* SX1262 high-power PA; output power then set by SetTxParams */
    uint8_t pa[4] = {0x04, 0x07, 0x00, 0x01};
    cmd0(r, OP_SET_PA_CONFIG, pa, 4);
    uint8_t txp[2] = {(uint8_t)cfg->tx_power_dbm, 0x04 /* 200 us ramp */};
    cmd0(r, OP_SET_TX_PARAMS, txp, 2);

    /* Errata 15.2: better tolerance of antenna mismatch */
    uint8_t clamp;
    read_register(r, REG_TX_CLAMP_CONFIG, &clamp, 1);
    clamp |= 0x1E;
    write_register(r, REG_TX_CLAMP_CONFIG, &clamp, 1);

    uint8_t mod[4] = {cfg->spreading_factor, (uint8_t)bw, cfg->coding_rate,
                      (uint8_t)(low_data_rate_optimize(cfg) ? 1 : 0)};
    cmd0(r, OP_SET_MODULATION_PARAMS, mod, 4);

    /* Errata 15.1: modulation quality bit depends on 500 kHz or not */
    uint8_t txmod;
    read_register(r, REG_TX_MODULATION, &txmod, 1);
    txmod = cfg->bandwidth_hz == 500000 ? (uint8_t)(txmod & ~0x04) : (uint8_t)(txmod | 0x04);
    write_register(r, REG_TX_MODULATION, &txmod, 1);

    set_packet_params(r, 0xFF);
    uint8_t base[2] = {0x00, 0x00};
    cmd0(r, OP_SET_BUFFER_BASE_ADDR, base, 2);

    /* Route the IRQs we poll to DIO1 as well, for anyone who wires it. */
    uint16_t mask = IRQ_TX_DONE | IRQ_RX_DONE | IRQ_HEADER_ERR | IRQ_CRC_ERR | IRQ_TIMEOUT;
    uint8_t irq[8] = {(uint8_t)(mask >> 8), (uint8_t)mask, (uint8_t)(mask >> 8), (uint8_t)mask,
                      0, 0, 0, 0};
    cmd0(r, OP_SET_DIO_IRQ_PARAMS, irq, 8);
    clear_irq(r, IRQ_ALL);

    return r->busy_timeout ? SX1262_ERR_BUSY : SX1262_OK;
}

void sx1262_set_frequency(sx1262_t *r, uint32_t freq_hz)
{
    if (r->rx_active) {
        standby(r); /* frequency changes are made from standby */
    }
    write_rf_frequency(r, freq_hz);
    if (r->rx_active) {
        enter_rx(r);
    }
}

void sx1262_start_rx(sx1262_t *r)
{
    standby(r);
    set_packet_params(r, 0xFF);
    clear_irq(r, IRQ_ALL);
    enter_rx(r);
    r->rx_active = true;
}

sx1262_err_t sx1262_send(sx1262_t *r, const uint8_t *data, uint8_t len)
{
    if (len == 0) {
        return SX1262_ERR_BAD_CONFIG;
    }
    standby(r);
    r->rx_active = false;

    set_packet_params(r, len);
    uint8_t offset = 0;
    command(r, OP_WRITE_BUFFER, &offset, 1, data, len);
    clear_irq(r, IRQ_ALL);
    uint8_t t[3] = {0, 0, 0}; /* no timeout: we poll */
    cmd0(r, OP_SET_TX, t, 3);

    uint32_t start = HAL_GetTick();
    uint32_t timeout = sx1262_airtime_ms(&r->cfg, len) + TX_TIMEOUT_MARGIN_MS;
    sx1262_err_t err = SX1262_OK;
    while (!(get_irq(r) & IRQ_TX_DONE)) {
        if (HAL_GetTick() - start > timeout) {
            err = SX1262_ERR_TIMEOUT;
            break;
        }
    }
    clear_irq(r, IRQ_ALL);
    sx1262_start_rx(r);
    return err;
}

int sx1262_read_packet(sx1262_t *r, uint8_t *buf, uint8_t max, int16_t *rssi_dbm)
{
    uint16_t irq = get_irq(r);
    if (irq & IRQ_HEADER_ERR) {
        clear_irq(r, IRQ_ALL);
        return SX1262_ERR_CRC;
    }
    if (!(irq & IRQ_RX_DONE)) {
        return 0;
    }
    clear_irq(r, IRQ_ALL);
    if (irq & IRQ_CRC_ERR) {
        return SX1262_ERR_CRC;
    }

    uint8_t st[3];
    query(r, OP_GET_RX_BUFFER_STATUS, 0, 0, st, 3);
    uint8_t len = st[1], start = st[2];
    if (len > max || len > 254) {
        return SX1262_ERR_TOO_LONG;
    }

    uint8_t tmp[1 + 255];
    query(r, OP_READ_BUFFER, &start, 1, tmp, (uint8_t)(1 + len));
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = tmp[1 + i];
    }

    if (rssi_dbm) {
        uint8_t ps[4];
        query(r, OP_GET_PACKET_STATUS, 0, 0, ps, 4);
        *rssi_dbm = (int16_t)(-(int16_t)ps[1] / 2);
    }
    return len;
}

sx1262_err_t sx1262_check_pll(sx1262_t *r, uint32_t freq_hz)
{
    bool was_rx = r->rx_active;
    standby(r);
    uint8_t clr[2] = {0, 0};
    cmd0(r, OP_CLEAR_DEVICE_ERRORS, clr, 2);
    write_rf_frequency(r, freq_hz);
    cmd0(r, OP_SET_FS, 0, 0);
    HAL_Delay(1); /* PLL settles in well under 100 us */
    uint16_t errs = get_device_errors(r);
    standby(r);
    if (was_rx) {
        enter_rx(r);
    }
    return (errs & ERR_PLL_LOCK) ? SX1262_ERR_PLL : SX1262_OK;
}

int16_t sx1262_rssi_inst(sx1262_t *r)
{
    uint8_t b[2];
    query(r, OP_GET_RSSI_INST, 0, 0, b, 2);
    return (int16_t)(-(int16_t)b[1] / 2);
}

uint16_t sx1262_device_errors(sx1262_t *r)
{
    return get_device_errors(r);
}
