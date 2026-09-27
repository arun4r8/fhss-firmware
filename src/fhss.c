#include "fhss.h"

#include <stddef.h>
#include <string.h>

#define DEFAULT_MAX_MISSED_HOPS 3u

/* ---- PRNG: seed-mixed xorshift32, identical on every platform ---------- */

static uint32_t mix_seed(uint32_t x)
{
    /* murmur3 finalizer so nearby seeds give unrelated sequences */
    x ^= x >> 16;
    x *= 0x85ebca6bu;
    x ^= x >> 13;
    x *= 0xc2b2ae35u;
    x ^= x >> 16;
    return x ? x : 0x6d2b79f5u; /* xorshift state must be nonzero */
}

static uint32_t xorshift32(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

/* Uniform integer in [0, bound) without modulo bias. */
static uint32_t rand_below(uint32_t *s, uint32_t bound)
{
    uint32_t limit = UINT32_MAX - (UINT32_MAX % bound);
    uint32_t r;
    do {
        r = xorshift32(s);
    } while (r >= limit);
    return r % bound;
}

static void build_sequence(fhss_t *f)
{
    uint32_t s = mix_seed(f->cfg.seed);
    uint16_t n = f->cfg.num_channels;

    for (uint16_t i = 0; i < n; i++) {
        f->sequence[i] = (uint8_t)i;
    }
    /* Fisher-Yates: every permutation equally likely */
    for (uint16_t i = n - 1; i > 0; i--) {
        uint16_t j = (uint16_t)rand_below(&s, (uint32_t)i + 1);
        uint8_t tmp = f->sequence[i];
        f->sequence[i] = f->sequence[j];
        f->sequence[j] = tmp;
    }
}

/* ---- helpers ----------------------------------------------------------- */

static void retune(fhss_t *f)
{
    if (f->hal.set_frequency) {
        f->hal.set_frequency(f->hal.user, fhss_current_frequency(f));
    }
}

static void enter_search(fhss_t *f)
{
    /* Park on the first channel of the sequence. The leader lands on it
     * once per cycle, so the follower hears a sync within one cycle. */
    f->state = FHSS_STATE_SEARCHING;
    f->hop_index = 0;
    f->missed_hops = 0;
    f->synced_this_dwell = false;
    retune(f);
}

/* ---- public API -------------------------------------------------------- */

fhss_err_t fhss_init(fhss_t *f, const fhss_config_t *cfg, const fhss_hal_t *hal)
{
    if (!f || !cfg || !hal) {
        return FHSS_ERR_NULL;
    }

    memset(f, 0, sizeof(*f));
    f->cfg = *cfg;
    f->hal = *hal;

    if (f->cfg.dwell_ms == 0) {
        f->cfg.dwell_ms = FHSS_DWELL_MS;
    }
    if (f->cfg.dwell_ms > FHSS_DWELL_MS) {
        return FHSS_ERR_BAD_DWELL;
    }
    if (f->cfg.guard_ms >= f->cfg.dwell_ms) {
        return FHSS_ERR_BAD_DWELL;
    }
    if (f->cfg.max_missed_hops == 0) {
        f->cfg.max_missed_hops = DEFAULT_MAX_MISSED_HOPS;
    }
    if (f->cfg.num_channels > FHSS_MAX_CHANNELS || f->cfg.num_channels > 256) {
        return FHSS_ERR_TOO_MANY_CHANNELS;
    }
    /* Walking a permutation revisits a channel every num_channels hops. */
    if ((uint32_t)f->cfg.num_channels * f->cfg.dwell_ms < FHSS_REVISIT_WINDOW_MS) {
        return FHSS_ERR_TOO_FEW_CHANNELS;
    }

    uint64_t top = (uint64_t)f->cfg.base_freq_hz +
                   (uint64_t)(f->cfg.num_channels - 1) * f->cfg.channel_spacing_hz;
    if (f->cfg.channel_spacing_hz == 0 || f->cfg.base_freq_hz < FHSS_BAND_MIN_HZ ||
        top > FHSS_BAND_MAX_HZ) {
        return FHSS_ERR_OUT_OF_BAND;
    }

    build_sequence(f);
    return FHSS_OK;
}

void fhss_start(fhss_t *f, uint32_t now_ms)
{
    f->dwell_start_ms = now_ms;
    if (f->cfg.role == FHSS_ROLE_LEADER) {
        f->state = FHSS_STATE_HOPPING;
        f->hop_index = 0;
        retune(f);
    } else {
        enter_search(f);
    }
}

bool fhss_tick(fhss_t *f, uint32_t now_ms)
{
    if (f->state != FHSS_STATE_HOPPING) {
        return false;
    }

    bool hopped = false;
    /* Advance dwell_start by whole dwells so timing never drifts, even if a
     * tick arrives late. Unsigned subtraction handles clock wraparound. */
    while ((uint32_t)(now_ms - f->dwell_start_ms) >= f->cfg.dwell_ms) {
        f->dwell_start_ms += f->cfg.dwell_ms;
        f->hop_index = (uint16_t)((f->hop_index + 1) % f->cfg.num_channels);
        hopped = true;

        if (f->cfg.role == FHSS_ROLE_FOLLOWER) {
            f->missed_hops = f->synced_this_dwell ? 0 : (uint8_t)(f->missed_hops + 1);
            f->synced_this_dwell = false;
            if (f->missed_hops >= f->cfg.max_missed_hops) {
                enter_search(f);
                return true;
            }
        }
    }

    if (hopped) {
        retune(f);
    }
    return hopped;
}

void fhss_get_sync(const fhss_t *f, uint32_t now_ms, fhss_sync_t *out)
{
    out->hop_index = f->hop_index;
    out->ms_into_dwell = (uint16_t)(now_ms - f->dwell_start_ms);
}

fhss_err_t fhss_on_sync(fhss_t *f, uint32_t now_ms, const fhss_sync_t *sync)
{
    if (!f || !sync) {
        return FHSS_ERR_NULL;
    }
    if (sync->hop_index >= f->cfg.num_channels || sync->ms_into_dwell >= f->cfg.dwell_ms) {
        return FHSS_ERR_BAD_SYNC;
    }
    if (f->cfg.role == FHSS_ROLE_LEADER) {
        return FHSS_OK;
    }

    uint16_t old_index = f->hop_index;
    fhss_state_t old_state = f->state;

    f->hop_index = sync->hop_index;
    f->dwell_start_ms = now_ms - sync->ms_into_dwell - f->cfg.rx_latency_ms;
    f->state = FHSS_STATE_HOPPING;
    f->synced_this_dwell = true; /* before tick, so a catch-up hop isn't a miss */

    /* Latency may push us past the leader's hop boundary; catch up. */
    bool hopped = fhss_tick(f, now_ms);
    f->synced_this_dwell = true;
    f->missed_hops = 0;

    if (!hopped && (old_state != FHSS_STATE_HOPPING || old_index != f->hop_index)) {
        retune(f);
    }
    return FHSS_OK;
}

bool fhss_can_transmit(const fhss_t *f, uint32_t now_ms)
{
    /* A searching follower sits on one channel indefinitely, so it must stay
     * silent until locked or it would exceed the per-channel dwell limit. */
    if (f->state != FHSS_STATE_HOPPING) {
        return false;
    }
    return fhss_ms_until_hop(f, now_ms) > f->cfg.guard_ms;
}

uint16_t fhss_current_channel(const fhss_t *f)
{
    return f->sequence[f->hop_index];
}

uint32_t fhss_channel_frequency(const fhss_t *f, uint16_t channel)
{
    return f->cfg.base_freq_hz + (uint32_t)channel * f->cfg.channel_spacing_hz;
}

uint32_t fhss_current_frequency(const fhss_t *f)
{
    return fhss_channel_frequency(f, fhss_current_channel(f));
}

uint32_t fhss_ms_until_hop(const fhss_t *f, uint32_t now_ms)
{
    if (f->state != FHSS_STATE_HOPPING) {
        return UINT32_MAX;
    }
    uint32_t elapsed = now_ms - f->dwell_start_ms;
    return elapsed >= f->cfg.dwell_ms ? 0 : f->cfg.dwell_ms - elapsed;
}

fhss_state_t fhss_state(const fhss_t *f)
{
    return f->state;
}

void fhss_sync_encode(const fhss_sync_t *s, uint8_t out[FHSS_SYNC_WIRE_SIZE])
{
    /* little-endian on the wire regardless of MCU */
    out[0] = (uint8_t)(s->hop_index & 0xff);
    out[1] = (uint8_t)(s->hop_index >> 8);
    out[2] = (uint8_t)(s->ms_into_dwell & 0xff);
    out[3] = (uint8_t)(s->ms_into_dwell >> 8);
}

void fhss_sync_decode(const uint8_t in[FHSS_SYNC_WIRE_SIZE], fhss_sync_t *s)
{
    s->hop_index = (uint16_t)(in[0] | (in[1] << 8));
    s->ms_into_dwell = (uint16_t)(in[2] | (in[3] << 8));
}
