/*
 * Host-side tests: simulate the hop clock millisecond by millisecond and
 * check the FCC-derived requirements directly against what the radio was
 * told to do.
 */
#include "fhss.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

/* ---- fake radio: records every retune ---------------------------------- */

#define MAX_EVENTS 20000

typedef struct {
    uint32_t t_ms[MAX_EVENTS];
    uint32_t freq[MAX_EVENTS];
    int n;
    uint32_t now;
} radio_log_t;

static void log_set_frequency(void *user, uint32_t hz)
{
    radio_log_t *r = user;
    if (r->n > 0 && r->freq[r->n - 1] == hz) {
        return; /* retune to same frequency is not a new visit */
    }
    if (r->n < MAX_EVENTS) {
        r->t_ms[r->n] = r->now;
        r->freq[r->n] = hz;
        r->n++;
    }
}

static fhss_config_t default_cfg(fhss_role_t role)
{
    fhss_config_t c = {
        .base_freq_hz = 902200000u,
        .channel_spacing_hz = 500000u, /* 50 ch: 902.2 - 926.7 MHz */
        .num_channels = 50,
        .dwell_ms = 400,
        .seed = 0xC0FFEEu,
        .role = role,
        .guard_ms = 10,
        .rx_latency_ms = 0,
        .max_missed_hops = 3,
    };
    return c;
}

static uint16_t chan_of(const fhss_config_t *c, uint32_t hz)
{
    return (uint16_t)((hz - c->base_freq_hz) / c->channel_spacing_hz);
}

/* Run a leader from start_ms for duration_ms, logging retunes. */
static void run_leader(fhss_t *f, radio_log_t *r, uint32_t start_ms, uint32_t duration_ms)
{
    r->now = start_ms;
    fhss_start(f, start_ms);
    for (uint32_t i = 1; i <= duration_ms; i++) {
        r->now = start_ms + i;
        fhss_tick(f, r->now);
    }
}

/* Verify dwell / revisit / occupancy rules from a retune log. */
static void check_hop_log(const char *name, const fhss_config_t *c, const radio_log_t *r)
{
    uint32_t last_start[256];
    uint32_t visits[256];
    memset(visits, 0, sizeof(visits));

    for (int i = 0; i < r->n; i++) {
        uint16_t ch = chan_of(c, r->freq[i]);

        CHECK(ch < c->num_channels, "%s: channel %u out of range", name, ch);
        CHECK(r->freq[i] >= FHSS_BAND_MIN_HZ && r->freq[i] <= FHSS_BAND_MAX_HZ,
              "%s: %u Hz out of band", name, r->freq[i]);

        if (i + 1 < r->n) {
            uint32_t dwell = r->t_ms[i + 1] - r->t_ms[i];
            CHECK(dwell == c->dwell_ms, "%s: dwell %u ms at hop %d", name, dwell, i);
            CHECK(dwell <= 400, "%s: dwell %u ms exceeds 400 ms", name, dwell);
        }
        if (visits[ch]) {
            uint32_t gap = r->t_ms[i] - last_start[ch];
            CHECK(gap >= FHSS_REVISIT_WINDOW_MS, "%s: channel %u revisited after %u ms",
                  name, ch, gap);
        }
        last_start[ch] = r->t_ms[i];
        visits[ch]++;
    }

    /* Equal probability: over whole cycles, each channel used equally. */
    int full_cycles = (r->n - 1) / c->num_channels;
    for (uint16_t ch = 0; ch < c->num_channels; ch++) {
        CHECK(visits[ch] >= (uint32_t)full_cycles && visits[ch] <= (uint32_t)full_cycles + 1,
              "%s: channel %u used %u times over %d cycles", name, ch, visits[ch],
              full_cycles);
    }
}

/* ---- tests ------------------------------------------------------------- */

static void test_sequence_is_permutation(void)
{
    printf("sequence is a permutation (1000 seeds)\n");
    fhss_config_t c = default_cfg(FHSS_ROLE_LEADER);
    fhss_hal_t hal = {0};
    for (uint32_t seed = 0; seed < 1000; seed++) {
        c.seed = seed * 2654435761u;
        fhss_t f;
        CHECK(fhss_init(&f, &c, &hal) == FHSS_OK, "init seed %u", seed);
        uint8_t seen[256] = {0};
        for (int i = 0; i < c.num_channels; i++) {
            seen[f.sequence[i]]++;
        }
        for (int i = 0; i < c.num_channels; i++) {
            CHECK(seen[i] == 1, "seed %u: channel %d appears %d times", seed, i, seen[i]);
        }
    }
}

static void test_same_seed_same_sequence(void)
{
    printf("same seed -> same sequence, different seed -> different\n");
    fhss_config_t c = default_cfg(FHSS_ROLE_LEADER);
    fhss_hal_t hal = {0};
    fhss_t a, b, d;
    fhss_init(&a, &c, &hal);
    fhss_init(&b, &c, &hal);
    c.seed ^= 1;
    fhss_init(&d, &c, &hal);
    CHECK(memcmp(a.sequence, b.sequence, c.num_channels) == 0, "same seed differs");
    CHECK(memcmp(a.sequence, d.sequence, c.num_channels) != 0, "adjacent seeds match");
}

static void test_leader_timing(void)
{
    printf("leader: 400 ms dwell, no revisit within 20 s, equal use (10 min)\n");
    fhss_config_t c = default_cfg(FHSS_ROLE_LEADER);
    static radio_log_t r;
    memset(&r, 0, sizeof(r));
    fhss_hal_t hal = {log_set_frequency, &r};
    fhss_t f;
    CHECK(fhss_init(&f, &c, &hal) == FHSS_OK, "init");
    run_leader(&f, &r, 0, 10u * 60u * 1000u);
    CHECK(r.n == 1501, "expected 1501 retunes in 10 min, got %d", r.n);
    check_hop_log("leader", &c, &r);
}

static void test_more_channels(void)
{
    printf("leader with 64 channels\n");
    fhss_config_t c = default_cfg(FHSS_ROLE_LEADER);
    c.num_channels = 64;
    c.channel_spacing_hz = 400000u;
    static radio_log_t r;
    memset(&r, 0, sizeof(r));
    fhss_hal_t hal = {log_set_frequency, &r};
    fhss_t f;
    CHECK(fhss_init(&f, &c, &hal) == FHSS_OK, "init");
    run_leader(&f, &r, 0, 5u * 60u * 1000u);
    check_hop_log("64ch", &c, &r);
}

static void test_sliding_window_occupancy(void)
{
    printf("any 20 s window: <= 400 ms on each channel\n");
    fhss_config_t c = default_cfg(FHSS_ROLE_LEADER);
    static radio_log_t r;
    memset(&r, 0, sizeof(r));
    fhss_hal_t hal = {log_set_frequency, &r};
    fhss_t f;
    fhss_init(&f, &c, &hal);

    const uint32_t total = 120000;
    static uint8_t chan_at[120000];
    r.now = 0;
    fhss_start(&f, 0);
    for (uint32_t t = 0; t < total; t++) {
        fhss_tick(&f, t);
        chan_at[t] = (uint8_t)fhss_current_channel(&f);
    }

    uint32_t occ[256] = {0};
    uint32_t worst = 0;
    for (uint32_t t = 0; t < total; t++) {
        occ[chan_at[t]]++;
        if (t >= FHSS_REVISIT_WINDOW_MS) {
            occ[chan_at[t - FHSS_REVISIT_WINDOW_MS]]--;
        }
        if (occ[chan_at[t]] > worst) {
            worst = occ[chan_at[t]];
        }
    }
    CHECK(worst <= 400, "worst 20 s occupancy %u ms", worst);
}

static void test_first_hop_uniform_across_seeds(void)
{
    printf("hop position -> channel is uniform across seeds (chi-square)\n");
    fhss_config_t c = default_cfg(FHSS_ROLE_LEADER);
    fhss_hal_t hal = {0};
    enum { SEEDS = 50000 };
    uint32_t counts[50] = {0};
    for (uint32_t s = 0; s < SEEDS; s++) {
        c.seed = s;
        fhss_t f;
        fhss_init(&f, &c, &hal);
        counts[f.sequence[0]]++;
    }
    double expected = (double)SEEDS / 50.0, chi2 = 0;
    for (int i = 0; i < 50; i++) {
        double d = counts[i] - expected;
        chi2 += d * d / expected;
    }
    /* 49 dof, p = 0.001 critical value ~ 85.4 */
    CHECK(chi2 < 85.4, "chi-square %.1f too high", chi2);
}

static void test_config_validation(void)
{
    printf("config validation\n");
    fhss_hal_t hal = {0};
    fhss_t f;
    fhss_config_t c = default_cfg(FHSS_ROLE_LEADER);

    c.num_channels = 49;
    CHECK(fhss_init(&f, &c, &hal) == FHSS_ERR_TOO_FEW_CHANNELS, "49 channels accepted");

    c = default_cfg(FHSS_ROLE_LEADER);
    c.dwell_ms = 401;
    CHECK(fhss_init(&f, &c, &hal) == FHSS_ERR_BAD_DWELL, "401 ms dwell accepted");

    c = default_cfg(FHSS_ROLE_LEADER);
    c.dwell_ms = 0;
    CHECK(fhss_init(&f, &c, &hal) == FHSS_OK && f.cfg.dwell_ms == 400, "default dwell");

    c = default_cfg(FHSS_ROLE_LEADER);
    c.base_freq_hz = 915000000u; /* 50 * 500k runs past 928 MHz */
    CHECK(fhss_init(&f, &c, &hal) == FHSS_ERR_OUT_OF_BAND, "out of band accepted");

    c = default_cfg(FHSS_ROLE_LEADER);
    c.num_channels = FHSS_MAX_CHANNELS + 1;
    CHECK(fhss_init(&f, &c, &hal) == FHSS_ERR_TOO_MANY_CHANNELS, "too many accepted");

    c = default_cfg(FHSS_ROLE_LEADER);
    CHECK(fhss_init(NULL, &c, &hal) == FHSS_ERR_NULL, "null accepted");
}

static void test_clock_wraparound(void)
{
    printf("millisecond clock wraparound\n");
    fhss_config_t c = default_cfg(FHSS_ROLE_LEADER);
    static radio_log_t r;
    memset(&r, 0, sizeof(r));
    fhss_hal_t hal = {log_set_frequency, &r};
    fhss_t f;
    fhss_init(&f, &c, &hal);
    run_leader(&f, &r, UINT32_MAX - 30000u, 60000u);
    check_hop_log("wrap", &c, &r);
}

static void test_late_ticks_keep_schedule(void)
{
    printf("late ticks do not drift the schedule\n");
    fhss_config_t c = default_cfg(FHSS_ROLE_LEADER);
    fhss_hal_t hal = {0};
    fhss_t f;
    fhss_init(&f, &c, &hal);
    fhss_start(&f, 0);
    for (uint32_t t = 0; t <= 100000; t += 37) { /* irregular 37 ms tick */
        fhss_tick(&f, t);
    }
    fhss_tick(&f, 100000);
    CHECK(f.hop_index == (100000 / 400) % 50, "hop index %u", f.hop_index);
    CHECK(f.dwell_start_ms == 100000, "dwell start %u", f.dwell_start_ms);
}

static void test_leader_follower_sync(void)
{
    printf("follower acquires, tracks, loses and reacquires leader\n");
    fhss_config_t lc = default_cfg(FHSS_ROLE_LEADER);
    fhss_config_t fc = default_cfg(FHSS_ROLE_FOLLOWER);
    fc.rx_latency_ms = 5;
    fhss_hal_t nohal = {0};
    fhss_t lead, fol;
    fhss_init(&lead, &lc, &nohal);
    fhss_init(&fol, &fc, &nohal);

    uint32_t t0 = 1000;
    fhss_start(&lead, 0);
    fhss_start(&fol, t0); /* follower powers up later, out of phase */
    CHECK(fhss_state(&fol) == FHSS_STATE_SEARCHING, "follower should start searching");
    CHECK(!fhss_can_transmit(&fol, t0), "searching follower must not transmit");

    uint32_t locked_at = 0, mismatched_ms = 0;
    bool link_up = true;
    const uint32_t outage_start = 60000, outage_end = 63000, end = 120000;

    for (uint32_t t = 0; t < end; t++) {
        fhss_tick(&lead, t);
        if (t >= t0) {
            fhss_tick(&fol, t);
        }
        link_up = !(t >= outage_start && t < outage_end);

        /* Leader sends a packet every 50 ms; it arrives rx_latency later. */
        if (t >= t0 && link_up && t % 50 == 0 && fhss_can_transmit(&lead, t)) {
            uint32_t rx_t = t + fc.rx_latency_ms;
            bool same_freq = fhss_current_frequency(&lead) == fhss_current_frequency(&fol);
            if (same_freq) {
                fhss_sync_t s;
                uint8_t wire[FHSS_SYNC_WIRE_SIZE];
                fhss_get_sync(&lead, t, &s);
                fhss_sync_encode(&s, wire);
                fhss_sync_decode(wire, &s);
                /* deliver after latency: advance follower clock first */
                for (uint32_t k = t + 1; k <= rx_t; k++) {
                    fhss_tick(&fol, k);
                }
                CHECK(fhss_on_sync(&fol, rx_t, &s) == FHSS_OK, "on_sync");
                if (!locked_at) {
                    locked_at = rx_t;
                }
            }
        }

        if (locked_at && t > locked_at && t < outage_start) {
            if (fhss_current_frequency(&lead) != fhss_current_frequency(&fol)) {
                mismatched_ms++;
            }
        }
    }

    CHECK(locked_at != 0, "never locked");
    CHECK(locked_at - t0 <= 50u * 400u + 400u, "acquisition took %u ms", locked_at - t0);
    CHECK(mismatched_ms == 0, "out of step for %u ms while locked", mismatched_ms);
    CHECK(fhss_state(&fol) == FHSS_STATE_HOPPING, "follower should have reacquired");
    CHECK(fhss_current_frequency(&lead) == fhss_current_frequency(&fol),
          "follower on wrong channel at end");
}

static void test_follower_drops_to_search(void)
{
    printf("follower returns to search after max_missed_hops\n");
    fhss_config_t c = default_cfg(FHSS_ROLE_FOLLOWER);
    fhss_hal_t hal = {0};
    fhss_t f;
    fhss_init(&f, &c, &hal);
    fhss_start(&f, 0);
    fhss_sync_t s = {.hop_index = 10, .ms_into_dwell = 0};
    fhss_on_sync(&f, 0, &s);
    CHECK(fhss_state(&f) == FHSS_STATE_HOPPING, "should lock");
    /* dwell 0 was synced; dwells 1-3 are silent, so drop at the 4th boundary */
    for (uint32_t t = 0; t < 4 * 400; t++) {
        fhss_tick(&f, t);
    }
    CHECK(fhss_state(&f) == FHSS_STATE_HOPPING, "dropped too early");
    fhss_tick(&f, 4 * 400);
    CHECK(fhss_state(&f) == FHSS_STATE_SEARCHING, "did not drop after 3 missed hops");
}

static void test_bad_sync_rejected(void)
{
    printf("malformed sync rejected\n");
    fhss_config_t c = default_cfg(FHSS_ROLE_FOLLOWER);
    fhss_hal_t hal = {0};
    fhss_t f;
    fhss_init(&f, &c, &hal);
    fhss_start(&f, 0);
    fhss_sync_t bad1 = {.hop_index = 50, .ms_into_dwell = 0};
    fhss_sync_t bad2 = {.hop_index = 0, .ms_into_dwell = 400};
    CHECK(fhss_on_sync(&f, 0, &bad1) == FHSS_ERR_BAD_SYNC, "hop index 50 accepted");
    CHECK(fhss_on_sync(&f, 0, &bad2) == FHSS_ERR_BAD_SYNC, "ms_into_dwell 400 accepted");
    CHECK(fhss_state(&f) == FHSS_STATE_SEARCHING, "bad sync changed state");
}

static void test_guard_time(void)
{
    printf("no transmit within guard time of a hop\n");
    fhss_config_t c = default_cfg(FHSS_ROLE_LEADER);
    fhss_hal_t hal = {0};
    fhss_t f;
    fhss_init(&f, &c, &hal);
    fhss_start(&f, 0);
    CHECK(fhss_can_transmit(&f, 0), "should transmit at dwell start");
    CHECK(fhss_can_transmit(&f, 389), "should transmit 11 ms before hop");
    CHECK(!fhss_can_transmit(&f, 390), "transmitting 10 ms before hop");
    CHECK(!fhss_can_transmit(&f, 399), "transmitting 1 ms before hop");
}

int main(void)
{
    test_sequence_is_permutation();
    test_same_seed_same_sequence();
    test_leader_timing();
    test_more_channels();
    test_sliding_window_occupancy();
    test_first_hop_uniform_across_seeds();
    test_config_validation();
    test_clock_wraparound();
    test_late_ticks_keep_schedule();
    test_leader_follower_sync();
    test_follower_drops_to_search();
    test_bad_sync_rejected();
    test_guard_time();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
