/*
 * fhss.h - Portable Frequency Hopping Spread Spectrum layer.
 *
 * Hardware independent: the library never touches a radio or a timer
 * directly. The port supplies a millisecond clock (passed into each call)
 * and a set_frequency() callback. Everything else is plain C99.
 *
 * Hopping rules enforced (FCC 47 CFR 15.247(a)(1)(i), 902-928 MHz band):
 *   - Dwell time per channel:  FHSS_DWELL_MS (400 ms)
 *   - A channel is not revisited within FHSS_REVISIT_WINDOW_MS (20 s)
 *   - Every channel is used equally often (pseudo-random permutation,
 *     each channel exactly once per hop cycle)
 *
 * Because the sequence is a permutation walked in order, a channel comes
 * back exactly num_channels * dwell_ms after it was last used. fhss_init()
 * rejects any configuration where that is shorter than the revisit window,
 * so at 400 ms dwell you need at least 50 channels.
 */
#ifndef FHSS_H
#define FHSS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FHSS_MAX_CHANNELS
#define FHSS_MAX_CHANNELS 128
#endif

#ifndef FHSS_DWELL_MS
#define FHSS_DWELL_MS 400u
#endif

#ifndef FHSS_REVISIT_WINDOW_MS
#define FHSS_REVISIT_WINDOW_MS 20000u
#endif

/* US 902-928 MHz ISM band edges. Override for other bands. */
#ifndef FHSS_BAND_MIN_HZ
#define FHSS_BAND_MIN_HZ 902000000u
#endif
#ifndef FHSS_BAND_MAX_HZ
#define FHSS_BAND_MAX_HZ 928000000u
#endif

typedef enum {
    FHSS_OK = 0,
    FHSS_ERR_NULL,
    FHSS_ERR_TOO_FEW_CHANNELS, /* num_channels * dwell < revisit window */
    FHSS_ERR_TOO_MANY_CHANNELS,
    FHSS_ERR_OUT_OF_BAND,
    FHSS_ERR_BAD_DWELL,
    FHSS_ERR_BAD_SYNC
} fhss_err_t;

typedef enum {
    FHSS_ROLE_LEADER,   /* owns the hop clock, beacons sync info */
    FHSS_ROLE_FOLLOWER  /* locks onto the leader's sync info */
} fhss_role_t;

typedef enum {
    FHSS_STATE_HOPPING,   /* leader always, follower once locked */
    FHSS_STATE_SEARCHING  /* follower parked on one channel, waiting */
} fhss_state_t;

typedef struct {
    uint32_t base_freq_hz;       /* frequency of channel 0 */
    uint32_t channel_spacing_hz; /* spacing between adjacent channels */
    uint16_t num_channels;
    uint16_t dwell_ms;           /* 0 -> FHSS_DWELL_MS */
    uint32_t seed;               /* shared network seed, same on all nodes */
    fhss_role_t role;
    uint16_t guard_ms;           /* no TX this close to a hop boundary */
    uint16_t rx_latency_ms;      /* airtime + processing, added on sync */
    uint8_t max_missed_hops;     /* follower: dwells with no sync before searching */
} fhss_config_t;

typedef struct {
    /* Retune the radio. Called from fhss_start()/fhss_tick()/fhss_on_sync(). */
    void (*set_frequency)(void *user, uint32_t freq_hz);
    void *user;
} fhss_hal_t;

/* Sync info carried in every packet the leader sends (4 bytes on air). */
typedef struct {
    uint16_t hop_index;     /* position in the hop sequence */
    uint16_t ms_into_dwell; /* how long the leader has been on this hop */
} fhss_sync_t;

#define FHSS_SYNC_WIRE_SIZE 4

typedef struct {
    fhss_config_t cfg;
    fhss_hal_t hal;
    uint8_t sequence[FHSS_MAX_CHANNELS];
    uint16_t hop_index;
    uint32_t dwell_start_ms;
    fhss_state_t state;
    uint8_t missed_hops;
    bool synced_this_dwell;
} fhss_t;

fhss_err_t fhss_init(fhss_t *f, const fhss_config_t *cfg, const fhss_hal_t *hal);

/* Tune the first channel and start the hop clock. */
void fhss_start(fhss_t *f, uint32_t now_ms);

/*
 * Call often (main loop or a >= 1 kHz timer). Advances hops that are due
 * and retunes the radio. Returns true if the channel changed.
 */
bool fhss_tick(fhss_t *f, uint32_t now_ms);

/* Leader: fill the sync header for an outgoing packet. */
void fhss_get_sync(const fhss_t *f, uint32_t now_ms, fhss_sync_t *out);

/* Follower: feed the sync header from any received packet. */
fhss_err_t fhss_on_sync(fhss_t *f, uint32_t now_ms, const fhss_sync_t *sync);

/* True if a packet can start now without crossing a hop boundary. */
bool fhss_can_transmit(const fhss_t *f, uint32_t now_ms);

uint16_t fhss_current_channel(const fhss_t *f);
uint32_t fhss_current_frequency(const fhss_t *f);
uint32_t fhss_channel_frequency(const fhss_t *f, uint16_t channel);
uint32_t fhss_ms_until_hop(const fhss_t *f, uint32_t now_ms);
fhss_state_t fhss_state(const fhss_t *f);

void fhss_sync_encode(const fhss_sync_t *s, uint8_t out[FHSS_SYNC_WIRE_SIZE]);
void fhss_sync_decode(const uint8_t in[FHSS_SYNC_WIRE_SIZE], fhss_sync_t *s);

#ifdef __cplusplus
}
#endif

#endif /* FHSS_H */
