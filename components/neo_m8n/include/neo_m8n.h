/*
 * neo_m8n.h - u-blox NEO-M8N GNSS receiver driver.
 *
 * One component, two targets. Everything above the UART is shared; the four
 * dozen lines that differ live in port/neo_m8n_port_<target>.c and are picked
 * by NEO_M8N_TARGET_ESP32 / NEO_M8N_TARGET_PICO (see below).
 *
 * The module talks NMEA 0183 out of the box (9600 8N1, 1 Hz, GGA/GLL/GSA/GSV/
 * RMC/VTG). This driver reads that, and additionally speaks enough of u-blox's
 * UBX binary protocol to retune the receiver: update rate, baud rate, and which
 * NMEA sentences it bothers to send.
 *
 * Single instance: one NEO-M8N per build, state is static. That matches how the
 * module is actually wired on these boards and keeps the RX buffers out of the
 * heap.
 *
 * Typical use:
 *
 *      neo_m8n_config_t cfg = NEO_M8N_CONFIG_DEFAULT();
 *      cfg.nav_rate_ms = 200;                  // 5 Hz
 *      cfg.target_baud = 38400;                // 9600 cannot carry 5 Hz
 *      if (neo_m8n_init(&cfg) != NEO_M8N_OK) { ... }
 *
 *      neo_m8n_fix_t fix;
 *      while (1) {
 *          if (neo_m8n_read(&fix, 1500) == NEO_M8N_OK && fix.valid) {
 *              printf("%.6f %.6f  %.1f m\n", fix.latitude, fix.longitude, fix.altitude_m);
 *          }
 *      }
 */

#ifndef NEO_M8N_H
#define NEO_M8N_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "nmea.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Target selection                                                    */
/* ------------------------------------------------------------------ */
/*
 * The build system defines exactly one of these (see CMakeLists.txt). The
 * fallback below infers it from the SDK's own macros so the header still
 * resolves when it is included by an editor, a unit test or a source file
 * compiled outside the component.
 *
 * ESP_PLATFORM is defined by every ESP-IDF build; PICO_RP2040 / PICO_RP2350
 * come from the Pico SDK's platform definitions. Anything else is treated as a
 * host build, which uses the injection port in port/neo_m8n_port_host.c.
 */
#if !defined(NEO_M8N_TARGET_ESP32) && \
    !defined(NEO_M8N_TARGET_PICO)  && \
    !defined(NEO_M8N_TARGET_HOST)
#  if defined(ESP_PLATFORM)
#    define NEO_M8N_TARGET_ESP32 1
#  elif defined(PICO_RP2040) || defined(PICO_RP2350) || defined(PICO_ON_DEVICE)
#    define NEO_M8N_TARGET_PICO 1
#  else
#    define NEO_M8N_TARGET_HOST 1
#  endif
#endif

/* ------------------------------------------------------------------ */
/* Errors                                                              */
/* ------------------------------------------------------------------ */

typedef enum {
    NEO_M8N_OK              =  0,
    NEO_M8N_ERR_INVALID_ARG = -1,
    NEO_M8N_ERR_STATE       = -2,  /* called before init, or init twice */
    NEO_M8N_ERR_IO          = -3,  /* the UART itself failed */
    NEO_M8N_ERR_TIMEOUT     = -4,  /* no data, or no ACK, within the deadline */
    NEO_M8N_ERR_NACK        = -5,  /* receiver rejected the configuration */
    NEO_M8N_ERR_NO_FIX      = -6,  /* data arrived but the receiver has no fix */
} neo_m8n_err_t;

const char *neo_m8n_err_str(neo_m8n_err_t err);

/* ------------------------------------------------------------------ */
/* Defaults                                                            */
/* ------------------------------------------------------------------ */
/*
 * On ESP-IDF these come from menuconfig (see Kconfig). Elsewhere, and for any
 * symbol menuconfig did not define, the values below apply. Override any of
 * them with -DNEO_M8N_DEFAULT_xxx=... at build time, or just fill in the
 * config struct yourself.
 */
#if defined(CONFIG_NEO_M8N_UART_NUM) && !defined(NEO_M8N_DEFAULT_UART_ID)
#  define NEO_M8N_DEFAULT_UART_ID   CONFIG_NEO_M8N_UART_NUM
#endif
#if defined(CONFIG_NEO_M8N_TX_GPIO) && !defined(NEO_M8N_DEFAULT_TX_PIN)
#  define NEO_M8N_DEFAULT_TX_PIN    CONFIG_NEO_M8N_TX_GPIO
#endif
#if defined(CONFIG_NEO_M8N_RX_GPIO) && !defined(NEO_M8N_DEFAULT_RX_PIN)
#  define NEO_M8N_DEFAULT_RX_PIN    CONFIG_NEO_M8N_RX_GPIO
#endif
#if defined(CONFIG_NEO_M8N_BAUD) && !defined(NEO_M8N_DEFAULT_BAUD)
#  define NEO_M8N_DEFAULT_BAUD      CONFIG_NEO_M8N_BAUD
#endif

#ifndef NEO_M8N_DEFAULT_UART_ID
#  if NEO_M8N_TARGET_PICO
#    define NEO_M8N_DEFAULT_UART_ID 1        /* uart0 is usually the console */
#  else
#    define NEO_M8N_DEFAULT_UART_ID 2        /* UART_NUM_2; 0 is the monitor */
#  endif
#endif
#ifndef NEO_M8N_DEFAULT_TX_PIN
#  if NEO_M8N_TARGET_PICO
#    define NEO_M8N_DEFAULT_TX_PIN  4        /* GP4 -> module RX */
#  else
#    define NEO_M8N_DEFAULT_TX_PIN  17
#  endif
#endif
#ifndef NEO_M8N_DEFAULT_RX_PIN
#  if NEO_M8N_TARGET_PICO
#    define NEO_M8N_DEFAULT_RX_PIN  5        /* GP5 <- module TX */
#  else
#    define NEO_M8N_DEFAULT_RX_PIN  16
#  endif
#endif
#ifndef NEO_M8N_DEFAULT_BAUD
#  define NEO_M8N_DEFAULT_BAUD      9600     /* factory default */
#endif
#ifndef NEO_M8N_DEFAULT_RX_BUF
#  define NEO_M8N_DEFAULT_RX_BUF    1024
#endif

/* Longest UBX payload this driver sends or keeps (CFG-PRT is 20). */
#define NEO_M8N_UBX_MAX_PAYLOAD 64

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */

typedef struct {
    int      uart_id;      /* ESP32: UART_NUM_x. Pico: 0 or 1. */
    int      tx_pin;       /* MCU TX -> module RX. -1 leaves the pin unassigned. */
    int      rx_pin;       /* MCU RX <- module TX. Required. */
    uint32_t baud;         /* baud the module is speaking now (9600 from the factory) */

    /*
     * Applied by neo_m8n_init() when auto_config is set. Each is a no-op at 0.
     */
    bool     auto_config;  /* send the UBX configuration below during init */
    uint32_t target_baud;  /* switch the link to this rate, then keep talking */
    uint16_t nav_rate_ms;  /* solution interval: 1000 = 1 Hz, 200 = 5 Hz */
    bool     trim_nmea;    /* keep GGA+RMC+GSA, silence GLL/GSV/VTG */
    bool     save_config;  /* persist to battery-backed RAM and flash */

    size_t   rx_buf_size;  /* driver-side receive buffer */
} neo_m8n_config_t;

/*
 * Sensible starting point: the factory 9600 baud, 1 Hz, and just enough UBX to
 * quiet the sentences nobody parses. Designated initialisers keep this valid if
 * fields are added later.
 */
#define NEO_M8N_CONFIG_DEFAULT()                    \
    ((neo_m8n_config_t){                            \
        .uart_id     = NEO_M8N_DEFAULT_UART_ID,     \
        .tx_pin      = NEO_M8N_DEFAULT_TX_PIN,      \
        .rx_pin      = NEO_M8N_DEFAULT_RX_PIN,      \
        .baud        = NEO_M8N_DEFAULT_BAUD,        \
        .auto_config = true,                        \
        .target_baud = 0,                           \
        .nav_rate_ms = 1000,                        \
        .trim_nmea   = true,                        \
        .save_config = false,                       \
        .rx_buf_size = NEO_M8N_DEFAULT_RX_BUF,      \
    })

/* ------------------------------------------------------------------ */
/* Fix data                                                            */
/* ------------------------------------------------------------------ */

/* GGA quality indicator. */
typedef enum {
    NEO_M8N_FIX_NONE      = 0,
    NEO_M8N_FIX_GPS       = 1,
    NEO_M8N_FIX_DGPS      = 2,
    NEO_M8N_FIX_PPS       = 3,
    NEO_M8N_FIX_RTK       = 4,
    NEO_M8N_FIX_RTK_FLOAT = 5,
    NEO_M8N_FIX_ESTIMATED = 6,
} neo_m8n_fix_quality_t;

/*
 * A merged view of the last sentence group. Different sentences fill different
 * fields, so a field is only as fresh as the sentence that carries it; see
 * `age_ms` for how long ago the position itself was updated.
 */
typedef struct {
    bool        valid;          /* RMC reported 'A' and a position was decoded */
    double      latitude;       /* decimal degrees, north positive */
    double      longitude;      /* decimal degrees, east positive */
    float       altitude_m;     /* above mean sea level (GGA) */
    float       geoid_sep_m;    /* MSL minus WGS84 ellipsoid (GGA) */
    float       speed_mps;      /* ground speed (RMC, converted from knots) */
    float       speed_knots;
    float       course_deg;     /* true course over ground (RMC) */

    uint8_t     fix_quality;    /* neo_m8n_fix_quality_t, from GGA */
    uint8_t     fix_type;       /* 1 none, 2 = 2D, 3 = 3D, from GSA */
    uint8_t     satellites_used;
    uint8_t     satellites_visible;
    float       hdop;
    float       pdop;
    float       vdop;

    nmea_time_t time;           /* UTC; date only after an RMC has been seen */
    uint32_t    age_ms;         /* since the last position update, at snapshot time */
} neo_m8n_fix_t;

typedef struct {
    uint32_t bytes_rx;
    uint32_t sentences;         /* checksum-valid NMEA sentences */
    uint32_t checksum_errors;
    uint32_t overruns;
    uint32_t ubx_frames;        /* UBX frames decoded (ACK/NAK and anything else) */
    uint32_t ubx_errors;        /* UBX frames dropped on a bad checksum */
} neo_m8n_stats_t;

/*
 * Called for every checksum-valid sentence, before it is parsed. Handy for
 * logging the raw stream to an SD card. Runs on whichever task called
 * neo_m8n_read()/neo_m8n_poll(); keep it short.
 */
typedef void (*neo_m8n_sentence_cb_t)(const char *sentence, void *user);

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

/*
 * Brings up the UART and, if cfg->auto_config is set, configures the receiver.
 *
 * Configuration failures are logged but do not fail init: a receiver that
 * ignores UBX still produces a perfectly good default NMEA stream, and a
 * tracker should not refuse to boot over it. Call the neo_m8n_set_* functions
 * yourself if you need to check the result.
 */
neo_m8n_err_t neo_m8n_init(const neo_m8n_config_t *cfg);
neo_m8n_err_t neo_m8n_deinit(void);
bool          neo_m8n_is_initialized(void);

/* ------------------------------------------------------------------ */
/* Reading                                                             */
/* ------------------------------------------------------------------ */

/*
 * Drains the UART for up to timeout_ms and parses whatever arrives. Returns
 * NEO_M8N_OK once a position sentence (GGA or RMC) has been decoded, or
 * NEO_M8N_ERR_TIMEOUT if the deadline passes first. `out` may be NULL if you
 * only want to pump the parser.
 *
 * NEO_M8N_OK does not mean there is a fix - check `out->valid`. A receiver
 * that is still acquiring emits perfectly valid sentences with empty position
 * fields, typically for 30 s or so from cold.
 */
neo_m8n_err_t neo_m8n_read(neo_m8n_fix_t *out, uint32_t timeout_ms);

/*
 * As neo_m8n_read(), but keeps going until the receiver actually has a fix.
 * Returns NEO_M8N_ERR_TIMEOUT if it never does. Allow tens of seconds from a
 * cold start.
 */
neo_m8n_err_t neo_m8n_wait_for_fix(neo_m8n_fix_t *out, uint32_t timeout_ms);

/*
 * Non-blocking: pumps whatever bytes are already buffered, parses them, and
 * returns without waiting. Use it from a periodic task that has other work to
 * do. Returns the number of sentences parsed, or a negative neo_m8n_err_t.
 */
int neo_m8n_poll(void);

/* Most recent merged state, without touching the UART. Returns false if no
 * position sentence has been decoded yet. */
bool neo_m8n_get_fix(neo_m8n_fix_t *out);

/* Milliseconds since the last position sentence, or UINT32_MAX if there has
 * never been one. */
uint32_t neo_m8n_fix_age_ms(void);

/*
 * Parses bytes from somewhere other than this driver's UART - an I2C/DDC read,
 * a replayed log, a test. Safe to mix with the UART path.
 */
void neo_m8n_feed(const uint8_t *data, size_t len);

void neo_m8n_set_sentence_cb(neo_m8n_sentence_cb_t cb, void *user);

void neo_m8n_get_stats(neo_m8n_stats_t *out);
void neo_m8n_reset_stats(void);

/* ------------------------------------------------------------------ */
/* Configuration (UBX)                                                 */
/* ------------------------------------------------------------------ */

/* NMEA message IDs under UBX class 0xF0, for neo_m8n_set_nmea_rate(). */
typedef enum {
    NEO_M8N_NMEA_GGA = 0x00,
    NEO_M8N_NMEA_GLL = 0x01,
    NEO_M8N_NMEA_GSA = 0x02,
    NEO_M8N_NMEA_GSV = 0x03,
    NEO_M8N_NMEA_RMC = 0x04,
    NEO_M8N_NMEA_VTG = 0x05,
    NEO_M8N_NMEA_GRS = 0x06,
    NEO_M8N_NMEA_GST = 0x07,
    NEO_M8N_NMEA_ZDA = 0x08,
    NEO_M8N_NMEA_GBS = 0x09,
    NEO_M8N_NMEA_DTM = 0x0A,
    NEO_M8N_NMEA_GNS = 0x0D,
} neo_m8n_nmea_msg_t;

/*
 * UBX-CFG-RATE. measRate is the solution interval in ms; 1000 is the default
 * 1 Hz and 100 the M8 floor. Anything faster than 1 Hz needs more than 9600
 * baud to fit the sentences - raise the baud rate first.
 */
neo_m8n_err_t neo_m8n_set_nav_rate(uint16_t measure_ms);

/*
 * UBX-CFG-MSG. rate is how many solution intervals pass between sentences:
 * 1 = every solution, 0 = off.
 */
neo_m8n_err_t neo_m8n_set_nmea_rate(neo_m8n_nmea_msg_t msg, uint8_t rate);

/*
 * UBX-CFG-PRT. Reconfigures the module's UART1 and then retunes the MCU side to
 * match, so the link stays up across the change.
 *
 * The receiver sends its ACK at the *old* rate and switches immediately after,
 * which is inherently racy, so this call does not wait for one; it verifies by
 * confirming that sentences resume at the new rate. Not persistent unless you
 * follow it with neo_m8n_save_config().
 */
neo_m8n_err_t neo_m8n_set_baud(uint32_t baud);

/* UBX-CFG-CFG: copies the current configuration into battery-backed RAM and
 * flash, so it survives a power cycle. */
neo_m8n_err_t neo_m8n_save_config(void);

/* Sends an arbitrary UBX frame and, if wait_ack is set, waits for its ACK-ACK.
 * Returns NEO_M8N_ERR_NACK on ACK-NAK. */
neo_m8n_err_t neo_m8n_send_ubx(uint8_t msg_class, uint8_t msg_id,
                               const uint8_t *payload, uint16_t len,
                               bool wait_ack, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* NEO_M8N_H */
