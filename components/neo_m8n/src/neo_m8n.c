/*
 * neo_m8n.c - portable core of the NEO-M8N driver. See include/neo_m8n.h.
 *
 * Everything here is platform-independent; all hardware access goes through
 * neo_m8n_port.h.
 *
 * Incoming bytes are fed to two decoders at once, an NMEA framer and a UBX
 * framer. The receiver interleaves the two protocols on the same wire - an
 * ACK for a configuration command lands in the middle of a burst of sentences -
 * so running both over every byte is what keeps a configuration exchange from
 * dropping position data on the floor.
 */

#include <string.h>

#include "neo_m8n.h"
#include "neo_m8n_port.h"

/* ------------------------------------------------------------------ */
/* UBX protocol constants                                             */
/* ------------------------------------------------------------------ */

#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

#define UBX_CLASS_NAV 0x01
#define UBX_CLASS_ACK 0x05
#define UBX_CLASS_CFG 0x06

#define UBX_ACK_NAK 0x00
#define UBX_ACK_ACK 0x01

#define UBX_CFG_PRT  0x00
#define UBX_CFG_MSG  0x01
#define UBX_CFG_RATE 0x08
#define UBX_CFG_CFG  0x09

/* NMEA sentences live under their own UBX class for CFG-MSG purposes. */
#define UBX_CLASS_NMEA 0xF0

/* CFG-PRT, UART1. */
#define UBX_PORT_UART1 0x01
/* 8 bits, no parity, 1 stop bit. Bits 6-7 = 0b11 (8-bit), bits 9-11 = 0b001
 * (none), bits 12-13 = 0b00 (1 stop), plus the reserved bit 4 u-blox sets. */
#define UBX_PRT_MODE_8N1 0x000008D0u
#define UBX_PROTO_UBX  0x0001
#define UBX_PROTO_NMEA 0x0002

/* CFG-CFG device masks. */
#define UBX_CFG_DEV_BBR   0x01
#define UBX_CFG_DEV_FLASH 0x02

/* The receiver is specified to ACK within a second; allow for a busy link. */
#define UBX_ACK_TIMEOUT_MS 1200

/* ------------------------------------------------------------------ */
/* Driver state                                                       */
/* ------------------------------------------------------------------ */

typedef enum {
    UBX_RX_SYNC1 = 0,
    UBX_RX_SYNC2,
    UBX_RX_CLASS,
    UBX_RX_ID,
    UBX_RX_LEN1,
    UBX_RX_LEN2,
    UBX_RX_PAYLOAD,
    UBX_RX_CK_A,
    UBX_RX_CK_B,
} ubx_rx_state_t;

typedef struct {
    ubx_rx_state_t state;
    uint8_t  msg_class;
    uint8_t  msg_id;
    uint16_t len;
    uint16_t idx;
    uint8_t  ck_a, ck_b;          /* running checksum over class..payload */
    uint8_t  payload[NEO_M8N_UBX_MAX_PAYLOAD];
    bool     truncated;           /* payload longer than we keep; drain it */

    /* Last ACK/NAK seen, consumed by wait_ack(). */
    bool     ack_pending;
    bool     ack_ok;
    uint8_t  ack_class;
    uint8_t  ack_id;
} ubx_rx_t;

typedef struct {
    bool             initialized;
    neo_m8n_config_t cfg;

    nmea_stream_t    nmea;
    char             line[NMEA_MAX_SENTENCE];
    ubx_rx_t         ubx;

    neo_m8n_fix_t    fix;
    bool             have_position;    /* a GGA or RMC has been decoded */
    uint32_t         pos_count;        /* position sentences decoded, ever */
    uint32_t         fix_ms;           /* port_millis() at the last one */

    neo_m8n_stats_t  stats;

    neo_m8n_sentence_cb_t cb;
    void            *cb_user;
} neo_m8n_t;

static neo_m8n_t s_gps;

/* Scratch for port_read(); sized for one NMEA burst at 1 Hz. */
#ifndef NEO_M8N_CHUNK
#define NEO_M8N_CHUNK 128
#endif

/* ------------------------------------------------------------------ */
/* Small helpers                                                      */
/* ------------------------------------------------------------------ */

const char *neo_m8n_err_str(neo_m8n_err_t err)
{
    switch (err) {
    case NEO_M8N_OK:              return "ok";
    case NEO_M8N_ERR_INVALID_ARG: return "invalid argument";
    case NEO_M8N_ERR_STATE:       return "bad state";
    case NEO_M8N_ERR_IO:          return "uart error";
    case NEO_M8N_ERR_TIMEOUT:     return "timeout";
    case NEO_M8N_ERR_NACK:        return "rejected by receiver";
    case NEO_M8N_ERR_NO_FIX:      return "no fix";
    }
    return "unknown";
}

/* Wrap-safe: correct across the 49-day rollover of a 32-bit millisecond tick. */
static uint32_t elapsed_since(uint32_t start)
{
    return neo_m8n_port_millis() - start;
}

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

/* ------------------------------------------------------------------ */
/* UBX receive                                                        */
/* ------------------------------------------------------------------ */

static void ubx_rx_reset(ubx_rx_t *u)
{
    u->state = UBX_RX_SYNC1;
    u->idx = 0;
    u->len = 0;
    u->ck_a = 0;
    u->ck_b = 0;
    u->truncated = false;
}

/* Fletcher-8 over class, id, length and payload. */
static void ubx_checksum(uint8_t *ck_a, uint8_t *ck_b, uint8_t byte)
{
    *ck_a = (uint8_t)(*ck_a + byte);
    *ck_b = (uint8_t)(*ck_b + *ck_a);
}

static void ubx_frame_done(neo_m8n_t *g)
{
    ubx_rx_t *u = &g->ubx;

    g->stats.ubx_frames++;

    if (u->msg_class == UBX_CLASS_ACK && u->len >= 2 && !u->truncated) {
        u->ack_pending = true;
        u->ack_ok = (u->msg_id == UBX_ACK_ACK);
        u->ack_class = u->payload[0];
        u->ack_id = u->payload[1];
        NEO_M8N_LOGD("%s for CFG 0x%02X/0x%02X",
                     u->ack_ok ? "ACK" : "NAK", u->ack_class, u->ack_id);
    }
    /* Other UBX classes are decoded and discarded: the driver asks for NMEA
     * only, so anything else is a leftover from a previous configuration. */
}

static void ubx_push(neo_m8n_t *g, uint8_t c)
{
    ubx_rx_t *u = &g->ubx;

    switch (u->state) {
    case UBX_RX_SYNC1:
        if (c == UBX_SYNC1) u->state = UBX_RX_SYNC2;
        break;

    case UBX_RX_SYNC2:
        /* A repeated 0xB5 is a fresh sync attempt, not a failure. */
        if (c == UBX_SYNC2)      u->state = UBX_RX_CLASS;
        else if (c == UBX_SYNC1) u->state = UBX_RX_SYNC2;
        else                     ubx_rx_reset(u);
        break;

    case UBX_RX_CLASS:
        u->ck_a = 0;
        u->ck_b = 0;
        ubx_checksum(&u->ck_a, &u->ck_b, c);
        u->msg_class = c;
        u->state = UBX_RX_ID;
        break;

    case UBX_RX_ID:
        ubx_checksum(&u->ck_a, &u->ck_b, c);
        u->msg_id = c;
        u->state = UBX_RX_LEN1;
        break;

    case UBX_RX_LEN1:
        ubx_checksum(&u->ck_a, &u->ck_b, c);
        u->len = c;
        u->state = UBX_RX_LEN2;
        break;

    case UBX_RX_LEN2:
        ubx_checksum(&u->ck_a, &u->ck_b, c);
        u->len |= (uint16_t)((uint16_t)c << 8);
        u->idx = 0;
        /* Keep what fits and count the rest through, so the checksum still
         * lines up and the framer stays synchronised. */
        u->truncated = (u->len > NEO_M8N_UBX_MAX_PAYLOAD);
        u->state = (u->len == 0) ? UBX_RX_CK_A : UBX_RX_PAYLOAD;
        break;

    case UBX_RX_PAYLOAD:
        ubx_checksum(&u->ck_a, &u->ck_b, c);
        if (u->idx < NEO_M8N_UBX_MAX_PAYLOAD) u->payload[u->idx] = c;
        if (++u->idx >= u->len) u->state = UBX_RX_CK_A;
        break;

    case UBX_RX_CK_A:
        if (c != u->ck_a) {
            g->stats.ubx_errors++;
            ubx_rx_reset(u);
        } else {
            u->state = UBX_RX_CK_B;
        }
        break;

    case UBX_RX_CK_B:
        if (c != u->ck_b) g->stats.ubx_errors++;
        else              ubx_frame_done(g);
        ubx_rx_reset(u);
        break;
    }
}

/* ------------------------------------------------------------------ */
/* UBX transmit                                                       */
/* ------------------------------------------------------------------ */

static neo_m8n_err_t ubx_write(uint8_t msg_class, uint8_t msg_id,
                               const uint8_t *payload, uint16_t len)
{
    uint8_t frame[8 + NEO_M8N_UBX_MAX_PAYLOAD];
    uint8_t ck_a = 0, ck_b = 0;
    size_t n = 0;
    int written;

    if (len > NEO_M8N_UBX_MAX_PAYLOAD) return NEO_M8N_ERR_INVALID_ARG;
    if (len > 0 && !payload) return NEO_M8N_ERR_INVALID_ARG;

    frame[n++] = UBX_SYNC1;
    frame[n++] = UBX_SYNC2;
    frame[n++] = msg_class;
    frame[n++] = msg_id;
    frame[n++] = (uint8_t)(len & 0xFF);
    frame[n++] = (uint8_t)(len >> 8);
    if (len) {
        memcpy(&frame[n], payload, len);
        n += len;
    }
    for (size_t i = 2; i < n; i++) ubx_checksum(&ck_a, &ck_b, frame[i]);
    frame[n++] = ck_a;
    frame[n++] = ck_b;

    written = neo_m8n_port_write(frame, n);
    if (written < 0) return (neo_m8n_err_t)written;
    return ((size_t)written == n) ? NEO_M8N_OK : NEO_M8N_ERR_IO;
}

/* Pumps the UART until the matching ACK/NAK turns up. NMEA arriving meanwhile
 * is parsed as usual rather than discarded. */
static neo_m8n_err_t ubx_wait_ack(uint8_t msg_class, uint8_t msg_id, uint32_t timeout_ms)
{
    uint32_t start = neo_m8n_port_millis();

    s_gps.ubx.ack_pending = false;

    for (;;) {
        uint32_t spent = elapsed_since(start);
        uint32_t left;
        uint8_t buf[NEO_M8N_CHUNK];
        int n;

        if (spent >= timeout_ms) return NEO_M8N_ERR_TIMEOUT;
        left = timeout_ms - spent;

        n = neo_m8n_port_read(buf, sizeof(buf), left);
        if (n < 0) return (neo_m8n_err_t)n;
        if (n > 0) neo_m8n_feed(buf, (size_t)n);

        if (s_gps.ubx.ack_pending &&
            s_gps.ubx.ack_class == msg_class &&
            s_gps.ubx.ack_id == msg_id) {
            s_gps.ubx.ack_pending = false;
            return s_gps.ubx.ack_ok ? NEO_M8N_OK : NEO_M8N_ERR_NACK;
        }
        /* An ACK for some other message means the receiver is alive but we are
         * not done; keep waiting for ours. */
        s_gps.ubx.ack_pending = false;
    }
}

neo_m8n_err_t neo_m8n_send_ubx(uint8_t msg_class, uint8_t msg_id,
                               const uint8_t *payload, uint16_t len,
                               bool wait_ack, uint32_t timeout_ms)
{
    neo_m8n_err_t err;

    if (!s_gps.initialized) return NEO_M8N_ERR_STATE;

    err = ubx_write(msg_class, msg_id, payload, len);
    if (err != NEO_M8N_OK) return err;
    if (!wait_ack) return NEO_M8N_OK;

    return ubx_wait_ack(msg_class, msg_id, timeout_ms ? timeout_ms : UBX_ACK_TIMEOUT_MS);
}

/* ------------------------------------------------------------------ */
/* Sentence handling                                                  */
/* ------------------------------------------------------------------ */

#define KNOTS_TO_MPS 0.514444f

static void merge_time(nmea_time_t *dst, const nmea_time_t *src)
{
    /* Time and date arrive in different sentences, so keep whichever half is
     * present and never let a time-only sentence clear a known date. */
    if (src->time_valid) {
        dst->hours = src->hours;
        dst->minutes = src->minutes;
        dst->seconds = src->seconds;
        dst->millis = src->millis;
        dst->time_valid = true;
    }
    if (src->date_valid) {
        dst->year = src->year;
        dst->month = src->month;
        dst->day = src->day;
        dst->date_valid = true;
    }
}

/* Returns true when the sentence carried a position update. */
static bool handle_sentence(neo_m8n_t *g, const char *line)
{
    switch (nmea_sentence_id(line)) {
    case NMEA_GGA: {
        nmea_gga_t gga;
        if (!nmea_parse_gga(line, &gga)) return false;
        merge_time(&g->fix.time, &gga.time);
        g->fix.fix_quality = gga.fix_quality;
        g->fix.satellites_used = gga.satellites;
        g->fix.hdop = gga.hdop;
        if (gga.position_valid) {
            g->fix.latitude = gga.latitude;
            g->fix.longitude = gga.longitude;
            g->fix.altitude_m = gga.altitude_m;
            g->fix.geoid_sep_m = gga.geoid_sep_m;
        }
        /* Validity follows the most recent sentence that carried a position.
         * Taking it from GGA as well as RMC means a receiver trimmed down to
         * GGA only still reports a usable fix. */
        g->fix.valid = gga.position_valid;
        return true;
    }
    case NMEA_RMC: {
        nmea_rmc_t rmc;
        if (!nmea_parse_rmc(line, &rmc)) return false;
        merge_time(&g->fix.time, &rmc.time);
        g->fix.valid = rmc.position_valid;
        if (rmc.position_valid) {
            g->fix.latitude = rmc.latitude;
            g->fix.longitude = rmc.longitude;
        }
        g->fix.speed_knots = rmc.speed_knots;
        g->fix.speed_mps = rmc.speed_knots * KNOTS_TO_MPS;
        g->fix.course_deg = rmc.course_deg;
        return true;
    }
    case NMEA_GSA: {
        nmea_gsa_t gsa;
        if (!nmea_parse_gsa(line, &gsa)) return false;
        g->fix.fix_type = gsa.fix_type;
        g->fix.pdop = gsa.pdop;
        g->fix.hdop = gsa.hdop;
        g->fix.vdop = gsa.vdop;
        return false;
    }
    case NMEA_GSV: {
        nmea_gsv_t gsv;
        if (!nmea_parse_gsv(line, &gsv)) return false;
        /* Only the first sentence of a group carries the running total. */
        if (gsv.msg_num == 1) g->fix.satellites_visible = gsv.total_sats;
        return false;
    }
    case NMEA_VTG: {
        nmea_vtg_t vtg;
        if (!nmea_parse_vtg(line, &vtg)) return false;
        g->fix.speed_knots = vtg.speed_knots;
        g->fix.speed_mps = vtg.speed_knots * KNOTS_TO_MPS;
        g->fix.course_deg = vtg.course_true_deg;
        return false;
    }
    default:
        return false;
    }
}

void neo_m8n_feed(const uint8_t *data, size_t len)
{
    neo_m8n_t *g = &s_gps;

    if (!data) return;

    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];

        g->stats.bytes_rx++;

        /* Both framers see every byte: the two protocols share the wire. */
        ubx_push(g, data[i]);

        if (nmea_stream_push(&g->nmea, c, g->line, sizeof(g->line))) {
            if (g->cb) g->cb(g->line, g->cb_user);
            if (handle_sentence(g, g->line)) {
                g->have_position = true;
                g->pos_count++;
                g->fix_ms = neo_m8n_port_millis();
            }
        }
    }

    g->stats.sentences = g->nmea.sentences;
    g->stats.checksum_errors = g->nmea.checksum_errors;
    g->stats.overruns = g->nmea.overruns;
}

/* ------------------------------------------------------------------ */
/* Configuration                                                      */
/* ------------------------------------------------------------------ */

neo_m8n_err_t neo_m8n_set_nav_rate(uint16_t measure_ms)
{
    uint8_t payload[6];

    if (!s_gps.initialized) return NEO_M8N_ERR_STATE;
    if (measure_ms < 50) return NEO_M8N_ERR_INVALID_ARG;

    put_u16(&payload[0], measure_ms);   /* measRate, ms between solutions */
    put_u16(&payload[2], 1);            /* navRate, fixed at 1 on the M8 */
    put_u16(&payload[4], 1);            /* timeRef: 1 = GPS time */

    return neo_m8n_send_ubx(UBX_CLASS_CFG, UBX_CFG_RATE, payload, sizeof(payload),
                            true, UBX_ACK_TIMEOUT_MS);
}

neo_m8n_err_t neo_m8n_set_nmea_rate(neo_m8n_nmea_msg_t msg, uint8_t rate)
{
    uint8_t payload[3];

    if (!s_gps.initialized) return NEO_M8N_ERR_STATE;

    /* Short form of CFG-MSG: applies to the port the command arrived on. */
    payload[0] = UBX_CLASS_NMEA;
    payload[1] = (uint8_t)msg;
    payload[2] = rate;

    return neo_m8n_send_ubx(UBX_CLASS_CFG, UBX_CFG_MSG, payload, sizeof(payload),
                            true, UBX_ACK_TIMEOUT_MS);
}

neo_m8n_err_t neo_m8n_set_baud(uint32_t baud)
{
    uint8_t payload[20];
    neo_m8n_err_t err;
    uint32_t start;

    if (!s_gps.initialized) return NEO_M8N_ERR_STATE;
    if (baud < 4800 || baud > 921600) return NEO_M8N_ERR_INVALID_ARG;
    if (baud == s_gps.cfg.baud) return NEO_M8N_OK;

    memset(payload, 0, sizeof(payload));
    payload[0] = UBX_PORT_UART1;
    put_u32(&payload[4], UBX_PRT_MODE_8N1);
    put_u32(&payload[8], baud);
    put_u16(&payload[12], UBX_PROTO_UBX | UBX_PROTO_NMEA);   /* inProtoMask */
    put_u16(&payload[14], UBX_PROTO_UBX | UBX_PROTO_NMEA);   /* outProtoMask */

    /* No ACK wait: the receiver answers at the old rate and switches straight
     * after, so the reply is a coin flip. Send, let it drain, follow it over. */
    err = ubx_write(UBX_CLASS_CFG, UBX_CFG_PRT, payload, sizeof(payload));
    if (err != NEO_M8N_OK) return err;

    neo_m8n_port_delay_ms(100);

    err = neo_m8n_port_set_baud(baud);
    if (err != NEO_M8N_OK) return err;

    neo_m8n_port_flush_input();
    nmea_stream_init(&s_gps.nmea);
    ubx_rx_reset(&s_gps.ubx);
    s_gps.cfg.baud = baud;

    /* Confirm by waiting for traffic that actually frames at the new rate. A
     * wrong divisor yields bytes, but none of them pass a checksum. */
    start = neo_m8n_port_millis();
    while (elapsed_since(start) < 2000) {
        uint8_t buf[NEO_M8N_CHUNK];
        uint32_t before = s_gps.nmea.sentences;
        int n = neo_m8n_port_read(buf, sizeof(buf), 250);

        if (n < 0) return (neo_m8n_err_t)n;
        if (n > 0) neo_m8n_feed(buf, (size_t)n);
        if (s_gps.nmea.sentences > before) return NEO_M8N_OK;
    }
    NEO_M8N_LOGW("no valid sentences after switching to %lu baud", (unsigned long)baud);
    return NEO_M8N_ERR_TIMEOUT;
}

neo_m8n_err_t neo_m8n_save_config(void)
{
    uint8_t payload[13];

    if (!s_gps.initialized) return NEO_M8N_ERR_STATE;

    memset(payload, 0, sizeof(payload));
    put_u32(&payload[0], 0x00000000u);   /* clearMask: clear nothing */
    put_u32(&payload[4], 0x0000FFFFu);   /* saveMask:  save every section */
    put_u32(&payload[8], 0x00000000u);   /* loadMask:  load nothing */
    payload[12] = UBX_CFG_DEV_BBR | UBX_CFG_DEV_FLASH;

    /* Writing flash takes a moment, so the ACK is slower than usual. */
    return neo_m8n_send_ubx(UBX_CLASS_CFG, UBX_CFG_CFG, payload, sizeof(payload),
                            true, 3000);
}

/*
 * Applies cfg->* to the receiver. Individual steps are allowed to fail: a
 * module that ignores UBX still streams usable NMEA, and refusing to boot a
 * tracker over a declined configuration message would be the wrong trade.
 */
static void apply_config(const neo_m8n_config_t *cfg)
{
    neo_m8n_err_t err;

    if (cfg->target_baud && cfg->target_baud != s_gps.cfg.baud) {
        err = neo_m8n_set_baud(cfg->target_baud);
        if (err != NEO_M8N_OK) {
            NEO_M8N_LOGW("baud switch to %lu failed (%s), staying at %lu",
                         (unsigned long)cfg->target_baud, neo_m8n_err_str(err),
                         (unsigned long)s_gps.cfg.baud);
        }
    }

    if (cfg->trim_nmea) {
        /* GGA gives altitude and satellite count, RMC gives date, speed and
         * course, GSA gives fix type and DOP. GLL and VTG are redundant with
         * those, and GSV is four extra sentences per constellation. */
        static const struct { neo_m8n_nmea_msg_t msg; uint8_t rate; } want[] = {
            { NEO_M8N_NMEA_GGA, 1 },
            { NEO_M8N_NMEA_RMC, 1 },
            { NEO_M8N_NMEA_GSA, 1 },
            { NEO_M8N_NMEA_GLL, 0 },
            { NEO_M8N_NMEA_GSV, 0 },
            { NEO_M8N_NMEA_VTG, 0 },
        };
        for (size_t i = 0; i < sizeof(want) / sizeof(want[0]); i++) {
            err = neo_m8n_set_nmea_rate(want[i].msg, want[i].rate);
            if (err != NEO_M8N_OK) {
                NEO_M8N_LOGW("CFG-MSG 0x%02X failed: %s",
                             (unsigned)want[i].msg, neo_m8n_err_str(err));
            }
        }
    }

    if (cfg->nav_rate_ms) {
        err = neo_m8n_set_nav_rate(cfg->nav_rate_ms);
        if (err != NEO_M8N_OK) {
            NEO_M8N_LOGW("CFG-RATE %u ms failed: %s",
                         (unsigned)cfg->nav_rate_ms, neo_m8n_err_str(err));
        }
    }

    if (cfg->save_config) {
        err = neo_m8n_save_config();
        if (err != NEO_M8N_OK) NEO_M8N_LOGW("CFG-CFG save failed: %s", neo_m8n_err_str(err));
    }
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                          */
/* ------------------------------------------------------------------ */

neo_m8n_err_t neo_m8n_init(const neo_m8n_config_t *cfg)
{
    neo_m8n_err_t err;

    if (!cfg) return NEO_M8N_ERR_INVALID_ARG;
    if (s_gps.initialized) return NEO_M8N_ERR_STATE;
    if (cfg->rx_pin < 0) return NEO_M8N_ERR_INVALID_ARG;   /* receive-only still needs RX */
    if (cfg->baud == 0) return NEO_M8N_ERR_INVALID_ARG;
    /* Configuring the receiver means talking to it. */
    if (cfg->auto_config && cfg->tx_pin < 0) return NEO_M8N_ERR_INVALID_ARG;

    memset(&s_gps, 0, sizeof(s_gps));
    s_gps.cfg = *cfg;
    nmea_stream_init(&s_gps.nmea);
    ubx_rx_reset(&s_gps.ubx);

    err = neo_m8n_port_init(cfg);
    if (err != NEO_M8N_OK) {
        NEO_M8N_LOGE("uart init failed: %s", neo_m8n_err_str(err));
        return err;
    }
    s_gps.initialized = true;

    NEO_M8N_LOGI("uart%d up at %lu baud (tx=%d rx=%d)",
                 cfg->uart_id, (unsigned long)cfg->baud, cfg->tx_pin, cfg->rx_pin);

    if (cfg->auto_config) apply_config(cfg);

    return NEO_M8N_OK;
}

neo_m8n_err_t neo_m8n_deinit(void)
{
    if (!s_gps.initialized) return NEO_M8N_ERR_STATE;
    neo_m8n_port_deinit();
    memset(&s_gps, 0, sizeof(s_gps));
    return NEO_M8N_OK;
}

bool neo_m8n_is_initialized(void)
{
    return s_gps.initialized;
}

/* ------------------------------------------------------------------ */
/* Reading                                                            */
/* ------------------------------------------------------------------ */

static void snapshot(neo_m8n_fix_t *out)
{
    *out = s_gps.fix;
    out->age_ms = s_gps.have_position ? elapsed_since(s_gps.fix_ms) : UINT32_MAX;
}

int neo_m8n_poll(void)
{
    uint8_t buf[NEO_M8N_CHUNK];
    uint32_t before;
    int n;

    if (!s_gps.initialized) return NEO_M8N_ERR_STATE;

    before = s_gps.nmea.sentences;
    n = neo_m8n_port_read(buf, sizeof(buf), 0);
    if (n < 0) return n;
    if (n > 0) neo_m8n_feed(buf, (size_t)n);

    return (int)(s_gps.nmea.sentences - before);
}

neo_m8n_err_t neo_m8n_read(neo_m8n_fix_t *out, uint32_t timeout_ms)
{
    uint32_t start;

    if (!s_gps.initialized) return NEO_M8N_ERR_STATE;

    start = neo_m8n_port_millis();
    for (;;) {
        uint32_t spent = elapsed_since(start);
        uint8_t buf[NEO_M8N_CHUNK];
        uint32_t before = s_gps.pos_count;
        int n;

        n = neo_m8n_port_read(buf, sizeof(buf),
                              spent < timeout_ms ? timeout_ms - spent : 0);
        if (n < 0) return (neo_m8n_err_t)n;
        if (n > 0) neo_m8n_feed(buf, (size_t)n);

        /* Count sentences rather than compare timestamps: at 10 Hz a GGA and
         * its RMC can land inside the same millisecond. */
        if (s_gps.pos_count != before) {
            if (out) snapshot(out);
            return NEO_M8N_OK;
        }

        /* Checked after the read so that a zero timeout still polls once, and
         * before looping so that it always terminates. */
        if (elapsed_since(start) >= timeout_ms) return NEO_M8N_ERR_TIMEOUT;
    }
}

neo_m8n_err_t neo_m8n_wait_for_fix(neo_m8n_fix_t *out, uint32_t timeout_ms)
{
    uint32_t start;

    if (!s_gps.initialized) return NEO_M8N_ERR_STATE;

    start = neo_m8n_port_millis();
    for (;;) {
        uint32_t spent = elapsed_since(start);
        neo_m8n_fix_t fix;
        neo_m8n_err_t err;

        if (spent >= timeout_ms) return NEO_M8N_ERR_TIMEOUT;

        err = neo_m8n_read(&fix, timeout_ms - spent);
        if (err != NEO_M8N_OK) return err;
        if (fix.valid) {
            if (out) *out = fix;
            return NEO_M8N_OK;
        }
    }
}

bool neo_m8n_get_fix(neo_m8n_fix_t *out)
{
    if (!out || !s_gps.have_position) return false;
    snapshot(out);
    return true;
}

uint32_t neo_m8n_fix_age_ms(void)
{
    return s_gps.have_position ? elapsed_since(s_gps.fix_ms) : UINT32_MAX;
}

void neo_m8n_set_sentence_cb(neo_m8n_sentence_cb_t cb, void *user)
{
    s_gps.cb = cb;
    s_gps.cb_user = user;
}

void neo_m8n_get_stats(neo_m8n_stats_t *out)
{
    if (out) *out = s_gps.stats;
}

void neo_m8n_reset_stats(void)
{
    memset(&s_gps.stats, 0, sizeof(s_gps.stats));
    s_gps.nmea.sentences = 0;
    s_gps.nmea.checksum_errors = 0;
    s_gps.nmea.overruns = 0;
}
