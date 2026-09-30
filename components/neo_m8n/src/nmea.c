/*
 * nmea.c - NMEA 0183 sentence parser. See include/nmea.h.
 *
 * Numbers are parsed by hand rather than with strtod/atoi: those pull in locale
 * handling (a decimal comma would silently corrupt every coordinate) and, on
 * some newlib-nano builds, are larger than this whole file.
 */

#include <string.h>

#include "nmea.h"

/* ------------------------------------------------------------------ */
/* Scalar parsing helpers                                             */
/* ------------------------------------------------------------------ */

static bool is_digit(char c)
{
    return c >= '0' && c <= '9';
}

static int hex_val(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* Unsigned decimal, no sign and no dot. Rejects an empty or non-numeric field. */
static bool parse_uint(const char *p, size_t len, uint32_t *out)
{
    uint32_t v = 0;

    if (len == 0) return false;
    for (size_t i = 0; i < len; i++) {
        if (!is_digit(p[i])) return false;
        v = v * 10u + (uint32_t)(p[i] - '0');
    }
    *out = v;
    return true;
}

/* Signed fixed-point decimal: "-12.345", ".5" and "7" all parse. */
static bool parse_decimal(const char *p, size_t len, double *out)
{
    size_t i = 0;
    double sign = 1.0;
    double v = 0.0;
    bool   digits = false;

    if (len == 0) return false;
    if (p[0] == '-') { sign = -1.0; i = 1; }
    else if (p[0] == '+') { i = 1; }

    for (; i < len && p[i] != '.'; i++) {
        if (!is_digit(p[i])) return false;
        v = v * 10.0 + (double)(p[i] - '0');
        digits = true;
    }
    if (i < len && p[i] == '.') {
        double scale = 0.1;
        for (i++; i < len; i++) {
            if (!is_digit(p[i])) return false;
            v += (double)(p[i] - '0') * scale;
            scale *= 0.1;
            digits = true;
        }
    }
    if (!digits) return false;
    *out = sign * v;
    return true;
}

static bool parse_float_field(const char *sentence, int index, float *out)
{
    const char *f;
    size_t len;
    double v;

    if (!nmea_field(sentence, index, &f, &len)) return false;
    if (!parse_decimal(f, len, &v)) return false;
    *out = (float)v;
    return true;
}

static bool parse_uint_field(const char *sentence, int index, uint32_t *out)
{
    const char *f;
    size_t len;

    if (!nmea_field(sentence, index, &f, &len)) return false;
    return parse_uint(f, len, out);
}

/*
 * DDMM.MMMM (latitude) or DDDMM.MMMM (longitude) plus a hemisphere field.
 * The degrees field is variable width, so the split is found from the decimal
 * point: minutes are always the two digits before it.
 */
static bool parse_coord(const char *p, size_t len,
                        const char *hemi, size_t hemi_len,
                        double *out)
{
    size_t dot = len;
    size_t deg_len;
    uint32_t deg;
    double minutes;
    double value;

    if (len < 3 || hemi_len != 1) return false;

    for (size_t i = 0; i < len; i++) {
        if (p[i] == '.') { dot = i; break; }
    }
    if (dot < 2) return false;

    deg_len = dot - 2;
    if (deg_len == 0 || deg_len > 3) return false;

    if (!parse_uint(p, deg_len, &deg)) return false;
    if (!parse_decimal(p + deg_len, len - deg_len, &minutes)) return false;
    if (minutes >= 60.0) return false;

    value = (double)deg + minutes / 60.0;

    switch (hemi[0]) {
    case 'N': case 'E': break;
    case 'S': case 'W': value = -value; break;
    default: return false;
    }
    if (value > 180.0 || value < -180.0) return false;

    *out = value;
    return true;
}

/* Reads the lat/lon/hemisphere quad starting at field `first`. */
static bool parse_position(const char *sentence, int first,
                           double *lat, double *lon)
{
    const char *lat_f, *ns_f, *lon_f, *ew_f;
    size_t lat_len, ns_len, lon_len, ew_len;

    if (!nmea_field(sentence, first + 0, &lat_f, &lat_len) ||
        !nmea_field(sentence, first + 1, &ns_f,  &ns_len)  ||
        !nmea_field(sentence, first + 2, &lon_f, &lon_len) ||
        !nmea_field(sentence, first + 3, &ew_f,  &ew_len)) {
        return false;
    }
    if (!parse_coord(lat_f, lat_len, ns_f, ns_len, lat)) return false;
    if (!parse_coord(lon_f, lon_len, ew_f, ew_len, lon)) return false;
    return true;
}

/* hhmmss or hhmmss.sss */
static bool parse_time(const char *p, size_t len, nmea_time_t *t)
{
    uint32_t hh, mm, ss;

    if (len < 6) return false;
    if (!parse_uint(p + 0, 2, &hh) ||
        !parse_uint(p + 2, 2, &mm) ||
        !parse_uint(p + 4, 2, &ss)) {
        return false;
    }
    /* Second 60 is legal: it is how a leap second is reported. */
    if (hh > 23 || mm > 59 || ss > 60) return false;

    t->hours = (uint8_t)hh;
    t->minutes = (uint8_t)mm;
    t->seconds = (uint8_t)ss;
    t->millis = 0;

    if (len > 6 && p[6] == '.') {
        double frac;
        if (parse_decimal(p + 6, len - 6, &frac)) {
            t->millis = (uint16_t)(frac * 1000.0 + 0.5);
        }
    }
    t->time_valid = true;
    return true;
}

/*
 * ddmmyy. NMEA carries no century. Pivot at 80, the same window gpsd and
 * minmea use: 80-99 are 1980-1999, 00-79 are 2000-2079. A live NEO-M8N only
 * ever reports the upper half, but logs replayed from older receivers parse
 * correctly too.
 */
static bool parse_date(const char *p, size_t len, nmea_time_t *t)
{
    uint32_t dd, mm, yy;

    if (len < 6) return false;
    if (!parse_uint(p + 0, 2, &dd) ||
        !parse_uint(p + 2, 2, &mm) ||
        !parse_uint(p + 4, 2, &yy)) {
        return false;
    }
    if (dd < 1 || dd > 31 || mm < 1 || mm > 12) return false;

    t->day = (uint8_t)dd;
    t->month = (uint8_t)mm;
    t->year = (uint16_t)((yy >= 80u ? 1900u : 2000u) + yy);
    t->date_valid = true;
    return true;
}

static bool parse_time_field(const char *sentence, int index, nmea_time_t *t)
{
    const char *f;
    size_t len;

    if (!nmea_field(sentence, index, &f, &len)) return false;
    return parse_time(f, len, t);
}

/* ------------------------------------------------------------------ */
/* Sentence framing                                                   */
/* ------------------------------------------------------------------ */

void nmea_stream_init(nmea_stream_t *s)
{
    if (s) memset(s, 0, sizeof(*s));
}

bool nmea_stream_push(nmea_stream_t *s, char c, char *out, size_t out_size)
{
    if (!s || !out || out_size == 0) return false;

    if (c == '$') {
        /* A '$' always restarts framing: a truncated sentence must not swallow
         * the one after it. This also resynchronises after UBX binary traffic
         * that happens to contain a 0x24 byte. */
        s->buf[0] = c;
        s->len = 1;
        s->in_sentence = true;
        return false;
    }
    if (!s->in_sentence) return false;

    if (c == '\r' || c == '\n') {
        bool ok = false;
        s->in_sentence = false;
        s->buf[s->len] = '\0';
        if (nmea_validate(s->buf)) {
            if (s->len + 1 <= out_size) {
                memcpy(out, s->buf, s->len + 1);
                s->sentences++;
                ok = true;
            } else {
                s->overruns++;
            }
        } else {
            s->checksum_errors++;
        }
        s->len = 0;
        return ok;
    }

    /* Keep one byte for the NUL. */
    if (s->len + 1 >= sizeof(s->buf)) {
        s->in_sentence = false;
        s->len = 0;
        s->overruns++;
        return false;
    }
    s->buf[s->len++] = c;
    return false;
}

bool nmea_validate(const char *sentence)
{
    uint8_t sum = 0;
    size_t i;
    int hi, lo;

    if (!sentence || sentence[0] != '$') return false;

    for (i = 1; sentence[i] != '\0' && sentence[i] != '*'; i++) {
        if (sentence[i] == '\r' || sentence[i] == '\n') return false;
        sum ^= (uint8_t)sentence[i];
    }
    if (sentence[i] != '*') return false;

    hi = hex_val(sentence[i + 1]);
    if (hi < 0) return false;
    lo = hex_val(sentence[i + 2]);
    if (lo < 0) return false;

    return sum == (uint8_t)((hi << 4) | lo);
}

nmea_sentence_id_t nmea_sentence_id(const char *sentence)
{
    const char *type;

    /* "$ttsss": two-character talker ID then the three-character type. The
     * talker varies with the constellation in use (GP, GL, GA, GB, GN), so
     * only the type is matched. */
    if (!sentence || strlen(sentence) < 6 || sentence[0] != '$') return NMEA_UNKNOWN;
    type = sentence + 3;

    if (!memcmp(type, "GGA", 3)) return NMEA_GGA;
    if (!memcmp(type, "RMC", 3)) return NMEA_RMC;
    if (!memcmp(type, "GSA", 3)) return NMEA_GSA;
    if (!memcmp(type, "GSV", 3)) return NMEA_GSV;
    if (!memcmp(type, "VTG", 3)) return NMEA_VTG;
    if (!memcmp(type, "GLL", 3)) return NMEA_GLL;
    return NMEA_UNKNOWN;
}

bool nmea_field(const char *sentence, int index, const char **field, size_t *len)
{
    const char *p = sentence;
    const char *e;

    if (!sentence || index < 0 || !field || !len) return false;

    for (int i = 0; i < index; i++) {
        p = strchr(p, ',');
        if (!p) return false;
        p++;
    }
    for (e = p; *e && *e != ',' && *e != '*'; e++) {
        /* scan to the field terminator */
    }
    *field = p;
    *len = (size_t)(e - p);
    return true;
}

/* ------------------------------------------------------------------ */
/* Sentence parsers                                                   */
/* ------------------------------------------------------------------ */

/*
 * $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
 *        1      2        3 4         5 6 7  8   9     10 11
 */
bool nmea_parse_gga(const char *sentence, nmea_gga_t *out)
{
    uint32_t u;

    if (!out || nmea_sentence_id(sentence) != NMEA_GGA) return false;
    memset(out, 0, sizeof(*out));

    parse_time_field(sentence, 1, &out->time);

    if (!parse_uint_field(sentence, 6, &u) || u > 8) return false;
    out->fix_quality = (uint8_t)u;

    /* Quality 0 means no fix; the position fields are then empty or stale. */
    out->position_valid = (out->fix_quality > 0) &&
                          parse_position(sentence, 2, &out->latitude, &out->longitude);

    if (parse_uint_field(sentence, 7, &u)) out->satellites = (uint8_t)(u > 255 ? 255 : u);
    parse_float_field(sentence, 8, &out->hdop);
    parse_float_field(sentence, 9, &out->altitude_m);
    parse_float_field(sentence, 11, &out->geoid_sep_m);
    return true;
}

/*
 * $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
 *        1      2 3        4 5         6 7     8     9      10    11
 */
bool nmea_parse_rmc(const char *sentence, nmea_rmc_t *out)
{
    const char *f;
    size_t len;

    if (!out || nmea_sentence_id(sentence) != NMEA_RMC) return false;
    memset(out, 0, sizeof(*out));

    parse_time_field(sentence, 1, &out->time);

    if (!nmea_field(sentence, 2, &f, &len) || len != 1) return false;
    out->valid = (f[0] == 'A');

    out->position_valid = out->valid &&
                          parse_position(sentence, 3, &out->latitude, &out->longitude);

    parse_float_field(sentence, 7, &out->speed_knots);
    parse_float_field(sentence, 8, &out->course_deg);

    if (nmea_field(sentence, 9, &f, &len)) parse_date(f, len, &out->time);

    /* Magnetic variation is signed by its own hemisphere field. */
    if (parse_float_field(sentence, 10, &out->variation_deg) &&
        nmea_field(sentence, 11, &f, &len) && len == 1) {
        if (f[0] == 'W') out->variation_deg = -out->variation_deg;
        out->variation_valid = (f[0] == 'W' || f[0] == 'E');
    }
    return true;
}

/*
 * $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39
 *        1 2 3.............14      15  16  17
 */
bool nmea_parse_gsa(const char *sentence, nmea_gsa_t *out)
{
    const char *f;
    size_t len;
    uint32_t u;

    if (!out || nmea_sentence_id(sentence) != NMEA_GSA) return false;
    memset(out, 0, sizeof(*out));

    if (nmea_field(sentence, 1, &f, &len) && len == 1) out->mode = f[0];

    if (!parse_uint_field(sentence, 2, &u) || u < 1 || u > 3) return false;
    out->fix_type = (uint8_t)u;

    /* Unused channels come through as empty fields, so gaps are expected. */
    for (int i = 0; i < NMEA_GSA_MAX_SV; i++) {
        if (parse_uint_field(sentence, 3 + i, &u) && u > 0 && u <= 255) {
            out->sv[out->sv_count++] = (uint8_t)u;
        }
    }

    parse_float_field(sentence, 15, &out->pdop);
    parse_float_field(sentence, 16, &out->hdop);
    parse_float_field(sentence, 17, &out->vdop);
    return true;
}

/*
 * $GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74
 *        1 2 3  4  5  6   7  ...
 */
bool nmea_parse_gsv(const char *sentence, nmea_gsv_t *out)
{
    uint32_t u;

    if (!out || nmea_sentence_id(sentence) != NMEA_GSV) return false;
    memset(out, 0, sizeof(*out));

    if (!parse_uint_field(sentence, 1, &u) || u == 0) return false;
    out->total_msgs = (uint8_t)u;
    if (!parse_uint_field(sentence, 2, &u) || u == 0) return false;
    out->msg_num = (uint8_t)u;
    if (!parse_uint_field(sentence, 3, &u)) return false;
    out->total_sats = (uint8_t)(u > 255 ? 255 : u);

    /* The last sentence of a group is short when the count is not a multiple
     * of four, so stop at the first block with no PRN. */
    for (int i = 0; i < NMEA_GSV_PER_MSG; i++) {
        int base = 4 + i * 4;
        nmea_sat_t *s = &out->sats[out->count];

        if (!parse_uint_field(sentence, base, &u) || u == 0) break;
        s->prn = (uint8_t)(u > 255 ? 255 : u);

        if (parse_uint_field(sentence, base + 1, &u) && u <= 90) s->elevation_deg = (uint8_t)u;
        if (parse_uint_field(sentence, base + 2, &u) && u < 360) s->azimuth_deg = (uint16_t)u;
        /* An empty SNR field means the satellite is visible but not tracked. */
        if (parse_uint_field(sentence, base + 3, &u) && u <= 99) {
            s->snr_dbhz = (uint8_t)u;
            s->snr_valid = true;
        }
        out->count++;
    }
    return true;
}

/*
 * $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
 *        1     2 3     4 5     6 7     8
 */
bool nmea_parse_vtg(const char *sentence, nmea_vtg_t *out)
{
    if (!out || nmea_sentence_id(sentence) != NMEA_VTG) return false;
    memset(out, 0, sizeof(*out));

    parse_float_field(sentence, 1, &out->course_true_deg);
    out->course_mag_valid = parse_float_field(sentence, 3, &out->course_mag_deg);
    parse_float_field(sentence, 5, &out->speed_knots);
    parse_float_field(sentence, 7, &out->speed_kph);
    return true;
}

/*
 * $GPGLL,4916.45,N,12311.12,W,225444,A*31
 *        1       2 3        4 5      6
 */
bool nmea_parse_gll(const char *sentence, nmea_gll_t *out)
{
    const char *f;
    size_t len;

    if (!out || nmea_sentence_id(sentence) != NMEA_GLL) return false;
    memset(out, 0, sizeof(*out));

    if (!nmea_field(sentence, 6, &f, &len) || len != 1) return false;
    out->valid = (f[0] == 'A');

    out->position_valid = out->valid &&
                          parse_position(sentence, 1, &out->latitude, &out->longitude);
    parse_time_field(sentence, 5, &out->time);
    return true;
}
