/*
 * nmea.h - NMEA 0183 sentence parser.
 *
 * Pure C99: no allocation, no I/O, no platform headers. Usable on its own if
 * you have a byte source other than the NEO-M8N driver in this component.
 *
 * Coordinates come out of the parser as decimal degrees (north and east
 * positive), not the DDMM.MMMM the wire format uses.
 */

#ifndef NMEA_H
#define NMEA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* NMEA 0183 caps a sentence at 82 bytes, but u-blox GSV and multi-GNSS talkers
 * routinely run longer, so leave headroom. */
#ifndef NMEA_MAX_SENTENCE
#define NMEA_MAX_SENTENCE 128
#endif

/* GSA reports at most 12 SVs; GSV packs 4 per sentence. */
#define NMEA_GSA_MAX_SV  12
#define NMEA_GSV_PER_MSG 4

typedef enum {
    NMEA_UNKNOWN = 0,
    NMEA_GGA,   /* fix data: position, altitude, satellites in use */
    NMEA_RMC,   /* recommended minimum: position, speed, course, date */
    NMEA_GSA,   /* fix mode and dilution of precision */
    NMEA_GSV,   /* satellites in view */
    NMEA_VTG,   /* course and ground speed */
    NMEA_GLL,   /* position and time only */
} nmea_sentence_id_t;

/* Date and time are filled in from different sentences: RMC carries both, GGA
 * and GLL carry time only. The two valid flags say which half is trustworthy. */
typedef struct {
    uint16_t year;          /* full year, e.g. 2026 */
    uint8_t  month;         /* 1-12 */
    uint8_t  day;           /* 1-31 */
    uint8_t  hours;         /* 0-23 UTC */
    uint8_t  minutes;       /* 0-59 */
    uint8_t  seconds;       /* 0-60, 60 during a leap second */
    uint16_t millis;        /* 0-999 */
    bool     time_valid;
    bool     date_valid;
} nmea_time_t;

typedef struct {
    nmea_time_t time;
    double      latitude;       /* decimal degrees, north positive */
    double      longitude;      /* decimal degrees, east positive */
    bool        position_valid; /* false when the receiver has no fix yet */
    uint8_t     fix_quality;    /* 0 none, 1 GPS, 2 DGPS, 4 RTK fixed, 5 RTK float, 6 estimated */
    uint8_t     satellites;     /* satellites used in the solution */
    float       hdop;
    float       altitude_m;     /* height above mean sea level */
    float       geoid_sep_m;    /* MSL minus WGS84 ellipsoid */
} nmea_gga_t;

typedef struct {
    nmea_time_t time;
    bool        valid;          /* status field 'A' (active) rather than 'V' (void) */
    double      latitude;
    double      longitude;
    bool        position_valid;
    float       speed_knots;
    float       course_deg;     /* true course over ground */
    float       variation_deg;  /* magnetic variation, east positive */
    bool        variation_valid;
} nmea_rmc_t;

typedef struct {
    char    mode;               /* 'M' manual 2D/3D, 'A' automatic */
    uint8_t fix_type;           /* 1 no fix, 2 = 2D, 3 = 3D */
    uint8_t sv[NMEA_GSA_MAX_SV];
    uint8_t sv_count;
    float   pdop;
    float   hdop;
    float   vdop;
} nmea_gsa_t;

typedef struct {
    uint8_t prn;
    uint8_t elevation_deg;      /* 0-90 */
    uint16_t azimuth_deg;       /* 0-359, true */
    uint8_t snr_dbhz;           /* 0-99, 0 when not tracked */
    bool    snr_valid;
} nmea_sat_t;

typedef struct {
    uint8_t    total_msgs;      /* sentences in this GSV group */
    uint8_t    msg_num;         /* 1-based index of this sentence */
    uint8_t    total_sats;      /* satellites in view for this constellation */
    uint8_t    count;           /* entries populated in sats[] */
    nmea_sat_t sats[NMEA_GSV_PER_MSG];
} nmea_gsv_t;

typedef struct {
    float course_true_deg;
    float course_mag_deg;
    bool  course_mag_valid;
    float speed_knots;
    float speed_kph;
} nmea_vtg_t;

typedef struct {
    nmea_time_t time;
    double      latitude;
    double      longitude;
    bool        position_valid;
    bool        valid;          /* status field 'A' */
} nmea_gll_t;

/*
 * Streaming collector. Feed it bytes as they arrive; it hands back whole
 * sentences once the checksum checks out.
 */
typedef struct {
    char     buf[NMEA_MAX_SENTENCE];
    size_t   len;
    bool     in_sentence;
    uint32_t sentences;         /* checksum-valid sentences emitted */
    uint32_t checksum_errors;
    uint32_t overruns;          /* sentences longer than NMEA_MAX_SENTENCE */
} nmea_stream_t;

void nmea_stream_init(nmea_stream_t *s);

/*
 * Push one byte. Returns true when a complete, checksum-valid sentence is
 * available; it is then copied into out (NUL-terminated, '$' included, line
 * ending stripped). Returns false for every other byte, including the last
 * byte of a sentence whose checksum failed.
 */
bool nmea_stream_push(nmea_stream_t *s, char c, char *out, size_t out_size);

/* XOR of everything between '$' and '*', compared against the two hex digits
 * that follow the '*'. A sentence with no '*' fails. */
bool nmea_validate(const char *sentence);

/* Talker ID is ignored, so "$GPGGA" and "$GNGGA" both return NMEA_GGA. */
nmea_sentence_id_t nmea_sentence_id(const char *sentence);

/*
 * Field accessor. Index 0 is the "$GPGGA" address field. Points into the
 * sentence and does not copy; an empty field returns true with *len == 0.
 */
bool nmea_field(const char *sentence, int index, const char **field, size_t *len);

/* Each returns false if the sentence is not of that type or is malformed.
 * Output structs are fully overwritten (zeroed first) on success. */
bool nmea_parse_gga(const char *sentence, nmea_gga_t *out);
bool nmea_parse_rmc(const char *sentence, nmea_rmc_t *out);
bool nmea_parse_gsa(const char *sentence, nmea_gsa_t *out);
bool nmea_parse_gsv(const char *sentence, nmea_gsv_t *out);
bool nmea_parse_vtg(const char *sentence, nmea_vtg_t *out);
bool nmea_parse_gll(const char *sentence, nmea_gll_t *out);

#ifdef __cplusplus
}
#endif

#endif /* NMEA_H */
