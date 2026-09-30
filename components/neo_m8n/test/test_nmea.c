/*
 * Host test for the portable half of the component (nmea.c + neo_m8n.c).
 * Build and run with test/run_tests.sh - no MCU or SDK needed.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "neo_m8n.h"
#include "nmea.h"

int g_failures;
int g_checks;

void run_driver_tests(void);

#define CHECK(cond)                                                     \
    do {                                                                \
        g_checks++;                                                     \
        if (!(cond)) {                                                  \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
            g_failures++;                                               \
        }                                                               \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                        \
    do {                                                                             \
        g_checks++;                                                                  \
        if (fabs((double)(a) - (double)(b)) > (eps)) {                               \
            printf("  FAIL %s:%d: %s == %.9f, expected %.9f\n",                      \
                   __FILE__, __LINE__, #a, (double)(a), (double)(b));                \
            g_failures++;                                                            \
        }                                                                            \
    } while (0)

static void test_checksum(void)
{
    printf("checksum\n");
    CHECK(nmea_validate("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"));
    CHECK(nmea_validate("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"));
    /* lower-case hex digits are legal */
    CHECK(nmea_validate("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6a"));
    /* one bit flipped in the payload */
    CHECK(!nmea_validate("$GPGGA,123519,4807.039,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"));
    CHECK(!nmea_validate("$GPGGA,123519,4807.038,N"));      /* no checksum at all */
    CHECK(!nmea_validate("GPGGA,123519*47"));               /* no leading $ */
    CHECK(!nmea_validate("$GPGGA,1*4"));                    /* truncated checksum */
    CHECK(!nmea_validate(""));
}

static void test_fields(void)
{
    const char *s = "$GPGGA,123519,,N,01131.000,E,1*4D";
    const char *f;
    size_t len;

    printf("field split\n");
    CHECK(nmea_field(s, 0, &f, &len) && len == 6 && !memcmp(f, "$GPGGA", 6));
    CHECK(nmea_field(s, 1, &f, &len) && len == 6 && !memcmp(f, "123519", 6));
    CHECK(nmea_field(s, 2, &f, &len) && len == 0);          /* empty field */
    CHECK(nmea_field(s, 6, &f, &len) && len == 1 && f[0] == '1');   /* stops at '*' */
    CHECK(!nmea_field(s, 7, &f, &len));                     /* past the end */
}

static void test_gga(void)
{
    nmea_gga_t g;

    printf("GGA\n");
    CHECK(nmea_parse_gga("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47", &g));
    CHECK(g.position_valid);
    CHECK_NEAR(g.latitude,   48.0 + 7.038 / 60.0,  1e-9);
    CHECK_NEAR(g.longitude,  11.0 + 31.0  / 60.0,  1e-9);
    CHECK(g.fix_quality == 1);
    CHECK(g.satellites == 8);
    CHECK_NEAR(g.hdop, 0.9, 1e-6);
    CHECK_NEAR(g.altitude_m, 545.4, 1e-3);
    CHECK_NEAR(g.geoid_sep_m, 46.9, 1e-3);
    CHECK(g.time.time_valid && g.time.hours == 12 && g.time.minutes == 35 && g.time.seconds == 19);
    CHECK(!g.time.date_valid);

    /* Southern and western hemispheres must come out negative. */
    CHECK(nmea_parse_gga("$GPGGA,041000.000,3339.7000,S,15100.0000,W,1,06,1.5,10.0,M,0.0,M,,*5D", &g));
    CHECK(g.latitude < 0 && g.longitude < 0);
    CHECK_NEAR(g.latitude,  -(33.0 + 39.7 / 60.0), 1e-9);
    CHECK_NEAR(g.longitude, -(151.0), 1e-9);

    /* No fix: quality 0 and blank position fields. Must parse, but not claim a position. */
    CHECK(nmea_parse_gga("$GPGGA,000000.00,,,,,0,00,99.99,,,,,,*66", &g));
    CHECK(!g.position_valid);
    CHECK(g.fix_quality == 0);
    CHECK(g.satellites == 0);

    /* Sub-second timestamps and a GN talker (multi-constellation). */
    CHECK(nmea_parse_gga("$GNGGA,092750.250,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*6F", &g));
    CHECK(g.time.hours == 9 && g.time.minutes == 27 && g.time.seconds == 50 && g.time.millis == 250);

    CHECK(!nmea_parse_gga("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A", &g));
}

static void test_rmc(void)
{
    nmea_rmc_t r;

    printf("RMC\n");
    CHECK(nmea_parse_rmc("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A", &r));
    CHECK(r.valid && r.position_valid);
    CHECK_NEAR(r.latitude,  48.0 + 7.038 / 60.0, 1e-9);
    CHECK_NEAR(r.speed_knots, 22.4, 1e-3);
    CHECK_NEAR(r.course_deg, 84.4, 1e-3);
    CHECK(r.time.date_valid && r.time.year == 1994 && r.time.month == 3 && r.time.day == 23);
    CHECK(r.variation_valid);
    CHECK_NEAR(r.variation_deg, -3.1, 1e-3);   /* 'W' makes it negative */

    /* Void status: the receiver has no fix, so the position must not be used. */
    CHECK(nmea_parse_rmc("$GPRMC,235947.000,V,,,,,,,041299,,*26", &r));
    CHECK(!r.valid && !r.position_valid);

    /* NMEA 2.3+ adds a mode indicator field after the variation. */
    CHECK(nmea_parse_rmc("$GNRMC,083559.00,A,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,A*49", &r));
    CHECK(r.valid && r.position_valid);
    CHECK_NEAR(r.latitude, 47.0 + 17.11437 / 60.0, 1e-9);
    CHECK(!r.variation_valid);                 /* variation field is empty here */
    CHECK(r.time.year == 2002 && r.time.month == 12 && r.time.day == 9);
}

static void test_gsa_gsv_vtg_gll(void)
{
    nmea_gsa_t a;
    nmea_gsv_t v;
    nmea_vtg_t t;
    nmea_gll_t l;

    printf("GSA / GSV / VTG / GLL\n");
    CHECK(nmea_parse_gsa("$GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39", &a));
    CHECK(a.mode == 'A' && a.fix_type == 3);
    CHECK(a.sv_count == 5);                    /* empty channels are skipped */
    CHECK(a.sv[0] == 4 && a.sv[4] == 24);
    CHECK_NEAR(a.pdop, 2.5, 1e-6);
    CHECK_NEAR(a.vdop, 2.1, 1e-6);

    CHECK(nmea_parse_gsv("$GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74", &v));
    CHECK(v.total_msgs == 3 && v.msg_num == 1 && v.total_sats == 11 && v.count == 4);
    CHECK(v.sats[0].prn == 3 && v.sats[0].elevation_deg == 3 && v.sats[0].azimuth_deg == 111);
    CHECK(v.sats[0].snr_valid && v.sats[0].snr_dbhz == 0);

    /* Last sentence of a group carries fewer than four satellites. */
    CHECK(nmea_parse_gsv("$GPGSV,3,3,11,22,42,067,42,24,14,311,43,27,05,244,00*4D", &v));
    CHECK(v.count == 3);
    CHECK(v.sats[2].prn == 27);

    CHECK(nmea_parse_vtg("$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48", &t));
    CHECK_NEAR(t.course_true_deg, 54.7, 1e-3);
    CHECK(t.course_mag_valid);
    CHECK_NEAR(t.speed_kph, 10.2, 1e-3);

    CHECK(nmea_parse_gll("$GPGLL,4916.45,N,12311.12,W,225444,A*31", &l));
    CHECK(l.valid && l.position_valid);
    CHECK_NEAR(l.longitude, -(123.0 + 11.12 / 60.0), 1e-9);
    CHECK(l.time.hours == 22 && l.time.minutes == 54 && l.time.seconds == 44);
}

static void test_stream(void)
{
    /* Two good sentences, one corrupted, and a truncated one that is cut short
     * by the '$' of its successor - exactly what a noisy UART delivers. */
    static const char feed[] =
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n"
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*FF\r\n"
        "$GPRMC,123519,A,4807.0"
        "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";

    nmea_stream_t s;
    char line[NMEA_MAX_SENTENCE];
    int got = 0;
    nmea_sentence_id_t last = NMEA_UNKNOWN;

    printf("stream framing\n");
    nmea_stream_init(&s);
    for (size_t i = 0; i < sizeof(feed) - 1; i++) {
        if (nmea_stream_push(&s, feed[i], line, sizeof(line))) {
            got++;
            last = nmea_sentence_id(line);
        }
    }
    CHECK(got == 2);
    CHECK(last == NMEA_RMC);
    CHECK(s.sentences == 2);
    CHECK(s.checksum_errors == 1);   /* the *FF one */
    CHECK(s.overruns == 0);

    /* An over-long run of bytes must not overflow the buffer. */
    nmea_stream_init(&s);
    nmea_stream_push(&s, '$', line, sizeof(line));
    for (int i = 0; i < NMEA_MAX_SENTENCE * 3; i++) {
        CHECK(!nmea_stream_push(&s, 'A', line, sizeof(line)));
    }
    CHECK(s.overruns > 0);
    CHECK(s.sentences == 0);
}

int main(void)
{
    test_checksum();
    test_fields();
    test_gga();
    test_rmc();
    test_gsa_gsv_vtg_gll();
    test_stream();
    run_driver_tests();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
