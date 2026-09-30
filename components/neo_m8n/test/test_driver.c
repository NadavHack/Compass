/*
 * Host test for the driver core: UBX framing, ACK/NAK handling, and the way
 * NMEA and UBX interleave on the same wire.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "neo_m8n.h"
#include "neo_m8n_port_host.h"

extern int g_failures;
extern int g_checks;

#define CHECK(cond)                                                     \
    do {                                                                \
        g_checks++;                                                     \
        if (!(cond)) {                                                  \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
            g_failures++;                                               \
        }                                                               \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                   \
    do {                                                                        \
        g_checks++;                                                             \
        if (fabs((double)(a) - (double)(b)) > (eps)) {                          \
            printf("  FAIL %s:%d: %s == %.9f, expected %.9f\n",                 \
                   __FILE__, __LINE__, #a, (double)(a), (double)(b));           \
            g_failures++;                                                       \
        }                                                                       \
    } while (0)

/* Golden frames, checksums worked out by hand from the UBX spec. */

/* UBX-CFG-RATE, measRate=1000 ms, navRate=1, timeRef=1 (GPS). */
static const uint8_t CFG_RATE_1HZ[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
    0xE8, 0x03, 0x01, 0x00, 0x01, 0x00,
    0x01, 0x39
};
/* UBX-ACK-ACK acknowledging class 0x06 id 0x08. */
static const uint8_t ACK_CFG_RATE[] = {
    0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x08, 0x16, 0x3F
};
/* UBX-ACK-NAK rejecting the same. */
static const uint8_t NAK_CFG_RATE[] = {
    0xB5, 0x62, 0x05, 0x00, 0x02, 0x00, 0x06, 0x08, 0x15, 0x3A
};

static const char GGA[] =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
static const char RMC[] =
    "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
static const char GSA[] =
    "$GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39\r\n";

static neo_m8n_config_t quiet_config(void)
{
    neo_m8n_config_t cfg = NEO_M8N_CONFIG_DEFAULT();

    cfg.auto_config = false;   /* tests drive the UBX calls explicitly */
    return cfg;
}

static void start(const neo_m8n_config_t *cfg)
{
    if (neo_m8n_is_initialized()) neo_m8n_deinit();
    neo_m8n_port_host_reset();
    CHECK(neo_m8n_init(cfg) == NEO_M8N_OK);
}

static void test_init(void)
{
    neo_m8n_config_t cfg = quiet_config();

    printf("init\n");
    start(&cfg);
    CHECK(neo_m8n_is_initialized());
    CHECK(neo_m8n_port_host_tx_len() == 0);        /* auto_config off: silent */
    CHECK(neo_m8n_init(&cfg) == NEO_M8N_ERR_STATE); /* no double init */
    CHECK(neo_m8n_deinit() == NEO_M8N_OK);
    CHECK(!neo_m8n_is_initialized());
    CHECK(neo_m8n_deinit() == NEO_M8N_ERR_STATE);

    /* RX is mandatory; TX is mandatory only when we intend to talk. */
    cfg = quiet_config();
    cfg.rx_pin = -1;
    CHECK(neo_m8n_init(&cfg) == NEO_M8N_ERR_INVALID_ARG);

    cfg = quiet_config();
    cfg.tx_pin = -1;
    cfg.auto_config = true;
    CHECK(neo_m8n_init(&cfg) == NEO_M8N_ERR_INVALID_ARG);

    cfg = quiet_config();
    cfg.tx_pin = -1;
    CHECK(neo_m8n_init(&cfg) == NEO_M8N_OK);   /* receive-only is fine */
    neo_m8n_deinit();
}

static void test_ubx_framing(void)
{
    neo_m8n_config_t cfg = quiet_config();
    uint8_t tx[64];
    size_t n;

    printf("UBX frame construction\n");
    start(&cfg);

    neo_m8n_port_host_inject(ACK_CFG_RATE, sizeof(ACK_CFG_RATE));
    CHECK(neo_m8n_set_nav_rate(1000) == NEO_M8N_OK);

    n = neo_m8n_port_host_take_tx(tx, sizeof(tx));
    CHECK(n == sizeof(CFG_RATE_1HZ));
    CHECK(n == sizeof(CFG_RATE_1HZ) && !memcmp(tx, CFG_RATE_1HZ, n));

    /* CFG-MSG short form: class 0xF0, the message id, then the rate. */
    neo_m8n_port_host_reset();
    neo_m8n_deinit();
    start(&cfg);
    neo_m8n_port_host_inject((const uint8_t[]){
        0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x01, 0x0F, 0x38 }, 10);
    CHECK(neo_m8n_set_nmea_rate(NEO_M8N_NMEA_GSV, 0) == NEO_M8N_OK);
    n = neo_m8n_port_host_take_tx(tx, sizeof(tx));
    CHECK(n == 11);
    if (n == 11) {
        CHECK(tx[2] == 0x06 && tx[3] == 0x01);   /* CFG-MSG */
        CHECK(tx[4] == 3 && tx[5] == 0);         /* length 3 */
        CHECK(tx[6] == 0xF0);                    /* NMEA class */
        CHECK(tx[7] == NEO_M8N_NMEA_GSV);
        CHECK(tx[8] == 0);                       /* rate 0 = off */
    }

    CHECK(neo_m8n_set_nav_rate(10) == NEO_M8N_ERR_INVALID_ARG);
    neo_m8n_deinit();
}

static void test_ack_nak_timeout(void)
{
    neo_m8n_config_t cfg = quiet_config();
    neo_m8n_stats_t st;

    printf("ACK / NAK / timeout\n");

    start(&cfg);
    neo_m8n_port_host_inject(NAK_CFG_RATE, sizeof(NAK_CFG_RATE));
    CHECK(neo_m8n_set_nav_rate(1000) == NEO_M8N_ERR_NACK);
    neo_m8n_deinit();

    /* Silence must time out rather than hang. */
    start(&cfg);
    CHECK(neo_m8n_set_nav_rate(1000) == NEO_M8N_ERR_TIMEOUT);
    neo_m8n_deinit();

    /* An ACK for a different message must not satisfy this one. */
    start(&cfg);
    neo_m8n_port_host_inject((const uint8_t[]){
        0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x00, 0x0E, 0x37 }, 10);
    CHECK(neo_m8n_set_nav_rate(1000) == NEO_M8N_ERR_TIMEOUT);
    neo_m8n_get_stats(&st);
    CHECK(st.ubx_frames == 1);      /* it was decoded, just not a match */
    neo_m8n_deinit();

    /* A corrupted ACK is dropped, not misread. */
    start(&cfg);
    {
        uint8_t bad[sizeof(ACK_CFG_RATE)];
        memcpy(bad, ACK_CFG_RATE, sizeof(bad));
        bad[sizeof(bad) - 1] ^= 0xFF;
        neo_m8n_port_host_inject(bad, sizeof(bad));
    }
    CHECK(neo_m8n_set_nav_rate(1000) == NEO_M8N_ERR_TIMEOUT);
    neo_m8n_get_stats(&st);
    CHECK(st.ubx_errors == 1);
    CHECK(st.ubx_frames == 0);
    neo_m8n_deinit();
}

static void test_interleaved(void)
{
    neo_m8n_config_t cfg = quiet_config();
    neo_m8n_stats_t st;
    neo_m8n_fix_t fix;

    /* The receiver does not pause its NMEA output to answer a configuration
     * command, so the ACK arrives wrapped in sentences. Both must survive. */
    printf("interleaved NMEA and UBX\n");
    start(&cfg);

    neo_m8n_port_host_inject_str(GGA);
    neo_m8n_port_host_inject(ACK_CFG_RATE, sizeof(ACK_CFG_RATE));
    neo_m8n_port_host_inject_str(RMC);

    CHECK(neo_m8n_set_nav_rate(1000) == NEO_M8N_OK);
    neo_m8n_poll();                     /* the ACK arrived mid-RMC; finish it */

    neo_m8n_get_stats(&st);
    CHECK(st.sentences == 2);           /* neither sentence was lost */
    CHECK(st.checksum_errors == 0);
    CHECK(st.ubx_frames == 1);
    CHECK(neo_m8n_get_fix(&fix));
    CHECK(fix.valid);
    neo_m8n_deinit();

    /* A UBX payload containing 0x24 ('$') must not derail the NMEA framer for
     * the sentence that follows it. */
    start(&cfg);
    neo_m8n_port_host_inject((const uint8_t[]){
        0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x24, 0x24, 0x50, 0x97 }, 10);
    neo_m8n_port_host_inject_str(GGA);
    CHECK(neo_m8n_read(&fix, 1000) == NEO_M8N_OK);
    neo_m8n_get_stats(&st);
    CHECK(st.sentences == 1);
    neo_m8n_deinit();
}

static void test_fix_merge(void)
{
    neo_m8n_config_t cfg = quiet_config();
    neo_m8n_fix_t fix;

    printf("fix aggregation\n");
    start(&cfg);

    /* No data yet. */
    CHECK(!neo_m8n_get_fix(&fix));
    CHECK(neo_m8n_fix_age_ms() == UINT32_MAX);
    CHECK(neo_m8n_read(&fix, 100) == NEO_M8N_ERR_TIMEOUT);

    neo_m8n_port_host_inject_str(GGA);
    neo_m8n_port_host_inject_str(GSA);
    neo_m8n_port_host_inject_str(RMC);
    CHECK(neo_m8n_read(&fix, 1000) == NEO_M8N_OK);
    neo_m8n_poll();                       /* drain the rest */
    CHECK(neo_m8n_get_fix(&fix));

    /* Fields from three different sentences, merged into one view. */
    CHECK(fix.valid);
    CHECK_NEAR(fix.latitude,  48.0 + 7.038 / 60.0, 1e-9);   /* GGA + RMC */
    CHECK_NEAR(fix.altitude_m, 545.4, 1e-3);                /* GGA only */
    CHECK(fix.satellites_used == 8);                        /* GGA only */
    CHECK(fix.fix_quality == NEO_M8N_FIX_GPS);              /* GGA only */
    CHECK(fix.fix_type == 3);                               /* GSA only */
    CHECK_NEAR(fix.pdop, 2.5, 1e-6);                        /* GSA only */
    CHECK_NEAR(fix.speed_knots, 22.4, 1e-3);                /* RMC only */
    CHECK_NEAR(fix.speed_mps, 22.4f * 0.514444f, 1e-3);     /* converted */
    CHECK_NEAR(fix.course_deg, 84.4, 1e-3);                 /* RMC only */

    /* Date comes from RMC, time from either; neither may clear the other. */
    CHECK(fix.time.time_valid && fix.time.date_valid);
    CHECK(fix.time.hours == 12 && fix.time.minutes == 35);
    CHECK(fix.time.year == 1994 && fix.time.month == 3 && fix.time.day == 23);

    /* Age tracks the clock. */
    neo_m8n_port_host_advance_ms(2500);
    CHECK(neo_m8n_get_fix(&fix) && fix.age_ms == 2500);
    neo_m8n_deinit();
}

static void test_no_fix_then_fix(void)
{
    neo_m8n_config_t cfg = quiet_config();
    neo_m8n_fix_t fix;

    /* A receiver that is still acquiring emits well-formed sentences with
     * empty position fields. read() must succeed and valid must stay false. */
    printf("acquiring, then locked\n");
    start(&cfg);

    neo_m8n_port_host_inject_str("$GPGGA,000000.00,,,,,0,00,99.99,,,,,,*66\r\n");
    neo_m8n_port_host_inject_str("$GPRMC,000000.00,V,,,,,,,060180,,,N*72\r\n");
    CHECK(neo_m8n_read(&fix, 1000) == NEO_M8N_OK);
    neo_m8n_poll();
    CHECK(neo_m8n_get_fix(&fix));
    CHECK(!fix.valid);
    CHECK(fix.fix_quality == NEO_M8N_FIX_NONE);
    CHECK(neo_m8n_wait_for_fix(&fix, 200) == NEO_M8N_ERR_TIMEOUT);

    neo_m8n_port_host_inject_str(GGA);
    neo_m8n_port_host_inject_str(RMC);
    CHECK(neo_m8n_wait_for_fix(&fix, 2000) == NEO_M8N_OK);
    CHECK(fix.valid);
    neo_m8n_deinit();
}

static void test_baud_switch(void)
{
    neo_m8n_config_t cfg = quiet_config();
    uint8_t tx[64];
    size_t n;

    printf("baud switch\n");
    start(&cfg);

    /* set_baud() discards whatever was in flight, then confirms the switch by
     * waiting for a sentence that frames correctly at the new rate. */
    neo_m8n_port_host_inject_after_flush(GGA);
    CHECK(neo_m8n_set_baud(38400) == NEO_M8N_OK);
    CHECK(neo_m8n_port_host_baud() == 38400);

    n = neo_m8n_port_host_take_tx(tx, sizeof(tx));
    CHECK(n == 28);                       /* 8 header/checksum + 20 payload */
    if (n == 28) {
        CHECK(tx[2] == 0x06 && tx[3] == 0x00);              /* CFG-PRT */
        CHECK(tx[4] == 20 && tx[5] == 0);
        CHECK(tx[6] == 0x01);                               /* UART1 */
        CHECK(tx[10] == 0xD0 && tx[11] == 0x08);            /* 8N1 mode */
        CHECK(tx[14] == 0x00 && tx[15] == 0x96 &&
              tx[16] == 0x00 && tx[17] == 0x00);            /* 38400 LE */
        CHECK(tx[18] == 0x03 && tx[20] == 0x03);            /* UBX + NMEA both ways */
    }

    CHECK(neo_m8n_set_baud(38400) == NEO_M8N_OK);   /* already there: no-op */
    CHECK(neo_m8n_port_host_tx_len() == 0);
    CHECK(neo_m8n_set_baud(100) == NEO_M8N_ERR_INVALID_ARG);

    /* Silence after the switch means the divisor is wrong. */
    CHECK(neo_m8n_set_baud(115200) == NEO_M8N_ERR_TIMEOUT);
    neo_m8n_deinit();
}

static void test_auto_config(void)
{
    neo_m8n_config_t cfg = NEO_M8N_CONFIG_DEFAULT();
    neo_m8n_stats_t st;

    /* auto_config sends CFG-MSG and CFG-RATE. With nothing acking, init must
     * still come back OK: default NMEA output is enough to be useful. */
    printf("auto-config survives an unresponsive module\n");
    neo_m8n_port_host_reset();
    if (neo_m8n_is_initialized()) neo_m8n_deinit();

    CHECK(neo_m8n_init(&cfg) == NEO_M8N_OK);
    CHECK(neo_m8n_port_host_tx_len() > 0);   /* it did try */

    /* Six CFG-MSG frames of 11 bytes, then one CFG-RATE of 14. */
    CHECK(neo_m8n_port_host_tx_len() == 6 * 11 + 14);

    neo_m8n_get_stats(&st);
    CHECK(st.ubx_frames == 0);
    neo_m8n_deinit();
}

static void test_uninitialized(void)
{
    neo_m8n_fix_t fix;

    printf("calls before init\n");
    if (neo_m8n_is_initialized()) neo_m8n_deinit();

    CHECK(neo_m8n_read(&fix, 10) == NEO_M8N_ERR_STATE);
    CHECK(neo_m8n_poll() == NEO_M8N_ERR_STATE);
    CHECK(neo_m8n_set_nav_rate(1000) == NEO_M8N_ERR_STATE);
    CHECK(neo_m8n_set_nmea_rate(NEO_M8N_NMEA_GGA, 1) == NEO_M8N_ERR_STATE);
    CHECK(neo_m8n_set_baud(38400) == NEO_M8N_ERR_STATE);
    CHECK(neo_m8n_save_config() == NEO_M8N_ERR_STATE);
    CHECK(!neo_m8n_get_fix(&fix));
    CHECK(neo_m8n_init(NULL) == NEO_M8N_ERR_INVALID_ARG);
}

void run_driver_tests(void);
void run_driver_tests(void)
{
    test_init();
    test_ubx_framing();
    test_ack_nak_timeout();
    test_interleaved();
    test_fix_merge();
    test_no_fix_then_fix();
    test_baud_switch();
    test_auto_config();
    test_uninitialized();
}
