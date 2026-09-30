/*
 * Host test for the SX127x register logic.
 *
 * Runs src/lora.c against the simulated register file in
 * port/lora_port_host.c, so the modem maths and register sequences are checked
 * without a radio. Build and run with test/run_tests.sh.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "lora.h"
#include "lora_port_host.h"

static int g_failures;
static int g_checks;

#define CHECK(cond)                                                     \
    do {                                                                \
        g_checks++;                                                     \
        if (!(cond)) {                                                  \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
            g_failures++;                                               \
        }                                                               \
    } while (0)

#define CHECK_REG(reg, expected)                                                  \
    do {                                                                          \
        unsigned got_ = lora_port_host_reg(reg);                                  \
        g_checks++;                                                               \
        if (got_ != (unsigned)(expected)) {                                       \
            printf("  FAIL %s:%d: reg 0x%02X == 0x%02X, expected 0x%02X\n",       \
                   __FILE__, __LINE__, (unsigned)(reg), got_, (unsigned)(expected)); \
            g_failures++;                                                         \
        }                                                                         \
    } while (0)

/* Register numbers, spelled out again here so the test does not lean on the
 * driver's own private definitions being right. */
#define R_FIFO           0x00
#define R_OP_MODE        0x01
#define R_FRF_MSB        0x06
#define R_FRF_MID        0x07
#define R_FRF_LSB        0x08
#define R_PA_CONFIG      0x09
#define R_LNA            0x0c
#define R_FIFO_ADDR_PTR  0x0d
#define R_FIFO_TX_BASE   0x0e
#define R_FIFO_RX_BASE   0x0f
#define R_IRQ_FLAGS      0x12
#define R_MODEM_CONFIG_1 0x1d
#define R_MODEM_CONFIG_2 0x1e
#define R_PREAMBLE_MSB   0x20
#define R_PREAMBLE_LSB   0x21
#define R_PAYLOAD_LENGTH 0x22
#define R_MODEM_CONFIG_3 0x26
#define R_DETECT_OPT     0x31
#define R_DETECT_THRESH  0x37
#define R_SYNC_WORD      0x39

#define MODE_LORA_STDBY  0x81
#define MODE_LORA_SLEEP  0x80
#define MODE_LORA_RX     0x85

static void begin(void)
{
    if (lora_initialized()) lora_close();
    lora_port_host_reset();
    CHECK(lora_init() == 1);
}

static void test_init(void)
{
    printf("init\n");
    lora_port_host_reset();

    CHECK(!lora_initialized());
    CHECK(lora_init() == 1);
    CHECK(lora_initialized());

    /* The chip must have been pulsed out of reset. */
    CHECK(lora_port_host_resets() == 1);

    /* Default configuration from the original driver. */
    CHECK_REG(R_FIFO_RX_BASE, 0x00);
    CHECK_REG(R_FIFO_TX_BASE, 0x00);
    CHECK_REG(R_LNA, 0x03);              /* boosted, OR'd into whatever was there */
    CHECK_REG(R_MODEM_CONFIG_3, 0x04);   /* low data rate optimise / AGC */
    CHECK_REG(R_PA_CONFIG, 0x8F);        /* PA_BOOST, 17 dBm */
    CHECK_REG(R_OP_MODE, MODE_LORA_STDBY);

    lora_close();
    CHECK(!lora_initialized());
}

static void test_missing_radio(void)
{
    printf("missing radio\n");

    /* A chip that never reports 0x12 must make init fail rather than spin or
     * abort the firmware, and must not leave the driver looking ready. */
    lora_port_host_reset();
    lora_port_host_set_version(0xFF);
    CHECK(lora_init() == 0);
    CHECK(!lora_initialized());

    lora_port_host_reset();
    lora_port_host_set_version(0x00);    /* bus held low / nothing connected */
    CHECK(lora_init() == 0);
    CHECK(!lora_initialized());

    /* And a working one still comes up afterwards. */
    lora_port_host_reset();
    CHECK(lora_init() == 1);
    lora_close();
}

static void test_frequency(void)
{
    printf("frequency\n");
    begin();

    /* frf = (freq << 19) / 32 MHz, split across three registers. */
    lora_set_frequency(868000000L);
    CHECK_REG(R_FRF_MSB, 0xD9);
    CHECK_REG(R_FRF_MID, 0x00);
    CHECK_REG(R_FRF_LSB, 0x00);

    lora_set_frequency(915000000L);
    CHECK_REG(R_FRF_MSB, 0xE4);
    CHECK_REG(R_FRF_MID, 0xC0);
    CHECK_REG(R_FRF_LSB, 0x00);

    lora_set_frequency(433000000L);
    CHECK_REG(R_FRF_MSB, 0x6C);
    CHECK_REG(R_FRF_MID, 0x40);
    CHECK_REG(R_FRF_LSB, 0x00);

    lora_close();
}

static void test_spreading_factor(void)
{
    printf("spreading factor\n");
    begin();

    lora_set_spreading_factor(7);
    CHECK_REG(R_MODEM_CONFIG_2, 0x70);
    CHECK_REG(R_DETECT_OPT, 0xC3);
    CHECK_REG(R_DETECT_THRESH, 0x0A);

    lora_set_spreading_factor(12);
    CHECK_REG(R_MODEM_CONFIG_2, 0xC0);

    /* SF6 needs different detection settings; it is the one special case. */
    lora_set_spreading_factor(6);
    CHECK_REG(R_MODEM_CONFIG_2, 0x60);
    CHECK_REG(R_DETECT_OPT, 0xC5);
    CHECK_REG(R_DETECT_THRESH, 0x0C);

    /* Out-of-range values clamp rather than corrupt the register. */
    lora_set_spreading_factor(3);
    CHECK_REG(R_MODEM_CONFIG_2, 0x60);   /* clamped to 6 */
    lora_set_spreading_factor(99);
    CHECK_REG(R_MODEM_CONFIG_2, 0xC0);   /* clamped to 12 */

    /* The low nibble of MODEM_CONFIG_2 belongs to CRC and symbol timeout and
     * must survive a spreading factor change. */
    lora_enable_crc();
    CHECK_REG(R_MODEM_CONFIG_2, 0xC4);
    lora_set_spreading_factor(9);
    CHECK_REG(R_MODEM_CONFIG_2, 0x94);   /* SF9 in the high nibble, CRC kept */

    lora_close();
}

static void test_bandwidth_and_coding_rate(void)
{
    printf("bandwidth and coding rate\n");
    begin();

    lora_set_bandwidth(125000L);
    CHECK_REG(R_MODEM_CONFIG_1, 0x70);
    lora_set_bandwidth(250000L);
    CHECK_REG(R_MODEM_CONFIG_1, 0x80);
    lora_set_bandwidth(500000L);
    CHECK_REG(R_MODEM_CONFIG_1, 0x90);
    lora_set_bandwidth(7800L);
    CHECK_REG(R_MODEM_CONFIG_1, 0x00);

    /* Boundaries: <= is the comparison, so the exact value picks the lower code. */
    lora_set_bandwidth(31250L);
    CHECK_REG(R_MODEM_CONFIG_1, 0x40);
    lora_set_bandwidth(31251L);
    CHECK_REG(R_MODEM_CONFIG_1, 0x50);

    /* Coding rate shares MODEM_CONFIG_1 with bandwidth and the header bit. */
    lora_set_bandwidth(125000L);
    lora_set_coding_rate(5);
    CHECK_REG(R_MODEM_CONFIG_1, 0x72);   /* bw 7 kept, cr 4/5 */
    lora_set_coding_rate(8);
    CHECK_REG(R_MODEM_CONFIG_1, 0x78);   /* bw 7 kept, cr 4/8 */
    lora_set_coding_rate(99);
    CHECK_REG(R_MODEM_CONFIG_1, 0x78);   /* clamped to 8 */

    lora_close();
}

static void test_power_and_misc(void)
{
    printf("tx power, preamble, sync word, CRC, header mode\n");
    begin();

    lora_set_tx_power(17);
    CHECK_REG(R_PA_CONFIG, 0x8F);
    lora_set_tx_power(2);
    CHECK_REG(R_PA_CONFIG, 0x80);
    lora_set_tx_power(0);                /* clamps up to 2 */
    CHECK_REG(R_PA_CONFIG, 0x80);
    lora_set_tx_power(30);               /* clamps down to 17 */
    CHECK_REG(R_PA_CONFIG, 0x8F);

    lora_set_preamble_length(8);
    CHECK_REG(R_PREAMBLE_MSB, 0x00);
    CHECK_REG(R_PREAMBLE_LSB, 0x08);
    lora_set_preamble_length(1024);
    CHECK_REG(R_PREAMBLE_MSB, 0x04);
    CHECK_REG(R_PREAMBLE_LSB, 0x00);

    lora_set_sync_word(0x34);
    CHECK_REG(R_SYNC_WORD, 0x34);

    lora_enable_crc();
    CHECK(lora_port_host_reg(R_MODEM_CONFIG_2) & 0x04);
    lora_disable_crc();
    CHECK(!(lora_port_host_reg(R_MODEM_CONFIG_2) & 0x04));

    lora_set_bandwidth(125000L);         /* leaves 0x70 in MODEM_CONFIG_1 */
    lora_implicit_header_mode(16);
    CHECK(lora_port_host_reg(R_MODEM_CONFIG_1) & 0x01);
    CHECK_REG(R_PAYLOAD_LENGTH, 16);
    lora_explicit_header_mode();
    CHECK(!(lora_port_host_reg(R_MODEM_CONFIG_1) & 0x01));
    CHECK_REG(R_MODEM_CONFIG_1, 0x70);   /* the rest of the register survived */

    lora_close();
}

static void test_modes(void)
{
    printf("mode transitions\n");
    begin();

    lora_sleep();
    CHECK_REG(R_OP_MODE, MODE_LORA_SLEEP);
    lora_idle();
    CHECK_REG(R_OP_MODE, MODE_LORA_STDBY);
    lora_receive();
    CHECK_REG(R_OP_MODE, MODE_LORA_RX);

    lora_close();
}

static void test_send_packet(void)
{
    static const uint8_t payload[] = { 'C', 'o', 'm', 'p', 'a', 's', 's' };
    uint8_t fifo[32];
    size_t n;

    printf("send packet\n");
    begin();

    lora_send_packet((uint8_t *)payload, (int)sizeof(payload));

    n = lora_port_host_fifo_take(fifo, sizeof(fifo));
    CHECK(n == sizeof(payload));
    CHECK(n == sizeof(payload) && !memcmp(fifo, payload, n));
    CHECK_REG(R_PAYLOAD_LENGTH, sizeof(payload));
    CHECK_REG(R_FIFO_ADDR_PTR, sizeof(payload));   /* advanced by the writes */

    /* TX_DONE is acknowledged, so the next transmission starts clean. */
    CHECK(!(lora_port_host_reg(R_IRQ_FLAGS) & 0x08));

    lora_close();
}

static void test_send_timeout(void)
{
    static const uint8_t payload[] = { 1, 2, 3 };
    uint32_t start, elapsed;

    printf("transmit that never completes\n");
    begin();

    /* A radio that stops raising TX_DONE. The original spun on that flag
     * forever, wedging the calling task; this must give up and return. */
    lora_port_host_set_tx_completes(false);

    start = lora_port_host_millis();
    lora_send_packet((uint8_t *)payload, (int)sizeof(payload));
    elapsed = lora_port_host_millis() - start;

    CHECK(elapsed >= 10000);                 /* it waited the full timeout */
    CHECK(elapsed < 11000);                  /* and not appreciably longer */
    CHECK_REG(R_OP_MODE, MODE_LORA_STDBY);   /* parked idle, not left in TX */

    /* Once the radio behaves again, transmission works as before. */
    lora_port_host_set_tx_completes(true);
    lora_send_packet((uint8_t *)payload, (int)sizeof(payload));
    CHECK(!(lora_port_host_reg(R_IRQ_FLAGS) & 0x08));

    lora_close();
}

static void test_receive_packet(void)
{
    static const uint8_t incoming[] = { 'h', 'e', 'l', 'l', 'o' };
    uint8_t buf[16];
    int n;

    printf("receive packet\n");
    begin();

    /* Nothing staged: no packet, and no bytes written into the buffer. */
    memset(buf, 0xAA, sizeof(buf));
    CHECK(lora_received() == 0);
    CHECK(lora_receive_packet(buf, sizeof(buf)) == 0);
    CHECK(buf[0] == 0xAA);

    lora_port_host_stage_rx(incoming, (uint8_t)sizeof(incoming));
    CHECK(lora_received() == 1);

    n = lora_receive_packet(buf, sizeof(buf));
    CHECK(n == (int)sizeof(incoming));
    CHECK(n == (int)sizeof(incoming) && !memcmp(buf, incoming, (size_t)n));

    /* The interrupt flag was acknowledged, so the packet is not read twice. */
    CHECK(lora_received() == 0);

    /* A packet larger than the caller's buffer is truncated, not overflowed. */
    lora_port_host_stage_rx(incoming, (uint8_t)sizeof(incoming));
    memset(buf, 0xAA, sizeof(buf));
    n = lora_receive_packet(buf, 3);
    CHECK(n == 3);
    CHECK(buf[3] == 0xAA);               /* nothing written past the limit */

    /* A CRC error means the payload is discarded. */
    lora_port_host_stage_rx(incoming, (uint8_t)sizeof(incoming));
    lora_port_host_set_reg(R_IRQ_FLAGS, 0x40 | 0x20);   /* RX_DONE + CRC error */
    CHECK(lora_receive_packet(buf, sizeof(buf)) == 0);

    lora_close();
}

static void test_rssi_snr(void)
{
    printf("RSSI and SNR\n");
    begin();

    /* Below 868 MHz the datasheet's low-band offset applies, above it the
     * high-band one. */
    lora_set_frequency(433000000L);
    lora_port_host_set_reg(0x1a, 200);
    CHECK(lora_packet_rssi() == 200 - 164);

    lora_set_frequency(868000000L);
    CHECK(lora_packet_rssi() == 200 - 157);

    /* SNR is a signed quarter-dB count. */
    lora_port_host_set_reg(0x19, 40);
    CHECK(fabs(lora_packet_snr() - 10.0) < 1e-6);
    lora_port_host_set_reg(0x19, 0xFF);          /* -1 */
    CHECK(fabs(lora_packet_snr() - (-0.25)) < 1e-6);
    lora_port_host_set_reg(0x19, 0x80);          /* -128 */
    CHECK(fabs(lora_packet_snr() - (-32.0)) < 1e-6);

    lora_close();
}

static void test_close(void)
{
    printf("close and re-init\n");
    begin();

    lora_close();
    CHECK(!lora_initialized());
    CHECK_REG(R_OP_MODE, MODE_LORA_SLEEP);   /* parked in sleep to save power */

    /* Closing twice is harmless, and the driver comes back up afterwards. */
    lora_close();
    CHECK(lora_init() == 1);
    CHECK(lora_initialized());
    lora_close();
}

int main(void)
{
    test_init();
    test_missing_radio();
    test_frequency();
    test_spreading_factor();
    test_bandwidth_and_coding_rate();
    test_power_and_misc();
    test_modes();
    test_send_packet();
    test_send_timeout();
    test_receive_packet();
    test_rssi_snr();
    test_close();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
