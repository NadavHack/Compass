/*
 * lora.h - Semtech SX1276/77/78/79 LoRa transceiver driver.
 *
 * Originally Inteform's esp32-lora-library, itself a port of sandeepmistry's
 * arduino-LoRa. Restructured here so one copy of the register logic serves both
 * the ESP32 and the Raspberry Pi Pico: everything above SPI is shared, and the
 * handful of lines that differ live in port/lora_port_<target>.c, picked by
 * LORA_TARGET_ESP32 / LORA_TARGET_PICO.
 *
 * Single instance: one radio per build, state is static.
 *
 * The original API is unchanged, so existing callers keep working:
 *
 *      if (!lora_init()) { ... }                  // pins from menuconfig
 *      lora_set_frequency(868E6);
 *      lora_enable_crc();
 *      lora_send_packet((uint8_t *)"hello", 5);
 *
 * Pass pins explicitly with lora_init_config() instead where there is no
 * menuconfig, which is to say on the Pico.
 */

#ifndef __LORA_H__
#define __LORA_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Target selection                                                    */
/* ------------------------------------------------------------------ */
/*
 * CMakeLists.txt defines exactly one of these. The fallback infers it from the
 * SDK's own macros so the header still resolves in an editor or a host test.
 */
#if !defined(LORA_TARGET_ESP32) && \
    !defined(LORA_TARGET_PICO)  && \
    !defined(LORA_TARGET_HOST)
#  if defined(ESP_PLATFORM)
#    define LORA_TARGET_ESP32 1
#  elif defined(PICO_RP2040) || defined(PICO_RP2350) || defined(PICO_ON_DEVICE)
#    define LORA_TARGET_PICO 1
#  else
#    define LORA_TARGET_HOST 1
#  endif
#endif

/* ------------------------------------------------------------------ */
/* Pin and bus defaults                                                */
/* ------------------------------------------------------------------ */
/*
 * On ESP-IDF these come from menuconfig (see Kconfig); the symbol names are the
 * original unprefixed ones, so saved sdkconfig values still apply. Elsewhere
 * the values below are used. Override any of them with -DLORA_DEFAULT_xxx=...
 */
#if defined(CONFIG_CS_GPIO) && !defined(LORA_DEFAULT_CS_PIN)
#  define LORA_DEFAULT_CS_PIN   CONFIG_CS_GPIO
#endif
#if defined(CONFIG_RST_GPIO) && !defined(LORA_DEFAULT_RST_PIN)
#  define LORA_DEFAULT_RST_PIN  CONFIG_RST_GPIO
#endif
#if defined(CONFIG_MISO_GPIO) && !defined(LORA_DEFAULT_MISO_PIN)
#  define LORA_DEFAULT_MISO_PIN CONFIG_MISO_GPIO
#endif
#if defined(CONFIG_MOSI_GPIO) && !defined(LORA_DEFAULT_MOSI_PIN)
#  define LORA_DEFAULT_MOSI_PIN CONFIG_MOSI_GPIO
#endif
#if defined(CONFIG_SCK_GPIO) && !defined(LORA_DEFAULT_SCK_PIN)
#  define LORA_DEFAULT_SCK_PIN  CONFIG_SCK_GPIO
#endif

#if LORA_TARGET_PICO
#  ifndef LORA_DEFAULT_SPI_ID
#    define LORA_DEFAULT_SPI_ID   0       /* spi0 */
#  endif
#  ifndef LORA_DEFAULT_CS_PIN
#    define LORA_DEFAULT_CS_PIN   17      /* GP17 */
#  endif
#  ifndef LORA_DEFAULT_RST_PIN
#    define LORA_DEFAULT_RST_PIN  20
#  endif
#  ifndef LORA_DEFAULT_MISO_PIN
#    define LORA_DEFAULT_MISO_PIN 16
#  endif
#  ifndef LORA_DEFAULT_MOSI_PIN
#    define LORA_DEFAULT_MOSI_PIN 19
#  endif
#  ifndef LORA_DEFAULT_SCK_PIN
#    define LORA_DEFAULT_SCK_PIN  18
#  endif
#else
#  ifndef LORA_DEFAULT_SPI_ID
#    define LORA_DEFAULT_SPI_ID   3       /* SPI3_HOST, as the original used */
#  endif
#  ifndef LORA_DEFAULT_CS_PIN
#    define LORA_DEFAULT_CS_PIN   15
#  endif
#  ifndef LORA_DEFAULT_RST_PIN
#    define LORA_DEFAULT_RST_PIN  32
#  endif
#  ifndef LORA_DEFAULT_MISO_PIN
#    define LORA_DEFAULT_MISO_PIN 13
#  endif
#  ifndef LORA_DEFAULT_MOSI_PIN
#    define LORA_DEFAULT_MOSI_PIN 12
#  endif
#  ifndef LORA_DEFAULT_SCK_PIN
#    define LORA_DEFAULT_SCK_PIN  14
#  endif
#endif

/* The SX127x tolerates up to 10 MHz on its SPI port. */
#ifndef LORA_DEFAULT_SPI_HZ
#define LORA_DEFAULT_SPI_HZ 9000000
#endif

typedef struct {
    int      spi_id;    /* ESP32: 2 or 3 (SPI2_HOST/SPI3_HOST). Pico: 0 or 1. */
    int      cs_pin;    /* NSS, driven by software on both targets */
    int      rst_pin;   /* NRST; -1 if the radio's reset is not wired up */
    int      miso_pin;
    int      mosi_pin;
    int      sck_pin;
    uint32_t spi_hz;
} lora_config_t;

#define LORA_CONFIG_DEFAULT()                   \
    ((lora_config_t){                           \
        .spi_id   = LORA_DEFAULT_SPI_ID,        \
        .cs_pin   = LORA_DEFAULT_CS_PIN,        \
        .rst_pin  = LORA_DEFAULT_RST_PIN,       \
        .miso_pin = LORA_DEFAULT_MISO_PIN,      \
        .mosi_pin = LORA_DEFAULT_MOSI_PIN,      \
        .sck_pin  = LORA_DEFAULT_SCK_PIN,       \
        .spi_hz   = LORA_DEFAULT_SPI_HZ,        \
    })

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/*
 * Brings up SPI, resets the radio and applies the default configuration.
 * Returns 1 on success and 0 if the radio does not answer.
 *
 * The original aborted through assert() when SPI failed or the chip did not
 * report version 0x12. It returns 0 now instead: a tracker that cannot find its
 * radio should be able to say so and carry on with GPS logging rather than
 * panic the firmware.
 */
int lora_init(void);

/* As lora_init(), with pins supplied rather than taken from the defaults.
 * This is the one to use on the Pico, which has no menuconfig. */
int lora_init_config(const lora_config_t *cfg);

int  lora_initialized(void);
void lora_close(void);

void lora_reset(void);
void lora_explicit_header_mode(void);
void lora_implicit_header_mode(int size);
void lora_idle(void);
void lora_sleep(void);
void lora_receive(void);

void lora_set_tx_power(int level);        /* 2-17 dBm, clamped */
void lora_set_frequency(long frequency);  /* Hz */
void lora_set_spreading_factor(int sf);   /* 6-12, clamped */
void lora_set_bandwidth(long sbw);        /* Hz, up to 500000 */
void lora_set_coding_rate(int denominator); /* 5-8, clamped */
void lora_set_preamble_length(long length);
void lora_set_sync_word(int sw);
void lora_enable_crc(void);
void lora_disable_crc(void);

void lora_send_packet(uint8_t *buf, int size);
int  lora_receive_packet(uint8_t *buf, int size);
int  lora_received(void);
int  lora_packet_rssi(void);
float lora_packet_snr(void);

void lora_dump_registers(void);

/* Direct register access, for debugging and for features this driver does not
 * wrap. Exposed by the original too, just not declared in its header. */
void lora_write_reg(int reg, int val);
int  lora_read_reg(int reg);

#ifdef __cplusplus
}
#endif

#endif
