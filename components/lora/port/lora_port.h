/*
 * lora_port.h - the platform contract for the SX127x driver.
 *
 * Six functions. To support another MCU, add a port/lora_port_<target>.c that
 * implements them and a branch in CMakeLists.txt; the register logic in
 * src/lora.c does not change.
 *
 * Internal to the component - applications include lora.h.
 */

#ifndef LORA_PORT_H
#define LORA_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lora.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Brings up the SPI bus and configures CS and RST as outputs, CS idle high.
 * Returns false if the bus could not be claimed. */
bool lora_port_init(const lora_config_t *cfg);

/* Releases the bus. Safe when init failed or never ran. */
void lora_port_deinit(void);

/*
 * Full-duplex transfer of len bytes with CS asserted for the whole exchange.
 * The port owns the chip select because every SX127x access is one framed
 * burst; splitting it would let a preempting task interleave a transfer.
 * rx may be NULL to discard what comes back.
 */
void lora_port_transfer(const uint8_t *tx, uint8_t *rx, size_t len);

/* Drives NRST. Reset timing lives in src/lora.c so both targets share it.
 * A no-op when rst_pin is -1. */
void lora_port_reset_pin(bool high);

void     lora_port_delay_ms(uint32_t ms);
uint32_t lora_port_millis(void);

/* ------------------------------------------------------------------ */
/* Logging                                                             */
/* ------------------------------------------------------------------ */

#if defined(LORA_LOG_NONE)
#  define LORA_LOGE(...) ((void)0)
#  define LORA_LOGW(...) ((void)0)
#  define LORA_LOGI(...) ((void)0)
#elif LORA_TARGET_ESP32
#  include "esp_log.h"
#  define LORA_LOG_TAG "lora"
#  define LORA_LOGE(...) ESP_LOGE(LORA_LOG_TAG, __VA_ARGS__)
#  define LORA_LOGW(...) ESP_LOGW(LORA_LOG_TAG, __VA_ARGS__)
#  define LORA_LOGI(...) ESP_LOGI(LORA_LOG_TAG, __VA_ARGS__)
#else
#  include <stdio.h>
#  define LORA_LOGE(...) do { printf("E lora: "); printf(__VA_ARGS__); printf("\n"); } while (0)
#  define LORA_LOGW(...) do { printf("W lora: "); printf(__VA_ARGS__); printf("\n"); } while (0)
#  define LORA_LOGI(...) do { printf("I lora: "); printf(__VA_ARGS__); printf("\n"); } while (0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* LORA_PORT_H */
