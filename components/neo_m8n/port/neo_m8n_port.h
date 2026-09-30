/*
 * neo_m8n_port.h - the platform contract.
 *
 * Everything the NEO-M8N driver needs from the hardware is behind these seven
 * functions. To support another MCU, add a port/neo_m8n_port_<target>.c that
 * implements them and a branch in CMakeLists.txt; nothing above this line
 * changes.
 *
 * Internal to the component - applications include neo_m8n.h.
 */

#ifndef NEO_M8N_PORT_H
#define NEO_M8N_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "neo_m8n.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Opens the UART at cfg->baud, 8N1, no flow control. */
neo_m8n_err_t neo_m8n_port_init(const neo_m8n_config_t *cfg);

/* Releases the UART. Safe to call when init failed or never ran. */
void neo_m8n_port_deinit(void);

/*
 * Reads up to len bytes. Blocks until at least one byte arrives or timeout_ms
 * elapses, then returns what it has rather than waiting to fill the buffer -
 * NMEA arrives in bursts and a partial burst still parses.
 *
 * Returns the byte count (0 on timeout) or a negative neo_m8n_err_t.
 */
int neo_m8n_port_read(uint8_t *buf, size_t len, uint32_t timeout_ms);

/* Writes len bytes, blocking until they are queued. Returns the count written
 * or a negative neo_m8n_err_t. */
int neo_m8n_port_write(const uint8_t *buf, size_t len);

/*
 * Retunes the MCU's UART, not the module's. Callers must have already told the
 * module to switch, and must let its transmit buffer drain first: this flushes
 * the TX path before changing the divisor.
 */
neo_m8n_err_t neo_m8n_port_set_baud(uint32_t baud);

/* Discards buffered input. Used after a baud change, when the bytes in flight
 * are framing garbage. */
void neo_m8n_port_flush_input(void);

/* Free-running millisecond counter. Only differences are meaningful; it is
 * allowed to wrap, and the driver's comparisons are wrap-safe. */
uint32_t neo_m8n_port_millis(void);

/* Blocks the caller. Yields to other tasks where the platform has them. */
void neo_m8n_port_delay_ms(uint32_t ms);

/* ------------------------------------------------------------------ */
/* Logging                                                             */
/* ------------------------------------------------------------------ */
/*
 * Mapped onto the platform's logger by each port. Define NEO_M8N_LOG_NONE to
 * compile all of it out.
 */
#if defined(NEO_M8N_LOG_NONE)
#  define NEO_M8N_LOGE(...) ((void)0)
#  define NEO_M8N_LOGW(...) ((void)0)
#  define NEO_M8N_LOGI(...) ((void)0)
#  define NEO_M8N_LOGD(...) ((void)0)
#elif NEO_M8N_TARGET_ESP32
#  include "esp_log.h"
#  define NEO_M8N_LOG_TAG "neo_m8n"
#  define NEO_M8N_LOGE(...) ESP_LOGE(NEO_M8N_LOG_TAG, __VA_ARGS__)
#  define NEO_M8N_LOGW(...) ESP_LOGW(NEO_M8N_LOG_TAG, __VA_ARGS__)
#  define NEO_M8N_LOGI(...) ESP_LOGI(NEO_M8N_LOG_TAG, __VA_ARGS__)
#  define NEO_M8N_LOGD(...) ESP_LOGD(NEO_M8N_LOG_TAG, __VA_ARGS__)
#else
/* Pico and host: stdio. On the Pico this is whatever pico_enable_stdio_* the
 * application selected, so it costs nothing extra. */
#  include <stdio.h>
#  define NEO_M8N_LOGE(...) do { printf("E neo_m8n: "); printf(__VA_ARGS__); printf("\n"); } while (0)
#  define NEO_M8N_LOGW(...) do { printf("W neo_m8n: "); printf(__VA_ARGS__); printf("\n"); } while (0)
#  define NEO_M8N_LOGI(...) do { printf("I neo_m8n: "); printf(__VA_ARGS__); printf("\n"); } while (0)
#  ifdef NEO_M8N_DEBUG
#    define NEO_M8N_LOGD(...) do { printf("D neo_m8n: "); printf(__VA_ARGS__); printf("\n"); } while (0)
#  else
#    define NEO_M8N_LOGD(...) ((void)0)
#  endif
#endif

#ifdef __cplusplus
}
#endif

#endif /* NEO_M8N_PORT_H */
