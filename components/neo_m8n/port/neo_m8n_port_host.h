/*
 * neo_m8n_port_host.h - test hooks for the host port.
 *
 * Only built when NEO_M8N_TARGET_HOST is selected (no MCU SDK present). It
 * fakes the UART with two byte queues and a virtual clock, so the driver's
 * framing, ACK handling and timeout logic can be exercised with plain gcc.
 */

#ifndef NEO_M8N_PORT_HOST_H
#define NEO_M8N_PORT_HOST_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Clears both queues, the clock and the recorded baud rate. */
void neo_m8n_port_host_reset(void);

/* Queues bytes for the driver to read, as if the module had sent them. */
void neo_m8n_port_host_inject(const void *data, size_t len);

/* Convenience wrapper for NUL-terminated NMEA text. */
void neo_m8n_port_host_inject_str(const char *s);

/*
 * Queues bytes that only become readable once the driver flushes the input.
 * Models a module that keeps transmitting across a baud change: whatever was
 * in flight is discarded, and fresh sentences show up immediately after.
 */
void neo_m8n_port_host_inject_after_flush(const char *s);

/* Drains up to len bytes the driver has written. Returns the count. */
size_t neo_m8n_port_host_take_tx(uint8_t *buf, size_t len);

/* Total bytes the driver has written since the last reset. */
size_t neo_m8n_port_host_tx_len(void);

/* Baud rate the driver last asked the port to switch to. */
uint32_t neo_m8n_port_host_baud(void);

/* Advances the virtual clock. Reads that find no data also advance it by their
 * timeout, so blocking calls terminate on their own. */
void neo_m8n_port_host_advance_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* NEO_M8N_PORT_HOST_H */
