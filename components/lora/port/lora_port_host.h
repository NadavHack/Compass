/*
 * lora_port_host.h - test hooks for the host port.
 *
 * Only built when LORA_TARGET_HOST is selected. Stands in for the SPI bus with
 * a simulated SX127x register file, so the driver's register sequences and
 * modem maths can be checked with plain gcc.
 */

#ifndef LORA_PORT_HOST_H
#define LORA_PORT_HOST_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LORA_HOST_NUM_REGS 0x80
#define LORA_HOST_FIFO_LEN 256

/* Clears the register file, the FIFO and the clock, and puts 0x12 back in the
 * version register so a radio appears present. */
void lora_port_host_reset(void);

/* Value the version register reports. Set it to something else to exercise the
 * "no radio on the bus" path. */
void lora_port_host_set_version(uint8_t v);

uint8_t lora_port_host_reg(int reg);
void    lora_port_host_set_reg(int reg, uint8_t v);

/* Bytes the driver pushed into the FIFO, i.e. the payload it transmitted. */
size_t  lora_port_host_fifo_take(uint8_t *buf, size_t len);

/* Stages a packet as though the radio had received it: fills the FIFO, sets
 * the length register and raises the RX_DONE interrupt flag. */
void    lora_port_host_stage_rx(const uint8_t *data, uint8_t len);

/* How many times the reset line has been pulsed low. */
/*
 * When false, entering transmit mode no longer raises TX_DONE, the way a radio
 * that has browned out mid-packet behaves. Defaults to true.
 */
void lora_port_host_set_tx_completes(bool completes);

/* Virtual clock, advanced by the driver's own delays. */
uint32_t lora_port_host_millis(void);

uint32_t lora_port_host_resets(void);

/* Total register writes and reads, for checking that a call touched the bus. */
uint32_t lora_port_host_writes(void);
uint32_t lora_port_host_reads(void);

void lora_port_host_advance_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* LORA_PORT_HOST_H */
