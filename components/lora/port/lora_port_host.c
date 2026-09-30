/*
 * lora_port_host.c - host/test port. Not for hardware.
 *
 * Models just enough of an SX127x to exercise the driver: a register file, the
 * auto-incrementing FIFO, and the TX_DONE interrupt that a real chip raises
 * when a transmission finishes.
 */

#include "lora_port.h"

#if LORA_TARGET_HOST

#include <string.h>

#include "lora_port_host.h"

/* Mirrors of the register numbers src/lora.c uses; kept local so the driver's
 * definitions stay private to it. */
#define H_REG_FIFO             0x00
#define H_REG_OP_MODE          0x01
#define H_REG_FIFO_ADDR_PTR    0x0d
#define H_REG_IRQ_FLAGS        0x12
#define H_REG_RX_NB_BYTES      0x13
#define H_REG_VERSION          0x42

#define H_MODE_TX              0x03
#define H_IRQ_TX_DONE          0x08
#define H_IRQ_RX_DONE          0x40

static uint8_t  s_regs[LORA_HOST_NUM_REGS];
static uint8_t  s_fifo[LORA_HOST_FIFO_LEN];
static size_t   s_fifo_written;     /* high-water mark of driver writes */
static uint8_t  s_version = 0x12;
static uint32_t s_millis;
static uint32_t s_resets;
static uint32_t s_writes, s_reads;
static bool     s_rst_level = true;
static bool     s_tx_completes = true;
static bool     s_open;

void lora_port_host_reset(void)
{
    memset(s_regs, 0, sizeof(s_regs));
    memset(s_fifo, 0, sizeof(s_fifo));
    s_fifo_written = 0;
    s_version = 0x12;
    s_millis = 0;
    s_resets = 0;
    s_writes = s_reads = 0;
    s_rst_level = true;
    s_tx_completes = true;
    s_open = false;
}

void lora_port_host_set_version(uint8_t v) { s_version = v; }

uint8_t lora_port_host_reg(int reg)
{
    return (reg >= 0 && reg < LORA_HOST_NUM_REGS) ? s_regs[reg] : 0;
}

void lora_port_host_set_reg(int reg, uint8_t v)
{
    if (reg >= 0 && reg < LORA_HOST_NUM_REGS) s_regs[reg] = v;
}

size_t lora_port_host_fifo_take(uint8_t *buf, size_t len)
{
    size_t n = (len < s_fifo_written) ? len : s_fifo_written;

    memcpy(buf, s_fifo, n);
    return n;
}

void lora_port_host_stage_rx(const uint8_t *data, uint8_t len)
{
    memcpy(s_fifo, data, len);
    s_regs[H_REG_RX_NB_BYTES] = len;
    s_regs[H_REG_IRQ_FLAGS] |= H_IRQ_RX_DONE;
}

void lora_port_host_set_tx_completes(bool completes) { s_tx_completes = completes; }

uint32_t lora_port_host_millis(void) { return s_millis; }

uint32_t lora_port_host_resets(void) { return s_resets; }
uint32_t lora_port_host_writes(void) { return s_writes; }
uint32_t lora_port_host_reads(void)  { return s_reads; }

void lora_port_host_advance_ms(uint32_t ms) { s_millis += ms; }

bool lora_port_init(const lora_config_t *cfg)
{
    (void)cfg;
    s_open = true;
    return true;
}

void lora_port_deinit(void)
{
    s_open = false;
}

void lora_port_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    uint8_t addr;
    bool write;

    if (!s_open || len < 2) return;

    addr = tx[0] & 0x7Fu;
    write = (tx[0] & 0x80u) != 0;

    if (write) {
        s_writes++;
        if (addr == H_REG_FIFO) {
            /* Writing the FIFO register stores at the pointer and advances it. */
            uint8_t ptr = s_regs[H_REG_FIFO_ADDR_PTR];
            s_fifo[ptr] = tx[1];
            s_regs[H_REG_FIFO_ADDR_PTR] = (uint8_t)(ptr + 1u);
            if ((size_t)ptr + 1u > s_fifo_written) s_fifo_written = (size_t)ptr + 1u;
        } else if (addr == H_REG_IRQ_FLAGS) {
            /* Interrupt flags are write-1-to-clear. */
            s_regs[addr] &= (uint8_t)~tx[1];
        } else {
            s_regs[addr] = tx[1];
            /* Entering transmit mode completes immediately here; a real chip
             * raises TX_DONE when the packet is off the air. */
            if (s_tx_completes && addr == H_REG_OP_MODE && (tx[1] & 0x07u) == H_MODE_TX) {
                s_regs[H_REG_IRQ_FLAGS] |= H_IRQ_TX_DONE;
            }
        }
        if (rx) { rx[0] = 0; rx[1] = 0; }
        return;
    }

    s_reads++;
    if (rx) {
        rx[0] = 0;
        if (addr == H_REG_VERSION) {
            rx[1] = s_version;
        } else if (addr == H_REG_FIFO) {
            uint8_t ptr = s_regs[H_REG_FIFO_ADDR_PTR];
            rx[1] = s_fifo[ptr];
            s_regs[H_REG_FIFO_ADDR_PTR] = (uint8_t)(ptr + 1u);
        } else {
            rx[1] = s_regs[addr];
        }
    }
}

void lora_port_reset_pin(bool high)
{
    /* Count the falling edge, which is what actually resets the chip. */
    if (s_rst_level && !high) s_resets++;
    s_rst_level = high;
}

void lora_port_delay_ms(uint32_t ms) { s_millis += ms; }

uint32_t lora_port_millis(void) { return s_millis; }

#endif /* LORA_TARGET_HOST */
