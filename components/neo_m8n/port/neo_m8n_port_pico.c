/*
 * neo_m8n_port_pico.c - Raspberry Pi Pico / RP2040 / RP2350 port.
 *
 * The RP2040's UART FIFO holds 32 bytes. A 1 Hz NMEA burst is several hundred,
 * which at 9600 baud takes roughly half a second to arrive - so polling the
 * FIFO directly would need the application to come back every ~30 ms forever,
 * and any longer piece of work (an SD card write, a LoRa transmit) would drop
 * bytes mid-sentence.
 *
 * This port therefore drains the FIFO from the UART interrupt into a ring
 * buffer. The application can then take its time.
 *
 * The interrupt is installed on whichever core calls neo_m8n_init(); read from
 * that same core.
 */

#include "neo_m8n_port.h"

#if NEO_M8N_TARGET_PICO

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/time.h"

/* Must be a power of two: the ring indices are masked, not divided. Holds
 * about two seconds of 1 Hz traffic. Override with -DNEO_M8N_PICO_RX_RING=... */
#ifndef NEO_M8N_PICO_RX_RING
#define NEO_M8N_PICO_RX_RING 1024
#endif
#if (NEO_M8N_PICO_RX_RING & (NEO_M8N_PICO_RX_RING - 1)) != 0
#error "NEO_M8N_PICO_RX_RING must be a power of two"
#endif
#define RING_MASK (NEO_M8N_PICO_RX_RING - 1)

static uart_inst_t *s_uart;
static uint         s_irq;
static bool         s_open;

static uint8_t           s_ring[NEO_M8N_PICO_RX_RING];
static volatile uint16_t s_head;   /* written by the ISR only */
static volatile uint16_t s_tail;   /* written by the reader only */
static volatile uint32_t s_dropped;

static void rx_isr(void)
{
    /* Empty the FIFO in one go; draining it is also what clears the receive
     * and receive-timeout interrupts. */
    while (uart_is_readable(s_uart)) {
        uint8_t c = (uint8_t)uart_getc(s_uart);
        uint16_t next = (uint16_t)((s_head + 1u) & RING_MASK);

        if (next == s_tail) {
            /* Full. Drop the newest byte rather than overwrite the oldest: a
             * half-written sentence at the tail is worse than a missing one. */
            s_dropped++;
            continue;
        }
        s_ring[s_head] = c;
        s_head = next;
    }
}

static size_t ring_take(uint8_t *buf, size_t len)
{
    size_t n = 0;

    while (n < len && s_tail != s_head) {
        buf[n++] = s_ring[s_tail];
        s_tail = (uint16_t)((s_tail + 1u) & RING_MASK);
    }
    return n;
}

neo_m8n_err_t neo_m8n_port_init(const neo_m8n_config_t *cfg)
{
    if (cfg->uart_id != 0 && cfg->uart_id != 1) return NEO_M8N_ERR_INVALID_ARG;

    s_uart = (cfg->uart_id == 1) ? uart1 : uart0;
    s_irq  = (cfg->uart_id == 1) ? UART1_IRQ : UART0_IRQ;
    s_head = s_tail = 0;
    s_dropped = 0;

    uart_init(s_uart, cfg->baud);
    uart_set_format(s_uart, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(s_uart, false, false);
    uart_set_fifo_enabled(s_uart, true);
    /* The driver frames sentences itself, so leave the SDK's CR/LF translation
     * off - it would rewrite the line endings out from under the parser. */
    uart_set_translate_crlf(s_uart, false);

    gpio_set_function((uint)cfg->rx_pin, GPIO_FUNC_UART);
    if (cfg->tx_pin >= 0) gpio_set_function((uint)cfg->tx_pin, GPIO_FUNC_UART);

    irq_set_exclusive_handler(s_irq, rx_isr);
    irq_set_enabled(s_irq, true);
    uart_set_irq_enables(s_uart, true, false);   /* RX and RX-timeout only */

    s_open = true;
    return NEO_M8N_OK;
}

void neo_m8n_port_deinit(void)
{
    if (!s_open) return;

    uart_set_irq_enables(s_uart, false, false);
    irq_set_enabled(s_irq, false);
    irq_remove_handler(s_irq, rx_isr);
    uart_deinit(s_uart);

    s_head = s_tail = 0;
    s_open = false;
}

int neo_m8n_port_read(uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    absolute_time_t deadline;
    size_t n;

    if (!s_open) return NEO_M8N_ERR_STATE;

    n = ring_take(buf, len);
    if (n > 0 || timeout_ms == 0) return (int)n;

    /* Nothing buffered: wait for the ISR to deliver something, then return it
     * without holding out for a full buffer. */
    deadline = make_timeout_time_ms(timeout_ms);
    while (s_tail == s_head) {
        if (time_reached(deadline)) return 0;
        tight_loop_contents();
    }
    return (int)ring_take(buf, len);
}

int neo_m8n_port_write(const uint8_t *buf, size_t len)
{
    if (!s_open) return NEO_M8N_ERR_STATE;

    uart_write_blocking(s_uart, buf, len);
    return (int)len;
}

neo_m8n_err_t neo_m8n_port_set_baud(uint32_t baud)
{
    if (!s_open) return NEO_M8N_ERR_STATE;

    /* Let the command that told the module to switch finish going out at the
     * old rate before the divisor moves. */
    uart_tx_wait_blocking(s_uart);
    uart_set_baudrate(s_uart, baud);
    return NEO_M8N_OK;
}

void neo_m8n_port_flush_input(void)
{
    if (!s_open) return;

    /* Stop the ISR before touching the indices it owns. */
    uart_set_irq_enables(s_uart, false, false);
    while (uart_is_readable(s_uart)) (void)uart_getc(s_uart);
    s_head = s_tail = 0;
    uart_set_irq_enables(s_uart, true, false);
}

uint32_t neo_m8n_port_millis(void)
{
    return to_ms_since_boot(get_absolute_time());
}

void neo_m8n_port_delay_ms(uint32_t ms)
{
    sleep_ms(ms);
}

/* Bytes lost to a full ring since init: if this climbs, the application is not
 * calling neo_m8n_read()/neo_m8n_poll() often enough, or the ring is too small
 * for the configured update rate. */
uint32_t neo_m8n_port_pico_dropped(void)
{
    return s_dropped;
}

#endif /* NEO_M8N_TARGET_PICO */
