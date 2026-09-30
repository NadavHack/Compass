/*
 * neo_m8n_port_host.c - host/test port. Not for hardware.
 *
 * Selected when neither ESP-IDF nor the Pico SDK is in play. See
 * neo_m8n_port_host.h.
 */

#include "neo_m8n_port.h"

#if NEO_M8N_TARGET_HOST

#include <string.h>

#include "neo_m8n_port_host.h"

#define HOST_QUEUE 8192

static uint8_t  s_rx[HOST_QUEUE];
static size_t   s_rx_head, s_rx_tail;
static uint8_t  s_tx[HOST_QUEUE];
static size_t   s_tx_head, s_tx_tail;
static uint32_t s_millis;
static uint32_t s_baud;
static bool     s_open;

/* Delivered into the RX queue by the next flush; see the header. */
static uint8_t  s_pending[HOST_QUEUE];
static size_t   s_pending_len;

void neo_m8n_port_host_reset(void)
{
    s_rx_head = s_rx_tail = 0;
    s_tx_head = s_tx_tail = 0;
    s_millis = 0;
    s_baud = 0;
    s_open = false;
    s_pending_len = 0;
}

void neo_m8n_port_host_inject(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;

    for (size_t i = 0; i < len && s_rx_head < HOST_QUEUE; i++) {
        s_rx[s_rx_head++] = p[i];
    }
}

void neo_m8n_port_host_inject_str(const char *s)
{
    neo_m8n_port_host_inject(s, strlen(s));
}

void neo_m8n_port_host_inject_after_flush(const char *s)
{
    size_t len = strlen(s);

    if (len > sizeof(s_pending)) len = sizeof(s_pending);
    memcpy(s_pending, s, len);
    s_pending_len = len;
}

size_t neo_m8n_port_host_take_tx(uint8_t *buf, size_t len)
{
    size_t n = 0;

    while (n < len && s_tx_tail < s_tx_head) buf[n++] = s_tx[s_tx_tail++];
    return n;
}

size_t neo_m8n_port_host_tx_len(void)
{
    return s_tx_head - s_tx_tail;
}

uint32_t neo_m8n_port_host_baud(void)
{
    return s_baud;
}

void neo_m8n_port_host_advance_ms(uint32_t ms)
{
    s_millis += ms;
}

neo_m8n_err_t neo_m8n_port_init(const neo_m8n_config_t *cfg)
{
    s_open = true;
    s_baud = cfg->baud;
    return NEO_M8N_OK;
}

void neo_m8n_port_deinit(void)
{
    s_open = false;
}

int neo_m8n_port_read(uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    size_t n = 0;

    if (!s_open) return NEO_M8N_ERR_STATE;

    while (n < len && s_rx_tail < s_rx_head) buf[n++] = s_rx[s_rx_tail++];

    /* Nothing queued: burn the timeout so the caller's deadline is reached
     * instead of spinning forever against a clock that never moves. */
    if (n == 0) s_millis += timeout_ms;

    return (int)n;
}

int neo_m8n_port_write(const uint8_t *buf, size_t len)
{
    size_t n = 0;

    if (!s_open) return NEO_M8N_ERR_STATE;
    while (n < len && s_tx_head < HOST_QUEUE) s_tx[s_tx_head++] = buf[n++];
    return (int)n;
}

neo_m8n_err_t neo_m8n_port_set_baud(uint32_t baud)
{
    s_baud = baud;
    return NEO_M8N_OK;
}

void neo_m8n_port_flush_input(void)
{
    s_rx_head = s_rx_tail = 0;

    if (s_pending_len) {
        neo_m8n_port_host_inject(s_pending, s_pending_len);
        s_pending_len = 0;
    }
}

uint32_t neo_m8n_port_millis(void)
{
    return s_millis;
}

void neo_m8n_port_delay_ms(uint32_t ms)
{
    s_millis += ms;
}

#endif /* NEO_M8N_TARGET_HOST */
