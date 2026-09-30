/*
 * neo_m8n_port_esp32.c - ESP-IDF port (ESP32 and friends).
 *
 * Uses the UART driver's own background ISR and ring buffer, so the
 * application only has to call into the driver often enough to keep that
 * buffer from filling.
 */

#include "neo_m8n_port.h"

#if NEO_M8N_TARGET_ESP32

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static uart_port_t s_uart = UART_NUM_MAX;

/* uart_driver_install() rejects anything smaller than the hardware FIFO. */
#define PORT_MIN_RX_BUF 256

neo_m8n_err_t neo_m8n_port_init(const neo_m8n_config_t *cfg)
{
    uart_config_t uc = {
        .baud_rate  = (int)cfg->baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int rx_buf = (int)cfg->rx_buf_size;
    esp_err_t err;

    if (cfg->uart_id < 0 || cfg->uart_id >= UART_NUM_MAX) return NEO_M8N_ERR_INVALID_ARG;
    if (rx_buf < PORT_MIN_RX_BUF) rx_buf = PORT_MIN_RX_BUF;

    s_uart = (uart_port_t)cfg->uart_id;

    /* TX buffer 0 makes writes blocking, which is what the UBX configuration
     * exchange wants: the frame must be on the wire before the ACK can come. */
    err = uart_driver_install(s_uart, rx_buf, 0, 0, NULL, 0);
    if (err != ESP_OK) goto fail;

    err = uart_param_config(s_uart, &uc);
    if (err != ESP_OK) goto fail_installed;

    err = uart_set_pin(s_uart,
                       cfg->tx_pin >= 0 ? cfg->tx_pin : UART_PIN_NO_CHANGE,
                       cfg->rx_pin,
                       UART_PIN_NO_CHANGE,   /* RTS */
                       UART_PIN_NO_CHANGE);  /* CTS */
    if (err != ESP_OK) goto fail_installed;

    uart_flush_input(s_uart);
    return NEO_M8N_OK;

fail_installed:
    uart_driver_delete(s_uart);
fail:
    NEO_M8N_LOGE("uart setup failed: %s", esp_err_to_name(err));
    s_uart = UART_NUM_MAX;
    return NEO_M8N_ERR_IO;
}

void neo_m8n_port_deinit(void)
{
    if (s_uart == UART_NUM_MAX) return;
    uart_driver_delete(s_uart);
    s_uart = UART_NUM_MAX;
}

int neo_m8n_port_read(uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    int n;

    if (s_uart == UART_NUM_MAX) return NEO_M8N_ERR_STATE;

    /* Returns as soon as it has something; it does not wait to fill `len`. */
    n = uart_read_bytes(s_uart, buf, (uint32_t)len, pdMS_TO_TICKS(timeout_ms));
    if (n < 0) return NEO_M8N_ERR_IO;
    return n;
}

int neo_m8n_port_write(const uint8_t *buf, size_t len)
{
    int n;

    if (s_uart == UART_NUM_MAX) return NEO_M8N_ERR_STATE;

    n = uart_write_bytes(s_uart, buf, len);
    if (n < 0) return NEO_M8N_ERR_IO;
    return n;
}

neo_m8n_err_t neo_m8n_port_set_baud(uint32_t baud)
{
    if (s_uart == UART_NUM_MAX) return NEO_M8N_ERR_STATE;

    /* The command telling the module to switch must finish going out at the
     * old rate before the divisor changes underneath it. */
    if (uart_wait_tx_done(s_uart, pdMS_TO_TICKS(200)) != ESP_OK) return NEO_M8N_ERR_IO;
    if (uart_set_baudrate(s_uart, baud) != ESP_OK) return NEO_M8N_ERR_IO;
    return NEO_M8N_OK;
}

void neo_m8n_port_flush_input(void)
{
    if (s_uart != UART_NUM_MAX) uart_flush_input(s_uart);
}

uint32_t neo_m8n_port_millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void neo_m8n_port_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

#endif /* NEO_M8N_TARGET_ESP32 */
