/*
 * lora_port_esp32.c - ESP-IDF port for the SX127x driver.
 *
 * Same bus setup the original component used: SPI at 9 MHz, mode 0, with the
 * chip select driven by software rather than by the peripheral, because the
 * SX127x wants CS held across a whole two-byte register access.
 */

#include "lora_port.h"

#if LORA_TARGET_ESP32

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static spi_device_handle_t s_spi;
static spi_host_device_t   s_host;
static int  s_cs  = -1;
static int  s_rst = -1;
static bool s_open;

bool lora_port_init(const lora_config_t *cfg)
{
    esp_err_t err;

    spi_bus_config_t bus = {
        .miso_io_num = cfg->miso_pin,
        .mosi_io_num = cfg->mosi_pin,
        .sclk_io_num = cfg->sck_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    spi_device_interface_config_t dev = {
        .clock_speed_hz = (int)cfg->spi_hz,
        .mode = 0,
        .spics_io_num = -1,     /* CS driven by hand, see below */
        .queue_size = 1,
        .flags = 0,
        .pre_cb = NULL,
    };

    if (s_open) return true;
    if (cfg->spi_id != 2 && cfg->spi_id != 3) {
        LORA_LOGE("spi_id must be 2 or 3 on ESP32, got %d", cfg->spi_id);
        return false;
    }
    s_host = (cfg->spi_id == 2) ? SPI2_HOST : SPI3_HOST;
    s_cs = cfg->cs_pin;
    s_rst = cfg->rst_pin;

    if (s_rst >= 0) {
        gpio_reset_pin(s_rst);
        gpio_set_direction(s_rst, GPIO_MODE_OUTPUT);
        gpio_set_level(s_rst, 1);
    }
    gpio_reset_pin(s_cs);
    gpio_set_direction(s_cs, GPIO_MODE_OUTPUT);
    gpio_set_level(s_cs, 1);        /* idle high */

    err = spi_bus_initialize(s_host, &bus, SPI_DMA_DISABLED);
    if (err != ESP_OK) {
        LORA_LOGE("spi_bus_initialize: %s", esp_err_to_name(err));
        return false;
    }
    err = spi_bus_add_device(s_host, &dev, &s_spi);
    if (err != ESP_OK) {
        LORA_LOGE("spi_bus_add_device: %s", esp_err_to_name(err));
        spi_bus_free(s_host);
        return false;
    }

    s_open = true;
    return true;
}

void lora_port_deinit(void)
{
    if (!s_open) return;

    spi_bus_remove_device(s_spi);
    spi_bus_free(s_host);
    s_spi = NULL;
    s_open = false;
}

void lora_port_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = {
        .flags = 0,
        .length = 8 * len,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    if (!s_open) return;

    gpio_set_level(s_cs, 0);
    spi_device_transmit(s_spi, &t);
    gpio_set_level(s_cs, 1);
}

void lora_port_reset_pin(bool high)
{
    if (s_rst >= 0) gpio_set_level(s_rst, high ? 1 : 0);
}

void lora_port_delay_ms(uint32_t ms)
{
    /* A tick is 10 ms by default, so short delays would round to nothing.
     * Spin for those: the radio needs a real 1 ms on its reset line. */
    if (ms < portTICK_PERIOD_MS) {
        esp_rom_delay_us(ms * 1000);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(ms));
}

uint32_t lora_port_millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

#endif /* LORA_TARGET_ESP32 */
