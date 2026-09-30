/*
 * lora_port_pico.c - Raspberry Pi Pico / RP2040 / RP2350 port for the SX127x.
 *
 * Chip select is an ordinary GPIO rather than the SPI block's own CSn: the
 * RP2040 deasserts that line between bytes, and the SX127x needs it held low
 * across the address byte and the data byte of one register access.
 */

#include "lora_port.h"

#if LORA_TARGET_PICO

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/time.h"

static spi_inst_t *s_spi;
static int  s_cs  = -1;
static int  s_rst = -1;
static bool s_open;

bool lora_port_init(const lora_config_t *cfg)
{
    if (s_open) return true;
    if (cfg->spi_id != 0 && cfg->spi_id != 1) {
        LORA_LOGE("spi_id must be 0 or 1 on the Pico, got %d", cfg->spi_id);
        return false;
    }

    s_spi = (cfg->spi_id == 1) ? spi1 : spi0;
    s_cs = cfg->cs_pin;
    s_rst = cfg->rst_pin;

    spi_init(s_spi, cfg->spi_hz);
    /* Mode 0: idle-low clock, sample on the rising edge, MSB first. */
    spi_set_format(s_spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function((uint)cfg->sck_pin,  GPIO_FUNC_SPI);
    gpio_set_function((uint)cfg->mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function((uint)cfg->miso_pin, GPIO_FUNC_SPI);

    gpio_init((uint)s_cs);
    gpio_set_dir((uint)s_cs, GPIO_OUT);
    gpio_put((uint)s_cs, 1);            /* idle high */

    if (s_rst >= 0) {
        gpio_init((uint)s_rst);
        gpio_set_dir((uint)s_rst, GPIO_OUT);
        gpio_put((uint)s_rst, 1);
    }

    s_open = true;
    return true;
}

void lora_port_deinit(void)
{
    if (!s_open) return;

    spi_deinit(s_spi);
    s_open = false;
}

void lora_port_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    if (!s_open) return;

    gpio_put((uint)s_cs, 0);
    if (rx) spi_write_read_blocking(s_spi, tx, rx, len);
    else    spi_write_blocking(s_spi, tx, len);
    gpio_put((uint)s_cs, 1);
}

void lora_port_reset_pin(bool high)
{
    if (s_rst >= 0) gpio_put((uint)s_rst, high ? 1 : 0);
}

void lora_port_delay_ms(uint32_t ms)
{
    sleep_ms(ms);
}

uint32_t lora_port_millis(void)
{
    return to_ms_since_boot(get_absolute_time());
}

#endif /* LORA_TARGET_PICO */
