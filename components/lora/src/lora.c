/*
 * lora.c - Semtech SX1276/77/78/79 register logic. See include/lora.h.
 *
 * Derived from Inteform's esp32-lora-library, which is a port of
 * sandeepmistry's arduino-LoRa. The register sequences and the modem
 * configuration maths are that code's, unchanged; what changed is that SPI and
 * GPIO now go through lora_port.h instead of calling ESP-IDF directly, so the
 * same file builds for the Pico.
 */

#include <stdio.h>
#include <string.h>

#include "lora.h"
#include "lora_port.h"

/*
 * Register definitions
 */
#define REG_FIFO                       0x00
#define REG_OP_MODE                    0x01
#define REG_FRF_MSB                    0x06
#define REG_FRF_MID                    0x07
#define REG_FRF_LSB                    0x08
#define REG_PA_CONFIG                  0x09
#define REG_LNA                        0x0c
#define REG_FIFO_ADDR_PTR              0x0d
#define REG_FIFO_TX_BASE_ADDR          0x0e
#define REG_FIFO_RX_BASE_ADDR          0x0f
#define REG_FIFO_RX_CURRENT_ADDR       0x10
#define REG_IRQ_FLAGS                  0x12
#define REG_RX_NB_BYTES                0x13
#define REG_PKT_SNR_VALUE              0x19
#define REG_PKT_RSSI_VALUE             0x1a
#define REG_MODEM_CONFIG_1             0x1d
#define REG_MODEM_CONFIG_2             0x1e
#define REG_PREAMBLE_MSB               0x20
#define REG_PREAMBLE_LSB               0x21
#define REG_PAYLOAD_LENGTH             0x22
#define REG_MODEM_CONFIG_3             0x26
#define REG_RSSI_WIDEBAND              0x2c
#define REG_DETECTION_OPTIMIZE         0x31
#define REG_DETECTION_THRESHOLD        0x37
#define REG_SYNC_WORD                  0x39
#define REG_DIO_MAPPING_1              0x40
#define REG_VERSION                    0x42

/*
 * Transceiver modes
 */
#define MODE_LONG_RANGE_MODE           0x80
#define MODE_SLEEP                     0x00
#define MODE_STDBY                     0x01
#define MODE_TX                        0x03
#define MODE_RX_CONTINUOUS             0x05
#define MODE_RX_SINGLE                 0x06

/*
 * PA configuration
 */
#define PA_BOOST                       0x80

/*
 * IRQ masks
 */
#define IRQ_TX_DONE_MASK               0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK     0x20
#define IRQ_RX_DONE_MASK               0x40

#define PA_OUTPUT_RFO_PIN              0
#define PA_OUTPUT_PA_BOOST_PIN         1

/* The SX127x is ready well inside a second; the original spun 100 times with a
 * 2-tick delay, which came to about the same on a default 100 Hz tick. */
#define TIMEOUT_RESET_MS               1000

/* A transmission that never reports TX_DONE must not wedge the caller. At the
 * slowest settings (SF12, 125 kHz) a maximum-length packet is a few seconds. */
#define TIMEOUT_TX_MS                  10000

static int  __implicit;
static long __frequency;
static bool __initialized;

/**
 * Write a value to a register.
 * @param reg Register index.
 * @param val Value to write.
 */
void
lora_write_reg(int reg, int val)
{
   uint8_t out[2] = { (uint8_t)(0x80 | reg), (uint8_t)val };
   uint8_t in[2];

   lora_port_transfer(out, in, sizeof(out));
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 */
int
lora_read_reg(int reg)
{
   uint8_t out[2] = { (uint8_t)reg, 0xff };
   uint8_t in[2];

   lora_port_transfer(out, in, sizeof(out));
   return in[1];
}

/**
 * Perform physical reset on the Lora chip
 */
void
lora_reset(void)
{
   lora_port_reset_pin(false);
   lora_port_delay_ms(1);
   lora_port_reset_pin(true);
   lora_port_delay_ms(10);
}

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
void
lora_explicit_header_mode(void)
{
   __implicit = 0;
   lora_write_reg(REG_MODEM_CONFIG_1, lora_read_reg(REG_MODEM_CONFIG_1) & 0xfe);
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
void
lora_implicit_header_mode(int size)
{
   __implicit = 1;
   lora_write_reg(REG_MODEM_CONFIG_1, lora_read_reg(REG_MODEM_CONFIG_1) | 0x01);
   lora_write_reg(REG_PAYLOAD_LENGTH, size);
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
void
lora_idle(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
void
lora_sleep(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
void
lora_receive(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
void
lora_set_tx_power(int level)
{
   // RF9x module uses PA_BOOST pin
   if (level < 2) level = 2;
   else if (level > 17) level = 17;
   lora_write_reg(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
void
lora_set_frequency(long frequency)
{
   __frequency = frequency;

   uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

   lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
   lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
   lora_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
void
lora_set_spreading_factor(int sf)
{
   if (sf < 6) sf = 6;
   else if (sf > 12) sf = 12;

   if (sf == 6) {
      lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
      lora_write_reg(REG_DETECTION_THRESHOLD, 0x0c);
   } else {
      lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
      lora_write_reg(REG_DETECTION_THRESHOLD, 0x0a);
   }

   lora_write_reg(REG_MODEM_CONFIG_2, (lora_read_reg(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

/**
 * Set bandwidth (bit rate)
 * @param sbw Bandwidth in Hz (up to 500000)
 */
void
lora_set_bandwidth(long sbw)
{
   int bw;

   /* Integer thresholds, same values the original wrote as floats. */
   if (sbw <= 7800L) bw = 0;
   else if (sbw <= 10400L) bw = 1;
   else if (sbw <= 15600L) bw = 2;
   else if (sbw <= 20800L) bw = 3;
   else if (sbw <= 31250L) bw = 4;
   else if (sbw <= 41700L) bw = 5;
   else if (sbw <= 62500L) bw = 6;
   else if (sbw <= 125000L) bw = 7;
   else if (sbw <= 250000L) bw = 8;
   else bw = 9;
   lora_write_reg(REG_MODEM_CONFIG_1, (lora_read_reg(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

/**
 * Set coding rate
 * @param denominator 5-8, Denominator for the coding rate 4/x
 */
void
lora_set_coding_rate(int denominator)
{
   if (denominator < 5) denominator = 5;
   else if (denominator > 8) denominator = 8;

   int cr = denominator - 4;
   lora_write_reg(REG_MODEM_CONFIG_1, (lora_read_reg(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
void
lora_set_preamble_length(long length)
{
   lora_write_reg(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
   lora_write_reg(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
void
lora_set_sync_word(int sw)
{
   lora_write_reg(REG_SYNC_WORD, sw);
}

/**
 * Enable appending/verifying packet CRC.
 */
void
lora_enable_crc(void)
{
   lora_write_reg(REG_MODEM_CONFIG_2, lora_read_reg(REG_MODEM_CONFIG_2) | 0x04);
}

/**
 * Disable appending/verifying packet CRC.
 */
void
lora_disable_crc(void)
{
   lora_write_reg(REG_MODEM_CONFIG_2, lora_read_reg(REG_MODEM_CONFIG_2) & 0xfb);
}

/**
 * Perform hardware initialization with explicit pins.
 * @return 1 on success, 0 if the bus or the radio did not come up.
 */
int
lora_init_config(const lora_config_t *cfg)
{
   uint32_t start;
   int version = 0;

   if (!cfg) return 0;
   if (__initialized) return 1;

   if (!lora_port_init(cfg)) {
      LORA_LOGE("spi init failed");
      return 0;
   }

   /*
    * Perform hardware reset.
    */
   lora_reset();

   /*
    * Check version. A missing or miswired radio reads back 0x00 or 0xff
    * forever, so this is the point where bad wiring is caught.
    */
   start = lora_port_millis();
   for (;;) {
      version = lora_read_reg(REG_VERSION);
      if (version == 0x12) break;
      if (lora_port_millis() - start >= TIMEOUT_RESET_MS) {
         LORA_LOGE("no SX127x found: version register reads 0x%02X, expected 0x12",
                   (unsigned)version);
         lora_port_deinit();
         return 0;
      }
      lora_port_delay_ms(2);
   }

   __initialized = true;

   /*
    * Default configuration.
    */
   lora_sleep();
   lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
   lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0);
   lora_write_reg(REG_LNA, lora_read_reg(REG_LNA) | 0x03);
   lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
   lora_set_tx_power(17);

   lora_idle();

   LORA_LOGI("SX127x ready on spi%d (cs=%d rst=%d sck=%d mosi=%d miso=%d)",
             cfg->spi_id, cfg->cs_pin, cfg->rst_pin,
             cfg->sck_pin, cfg->mosi_pin, cfg->miso_pin);
   return 1;
}

/**
 * Perform hardware initialization using the compile-time defaults.
 */
int
lora_init(void)
{
   lora_config_t cfg = LORA_CONFIG_DEFAULT();

   return lora_init_config(&cfg);
}

int
lora_initialized(void)
{
   return __initialized ? 1 : 0;
}

/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 */
void
lora_send_packet(uint8_t *buf, int size)
{
   uint32_t start;

   /*
    * Transfer data to radio.
    */
   lora_idle();
   lora_write_reg(REG_FIFO_ADDR_PTR, 0);

   for(int i=0; i<size; i++)
      lora_write_reg(REG_FIFO, *buf++);

   lora_write_reg(REG_PAYLOAD_LENGTH, size);

   /*
    * Start transmission and wait for conclusion.
    */
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

   start = lora_port_millis();
   while((lora_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
      /* The original looped forever here. A radio that browns out mid-transmit
       * would hang the calling task; give up instead. */
      if (lora_port_millis() - start >= TIMEOUT_TX_MS) {
         LORA_LOGW("transmit timed out after %d ms", TIMEOUT_TX_MS);
         lora_idle();
         return;
      }
      lora_port_delay_ms(2);
   }

   lora_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}

/**
 * Read a received packet.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @return Number of bytes received (zero if no packet available).
 */
int
lora_receive_packet(uint8_t *buf, int size)
{
   int len = 0;

   /*
    * Check interrupts.
    */
   int irq = lora_read_reg(REG_IRQ_FLAGS);
   lora_write_reg(REG_IRQ_FLAGS, irq);
   if((irq & IRQ_RX_DONE_MASK) == 0) return 0;
   if(irq & IRQ_PAYLOAD_CRC_ERROR_MASK) return 0;

   /*
    * Find packet size.
    */
   if (__implicit) len = lora_read_reg(REG_PAYLOAD_LENGTH);
   else len = lora_read_reg(REG_RX_NB_BYTES);

   /*
    * Transfer data from radio.
    */
   lora_idle();
   lora_write_reg(REG_FIFO_ADDR_PTR, lora_read_reg(REG_FIFO_RX_CURRENT_ADDR));
   if(len > size) len = size;
   for(int i=0; i<len; i++)
      *buf++ = (uint8_t)lora_read_reg(REG_FIFO);

   return len;
}

/**
 * Returns non-zero if there is data to read (packet received).
 */
int
lora_received(void)
{
   if(lora_read_reg(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) return 1;
   return 0;
}

/**
 * Return last packet's RSSI.
 */
int
lora_packet_rssi(void)
{
   return (lora_read_reg(REG_PKT_RSSI_VALUE) - (__frequency < 868000000L ? 164 : 157));
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
float
lora_packet_snr(void)
{
   return ((int8_t)lora_read_reg(REG_PKT_SNR_VALUE)) * 0.25f;
}

/**
 * Shutdown hardware.
 */
void
lora_close(void)
{
   if (!__initialized) return;

   lora_sleep();
   /* The original left the bus and pins claimed here, with a FIXME. Releasing
    * them lets the SPI bus be reused and makes a re-init work. */
   lora_port_deinit();
   __initialized = false;
   __implicit = 0;
   __frequency = 0;
}

void
lora_dump_registers(void)
{
   int i;
   printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
   for(i=0; i<0x40; i++) {
      printf("%02X ", lora_read_reg(i));
      if((i & 0x0f) == 0x0f) printf("\n");
   }
   printf("\n");
}
