# lora — Semtech SX1276/77/78/79 driver

One component, two targets. The register logic and modem maths are shared
source; only the SPI access differs, and that lives behind six functions in
`port/`.

```
include/lora.h          public API
src/lora.c              register sequences and modem maths   (portable)
port/lora_port.h        the platform contract
port/lora_port_esp32.c  ESP-IDF
port/lora_port_pico.c   Pico SDK (RP2040 / RP2350)
port/lora_port_host.c   simulated SX127x for the host tests
test/run_tests.sh       builds and runs the tests with plain gcc
```

## Provenance

This started as [Inteform's esp32-lora-library](https://github.com/Inteform/esp32-lora-library),
which is itself a port of [sandeepmistry's arduino-LoRa](https://github.com/sandeepmistry/arduino-LoRa).
Those register sequences — the frequency synthesiser maths, the modem config
masks, the FIFO handling — are unchanged and carry years of field use behind
them. What changed here is the structure: SPI and GPIO now go through
`lora_port.h` instead of calling ESP-IDF directly, so the same file builds for
the Pico.

## The target macro

`CMakeLists.txt` defines exactly one of `LORA_TARGET_ESP32`, `LORA_TARGET_PICO`
or `LORA_TARGET_HOST`, and each port file is wrapped in `#if` on its own, so
compiling all three is harmless. `lora.h` also infers the target from the SDK's
own macros (`ESP_PLATFORM`, `PICO_RP2040`) when the build system has not said,
which is what makes the header resolve in an editor or a unit test.

This matches `components/neo_m8n`, deliberately: same layout, same macro scheme,
same `port/` contract idea.

## Behaviour changes from upstream

Three, all in the direction of not taking the firmware down with the radio:

- **`lora_init()` returns 0 instead of aborting.** Upstream used `assert()` when
  SPI failed or the chip did not report version `0x12`, which panics the
  firmware. A tracker that cannot find its radio should be able to say so and
  carry on logging GPS.
- **`lora_send_packet()` gives up after 10 s.** Upstream spun on the TX_DONE
  flag forever, so a radio that browned out mid-transmit would wedge the calling
  task.
- **`lora_close()` releases the bus.** Upstream left it claimed, with a `FIXME`.
  Releasing it means the SPI bus can be reused and a re-init works.

The API is otherwise unchanged, so existing callers keep compiling. Two
additions: `lora_init_config()` takes pins explicitly, and `lora_initialized()`
reports state.

## Wiring

3.3 V SPI. Chip select is driven by software on both targets, because the
SX127x needs CS held low across the address byte and the data byte of one
register access, and neither chip's hardware CS does that by default.

| SX127x | ESP32 (menuconfig default) | Pico (default) |
|--------|----------------------------|----------------|
| VCC    | 3V3                        | 3V3            |
| GND    | GND                        | GND            |
| SCK    | GPIO14                     | GP18           |
| MOSI   | GPIO12                     | GP19           |
| MISO   | GPIO13                     | GP16           |
| NSS    | GPIO15                     | GP17           |
| NRST   | GPIO32                     | GP20           |

Set `rst_pin` to `-1` (or `CONFIG_RST_GPIO=-1`) if the reset line is not wired.

## Use

### ESP-IDF

`EXTRA_COMPONENT_DIRS` already points at `components/`, so nothing changes:

```cmake
idf_component_register(SRCS "main.c" INCLUDE_DIRS "." REQUIRES lora)
```

Pins are in `idf.py menuconfig` under **LoRa Configuration**. The symbol names
are the original unprefixed ones (`CONFIG_CS_GPIO` and friends), so saved
`sdkconfig` values still apply.

```c
if (!lora_init()) { /* no radio on the bus */ }
lora_set_frequency(868E6);
lora_enable_crc();
lora_send_packet((uint8_t *)"hello", 5);
```

### Raspberry Pi Pico

`Tracker/CMakeLists.txt` already does this:

```cmake
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../components/lora lora)
target_link_libraries(Tracker pico_stdlib lora)
```

There is no menuconfig, so pass the pins:

```c
lora_config_t cfg = LORA_CONFIG_DEFAULT();
cfg.spi_id = 0;          /* spi0 */
cfg.cs_pin = 17;
if (!lora_init_config(&cfg)) { /* no radio on the bus */ }
```

### Receiving

```c
lora_receive();
for (;;) {
    uint8_t buf[256];
    if (lora_received()) {
        int n = lora_receive_packet(buf, sizeof(buf));
        printf("%d bytes, RSSI %d dBm, SNR %.2f\n",
               n, lora_packet_rssi(), lora_packet_snr());
        lora_receive();
    }
}
```

`lora_receive_packet()` returns 0 both when no packet is waiting and when one
arrived with a bad CRC, so a 0 is not an error.

Both ends must agree on frequency, spreading factor, bandwidth, coding rate,
preamble length, sync word and CRC setting, or they will not hear each other.

## Tests

```sh
components/lora/test/run_tests.sh
```

Builds `src/lora.c` against a simulated SX127x register file and runs 98
assertions: the init sequence and its default register values, the missing-radio
path, frequency synthesiser values for 433/868/915 MHz checked against
independently computed constants, spreading factor including the SF6 special
case, bandwidth boundaries, coding rate, power clamping, CRC and header-mode
bits preserving the rest of their shared registers, FIFO round-trips for send
and receive, truncation of an over-long packet, CRC-error discard, and the
RSSI/SNR conversions.

What the tests cannot cover: real SPI timing, the reset pulse width, and
whether two radios actually talk to each other over the air.
