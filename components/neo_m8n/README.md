# neo_m8n — u-blox NEO-M8N GNSS driver

One component, two targets. The NMEA parser, the UBX framing and the fix
aggregation are shared source; only the UART access differs, and that lives
behind seven functions in `port/`.

```
include/neo_m8n.h        public API
include/nmea.h           NMEA 0183 parser, usable on its own
src/neo_m8n.c            framing, UBX configuration, fix aggregation  (portable)
src/nmea.c               sentence parsing                             (portable)
port/neo_m8n_port.h      the platform contract
port/neo_m8n_port_esp32.c    ESP-IDF
port/neo_m8n_port_pico.c     Pico SDK (RP2040 / RP2350)
port/neo_m8n_port_host.c     fake UART for the host tests
test/run_tests.sh        builds and runs the tests with plain gcc
```

## Why one component instead of two

The split was worth measuring rather than guessing. Counting comments, the
portable half — NMEA parser, UBX state machines, fix aggregation, public
headers — is about 1,800 lines, none of which touch hardware. What actually
differs between an ESP32 and a Pico is opening a UART, reading it with a
timeout, writing it, changing its baud rate and reading a millisecond clock:
**124 lines for ESP-IDF and 184 for the Pico**, the difference being the Pico's
interrupt ring buffer.

Duplicating 1,800 lines of parser into `components/gps_esp32` and
`components/gps_pico` would mean fixing every parser bug twice, so the code is
shared and the target picks a port file. If a future module needs genuinely
different logic rather than a different UART, that one is worth its own
directory — this one is not.

## The target macro

`CMakeLists.txt` defines exactly one of `NEO_M8N_TARGET_ESP32`,
`NEO_M8N_TARGET_PICO` or `NEO_M8N_TARGET_HOST`, and each port file is wrapped in
`#if` on its own. Compiling all three is harmless — two of them compile to
nothing.

`neo_m8n.h` also infers the target from the SDK's own macros (`ESP_PLATFORM`,
`PICO_RP2040`) when the build system has not said. That is what makes the header
resolve in an editor or a unit test rather than erroring out.

Adding a third MCU means one new `port/neo_m8n_port_<target>.c` and one branch
in `CMakeLists.txt`. Nothing above the port layer changes.

## Wiring

The module is a 3.3 V UART device. Cross the data lines:

| NEO-M8N | ESP32 (default)   | Pico (default) |
|---------|-------------------|----------------|
| VCC     | 3V3               | 3V3 (pin 36)   |
| GND     | GND               | GND            |
| TX      | GPIO16 (MCU RX)   | GP5 (MCU RX)   |
| RX      | GPIO17 (MCU TX)   | GP4 (MCU TX)   |

TX is only needed to configure the receiver. Leave it unconnected, set
`tx_pin = -1` and `auto_config = false`, and the driver runs receive-only on the
module's factory defaults.

Give the module its own clear view of the sky and keep the ceramic patch antenna
away from switching regulators and the LoRa module — a NEO-M8N that cannot see
satellites still emits perfectly valid sentences, just with empty position
fields, which looks exactly like a software bug.

## Use

### Raspberry Pi Pico

`Tracker/CMakeLists.txt` already does this:

```cmake
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../components/neo_m8n neo_m8n)
target_link_libraries(Tracker pico_stdlib neo_m8n)
```

### ESP-IDF

`EXTRA_COMPONENT_DIRS` in `Compass/CMakeLists.txt` and `Reciever/CMakeLists.txt`
already points at `components/`, so the component is found automatically. Add it
to the requirements of whichever component uses it:

```cmake
idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES lora neo_m8n)
```

Pins and the initial baud rate are in `idf.py menuconfig` under
**NEO-M8N GPS Configuration**. (`MINIMAL_BUILD` is on in both projects, so a
component that nothing `REQUIRES` is not built at all.)

### Code

```c
#include "neo_m8n.h"

neo_m8n_config_t cfg = NEO_M8N_CONFIG_DEFAULT();
if (neo_m8n_init(&cfg) != NEO_M8N_OK) { /* the UART failed */ }

neo_m8n_fix_t fix;
for (;;) {
    if (neo_m8n_read(&fix, 1500) != NEO_M8N_OK) {
        continue;                      /* nothing arrived — check the wiring */
    }
    if (!fix.valid) {
        continue;                      /* still acquiring */
    }
    printf("%.6f, %.6f  %.1f m  %u sats\n",
           fix.latitude, fix.longitude, fix.altitude_m, fix.satellites_used);
}
```

`neo_m8n_read()` returning `NEO_M8N_OK` means a sentence was decoded, not that
there is a fix — always check `fix.valid`. A cold start takes 25–30 s with a
clear sky, and longer behind a window.

If the caller has other work to do, use `neo_m8n_poll()` (non-blocking) on a
timer and `neo_m8n_get_fix()` to read the latest state.

## Going faster than 1 Hz

The factory default is 9600 baud, which cannot carry the default sentence set
faster than about 1 Hz. Raise the baud rate first — the driver switches both
ends:

```c
neo_m8n_config_t cfg = NEO_M8N_CONFIG_DEFAULT();
cfg.target_baud = 38400;     /* module and MCU both move up */
cfg.nav_rate_ms = 200;       /* 5 Hz */
cfg.trim_nmea   = true;      /* keep GGA+RMC+GSA, silence GLL/GSV/VTG */
cfg.save_config = true;      /* persist, so a power cycle keeps it */
```

`target_baud` is the rate to move *to*; `baud` stays at whatever the module is
speaking on power-up (9600, unless a previous `save_config` changed it). If you
save a new rate to flash and then lose track of it, the module ignores you at
every other rate — recover by trying each in turn, or by pulling its backup
battery to force a factory reset.

Configuration is best-effort by design: `neo_m8n_init()` logs a warning and
carries on if the receiver does not acknowledge, because default NMEA output is
still useful and a tracker should not refuse to boot over a declined command.
Call `neo_m8n_set_nav_rate()` and friends directly if you want to check.

## Pico interrupt note

The RP2040's UART FIFO is 32 bytes; a 1 Hz burst is several hundred. The Pico
port therefore drains the FIFO from the UART interrupt into a ring buffer
(`NEO_M8N_PICO_RX_RING`, 1024 bytes by default, must be a power of two), so the
application can take its time between calls. The handler is installed on
whichever core calls `neo_m8n_init()`; read from that same core.

`neo_m8n_port_pico_dropped()` counts bytes lost to a full ring. If it climbs,
either poll more often or enlarge the ring.

The ESP-IDF port relies on the IDF UART driver's own ISR and ring buffer, sized
by `cfg.rx_buf_size`.

## Tests

```sh
components/neo_m8n/test/run_tests.sh
```

Builds the portable sources against the host port with `-Wall -Wextra
-Wpedantic -Wshadow -Wconversion` and runs 560 assertions: checksum validation,
every sentence type, hemisphere signs, sub-second timestamps, the no-fix case,
buffer overrun handling, UBX frame construction against hand-computed golden
frames, ACK/NAK/timeout paths, and the interleaving of NMEA and UBX on one wire.

## Protocol references

- [u-blox 8 / M8 Receiver Description and Protocol Specification (UBX-13003221)](https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf) — UBX-CFG-PRT §32.10.25, UBX-CFG-MSG §32.10.14, UBX-CFG-RATE §32.10.27, NMEA message class `0xF0`
- [NEO-M8 Hardware Integration Manual (UBX-13003557)](https://content.u-blox.com/sites/default/files/NEO-M8_HardwareIntegrationManual_(UBX-13003557).pdf) — supply, antenna and layout
- [PX4/PX4-GPSDrivers `ubx.h`](https://github.com/PX4/PX4-GPSDrivers/blob/main/src/ubx.h) — cross-checked the CFG message ids and the `0x000008D0` 8N1 mode word
- [KumarRobotics/ublox `CfgPRT.msg`](https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/CfgPRT.msg) — cross-checked the CFG-PRT field offsets
- [kosma/minmea](https://github.com/kosma/minmea) — reference for NMEA parser API shape; no code taken
