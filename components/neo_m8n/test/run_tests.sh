#!/usr/bin/env bash
# Builds the portable half of the component for the host and runs its tests.
# No ESP-IDF or Pico SDK required: the target macro resolves to
# NEO_M8N_TARGET_HOST, which swaps the UART for the fake in
# port/neo_m8n_port_host.c.
set -euo pipefail

cd "$(dirname "$0")/.."
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

CC=${CC:-cc}
$CC -std=c99 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -O1 \
    -Iinclude -Iport \
    -o "$out/test_neo_m8n" \
    src/nmea.c src/neo_m8n.c port/neo_m8n_port_host.c \
    test/test_nmea.c test/test_driver.c \
    -lm

"$out/test_neo_m8n"
