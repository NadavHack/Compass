#!/usr/bin/env bash
# Builds the portable half of the component for the host and runs its tests.
# No ESP-IDF or Pico SDK required: the target macro resolves to LORA_TARGET_HOST,
# which swaps the SPI bus for the simulated SX127x in port/lora_port_host.c.
set -euo pipefail

cd "$(dirname "$0")/.."
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

CC=${CC:-cc}
$CC -std=c99 -Wall -Wextra -Wpedantic -Wshadow -O1 \
    -Iinclude -Iport \
    -o "$out/test_lora" \
    src/lora.c port/lora_port_host.c test/test_lora.c \
    -lm

"$out/test_lora"
