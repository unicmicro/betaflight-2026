#!/usr/bin/env bash
set -euo pipefail

test_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
repo=$(cd -- "${1:-$test_dir/../..}" && pwd -P)
um32="$repo/src/platform/UM32"
cherry="$repo/lib/modules/UM324xF/Middlewares/UM/CherryUSB-1.6.0"

if [[ ! -f "$um32/serial_usb_vcp.c" ]]; then
    printf '%s\n' 'Pass the Betaflight repository root as the first argument.' >&2
    exit 2
fi
if [[ ! -f "$cherry/core/usbd_core.h" ]]; then
    printf '%s\n' 'Initialize lib/modules/UM324xF as described in README.md.' >&2
    exit 2
fi

build_dir=$(mktemp -d "${TMPDIR:-/tmp}/um32-vcp-host-test.XXXXXXXX")
test_exe="$build_dir/vcp_tx_test.exe"
trap 'rm -f -- "$test_exe"; rmdir -- "$build_dir"' EXIT

"${CC:-cc}" -std=gnu11 -O2 -Wall -Wextra -Werror -DUM324xF \
    -I"$test_dir" -I"$repo/src/main" -I"$um32" \
    -I"$cherry/port/um" -I"$cherry/common" -I"$cherry/core" -I"$cherry/class/cdc" \
    "$um32/serial_usb_vcp.c" "$um32/vcp_hal/cdc_acm.c" \
    "$test_dir/vcp_tx_test.c" -o "$test_exe"
"$test_exe"
