#!/bin/bash
# Build and run the host (PC) unit tests with gcc. From the repository:
#   bash mtk3smp-rp2040/tests/host/run_host_tests.sh
# Exit status: 0 when every test passes.
set -u
H="$(cd "$(dirname "$0")" && pwd)"
R="$(cd "$H/../.." && pwd)"              # mtk3smp-rp2040
A="$R/app_program"
L="$R/lib/libnet/lwip"
OUT="$(mktemp -d)"
CC="gcc -std=c99 -Wall -Wextra -Werror"
fail=0

t() {   # name, then the gcc arguments
    name=$1; shift
    echo "== $name"
    if $CC "$@" -o "$OUT/$name" && "$OUT/$name"; then
        :
    else
        echo "== $name FAILED"; fail=1
    fi
}

t telemetry_codec -I"$L" "$L/telemetry_codec_test.c" "$L/telemetry_codec.c"
t cmd_map -I"$H/stub" -I"$A" "$H/cmd_map_test.c" \
    "$A/buddy1_telemetry/cmd_map.c"
t terrain_lift -I"$H/stub" -I"$A" "$H/terrain_lift_test.c" \
    "$A/buddy4_imu/terrain.c"
if [ -f "$H/barcode_decode_test.c" ]; then
    t barcode_decode -I"$A/buddy3_line_barcode" "$H/barcode_decode_test.c" \
        "$A/buddy3_line_barcode/barcode.c"
fi

rm -rf "$OUT"
[ $fail -eq 0 ] && echo "HOST TESTS: ALL PASS" || echo "HOST TESTS: FAILED"
exit $fail
