#!/usr/bin/env bash
# Build + Flash via 1200-Baud-BOOTSEL-Trick (keine sudo/udev-Rechte nötig).
set -euo pipefail

PORT="${1:-/dev/ttyACM0}"
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "${IN_PICO_SHELL:-}" ]; then
    exec env IN_PICO_SHELL=1 nix develop "$SRC_DIR/.." -c bash "${BASH_SOURCE[0]}" "$@"
fi

cmake --build "$SRC_DIR/build"

if [ -e "$PORT" ]; then
    stty -F "$PORT" 1200
fi

# Warten bis die RP2350-Boot-Partition gemountet ist
for i in $(seq 1 50); do
    MOUNT=$(lsblk -lno LABEL,MOUNTPOINT 2>/dev/null | awk '$1=="RP2350" && $2!="" {print $2}')
    [ -n "${MOUNT:-}" ] && break
    sleep 0.2
done

if [ -z "${MOUNT:-}" ]; then
    echo "RP2350-Bootlaufwerk nicht gefunden" >&2
    exit 1
fi

cp -v "$SRC_DIR/build/my_program.uf2" "$MOUNT/"
