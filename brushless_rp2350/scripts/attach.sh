#!/usr/bin/env sh
# reattaches rtt/defmt logging to an already-running rp2350
set -eu

script_dir=$(cd "$(dirname "$0")" && pwd)
elf="${1:-$script_dir/../target/thumbv8m.main-none-eabihf/release/brushless_rp2350}"

if [ ! -f "$elf" ]; then
    echo "error: elf not found at $elf - build it first, or pass the right path as an argument" >&2
    exit 1
fi

exec probe-rs attach --chip RP235x --protocol swd "$elf"
