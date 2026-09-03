#!/usr/bin/env sh
# resets the rp2350 over the debug probe (swd)
set -eu

exec probe-rs reset --chip RP235x --protocol swd
