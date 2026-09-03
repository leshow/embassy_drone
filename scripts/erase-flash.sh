#!/usr/bin/env sh
# wipes the esp32-s3's whole flash region (0x300000 bytes starting at 0x10000) - used to be
# `cargo erase-flash`, but cargo aliases can't shell out to an arbitrary command like espflash;
# see erase-calibration.sh for the same story with a narrower region.
set -eu

exec espflash erase-region 0x10000 0x300000
