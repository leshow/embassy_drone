#!/usr/bin/env sh
# clears persisted accel calibration back to "uncalibrated" - see
# firmware/src/calibration_storage.rs. used to be `cargo erase-calibration`, but cargo aliases
# can't shell out to an arbitrary command like espflash.
set -eu

exec espflash erase-region 0x9000 0x6000
