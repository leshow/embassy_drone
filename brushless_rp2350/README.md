# brushless mini quad on RP2350 (WIP)

Brushless follow-up to the main [esp32-s3 quad](../README.md) - same overall goal, different everything else. Individual ESCs instead of MOSFET-driven brushed motors, RP2350 instead of ESP32-S3, plain `rustup`/stable instead of the Xtensa toolchain.

Currently at the "3 of 4 motors spin on the bench via Oneshot125" stage - no IMU, no mixer, no control link wired up yet.

## Parts list (planned)

| Part                                                    | Qty | Notes                                                                                   |
| ------------------------------------------------------- | --- | --------------------------------------------------------------------------------------- |
| RP2350 "mini zero"                                      | 1   | Pro Micro-footprint clone, 29 GPIO broken out                                           |
| Happymodel SE0802 19000KV                               | 4   | brushless motor, matches Mobula7 spec                                                   |
| MX-5A / MX-5A-L (Buzzard Models)                        | 4   | individual 1S ESC, likely BLHeli_S/Bluejay ("BLS" in listing), one per motor            |
| ICM-20948                                               | 1   | same IMU as the esp32-s3 build                                                          |
| Happymodel Nano ELRS EP2                                | 1   | ELRS receiver, CRSF over UART - pairs with the RadioMaster Pocket's built-in ELRS radio |
| 1S LiPo battery                                         | 1   |                                                                                         |
| Happymodel Mobula7 frame or 3d printed frame in stl dir | 1   | 75mm wheelbase, 45mm props                                                              |

## Status

- [x] single motor spins via standard servo-PWM on the bench (superseded, see below)
- [x] 3 of 4 motors spin via Oneshot125 on the bench (`src/main.rs`, GPIO10/11/12) - motor 4 pending ESC replacement (first unit died during bench-PSU testing)
- [ ] all 4 motors + mixer (reusing `libs::mixer`)
- [ ] IMU + fusion (reusing `libs::flight::fusion`)
- [ ] ELRS/CRSF control link via the Nano EP2, replacing `ground_control`'s WiFi/UDP link for this build
- [ ] DShot via PIO (see `../docs/todo.md` for the reasoning - RP2350's 12 PIO state machines give each motor its own DShot channel)

## Building

Standalone crate, not part of the root workspace - needs its own `stable` + `thumbv8m.main-none-eabihf` toolchain instead of the root's Xtensa `esp` channel.

```sh
cd brushless_rp2350
cargo build --release
```

Flash over USB/BOOTSEL (no debug probe needed - requires [picotool](https://github.com/raspberrypi/picotool), install it yourself):

```sh
cargo flash-usb
```

Or, once SWD is soldered up to a raspberry pi debug probe, flash and get live `defmt` output:

```sh
cargo flash-probe
```

See `.cargo/config.toml` for the alias definitions and notes on why the toolchain/tooling differs from the rest of the repo.
