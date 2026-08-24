# brushless mini quad on RP2350 (WIP)

Brushless follow-up to the main [esp32-s3 quad](../README.md) - same overall goal, different everything else. Individual ESCs instead of MOSFET-driven brushed motors, RP2350 instead of ESP32-S3, plain `rustup`/stable instead of the Xtensa toolchain.

Currently at the "3 of 4 motors spin on the bench via Oneshot125" stage - no IMU, no mixer, no control link wired up yet.

## Parts list (planned)

| Part                                                    | Qty | Notes                                                                                   |
| ------------------------------------------------------- | --- | --------------------------------------------------------------------------------------- |
| RP2350 "mini zero"                                      | 1   | Pro Micro-footprint clone, 29 GPIO broken out                                           |
| Happymodel SE0802 19000KV                               | 4   | brushless motor, matches Mobula7 spec                                                   |
| MX-5A / MX-5A-L                                         | 4   | individual 1S ESC, BLHeli_S/Bluejay ("BLS" in listing), one per motor                   |
| ICM-20948                                               | 1   | same IMU as the esp32-s3 build                                                          |
| Happymodel Nano ELRS EP2                                | 1   | ELRS receiver, CRSF over UART - pairs with the RadioMaster Pocket's built-in ELRS radio |
| 1S LiPo battery                                         | 1   |                                                                                         |
| TPS63802 buck-boost module                              | 1   | 1S -> 5V, powers RP2350 + EP2 - see Power section below                                 |
| Happymodel Mobula7 frame or 3d printed frame in stl dir | 1   | 75mm wheelbase, 45mm props                                                              |

## Status

- [x] single motor spins via standard servo-PWM on the bench (superseded, see below)
- [x] 4 of 4 motors spin via Oneshot125 on the bench (`src/main.rs`, GPIO10/11/12) - motor 4 pending ESC replacement (first unit died during bench-PSU testing)
- [ ] all 4 motors + mixer (reusing `libs::mixer`)
- [ ] IMU + fusion (reusing `libs::flight::fusion`)
- [ ] ELRS/CRSF control link via the Nano EP2, replacing `ground_control`'s WiFi/UDP link for this build
- [ ] DShot via PIO (see `../docs/todo.md` for the reasoning - RP2350's 12 PIO state machines give each motor its own DShot channel)

## Power

Two separate power domains, not one shared rail:

- **Motors/ESCs**: direct off the 1S battery, unregulated - no boost converter in this path. Each MX-5A's power pads go straight to battery+/-, same raw-rail topology the brushed-motor build used minus the MOSFETs.
- **RP2350 + IMU + EP2 receiver**: 1S battery -> TPS63802 buck-boost module (bridged for 5V output) -> RP2350's `VSYS` pin and the EP2's `+5V` pad, off the same boosted rail. Not `VBUS` - that's raw USB pass-through only, unavailable once flying off battery. Using a boost converter here sidesteps needing to identify whether this board's own onboard regulator is a proper buck-boost or just a plain LDO (either handles a clean 5V input fine) - this was an open question in `../docs/todo.md`'s original power notes, resolved by just always boosting rather than relying on the board's regulator to tolerate raw 1S sag.

All grounds - battery, ESCs, RP2350, EP2, boost converter - share one common reference.

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

Or, once SWD is soldered up to a debug probe, flash and get live `defmt` output instead of BOOTSEL/picotool:

```sh
cargo flash-probe
```

### Setting up the debug probe

For a Raspberry Pi Debug Probe or other CMSIS-DAP-compatible probe:

1. Install `probe-rs`: `cargo install probe-rs-tools --locked`
2. Linux udev rules, so the probe is accessible without root:

   ```sh
   sudo curl -fsSL https://probe.rs/files/69-probe-rs.rules -o /etc/udev/rules.d/69-probe-rs.rules
   sudo udevadm control --reload
   sudo udevadm trigger
   ```

3. Wire it up: use the probe's **"D"** (debug/SWD) port, not "U" (that's a separate plain UART). Standard 3-pin JST-SH cable: orange → SWCLK, black → GND, yellow → SWDIO. This board's SWD pins are bare pads, not a proper JST-SH connector, so you'll want the JST-SH-to-0.1"-header (female) cable variant to land on them.
4. SWD carries no power - the RP2350 still needs its own USB cable connected separately for power, same as the BOOTSEL/picotool path. So two USB cables total: one to the RP2350, one to the probe itself.
5. Raspberry Pi Debug Probes need firmware ≥2.2.0 for `probe-rs` to talk to them (older units commonly ship below that). Update via the [debugprobe releases page](https://github.com/raspberrypi/debugprobe/releases/latest): hold the probe's own BOOTSEL button while plugging it into USB, then copy `debugprobe.uf2` onto the mounted drive.
6. Sanity check before flashing: `probe-rs list` should show the probe.

See `.cargo/config.toml` for the alias definitions and notes on why the toolchain/tooling differs from the rest of the repo.
