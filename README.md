# mini quadcopter in Rust w/ embassy for esp32 (WIP)

currently still a work in progress, building a (very) mini quadcopter for esp32 on bare metal with [embassy.dev](https://embassy.dev). right now using the esp32-c3 supermini, I may upgrade this to the s3 if the extra core is necessary.

## Quad assets

![drone frame](/images/drone_frame.png)

The frame stl & 3mf files are in the `stl/` dir and can be used to 3d print the frame. I created them in OnShape. For the battery compartment, I opted for a friction fit with pegs that you can also glue in. This got around some issues with printing supports that were a pain to remove.

There's a top stand that serves as a mount for the microcontroller and MPU6050 or ICM20948. I'll include a full parts list as the project progresses.

I've also created a prop guard you can mount as a single piece or in two halves, see the 3mf file in `stl/`

![prop guard](/images/drone_top.png)

They have a C-snap fit to the motor holders that should lock them into place pretty well. I've printed with a 0.4 and 0.6 nozzle on a bambu a1 and both results are usable. Obviously if you want cleaner layer lines go for the smaller nozzle.

I've printed in PLA, PETG and PETG-CF & GF. PLA seems totally fine, the drone frame is around 16 grams. The props are heavy at an additional 16 grams. They might require a bit of re-design but for indoor testing not so bad.

## Parts list

### Microcontrollers (choose one)

Currently only building on the c3 and c6. I'll try the s3 if I have time to attempt a build on the xtensa cores but RISC-V is better supported by embassy & Rust. The s3 supermini might be interesting because it's dual-core and has the same footprint as the c3.

| Part               | Chip     | Cores   | Flash  | RAM        | WiFi      | Notes                                                          |
| ------------------ | -------- | ------- | ------ | ---------- | --------- | -------------------------------------------------------------- |
| ESP32-C3 SuperMini | ESP32-C3 | 1× RV32 | 4 MB   | 400 KB     | 2.4 GHz   | **Current build** - `c3` feature                               |
| ESP32-C6 SuperMini | ESP32-C6 | 1× RV32 | 4 MB   | 512 KB     | 2.4/5 GHz | Drop-in upgrade - `c6` feature                                 |
| ESP32-S3 (16R8)    | ESP32-S3 | 2× LX7  | 16 MB  | 8 MB PSRAM | 2.4 GHz   | Dual-core; needs Xtensa toolchain - see `docs/s3-migration.md` |
| ESP32-S3 Mini      | ESP32-S3 | 2× LX7  | 4/8 MB | - / 2 MB   | 2.4 GHz   | Compact form factor                                            |

### IMU Sensors (choose)

| Part      | DOF  | Interface | Accel | Gyro | Mag       | Notes                                    |
| --------- | ---- | --------- | ----- | ---- | --------- | ---------------------------------------- |
| ICM-20948 | 9DOF | I²C / SPI | ✓     | ✓    | ✓ AK09916 | **Current build** - yaw via magnetometer |
| MPU-6050  | 6DOF | I²C       | ✓     | ✓    | ✗         | No yaw reference; cheaper and common     |

### Other Components

| Part                   | Qty | Notes                                         |
| ---------------------- | --- | --------------------------------------------- |
| 8520 brushed motor     | 4   | find it on aliexpress like everything else    |
| mosfet 100N03A         | 4   | One per motor                                 |
| 1S LiPo battery (3.7v) | 1   | 3.7v 1s battery 25C or more discharge rate\*  |
| Propeller              | 4   | 55 or 65mm                                    |
| 3D printed frame       | 1   | STL/3MF files in `stl/` - designed in OnShape |

Requires [espflash](https://github.com/esp-rs/espflash) for flashing (`cargo install espflash`).

\*I tried with a 503040 3.7v lipo recycled from a keyboard build but the BMS (battery management system) on it will automatically shut off after a few seconds. It's not really build to power these motors.

### ESP32-C3 (default)

```sh
cargo build -p embassy_quad --target riscv32imc-unknown-none-elf
cargo run -p embassy_quad --target riscv32imc-unknown-none-elf
```

Or using the aliases:

```sh
cargo build-c3
cargo flash-c3
```

### ESP32-C6

The C6 uses a different RISC-V target (`riscv32imac` vs `riscv32imc`). The
`c3` and `c6` features are mutually exclusive — always use
`--no-default-features` when building for C6 to avoid activating both:

```sh
cargo build-c6
cargo flash-c6
```

See .cargo/config.toml to see expansion of aliases

### Log level

`DEFMT_LOG` is read at compile time by `esp-println`:

```sh
DEFMT_LOG=debug cargo flash-c3
```

### Throttle cap

`THROTTLE_CAP` sets the max throttle at compile time, it will map your throttle input from 0-100 over the range you specify. For example, if you set `THROTTLE_CAP=50` and push the throttle to 80% it will translate into `.8 * 50 = 40%`.

## Visualizer

to see a 3d rendering of the orientation run:

```bash
DEFMT_LOG="info" LOG_RATE_MS=1 cargo flash-c3 --features visualize | (cd visualizer && cargo run)
```

It feeds the esp32 log output to a binary reading stdin and rendering a cube on screen

By default, sensor readings from the ICM-20948 are sent to the ESP32-C3 where either a Madgwick filter fuses accel and gyro data in software to correct orientation or the IMU's DMP is used to get the already fused output (`dmp` feature is on by default). The ICM-20948 also has an onboard DMP that fuses data directly on the sensor board. Output looks pretty good and doesn't have the yaw drift that the software fusion does when the magnetometer is enabled.

```sh
DEFMT_LOG="info" LOG_RATE_MS=1 cargo flash-c3 --features visualize | (cd visualizer && cargo run)
```

## Ground control

The `ground_control/` crate is a PC-side sender that reads a gamepad and streams control packets to the drone over UDP.

The drone runs a wifi AP (`esp-quad`, WPA2) with a static IP of `192.168.4.1`. There is no DHCP server, so you must assign yourself a static IP when connecting. The trick is to do this in a single `nmcli con add` — adding security and static IP separately doesn't work reliably:

```sh
nmcli con add type wifi con-name esp-quad ssid esp-quad \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk <same as AP_PASSWORD> \
  ipv4.method manual \
  ipv4.addresses "192.168.4.2/24" \
  ipv4.gateway "192.168.4.1"
```

The AP password is set at build time via the `AP_PASSWORD` environment variable. If unset, the AP is open (remove the `wifi-sec.*` lines above).

Once the profile exists, connecting is just:

```sh
nmcli dev wifi rescan
nmcli con up esp-quad
```

Then run the ground control sender:

```sh
cd ground_control && RUST_LOG="info" cargo run --release
```

The left stick Y axis controls throttle (center = 0%, full up = 100%), left stick X controls yaw, and the right stick controls pitch/roll. The Start button toggles arm. Motors only spin when armed and throttle is above 0.

The drone IP and port can be overridden at build time:

```sh
RUST_LOG="info" GATEWAY_IP=192.168.4.1 UDP_PORT=4444 cargo run --release
```

## Accelerometer Calibration

flash calibration:

```sh
DEFMT_LOG="debug" LOG_RATE_MS=100 AP_PASSWORD="testtest" cargo flash-c3 --features calibrate
```

You can then either leave the quad plugged in or unplug and calibrate using the wifi outputs, either way, you need to connect ground_control

after the `esp-quad` network shows up on wifi, connect to it and fire up ground control in calibration mode:

```sh
~/dev/rust/drone_embassy calibrate_on_wifi
❯ RUST_LOG="info" cargo gc -- --calibrate
    Finished `release` profile [optimized] target(s) in 0.02s
     Running `ground_control/target/x86_64-unknown-linux-gnu/release/ground_control --calibrate`
2026-07-19T04:19:17.472588Z  INFO ground_control: connected to 192.168.4.1:4444 - waiting to start calibration
2026-07-19T04:19:17.595122Z  WARN ground_control: place: level
2026-07-19T04:19:32.976684Z  WARN ground_control: place: front side up
2026-07-19T04:19:48.332827Z  WARN ground_control: place: back side up
2026-07-19T04:20:03.798211Z  WARN ground_control: place: right side up
2026-07-19T04:20:19.373504Z  WARN ground_control: place: left side up
2026-07-19T04:20:34.615814Z  WARN ground_control: place: upside down
2026-07-19T04:20:50.079828Z  INFO ground_control: === CALIBRATION COMPLETE - saved ===
```

follow the instructions and tilt the drone around the axis to get readings. The readings are saved in non-volatile flash memory on the esp32.

## Testing

Unit tests live in `libs`, the crate shared between `firmware` and `ground_control` that defines the control/telemetry packet wire formats. It's the only crate that can build for the host, `firmware` can't (esp-hal's build script refuses to build for a non-embedded target), so there's nothing to run tests against there yet.

`telemetry` and `telemetry-verbose` each change the telemetry packet's wire format, so they're tested separately:

```sh
cargo test -p libs
cargo test -p libs --features telemetry
cargo test -p libs --features telemetry-verbose
```

## Docs

Check the `docs/` directory for some more info on assembly, setting up different controllers, sensors, etc.

## LLM usage

Docs and tests are sometimes generated with the use of LLMs, along with explanation/discovery, but the purpose of this project is to actually learn, so the code is still written by a human (hi!)
