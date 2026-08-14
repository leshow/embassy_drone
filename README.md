# mini quadcopter in Rust w/ embassy for esp32 (WIP)

currently still a work in progress, building a (very) mini quadcopter for esp32 on bare metal with [embassy.dev](https://embassy.dev). running on the esp32-s3 - the extra core lets the flight loop and WiFi run fully independently, and the hardware FPU matters a lot for the fusion/filter math (see `docs/s3-migration.md`).

## rp2350 brushless version

I've also started a version with brushless motors, a proper ESC and ELRS transmitter under `brushless_rp2350/`. This one is currently WIP as well.

## Quad assets

![drone frame](/images/drone_frame.png)

The frame stl & 3mf files are in the `stl/` dir and can be used to 3d print the frame. I created them in OnShape. For the battery compartment, I opted for a friction fit with pegs that you can also glue in. This got around some issues with printing supports that were a pain to remove.

There's a top stand that serves as a mount for the microcontroller and MPU6050 or ICM20948. I'll include a full parts list as the project progresses.

I've also created a prop guard you can mount as a single piece or in two halves, see the 3mf file in `stl/`

![prop guard](/images/drone_top.png)

They have a C-snap fit to the motor holders that should lock them into place pretty well. I've printed with a 0.4 and 0.6 nozzle on a bambu a1 and both results are usable. Obviously if you want cleaner layer lines go for the smaller nozzle.

I've printed in PLA, PETG and PETG-CF & GF. PLA seems totally fine, the drone frame is around ~13 grams. The prop guards are heavy at an additional 14 grams (I have a slimmed down version that still give some protection in the 3mf project file). Recommend running without the prop guards to get better battery and flight time.

## Parts list

### Microcontroller

S3-only - the earlier C3/C6 (RISC-V) builds were dropped in favor of the
S3's hardware FPU and second core. Needs the Xtensa toolchain (`espup`),
not plain `rustup` - see `docs/s3-migration.md` for the full writeup and
rationale.

| Part     | Chip     | Cores  | Flash | RAM        | WiFi    | Notes                      |
| -------- | -------- | ------ | ----- | ---------- | ------- | -------------------------- |
| ESP32-S3 | ESP32-S3 | 2x LX7 | 4+ MB | 512K+PSRAM | 2.4 GHz | Current build - s3 feature |

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

### ESP32-S3 (default)

Needs the Xtensa toolchain, not plain `rustup`:

```sh
cargo install espup
espup install
# then source the exports it prints (or ~/export-esp.sh) before building
```

```sh
cargo build -p embassy_quad --target xtensa-esp32s3-none-elf
cargo run -p embassy_quad --target xtensa-esp32s3-none-elf
```

Or using the aliases:

```sh
cargo build-s3
cargo flash-s3
```

See .cargo/config.toml to see expansion of aliases

### Log level

`DEFMT_LOG` is read at compile time by `esp-println`:

```sh
DEFMT_LOG=debug cargo flash-s3
```

### Throttle cap

`THROTTLE_CAP` sets the max throttle at compile time, it will map your throttle input from 0-100 over the range you specify. For example, if you set `THROTTLE_CAP=50` and push the throttle to 80% it will translate into `.8 * 50 = 40%`.

## Visualizer

to see a 3d rendering of the orientation run:

```bash
DEFMT_LOG="info" LOG_RATE_MS=1 cargo flash-s3 --features visualize | (cd visualizer && cargo run)
```

It feeds the esp32 log output to a binary reading stdin and rendering a cube on screen

Sensor readings from the ICM-20948 (over SPI) are sent to the ESP32-S3, where a Madgwick filter fuses accel and gyro data in software to correct orientation. (Earlier versions could also offload fusion to the ICM-20948's onboard DMP; that path has been dropped because DMP, although able to fuse mag input without calibration, is only capable of 100hz.) The icm20948's max speed is 1125hz.

```sh
DEFMT_LOG="info" LOG_RATE_MS=1 cargo flash-s3 --features visualize | (cd visualizer && cargo run)
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
DEFMT_LOG="debug" LOG_RATE_MS=100 AP_PASSWORD="testtest" cargo flash-s3 --features calibrate
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
