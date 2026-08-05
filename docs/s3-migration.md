# ESP32-S3 Migration Notes

## Decision (as of this planning session)

Going S3-**only** - not adding S3 alongside the existing C3/C6 builds. Port
`firmware/` in place; do not generate a separate crate via `esp-generate`.
The `fusion-lib` extraction idea from an earlier draft of this doc is moot -
`libs/` already serves that role (PID/mixer/calibration/telemetry, already
chip-agnostic, already shared).

## Sequencing

1. **SPI migration first, on the current C3 hardware.** Swap the ICM-20948
   from I2C to SPI (`embedded-hal-bus` + `ExclusiveDevice` for CS,
   `sensors.rs`'s `Icm20948<T, MAG>` widened from `I2cDevice<I>` to
   `T: Transport` so both transports share the same read/calibration code,
   `disable_i2c_interface()` already added to the vendored
   `vendor/icm20948-async` fork - just needs wiring into `init_icm20948`'s
   SPI path). Measure real flight-loop dt/i2c numbers on C3 before touching
   the chip.
2. **Then port to S3.** See below.

Rationale for this order: isolates the SPI win (removes esp-hal's I2C
`yield_now`-based completion-wait tax and the doubled
`reset_before_transmission()` per combined read - see the `esp_i2c` repro at
`~/dev/esp_i2c` for the measured numbers and upstream bug writeup) from the
chip swap, so if something looks off after the S3 port, it's not conflated
with the transport change.

## Why S3

- Hardware single-precision FPU (C3 has none) - directly addresses the
  measured cost of Madgwick fusion + `apply_level_prior`'s trig calls +
  `Lpf3`'s `expf()` calls, all currently running in software float.
- Dual-core - lets WiFi and the flight loop run on physically separate
  cores instead of competing for time on one cooperative executor (see
  "Dual executor" below - this is what actually motivated finally deciding
  to commit to S3 rather than keep optimizing C3).

## Toolchain

Since this is S3-only (not coexisting with the RISC-V C3/C6 builds), skip
the "separate subdirectory with its own `rust-toolchain.toml`" workaround
from the original draft of this doc - just point the root
`rust-toolchain.toml` at the Xtensa toolchain directly:

```toml
[toolchain]
channel = "esp"
targets = ["xtensa-esp32s3-none-elf"]
```

Prerequisites:

```sh
cargo install espup
espup install
# exports LIBCLANG_PATH and PATH - add to shell profile, or source
# ~/export-esp.sh before building
```

`.cargo/config.toml`: swap the `riscv32` rustflags target selector for
`xtensa` (same `-Tdefmt.x -Tlinkall.x -C force-frame-pointers` flags,
verified working on Xtensa). Drop `c3`/`c6` aliases, add `build-s3`/
`flash-s3` against `xtensa-esp32s3-none-elf`. These aliases also need
`-Z build-std=core,alloc`: the distributed Xtensa Rust toolchain doesn't
ship a prebuilt `core`/`alloc` for this target (only for the host,
`x86_64-unknown-linux-gnu` - confirmed from the release archive's own
component manifest), so it has to be built from source via the `rust-src`
component instead, which `-Z build-std` triggers. rust-analyzer needs the
same flag (`rust-analyzer.cargo.extraArgs` in `.vscode/settings.json`) or
it'll show the same `E0463` as a false-positive error in the editor.

`Cargo.toml`: drop the `c3`/`c6` feature blocks, replace with `esp32s3` on
the same dependency set (`esp-hal`, `esp-rtos`, `esp-bootloader-esp-idf`,
`esp-backtrace`, `esp-println`, `esp-radio`, `esp-storage` - all confirmed
to support it).

## Board pin map - FINALIZED for the actual board in hand

The physical board has GPIO1-13 on the main labeled header; GPIO
14,15,16,17,18,21,33-42,45-48 are only exposed as small solder pads on the
back (harder to wire, avoided below - not needed anyway). This didn't
exactly match either the generic Seeed XIAO ESP32-S3 or "S3 SuperMini"
pinout research done earlier in this doc - it's specific to the actual
board, confirmed by directly reading its silkscreen/pads rather than
community pinout pages, and supersedes those generic notes for wiring
purposes.

Verified against the ESP32-S3 datasheet's own IO_MUX Pin Functions table:
GPIO10/11/12/13 are the chip's _actual dedicated FSPI (SPI2) pins_
(FSPICS0/FSPID/FSPICLK/FSPIQ respectively) - not GPIO-matrix-routed, so no
repeat of the C3 matrix-input-delay ceiling that capped reliable SPI clock
there. All four are on the easy main header, not the back pads.

Final pinout (9 pins total: 4 SPI + 1 interrupt + 4 motors, all within the
easy GPIO1-13 header range):

| Signal             | GPIO |
| ------------------ | ---- |
| SCLK               | 12   |
| MOSI               | 11   |
| MISO               | 13   |
| CS                 | 10   |
| Interrupt (INT1)   | 6    |
| Motor: rear left   | 1    |
| Motor: rear right  | 2    |
| Motor: front left  | 3    |
| Motor: front right | 4    |

GPIO5, 7, 8, 9 spare on the easy header for anything added later. GPIO9
(FSPIHD) and GPIO14 (FSPIWP) aren't needed - basic 4-wire SPI doesn't use
hold/write-protect.

## Dual executor (WiFi core + flight-loop core) - IMPLEMENTED

Flight loop stays on core 0 (unchanged - `main()` already ran there).
WiFi (`AP::init`, `listen_control`/`listen_calibrate`) moved to a new
`wifi_core_task`, spawned on core 1 via `esp_rtos::start_second_core` with
its own `Executor`/`Spawner`, verified against the real API in esp-hal's
own `examples/async/embassy_multicore/src/main.rs` (not guessed):

```rust
static APP_CORE_STACK: StaticCell<Stack<8192>> = StaticCell::new();
let app_core_stack = APP_CORE_STACK.init(Stack::new());
esp_rtos::start_second_core(
    peripherals.CPU_CTRL,
    sw_int.software_interrupt1,
    app_core_stack,
    move || {
        static EXECUTOR: StaticCell<Executor> = StaticCell::new();
        let executor = EXECUTOR.init(Executor::new());
        executor.run(|core1_spawner| {
            core1_spawner.spawn(wifi_core_task(peripherals.WIFI, core1_spawner).unwrap());
        });
    },
);
```

This gives the flight loop a fully dedicated core rather than
priority-based preemption within one shared executor - stronger isolation
than anything possible on single-core C3, and directly addresses the
WiFi-task-contention-via-`yield_now` effect measured this session (real
firmware's `i2c read` dropped from ~1.12ms to ~979us avg just from
disabling WiFi spawn entirely on C3 - a dedicated core should get similar
or better isolation without giving up WiFi/telemetry).

Two things flagged as worth verifying on real hardware, not yet confirmed:
`CONTROLS`/`TELEMETRY`'s `blocking_mutex::Mutex<CriticalSectionRawMutex,
Cell<T>>` now needs to be safe for genuine cross-core access (not just
single-core task interleaving) - should be fine since `critical-section`'s
whole purpose is providing this across single- and multi-core targets, but
not independently confirmed for this exact setup. Same caveat for
`esp_alloc`'s heap allocator being called from both cores.

`cargo build-s3` now compiles and links cleanly across all feature
combinations (default, `telemetry-verbose`, `mag,calibrate`, `calibrate`,
`visualize`), and `cargo clippy`/`cargo fmt --check` are clean too. Getting
here required `-Z build-std=core,alloc` (see the Toolchain section below) -
the distributed Xtensa Rust toolchain ships no prebuilt `core`/`alloc` for
`xtensa-esp32s3-none-elf` at all, confirmed by inspecting the release
archive's own component manifest, so `E0463: can't find crate for core`
wasn't an incomplete-install problem, it was expected until `-Z build-std`
was added to the `build-s3`/`flash-s3` aliases. Not yet flashed/tested on
real S3 hardware.

## NVS / calibration storage

Should port unchanged - `calibration_storage.rs` looks up the `nvs`
partition dynamically at runtime via `esp_bootloader_esp_idf::partitions`
rather than hardcoding an offset, and uses explicit `to_le_bytes()`
encoding, not native-endian. The one thing genuinely chip-specific under
the hood is `esp_hal::rom::crc::crc16_le` (a real mask-ROM function, not
just software config) - `esp-hal`'s `rom` module is designed to abstract
this per-chip, should resolve automatically, but verify by actually running
the accel calibration routine on S3 hardware and confirming it survives a
reboot before trusting it in the air.

## Key differences in main.rs for S3

- I2C/SPI peripheral APIs are the same `esp-hal` surface - no sensor driver
  code changes beyond pin numbers (and the SPI transport migration, which
  is chip-independent and happening first on C3 regardless).
- `esp_println`/`defmt` setup unchanged.
- `LOOP_PERIOD_MS` and the fusion builder setup are unchanged.

---

## Superseded: original "S3 alongside C3, separate crate" plan

The rest of this file (esp-generate instructions for a standalone `s3`/
`s3-mini` crate, `fusion-lib` extraction, the ESP-IDF/FreeRTOS alternative)
described adding S3 as a third target alongside the existing RISC-V builds.
That's no longer the plan - kept below only for reference in case the
S3-only decision changes.

### Generating a separate S3 crate

```sh
# For the ESP32-S3 16R8 (16MB flash, 8MB octal PSRAM)
esp-generate -c esp32s3-wroom-1-octal-psram \
  -o alloc -o wifi -o unstable-hal -o embassy -o log -o wokwi -o vscode \
  s3

# For the ESP32-S3 Mini with 2MB PSRAM
esp-generate -c esp32s3-mini-1-psram \
  -o alloc -o wifi -o unstable-hal -o embassy -o log -o wokwi -o vscode \
  s3-mini
```

### Alternative: ESP-IDF (FreeRTOS) instead of bare-metal Embassy

Rather than bare-metal Embassy, the S3 can run on top of Espressif's
**ESP-IDF** SDK, which uses **FreeRTOS** as its underlying RTOS. Rust code
runs as a standard application on top of it via the `esp-idf-hal` and
`esp-idf-svc` crates.

For a **flight controller** where the PID loop timing matters: stick with
bare-metal Embassy (the current, actual choice). ESP-IDF/FreeRTOS
introduces scheduling jitter and a much larger binary/RAM footprint - only
worth it for a companion-computer role, not the flight controller itself.
