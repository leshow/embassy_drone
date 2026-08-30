#![allow(clippy::too_many_arguments)]
#![cfg_attr(feature = "calibrate", allow(unused))]
#![no_std]
#![no_main]
extern crate alloc;

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_backtrace as _;
use esp_hal::{
    Async, gpio,
    interrupt::software::SoftwareInterruptControl,
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        timer::{self, TimerIFace, config::Duty},
    },
    peripherals::LEDC,
    spi::master::Spi,
    system::Stack,
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_println as _;
use esp_rtos::embassy::Executor;
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

mod calibration_storage;
mod flight;
mod motors;
mod panic_safety;
mod sensors;
mod wifi;

use crate::{motors::Motors, sensors::Sensor};
use libs::flight::AccelBias;

const LOOP_PERIOD_MS: u64 = 1; // 1000Hz target loop rate; shared by timer and Madgwick sample_period
// if changing duty cycle, change this value. currently 10 bit resolution
const PWM_BITS: u32 = 10;
const PWM_MAX_DUTY: u32 = (1 << PWM_BITS) - 1;

static TIMER: StaticCell<timer::Timer<'static, LowSpeed>> = StaticCell::new();

/// How many loop iterations to skip between log lines.
/// Override at build time: `LOG_RATE_MS=200 cargo flash-s3` (default: 500 ms).
const LOG_EVERY_N: u32 = {
    let ms = match option_env!("LOG_RATE_MS") {
        Some(s) => libs::flight::parse_u64(s),
        None => 500,
    };
    (ms / LOOP_PERIOD_MS) as u32
};

/// cap on throttle for testing
const THROTTLE_CAP: u8 = {
    match option_env!("THROTTLE_CAP") {
        Some(s) => {
            let v = libs::flight::parse_u64(s);
            assert!(v <= 100, "THROTTLE_CAP must be 0..=100");
            v as u8
        }
        None => 100, // no cap default
    }
};

const fn pwm_duty_config(bits: u32) -> Duty {
    match bits {
        1 => Duty::Duty1Bit,
        2 => Duty::Duty2Bit,
        3 => Duty::Duty3Bit,
        4 => Duty::Duty4Bit,
        5 => Duty::Duty5Bit,
        6 => Duty::Duty6Bit,
        7 => Duty::Duty7Bit,
        8 => Duty::Duty8Bit,
        9 => Duty::Duty9Bit,
        10 => Duty::Duty10Bit,
        11 => Duty::Duty11Bit,
        12 => Duty::Duty12Bit,
        13 => Duty::Duty13Bit,
        14 => Duty::Duty14Bit,
        _ => panic!("failed to pick PWM duty"),
    }
}

type SpiSensor<'a> = ExclusiveDevice<Spi<'a, Async>, gpio::Output<'a>, embassy_time::Delay>;

#[cfg(feature = "mag")]
pub(crate) type Sensor20948<'a> = Sensor<
    icm20948_async::Icm20948<icm20948_async::SpiDevice<SpiSensor<'a>>, icm20948_async::MagEnabled>,
>;

#[cfg(not(feature = "mag"))]
pub(crate) type Sensor20948<'a> = Sensor<
    icm20948_async::Icm20948<icm20948_async::SpiDevice<SpiSensor<'a>>, icm20948_async::MagDisabled>,
>;

// runs on core 1: everything WiFi-related. Kept off core 0 so the flight loop never competes
// with WiFi/UDP tasks for time on a shared executor - see docs/s3-migration.md "Dual executor"
#[cfg(not(feature = "visualize"))]
#[embassy_executor::task]
async fn wifi_core_task(wifi: esp_hal::peripherals::WIFI<'static>, spawner: Spawner) {
    let ap = wifi::AP::init(wifi, spawner).await;
    #[cfg(not(feature = "calibrate"))]
    ap.listen_control(spawner);
    #[cfg(feature = "calibrate")]
    ap.listen_calibrate(spawner);
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    // WiFi tasks get their own spawner on core 1 (see wifi_core_task) - core 0's spawner is
    // unused since main() drives the flight loop directly rather than spawning it as a task
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // wifi heap only needed when running the AP, visualize mode just logs over USB
    #[cfg(not(feature = "visualize"))]
    {
        esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);
        esp_alloc::heap_allocator!(size: 36 * 1024);
    }

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    #[cfg(not(feature = "visualize"))]
    {
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
    }

    // Wait for ICM20948 to power up before touching SPI at all
    Timer::after_millis(1000).await;

    // S3's actual IO_MUX FSPI pins - not GPIO-matrix routed, so no repeat of the C3 matrix-input
    // -delay ceiling that capped reliable SPI clock there. GPIO10=CS, GPIO11=MOSI, GPIO12=SCLK,
    // GPIO13=MISO, GPIO6=interrupt, GPIO1-4=motors (front left/front right/rear right/rear left)
    let spi = Spi::new(
        peripherals.SPI2,
        esp_hal::spi::master::Config::default().with_frequency(Rate::from_mhz(3)), // 3mhz sems reliable, faster crashed whoami on imu startup
    )
    .expect("failed to start spi")
    .with_sck(peripherals.GPIO12)
    .with_mosi(peripherals.GPIO11)
    .with_miso(peripherals.GPIO13)
    .into_async();

    let cs = gpio::Output::new(
        peripherals.GPIO10,
        gpio::Level::High,
        gpio::OutputConfig::default(),
    );
    let spi_device = ExclusiveDevice::new(spi, cs, embassy_time::Delay)
        .expect("failed to create SPI exclusive device");

    // normal run scenario
    #[cfg(not(any(feature = "calibrate", feature = "visualize")))]
    {
        let int_pin = gpio::Input::new(peripherals.GPIO6, gpio::InputConfig::default());
        run(
            spi_device,
            peripherals.LEDC,
            peripherals.FLASH,
            peripherals.GPIO4, // rear left
            peripherals.GPIO3, // rear right
            peripherals.GPIO1, // front left
            peripherals.GPIO2, // front right
            int_pin,
        )
        .await;
    }

    // calibrate and write to nvs
    #[cfg(feature = "calibrate")]
    {
        // ICM20948
        let mut sensor = Sensor::init_icm20948(spi_device)
            .await
            .expect("ICM20948 init failed");
        let mut flash_storage = esp_storage::FlashStorage::new(peripherals.FLASH);

        // accel pass - Some(()) on success so the (optional) mag pass below knows whether to
        // bother running at all
        let ok = match sensor.run_calibration().await {
            Some(accel_bias) => {
                let saved =
                    calibration_storage::store_accel_calibration(&mut flash_storage, &accel_bias);
                if saved {
                    defmt::info!(
                        "accel calibration saved: {}",
                        defmt::Debug2Format(&accel_bias)
                    );
                    Some(())
                } else {
                    defmt::error!("failed to save accel calibration to flash");
                    None
                }
            }
            None => {
                defmt::error!("accel calibration aborted, nothing to save");
                None
            }
        };

        // mag pass - same session, no second wait on wifi::calibrate::START (that already
        // happened inside run_calibration above). skipped entirely if accel already failed.
        #[cfg(feature = "mag")]
        let ok = match ok {
            Some(()) => match sensor.run_mag_calibration().await {
                Some(mag_bias) => {
                    let saved =
                        calibration_storage::store_mag_calibration(&mut flash_storage, &mag_bias);
                    if saved {
                        defmt::info!("mag calibration saved: {}", defmt::Debug2Format(&mag_bias));
                        Some(())
                    } else {
                        defmt::error!("failed to save mag calibration to flash");
                        None
                    }
                }
                None => {
                    defmt::error!("mag calibration aborted, nothing to save");
                    None
                }
            },
            None => None,
        };

        let mode = match ok {
            Some(()) => libs::calibrate::CalibrationMode::Ended,
            None => libs::calibrate::CalibrationMode::Failed,
        };
        wifi::calibrate::EVENTS
            .publisher()
            .expect("calibration publisher already taken")
            .publish(mode)
            .await;
    }

    // run the visualizer (just dump to stdout)
    #[cfg(feature = "visualize")]
    {
        let int_pin = gpio::Input::new(peripherals.GPIO6, gpio::InputConfig::default());
        run_visualizer(spi_device, peripherals.FLASH, int_pin).await;
    }
}

async fn run(
    spi: SpiSensor<'_>,
    ledc: LEDC<'static>,
    flash: esp_hal::peripherals::FLASH<'static>,
    rear_left_pin: impl gpio::interconnect::PeripheralOutput<'static>,
    rear_right_pin: impl gpio::interconnect::PeripheralOutput<'static>,
    front_left_pin: impl gpio::interconnect::PeripheralOutput<'static>,
    front_right_pin: impl gpio::interconnect::PeripheralOutput<'static>,
    int_pin: gpio::Input<'static>,
) {
    let mut ledc = Ledc::new(ledc);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    // Promote the configured timer to static
    let mut timer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    timer
        .configure(timer::config::Config {
            duty: pwm_duty_config(PWM_BITS),
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(78),
        })
        .expect("timer init failed");

    let timer_static = TIMER.init(timer);
    let motors = Motors::init_pwm(
        &ledc,
        timer_static,
        0,
        rear_left_pin,
        rear_right_pin,
        front_left_pin,
        front_right_pin,
    )
    .await;
    // ICM20948
    // gyro bias is tracked continuously in flight::run_control
    let sensor = match Sensor::init_icm20948(spi).await {
        Ok(s) => s,
        Err(e) => {
            defmt::error!("ICM20948 init failed: {}", defmt::Debug2Format(&e));
            panic!("ICM20948 init failed");
        }
    };

    // accel bias/scale from the last `--features calibrate` run, if any - see
    // calibration_storage and flight::AccelBias
    let mut flash_storage = esp_storage::FlashStorage::new(flash);
    let accel_bias = calibration_storage::load_accel_calibration(&mut flash_storage)
        .inspect(|a| {
            info!("accel loaded from bias {}", defmt::Debug2Format(a));
        })
        .unwrap_or_else(|| {
            info!("no accel found, run calibration");
            AccelBias::default()
        });
    flight::run_control(sensor, int_pin, motors, accel_bias).await;
}

#[cfg(feature = "visualize")]
async fn run_visualizer(
    spi: SpiSensor<'_>,
    flash: esp_hal::peripherals::FLASH<'static>,
    int_pin: gpio::Input<'static>,
) {
    let sensor = match Sensor::init_icm20948(spi).await {
        Ok(s) => s,
        Err(e) => {
            defmt::error!("ICM20948 init failed: {}", defmt::Debug2Format(&e));
            panic!("ICM20948 init failed");
        }
    };

    let mut flash_storage = esp_storage::FlashStorage::new(flash);
    let accel_bias =
        calibration_storage::load_accel_calibration(&mut flash_storage).unwrap_or_default();
    flight::run_fusion_visualizer(sensor, int_pin, accel_bias).await;
}
