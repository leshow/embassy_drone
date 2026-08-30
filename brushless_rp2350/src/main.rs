#![no_std]
#![no_main]

use crate::radio::Controls;
use defmt::error;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::PIO0,
    spi::{Config as SpiConfig, Spi},
    uart::InterruptHandler as UartInterruptHandler,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch};
use embassy_time::{Instant, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;

use {defmt_rtt as _, panic_probe as _};

mod flight;
mod imu;
mod motors;
mod radio;

use motors::Motors;

// physical wiring
pub type MotorFl = embassy_rp::peripherals::PIN_10;
pub type MotorFr = embassy_rp::peripherals::PIN_11;
pub type MotorRl = embassy_rp::peripherals::PIN_12;
pub type MotorRr = embassy_rp::peripherals::PIN_13;

pub type RadioUart = embassy_rp::peripherals::UART0;
pub type RadioDma = embassy_rp::peripherals::DMA_CH0;
pub type ImuTxDma = embassy_rp::peripherals::DMA_CH1;
pub type ImuRxDma = embassy_rp::peripherals::DMA_CH2;

pub type CtrlRx<'a> = watch::Receiver<'a, CriticalSectionRawMutex, (Controls, Instant), 1>;
pub type CtrlTx<'a> = watch::Sender<'a, CriticalSectionRawMutex, (Controls, Instant), 1>;

bind_interrupts!(struct Irqs {
    UART0_IRQ => UartInterruptHandler<RadioUart>;
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<RadioDma>, embassy_rp::dma::InterruptHandler<ImuTxDma>, embassy_rp::dma::InterruptHandler<ImuRxDma>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

/// How many loop iterations to skip between log lines.
/// Override at build time: `LOG_RATE=200 cargo flash-s3` (default: 500).
const LOG_EVERY_N: u32 = {
    let every = match option_env!("LOG_RATE") {
        Some(s) => libs::flight::parse_u64(s),
        None => 500,
    };
    every as u32
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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    spawner.spawn(radio::read_radio(p.UART0, p.PIN_1, p.DMA_CH0).unwrap());

    #[cfg(feature = "test_dshot")]
    test_dshot(p.PIO0, p.PIN_10, p.PIN_11, p.PIN_12, p.PIN_13).await;

    #[cfg(feature = "visualize")]
    {
        // 3MHz, under ICM42688's 24MHz SPI ceiling

        use crate::imu::Sensor;
        let mut spi_config = SpiConfig::default();
        spi_config.frequency = 3_000_000;
        let spi_bus = Spi::new(
            p.SPI0, p.PIN_2, p.PIN_3, p.PIN_4, p.DMA_CH1, p.DMA_CH2, Irqs, spi_config,
        );
        let cs = Output::new(p.PIN_5, Level::High);
        let device = ExclusiveDevice::new_no_delay(spi_bus, cs).unwrap();

        // initialize() soft-resets the sensor and checks WHO_AM_I internally (expects 0x47)
        match Sensor::init(device).await {
            Ok(icm) => visualize(icm, p.PIN_6).await,
            Err(e) => error!("ICM42688 init failed: {}", e),
        }
    }

    // normal run scenario
    #[cfg(not(any(feature = "visualize", feature = "test_dshot")))]
    {
        use crate::{imu::Sensor, radio::CONTROLS};
        let mut spi_config = SpiConfig::default();
        spi_config.frequency = 3_000_000;
        let spi_bus = Spi::new(
            p.SPI0, p.PIN_2, p.PIN_3, p.PIN_4, p.DMA_CH1, p.DMA_CH2, Irqs, spi_config,
        );
        let cs = Output::new(p.PIN_5, Level::High);
        let device = ExclusiveDevice::new_no_delay(spi_bus, cs).unwrap();
        let rx = CONTROLS.receiver().expect("failed to create receiver");

        // motors first
        let motors = Motors::init(p.PIO0, p.PIN_10, p.PIN_11, p.PIN_12, p.PIN_13).await;

        // initialize() soft-resets the sensor and checks WHO_AM_I internally (expects 0x47)
        let mut sensor = match Sensor::init(device).await {
            Ok(icm) => icm,
            Err(e) => {
                error!("ICM42688 init failed: {}", e);
                // TODO: should this panic?
                return;
            }
        };
        // flush whatever backlog built up since init
        if let Err(e) = sensor.reset_fifo().await {
            error!("ICM42688 reset_fifo failed: {}", e);
        }
        flight::run(sensor, motors, p.PIN_6, rx).await;
    }
    loop {
        Timer::after_secs(60).await;
    }
}

// live fusion dump
// it can be piped into `visualizer` the same way the esp32-s3 project does:
// `cargo flash-probe --features visualize | (cd ../visualizer && cargo run)`
//
// INT1 (GPIO6) drives sampling off the data-ready pulse - ICM42688 config default is 200Hz
// ODR, latched/active-high, and read_sample()'s first SPI byte reads INT_STATUS which clears
// the latch
#[cfg(feature = "visualize")]
async fn visualize<D>(
    mut imu: crate::imu::Sensor<icm426xx::ICM42688<D, icm426xx::Ready>>,
    int1: Peri<'static, PIN_6>,
) where
    D: embedded_hal_async::spi::SpiDevice,
    D::Error: defmt::Format,
{
    let mut int1 = Input::new(int1, Pull::None);
    let mut fusion = FusionBuilder::new().icm42688().madgwick().build();
    let mut last = Instant::now();

    loop {
        // TODO: can try without wait_for_high if there are issues
        int1.wait_for_high().await;
        match imu.read().await {
            Ok((accel, gyro)) => {
                let now = Instant::now();
                let dt = now.duration_since(last).as_micros() as f32 / 1_000_000.0;
                last = now;

                let quat = fusion.update(dt, accel, gyro);
                let (roll, pitch, yaw) = quat.euler_angles();
                info!(
                    "roll: {}\u{b0} pitch: {}\u{b0} yaw: {}\u{b0}",
                    roll * RAD_TO_DEG,
                    pitch * RAD_TO_DEG,
                    yaw * RAD_TO_DEG
                );
            }
            Err(e) => error!("ICM42688 read_sample failed: {}", e),
        }
    }
}

// same GPIO10-13 wiring as test_oneshot, but driven through the real Motors abstraction
// (motors.rs) instead of raw PWM_SLICE5/6 - each motor gets its own independent PIO state
// machine. Motors::init already arms; unlike PWM (a free-running hardware counter that keeps
// outputting once configured), DShot frames are one-shot pushes to the PIO FIFO - nothing
// repeats on its own, so every phase (including the hold) has to keep calling set_motors at a
// steady rate or the ESCs will fail-safe
#[cfg(feature = "test_dshot")]
async fn test_dshot(
    pio: Peri<'static, PIO0>,
    fl: Peri<'static, MotorFl>,
    fr: Peri<'static, MotorFr>,
    rl: Peri<'static, MotorRl>,
    rr: Peri<'static, MotorRr>,
) {
    // Motors::set_motors takes normalized [0.0, 1.0], not raw DShot 0-1999 values
    const TARGET_THROTTLE: f32 = 0.2; // ~20%, same bench-spin level as the Oneshot125 test
    const RAMP_STEPS: u32 = 100;
    const HOLD_ITERATIONS: u32 = 100; // 100 * 20ms = 2s
    const STEP_DELAY_MS: u64 = 20;

    let mut motors = Motors::init(pio, fl, fr, rl, rr).await;

    // straight to target instead of ramping - diagnostic for whether the staggered start seen
    // during the ramp is just different motors crossing their own spin-up threshold at
    // different points along it, rather than a real per-channel timing difference
    info!("jumping straight to bench spin, no ramp");
    motors
        .set_motors(
            TARGET_THROTTLE,
            TARGET_THROTTLE,
            TARGET_THROTTLE,
            TARGET_THROTTLE,
        )
        .await;

    info!("holding bench spin");
    for _ in 0..HOLD_ITERATIONS {
        motors
            .set_motors(
                TARGET_THROTTLE,
                TARGET_THROTTLE,
                TARGET_THROTTLE,
                TARGET_THROTTLE,
            )
            .await;
        Timer::after_millis(STEP_DELAY_MS).await;
    }

    info!("ramping down to idle");
    for i in (0..=RAMP_STEPS).rev() {
        let t = TARGET_THROTTLE * (i as f32 / RAMP_STEPS as f32);
        motors.set_motors(t, t, t, t).await;
        Timer::after_millis(STEP_DELAY_MS).await;
    }

    motors.turn_off().await;
    info!("done, motors stopped");
}

// dead oneshot125 code. dshot is better :)
// #[cfg(feature = "test_oneshot")]
// {
//     let mut config = PwmConfig::default();
//     // RP2350 default sys clock is 150MHz - divider 150 makes 1 tick = 1us,
//     // top 299 makes one period 300us (~3.3kHz refresh) under
//     // Oneshot125's ~4kHz ceiling
//     config.divider = 150u8.into();
//     config.top = 299;
//     config.compare_a = ARM_US;
//     config.compare_b = ARM_US;

//     test_oneshot(
//         p.PWM_SLICE5,
//         p.PIN_10,
//         p.PIN_11,
//         p.PWM_SLICE6,
//         p.PIN_12,
//         p.PIN_13,
//         config,
//     )
//     .await;
// }
// Oneshot125 ESC convention: much faster refresh than standard PWM, pulse width in microseconds
// const ARM_US: u16 = 125;
// const BENCH_SPIN_US: u16 = 150; // 20% throttle: 125 + 0.20 * (250 - 125)
// min + (percent * (max - min))

// async fn test_oneshot(
//     slice_a: Peri<'static, PWM_SLICE5>,
//     pin_10: Peri<'static, MotorFl>,
//     pin_11: Peri<'static, MotorFr>,
//     slice_b: Peri<'static, PWM_SLICE6>,
//     pin_12: Peri<'static, MotorRl>,
//     pin_13: Peri<'static, MotorRr>,
//     mut config: PwmConfig,
// ) {
//     // gpio 10/11 on pwm_slice5
//     let mut esc_fl_fr = Pwm::new_output_ab(slice_a, pin_10, pin_11, config.clone());
//     // gpio 12/13 are on pwm slice 6 - PIN_12 is hardware-fixed as channel A, PIN_13 as
//     // channel B (must be passed in that order), so compare_a controls GPIO12 (RL) and
//     // compare_b controls GPIO13 (RR)
//     let mut esc_rl_rr = Pwm::new_output_ab(slice_b, pin_12, pin_13, config.clone());

//     info!("arming ESCs, holding min throttle");
//     Timer::after_secs(2).await;

//     // straight to target instead of ramping - PWM's a free-running hardware counter, so unlike
//     // DShot there's no failsafe-timeout concern either way; this matches what the direct-jump
//     // DShot test already confirmed: the staggered start seen during a slow ramp was just
//     // different motors crossing their own spin-up threshold at different points along it
//     info!("jumping straight to bench spin, no ramp");
//     config.compare_a = BENCH_SPIN_US;
//     config.compare_b = BENCH_SPIN_US;
//     esc_fl_fr.set_config(&config);
//     esc_rl_rr.set_config(&config);

//     info!("holding bench spin");
//     Timer::after_secs(2).await;

//     info!("ramping down to idle");
//     for us in (ARM_US..=BENCH_SPIN_US).rev() {
//         config.compare_a = us;
//         config.compare_b = us;
//         esc_fl_fr.set_config(&config);
//         esc_rl_rr.set_config(&config);
//         Timer::after_millis(100).await;
//     }

//     config.compare_a = ARM_US;
//     config.compare_b = ARM_US;
//     esc_fl_fr.set_config(&config);
//     esc_rl_rr.set_config(&config);
//     info!("done, idling at min throttle");
// }
