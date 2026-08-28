#![no_std]
#![no_main]

use defmt::{error, info};
use embassy_dshot::{Command, DshotPioAsync, DshotSpeed, rp::DshotPio};
use embassy_executor::Spawner;
use embassy_rp::{
    Peri, bind_interrupts,
    gpio::{Input, Level, Output, Pull},
    peripherals::{DMA_CH0, DMA_CH1, DMA_CH2, PIN_6, PIO0, UART0},
    spi::{Config as SpiConfig, Spi},
    uart::InterruptHandler as UartInterruptHandler,
};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use icm426xx::{Config as ImuConfig, ICM42688, OutputDataRate};
use libs::flight::fusion::{FusionBuilder, RAD_TO_DEG};
use nalgebra::Vector3;
use {defmt_rtt as _, panic_probe as _};

mod radio;

// DShot throttle range is 0-1999 (embassy-dshot's own 0-based abstraction over the raw
// 48-2047 DShot wire values - THROTTLE_IDLE is already 0, no offset needed on our end)
const DSHOT_TARGET_THROTTLE: u16 = 400; // ~20%, same bench-spin level as the Oneshot125 test
const DSHOT_RAMP_STEPS: u32 = 100;
const DSHOT_HOLD_ITERATIONS: u32 = 100; // 100 * 20ms = 2s
const DSHOT_STEP_DELAY_MS: u64 = 20;

bind_interrupts!(struct Irqs {
    UART0_IRQ => UartInterruptHandler<UART0>;
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH0>, embassy_rp::dma::InterruptHandler<DMA_CH1>, embassy_rp::dma::InterruptHandler<DMA_CH2>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    spawner.spawn(radio::read_radio(p.UART0, p.PIN_1, p.DMA_CH0).unwrap());

    #[cfg(feature = "test_dshot")]
    test_dshot(p.PIO0, p.PIN_10, p.PIN_11, p.PIN_12, p.PIN_13).await;

    #[cfg(feature = "visualize")]
    {
        // 3MHz, under ICM42688's 24MHz SPI ceiling
        let mut spi_config = SpiConfig::default();
        spi_config.frequency = 3_000_000;
        let spi_bus = Spi::new(
            p.SPI0, p.PIN_2, p.PIN_3, p.PIN_4, p.DMA_CH1, p.DMA_CH2, Irqs, spi_config,
        );
        let cs = Output::new(p.PIN_5, Level::High);
        let device = ExclusiveDevice::new_no_delay(spi_bus, cs).unwrap();

        // 1kHz read rate
        let imu_config = ImuConfig {
            rate: OutputDataRate::Hz1000,
            ..Default::default()
        };
        // initialize() soft-resets the sensor and checks WHO_AM_I internally (expects 0x47)
        match ICM42688::new(device).initialize(Delay, imu_config).await {
            Ok(icm) => visualize(icm, p.PIN_6).await,
            Err(e) => error!("ICM42688 init failed: {}", e),
        }
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
async fn visualize<SPI>(mut icm: ICM42688<SPI, icm426xx::Ready>, int1: Peri<'static, PIN_6>)
where
    SPI: embedded_hal_async::spi::SpiDevice,
    SPI::Error: defmt::Format,
{
    info!("ICM42688 init OK, WHO_AM_I matched");

    let mut int1 = Input::new(int1, Pull::None);
    let mut fusion = FusionBuilder::new().icm42688().madgwick().build();
    let mut last = Instant::now();

    loop {
        int1.wait_for_high().await;
        match icm.read_sample().await {
            Ok(Some((sample, _more_in_fifo))) => {
                let (Some(accel), Some(gyro)) = (sample.accel, sample.gyro) else {
                    continue;
                };
                let now = Instant::now();
                let dt = now.duration_since(last).as_micros() as f32 / 1_000_000.0;
                last = now;

                let quat = fusion.update(
                    dt,
                    Vector3::new(accel.0, accel.1, accel.2),
                    Vector3::new(gyro.0, gyro.1, gyro.2),
                );
                let (roll, pitch, yaw) = quat.euler_angles();
                info!(
                    "roll: {}\u{b0} pitch: {}\u{b0} yaw: {}\u{b0}",
                    roll * RAD_TO_DEG,
                    pitch * RAD_TO_DEG,
                    yaw * RAD_TO_DEG
                );
            }
            Ok(None) => {}
            Err(e) => error!("ICM42688 read_sample failed: {}", e),
        }
    }
}

// same GPIO10-13 wiring as test_oneshot, but driven by PIO instead of PWM_SLICE5/6 - each
// motor gets its own independent state machine
// unlike PWM (a free-running hardware counter that keeps outputting once configured), DShot
// frames are one-shot pushes to the PIO FIFO - nothing repeats on its own, so every phase
// (including the hold) has to keep sending frames at a steady rate or the ESCs will fail-safe
#[cfg(feature = "test_dshot")]
async fn test_dshot(
    pio: Peri<'static, PIO0>,
    pin_10: Peri<'static, embassy_rp::peripherals::PIN_10>,
    pin_11: Peri<'static, embassy_rp::peripherals::PIN_11>,
    pin_12: Peri<'static, embassy_rp::peripherals::PIN_12>,
    pin_13: Peri<'static, embassy_rp::peripherals::PIN_13>,
) {
    let mut dshot = DshotPio::<4, PIO0>::new(
        pio,
        Irqs,
        pin_10,
        pin_11,
        pin_12,
        pin_13,
        DshotSpeed::DShot300,
    );

    info!("arming ESCs");
    dshot.arm_async(Duration::from_secs(2)).await;

    // straight to target instead of ramping - diagnostic for whether the staggered start seen
    // during the ramp is just different motors crossing their own spin-up threshold at
    // different points along it, rather than a real per-channel timing difference
    info!("jumping straight to bench spin, no ramp");
    dshot
        .throttle_async([DSHOT_TARGET_THROTTLE; 4])
        .await
        .unwrap();

    info!("holding bench spin");
    for _ in 0..DSHOT_HOLD_ITERATIONS {
        dshot
            .throttle_async([DSHOT_TARGET_THROTTLE; 4])
            .await
            .unwrap();
        Timer::after_millis(DSHOT_STEP_DELAY_MS).await;
    }

    info!("ramping down to idle");
    for i in (0..=DSHOT_RAMP_STEPS).rev() {
        let t = (DSHOT_TARGET_THROTTLE as u32 * i / DSHOT_RAMP_STEPS) as u16;
        dshot.throttle_async([t; 4]).await.unwrap();
        Timer::after_millis(DSHOT_STEP_DELAY_MS).await;
    }

    dshot.send_command_async(Command::MotorStop).await;
    info!("done, motors stopped");
}

// oneshot125:
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
//     pin_10: Peri<'static, embassy_rp::peripherals::PIN_10>,
//     pin_11: Peri<'static, embassy_rp::peripherals::PIN_11>,
//     slice_b: Peri<'static, PWM_SLICE6>,
//     pin_12: Peri<'static, embassy_rp::peripherals::PIN_12>,
//     pin_13: Peri<'static, embassy_rp::peripherals::PIN_13>,
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
