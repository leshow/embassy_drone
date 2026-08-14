#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

// standard servo-PWM ESC convention: 50Hz refresh, pulse width in microseconds
const ARM_US: u16 = 1000;
const BENCH_SPIN_US: u16 = 1200; // 20% throttle: 1000 + 0.20 * (2000 - 1000)

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut config = PwmConfig::default();
    // RP2350 default sys clock is 150MHz - divider 150 makes 1 tick = 1us,
    // top 19_999 makes one period 20ms (50Hz), so compare_a is a pulse width in us
    config.divider = 150u8.into();
    config.top = 19_999;
    config.compare_a = ARM_US;

    let mut esc = Pwm::new_output_a(p.PWM_SLICE6, p.PIN_28, config.clone());

    info!("arming ESC, holding min throttle");
    Timer::after_secs(2).await;

    info!("ramping up to bench spin");
    for us in (ARM_US..=BENCH_SPIN_US).step_by(10) {
        config.compare_a = us;
        esc.set_config(&config);
        Timer::after_millis(50).await;
    }

    info!("holding bench spin");
    Timer::after_secs(2).await;

    info!("ramping down to idle");
    for us in (ARM_US..=BENCH_SPIN_US).rev().step_by(10) {
        config.compare_a = us;
        esc.set_config(&config);
        Timer::after_millis(50).await;
    }

    config.compare_a = ARM_US;
    esc.set_config(&config);
    info!("done, idling at min throttle");

    loop {
        Timer::after_secs(60).await;
    }
}
