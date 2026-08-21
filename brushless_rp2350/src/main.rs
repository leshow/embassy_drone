#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

// Oneshot125 ESC convention: much faster refresh than standard PWM, pulse width in microseconds
const ARM_US: u16 = 125;
const BENCH_SPIN_US: u16 = 150; // 20% throttle: 125 + 0.20 * (250 - 125)

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut config = PwmConfig::default();
    // RP2350 default sys clock is 150MHz - divider 150 makes 1 tick = 1us,
    // top 299 makes one period 300us (~3.3kHz refresh) - comfortably under
    // Oneshot125's ~4kHz ceiling
    config.divider = 150u8.into();
    config.top = 299;
    config.compare_a = ARM_US;
    config.compare_b = ARM_US;

    // GPIO10/11 share PWM_SLICE5 (channel A/B) - one Pwm object drives both
    let mut esc_fl_fr = Pwm::new_output_ab(p.PWM_SLICE5, p.PIN_10, p.PIN_11, config.clone());
    // GPIO12 is alone on PWM_SLICE6 channel A - only have 3 ESCs on hand right
    // now (replacement for the smoked one hasn't arrived), so GPIO13/channel B
    // sits unused for the moment
    let mut esc_rl = Pwm::new_output_a(p.PWM_SLICE6, p.PIN_12, config.clone());

    info!("arming ESCs, holding min throttle");
    Timer::after_secs(2).await;

    info!("ramping up to bench spin");
    for us in (ARM_US..=BENCH_SPIN_US).step_by(1) {
        config.compare_a = us;
        config.compare_b = us;
        esc_fl_fr.set_config(&config);
        esc_rl.set_config(&config);
        Timer::after_millis(50).await;
    }

    info!("holding bench spin");
    Timer::after_secs(2).await;

    info!("ramping down to idle");
    for us in (ARM_US..=BENCH_SPIN_US).rev().step_by(1) {
        config.compare_a = us;
        config.compare_b = us;
        esc_fl_fr.set_config(&config);
        esc_rl.set_config(&config);
        Timer::after_millis(50).await;
    }

    config.compare_a = ARM_US;
    config.compare_b = ARM_US;
    esc_fl_fr.set_config(&config);
    esc_rl.set_config(&config);
    info!("done, idling at min throttle");

    loop {
        Timer::after_secs(60).await;
    }
}
