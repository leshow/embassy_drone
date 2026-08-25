#![no_std]
#![no_main]

use crsf::{Packet, PacketParser};
use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::{
    Peri, bind_interrupts,
    peripherals::{DMA_CH0, UART0},
    uart::InterruptHandler as UartInterruptHandler,
    uart::{Config as UartConfig, UartRx},
};
use embassy_rp::{
    peripherals::{PIN_1, PWM_SLICE5, PWM_SLICE6},
    pwm::{Config as PwmConfig, Pwm},
};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

// Oneshot125 ESC convention: much faster refresh than standard PWM, pulse width in microseconds
const ARM_US: u16 = 125;
const BENCH_SPIN_US: u16 = 150; // 20% throttle: 125 + 0.20 * (250 - 125)
// min + (percent * (max - min))

bind_interrupts!(struct Irqs {
    UART0_IRQ => UartInterruptHandler<UART0>;
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _p = embassy_rp::init(Default::default());

    #[cfg(feature = "test_motors")]
    {
        let mut config = PwmConfig::default();
        // RP2350 default sys clock is 150MHz - divider 150 makes 1 tick = 1us,
        // top 299 makes one period 300us (~3.3kHz refresh) under
        // Oneshot125's ~4kHz ceiling
        config.divider = 150u8.into();
        config.top = 299;
        config.compare_a = ARM_US;
        config.compare_b = ARM_US;

        _spawner.spawn(read_radio(_p.UART0, _p.PIN_1, _p.DMA_CH0).unwrap());
        test_motors(
            _p.PWM_SLICE5,
            _p.PIN_10,
            _p.PIN_11,
            _p.PWM_SLICE6,
            _p.PIN_12,
            _p.PIN_13,
            config,
        )
        .await;
    }

    loop {
        Timer::after_secs(60).await;
    }
}

// reads CRSF frames from EP2 receiver
#[embassy_executor::task]
async fn read_radio(
    uart: Peri<'static, UART0>,
    rx_pin: Peri<'static, PIN_1>,
    dma: Peri<'static, DMA_CH0>,
) {
    let mut uart_config = UartConfig::default();
    // CRSF: 420000 baud
    uart_config.baudrate = 420_000;

    let mut rx = UartRx::new(uart, rx_pin, Irqs, dma, uart_config);
    let mut parser = PacketParser::<64>::new();
    let mut byte = [0u8; 1];

    loop {
        if rx.read(&mut byte).await.is_ok() {
            parser.push_bytes(&byte);
            while let Some(Ok((_addr, packet))) = parser.next_packet() {
                if let Packet::RcChannels(channels) = packet {
                    info!("radio channels: {:?}", channels.0);
                }
            }
        }
    }
}

async fn test_motors(
    slice_a: Peri<'static, PWM_SLICE5>,
    pin_10: Peri<'static, embassy_rp::peripherals::PIN_10>,
    pin_11: Peri<'static, embassy_rp::peripherals::PIN_11>,
    slice_b: Peri<'static, PWM_SLICE6>,
    pin_12: Peri<'static, embassy_rp::peripherals::PIN_12>,
    pin_13: Peri<'static, embassy_rp::peripherals::PIN_13>,
    mut config: PwmConfig,
) {
    // gpio 10/11 on pwm_slice5
    let mut esc_fl_fr = Pwm::new_output_ab(slice_a, pin_10, pin_11, config.clone());
    // gpio 12/13 are on pwm slice 6 - PIN_12 is hardware-fixed as channel A, PIN_13 as
    // channel B (must be passed in that order), so compare_a controls GPIO12 (RL) and
    // compare_b controls GPIO13 (RR)
    let mut esc_rl_rr = Pwm::new_output_ab(slice_b, pin_12, pin_13, config.clone());

    info!("arming ESCs, holding min throttle");
    Timer::after_secs(2).await;

    info!("ramping up to bench spin");
    for us in ARM_US..=BENCH_SPIN_US {
        config.compare_a = us;
        config.compare_b = us;
        esc_fl_fr.set_config(&config);
        esc_rl_rr.set_config(&config);
        Timer::after_millis(100).await;
    }

    info!("holding bench spin");
    Timer::after_secs(2).await;

    info!("ramping down to idle");
    for us in (ARM_US..=BENCH_SPIN_US).rev() {
        config.compare_a = us;
        config.compare_b = us;
        esc_fl_fr.set_config(&config);
        esc_rl_rr.set_config(&config);
        Timer::after_millis(100).await;
    }

    config.compare_a = ARM_US;
    config.compare_b = ARM_US;
    esc_fl_fr.set_config(&config);
    esc_rl_rr.set_config(&config);
    info!("done, idling at min throttle");
}
