use crsf::{Packet, PacketParser, RcChannels};
use defmt::debug;
use embassy_rp::{
    Peri,
    peripherals::{DMA_CH0, PIN_1, UART0},
    uart::{Config as UartConfig, UartRx},
};
use {defmt_rtt as _, panic_probe as _};

use crate::Irqs;

// reads CRSF frames from EP2 receiver
#[embassy_executor::task]
pub async fn read_radio(
    uart: Peri<'static, UART0>,
    rx_pin: Peri<'static, PIN_1>,
    dma: Peri<'static, DMA_CH0>,
) {
    let mut uart_config = UartConfig::default();
    // CRSF: 420000 baud
    uart_config.baudrate = 420_000;

    let mut rx = UartRx::new(uart, rx_pin, Irqs, dma, uart_config);
    let mut parser = PacketParser::<64>::new();
    // read only 1 byte at a time since csrf is variable width packets (up to 64 len)
    let mut byte = [0u8; 1];
    // only log when a channel value actually changes - otherwise this floods with an
    // identical line every ~4ms even while every stick/switch is sitting still, which makes
    // it hard to tell which index moved when testing channel mapping by hand
    let mut last: Option<[u16; 16]> = None;

    loop {
        // read is opportunistic so it will return immediately if there are more bytes,
        // should be fine to read 1 at a time
        if rx.read(&mut byte).await.is_ok() {
            parser.push_bytes(&byte);
            while let Some(Ok((_addr, packet))) = parser.next_packet() {
                if let Packet::RcChannels(channels) = packet
                    && last != Some(channels.0)
                {
                    let ctrl = Controls::from(&channels);
                    debug!("control packet: {:?}", defmt::Debug2Format(&ctrl));
                    last = Some(channels.0);
                }
            }
        }
    }
}

// channel order: AETR (0=roll, 1=pitch, 2=throttle, 3=yaw), then AUX1-4 as SA/SB/SC/SD (4-7).
// Sticks are configured with extended travel (measured floor ~174, right at CRSF's true protocol minimum of 172)
// all switches, 2-position or 3-position land on that standard 191/992/1792 convention regardless.
#[derive(Clone, Copy, Debug)]
pub struct Controls {
    roll: f32,
    pitch: f32,
    throttle: f32,
    yaw: f32,
    armed: bool, // SA
    sb: ThreeToggle,
    sc: ThreeToggle,
    sd: bool,
}

#[derive(Clone, Copy, Debug)]
pub enum ThreeToggle {
    Low,
    Mid,
    High,
}

impl From<u16> for ThreeToggle {
    fn from(raw: u16) -> Self {
        // switches cluster around 191/992/1792
        // threshold at the midpoints between clusters  of values
        const LOW_MID_THRESHOLD: u16 = (191 + 992) / 2;
        const MID_HIGH_THRESHOLD: u16 = (992 + 1792) / 2;
        match raw {
            ..LOW_MID_THRESHOLD => Self::Low,
            LOW_MID_THRESHOLD..MID_HIGH_THRESHOLD => Self::Mid,
            MID_HIGH_THRESHOLD.. => Self::High,
        }
    }
}

// roll/pitch/yaw: -1.0..1.0, centered on CHANNEL_VALUE_MID.
fn normalize_axis(raw: u16) -> f32 {
    ((raw as f32 - RcChannels::CHANNEL_VALUE_MID as f32)
        / (RcChannels::CHANNEL_VALUE_MID as f32 - RcChannels::CHANNEL_VALUE_MIN as f32))
        .clamp(-1.0, 1.0)
}

// throttle: 0.0..1.0 across the full protocol range, these sticks measure extended travel
fn normalize_throttle(raw: u16) -> f32 {
    ((raw as f32 - RcChannels::CHANNEL_VALUE_MIN as f32)
        / (RcChannels::CHANNEL_VALUE_MAX as f32 - RcChannels::CHANNEL_VALUE_MIN as f32))
        .clamp(0.0, 1.0)
}

impl From<&RcChannels> for Controls {
    fn from(value: &RcChannels) -> Self {
        let ch = value.0;
        Self {
            roll: normalize_axis(ch[0]),
            pitch: normalize_axis(ch[1]),
            throttle: normalize_throttle(ch[2]),
            yaw: normalize_axis(ch[3]),
            // switches share the same 191/992/1792 clustering as ThreeToggle
            armed: ch[4] > RcChannels::CHANNEL_VALUE_MID,
            sb: ThreeToggle::from(ch[5]),
            sc: ThreeToggle::from(ch[6]),
            sd: ch[7] > RcChannels::CHANNEL_VALUE_MID,
        }
    }
}
