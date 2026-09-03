use crsf::{
    MAX_PACKET_LENGTH, PACKET_HEADER_LENGTH, Packet, PacketError, PacketParser, RcChannels,
};
use defmt::{trace, warn};
use embassy_rp::{
    Peri,
    peripherals::PIN_1,
    uart::{Config as UartConfig, UartRx},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::Instant;
use {defmt_rtt as _, panic_probe as _};

use crate::{CtrlTx, Irqs, RadioDma, RadioUart};

// latest control input radio w/ timestamp
// watch keeps only latest value for some const num of recvers
pub static CONTROLS: Watch<CriticalSectionRawMutex, (Controls, Instant), 1> = Watch::new();

// reads CRSF frames from EP2 receiver
#[embassy_executor::task]
pub async fn read_radio(
    uart: Peri<'static, RadioUart>,
    rx_pin: Peri<'static, PIN_1>,
    dma: Peri<'static, RadioDma>,
) {
    let mut uart_config = UartConfig::default();
    // CRSF: 420000 baud
    uart_config.baudrate = 420_000;

    let mut rx = UartRx::new(uart, rx_pin, Irqs, dma, uart_config);
    let mut parser = PacketParser::<64>::new();
    // only log when a channel value actually changes
    let mut last: Option<[u16; 16]> = None;
    let tx: CtrlTx = CONTROLS.sender();

    // legal body lengths: type + crc at minimum, whatever fits after the header at most
    const LEN_MIN: usize = 2;
    const LEN_MAX: usize = MAX_PACKET_LENGTH - PACKET_HEADER_LENGTH;

    let mut header = [0u8; PACKET_HEADER_LENGTH];
    let mut body_buf = [0u8; MAX_PACKET_LENGTH];
    let mut byte = [0u8; 1];

    // are we aligned on our reads
    let mut aligned = false;

    // INVARIANT: at most one frame is pushed into the parser per iteration, and always drained
    // before the next read. push_bytes silently drops when its buffer is full and
    // next_raw_packet pops up to MAX_PACKET_LENGTH per extraction, so letting frames pile up
    // would quietly lose bytes
    loop {
        if aligned {
            // a crsf frame is [addr][len][type + payload + crc]
            if rx.read(&mut header).await.is_ok() {
                let len = header[1] as usize;
                if (LEN_MIN..=LEN_MAX).contains(&len) {
                    let body = &mut body_buf[..len];
                    if rx.read(body).await.is_ok() {
                        parser.push_bytes(&header);
                        parser.push_bytes(body);
                    }
                } else {
                    // a length this can't be means bytes went missing
                    // push what we read, the parser needs an unbroken stream to resync on...
                    // next_packet below then comes up empty and we'll hit the single byte read case
                    parser.push_bytes(&header);
                }
            }
        } else {
            // one byte at a time until a frame parses again
            if rx.read(&mut byte).await.is_ok() {
                parser.push_bytes(&byte);
            }
        }

        let mut framed = false;
        while let Some(res) = parser.next_packet() {
            match res {
                Ok((_addr, packet)) => {
                    framed = true;
                    if let Packet::RcChannels(channels) = packet {
                        // publish on every valid packet to tx
                        let ctrl = Controls::from(&channels);
                        tx.send((ctrl, Instant::now()));
                        // only print a debug if something changed
                        if last != Some(channels.0) {
                            trace!("control packet: {:?}", defmt::Debug2Format(&ctrl));
                            last = Some(channels.0);
                        }
                    }
                }
                // correct packet but just one we dont handle
                Err(PacketError::UnknownType { .. }) => framed = true,
                Err(_) => {}
            }
        }

        if aligned && !framed {
            warn!("crsf: lost frame alignment, falling back to byte scan");
        }
        aligned = framed;
    }
}

// channel order: AETR (0=roll, 1=pitch, 2=throttle, 3=yaw), then AUX1-4 as SA/SB/SC/SD (4-7).
// Sticks are configured with extended travel (measured floor ~174, right at CRSF's true protocol minimum of 172)
// all switches, 2-position or 3-position land on that standard 191/992/1792 convention regardless.
#[allow(unused)]
#[derive(Clone, Copy, Debug)]
pub struct Controls {
    pub(crate) roll: f32,
    pub(crate) pitch: f32,
    pub(crate) throttle: f32,
    pub(crate) yaw: f32,
    pub(crate) armed: bool, // SA
    pub(crate) sb: ThreeToggle,
    pub(crate) sc: ThreeToggle,
    pub(crate) sd: bool,
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
