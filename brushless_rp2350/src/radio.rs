use crsf::{Packet, PacketParser};
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

    loop {
        // read is opportunistic so it will return immediately if there are more bytes,
        // should be fine to read 1 at a time
        if rx.read(&mut byte).await.is_ok() {
            parser.push_bytes(&byte);
            while let Some(Ok((_addr, packet))) = parser.next_packet() {
                if let Packet::RcChannels(channels) = packet {
                    debug!("radio channels: {:?}", channels.0);
                }
            }
        }
    }
}
