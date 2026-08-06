extern crate alloc;

use alloc::string::ToString;
use core::{cell::Cell, net::Ipv4Addr, str::FromStr};

use embassy_executor::Spawner;
use embassy_net::{
    Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4,
    udp::{PacketMetadata, UdpSocket},
};
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::Instant;
use esp_hal::{peripherals::WIFI, rng::Rng};
use esp_radio::wifi::{
    AccessPointStationEventInfo, AuthenticationMethod, Config, ControllerConfig, Interface,
    WifiController, ap::AccessPointConfig,
};
use static_cell::StaticCell;

use libs::control::{self, ControlPacket};
#[cfg(feature = "telemetry")]
use libs::telemetry::TelemetryPacket;

// latest control input from ground control, stamped at receive time for failsafe
pub static CONTROLS: Mutex<esp_sync::RawMutex, Cell<Option<(ControlPacket, Instant)>>> =
    Mutex::new(Cell::new(None));

// latest telemetry snapshot from the flight loop, sent back to ground control on each
// received control packet. None until the flight loop has produced a first sample.
#[cfg(feature = "telemetry")]
pub static TELEMETRY: Mutex<esp_sync::RawMutex, Cell<Option<(TelemetryPacket, Instant)>>> =
    Mutex::new(Cell::new(None));

macro_rules! mk_static {
    ($t:ty, $val:expr) => {{
        static STATIC_CELL: StaticCell<$t> = StaticCell::new();
        STATIC_CELL.uninit().write($val)
    }};
}

pub struct AP {
    stack: Stack<'static>,
}

impl AP {
    /// Initialise the WiFi AP and return an AP holding the network stack.
    /// Call esp_alloc::heap_allocator! in main before this.
    pub async fn init(wifi: WIFI<'static>, spawner: Spawner) -> Self {
        let gw_ip = Ipv4Addr::from_str(libs::GW_IP_ADDR_ENV.unwrap_or(libs::GW_IP_DEFAULT))
            .expect("no IP given for AP");
        let ap_config = Config::AccessPoint(match libs::AP_PASSWORD {
            Some(pw) => AccessPointConfig::default()
                .with_ssid(libs::SSID_DEFAULT)
                .with_password(pw.to_string())
                .with_auth_method(AuthenticationMethod::Wpa2Personal),
            None => AccessPointConfig::default().with_ssid(libs::SSID_DEFAULT),
        });

        let (controller, interfaces) = esp_radio::wifi::new(
            wifi,
            ControllerConfig::default().with_initial_config(ap_config),
        )
        .expect("failed to initialise WiFi");

        let net_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
            address: Ipv4Cidr::new(gw_ip, 24),
            gateway: Some(gw_ip),
            dns_servers: Default::default(),
        });

        let rng = Rng::new();
        let seed = (rng.random() as u64) << 32 | rng.random() as u64;

        let (stack, runner) = embassy_net::new(
            interfaces.access_point,
            net_config,
            mk_static!(StackResources<3>, StackResources::<3>::new()),
            seed,
        );

        spawner.spawn(ap_task(controller).expect("ap_task already spawned"));
        spawner.spawn(net_task(runner).expect("net_task already spawned"));

        stack.wait_config_up().await;
        defmt::info!(
            "AP up - SSID: {}, IP: {}",
            libs::SSID_DEFAULT,
            libs::GW_IP_ADDR_ENV.unwrap_or(libs::GW_IP_DEFAULT)
        );

        Self { stack }
    }

    /// Spawn the UDP listener task — updates CONTROLS on each valid packet
    pub fn listen_control(&self, spawner: Spawner) {
        spawner.spawn(udp_control_task(self.stack).expect("udp_task already spawned"));
    }

    /// Spawn the UDP task that waits for ground_control to say "start calibrating", then
    /// forwards pose updates from `sensors::run_calibration` back over UDP.
    #[cfg(feature = "calibrate")]
    pub fn listen_calibrate(&self, spawner: Spawner) {
        spawner.spawn(calibrate::task(self.stack).expect("calibrate_task already spawned"));
    }
}

#[embassy_executor::task]
async fn ap_task(controller: WifiController<'static>) {
    loop {
        match controller
            .wait_for_access_point_connected_event_async()
            .await
        {
            Ok(AccessPointStationEventInfo::Connected(_)) => defmt::info!("station connected"),
            Ok(AccessPointStationEventInfo::Disconnected(_)) => {
                defmt::info!("station disconnected")
            }
            Err(_) => {}
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface<'static>>) {
    runner.run().await
}

fn udp_soc(stack: Stack<'static>) -> UdpSocket<'static> {
    let rx_meta = mk_static!([PacketMetadata; 4], [PacketMetadata::EMPTY; 4]);
    let rx_buf = mk_static!([u8; 512], [0u8; 512]);
    let tx_meta = mk_static!([PacketMetadata; 4], [PacketMetadata::EMPTY; 4]);
    let tx_buf = mk_static!([u8; 512], [0u8; 512]);

    let port = libs::UDP_PORT_ENV
        .map(|o| o.parse::<u16>())
        .transpose()
        .expect("failed to parse UDP_PORT_ENV")
        .unwrap_or(libs::UDP_PORT_DEFAULT);

    let mut socket = UdpSocket::new(stack, rx_meta, rx_buf, tx_meta, tx_buf);
    socket.bind(port).expect("UDP bind failed");
    defmt::info!("UDP listening on port {}", libs::UDP_PORT_DEFAULT);
    socket
}

#[embassy_executor::task]
async fn udp_control_task(stack: Stack<'static>) {
    let soc = udp_soc(stack);
    let mut buf = [0u8; control::DEFAULT_SIZE]; // sized to exact packet
    loop {
        match soc.recv_from(&mut buf).await {
            Ok((n, meta)) if n == control::DEFAULT_SIZE => {
                if let Some(packet) = ControlPacket::from_bytes(&buf) {
                    defmt::trace!("packet received {:?}", defmt::Debug2Format(&packet));
                    CONTROLS.lock(|c| c.set(Some((packet, Instant::now()))));
                }
                #[cfg(feature = "telemetry")]
                if let Some((pkt, _ts)) = TELEMETRY.lock(|c| c.get())
                    && let Err(err) = soc.send_to(&pkt.to_bytes(), meta.endpoint).await
                {
                    defmt::warn!(
                        "failed to sent telemetry packet: {:?}",
                        defmt::Debug2Format(&err)
                    );
                }
                #[cfg(not(feature = "telemetry"))]
                let _ = meta;
            }
            Ok((n, _)) => defmt::warn!("unexpected UDP packet size: {}", n),
            Err(_) => {}
        }
    }
}

#[cfg(feature = "calibrate")]
pub mod calibrate {
    use embassy_net::{IpEndpoint, Stack};
    use embassy_sync::{
        blocking_mutex::raw::CriticalSectionRawMutex, pubsub::PubSubChannel, signal::Signal,
    };
    use libs::{
        calibrate::CalibrationMode,
        control::{self, ControlPacket},
    };

    // fires once we get task hears from ground_control
    // unblocking run_calibration to start the pose sequence
    pub static START: Signal<CriticalSectionRawMutex, IpEndpoint> = Signal::new();

    // pose transitions published by run_calibration, forwarded to ground_control here
    pub static EVENTS: PubSubChannel<CriticalSectionRawMutex, CalibrationMode, 8, 1, 1> =
        PubSubChannel::new();

    #[embassy_executor::task]
    pub(crate) async fn task(stack: Stack<'static>) {
        let soc = super::udp_soc(stack);

        // subscribe before signaling START
        let mut sub = EVENTS
            .subscriber()
            .expect("calibration subscriber already taken");

        // any valid control packet is the trigger
        let mut buf = [0u8; control::DEFAULT_SIZE];
        let endpoint = loop {
            match soc.recv_from(&mut buf).await {
                Ok((n, meta))
                    if n == control::DEFAULT_SIZE && ControlPacket::from_bytes(&buf).is_some() =>
                {
                    break meta.endpoint;
                }
                _ => {}
            }
        };
        START.signal(endpoint); // START CAlIBRATION!!

        loop {
            let mode = sub.next_message_pure().await;
            if let Err(err) = soc.send_to(&mode.to_bytes(), endpoint).await {
                defmt::warn!(
                    "failed to send calibration status: {:?}",
                    defmt::Debug2Format(&err)
                );
            }
        }
    }
}
