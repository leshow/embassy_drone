use std::io::ErrorKind;
use std::{
    net::{SocketAddr, UdpSocket},
    thread,
    time::Duration,
};

use gilrs::{Axis, Button, Event, EventType, Gilrs};
#[cfg(feature = "telemetry")]
use libs::telemetry::{TelemetryPacket, TELEMETRY_SIZE};
use libs::{
    calibrate::{CalibrationMode, CALIBRATION_SIZE},
    control::ControlPacket,
};
use tracing::{error, info, warn};

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt::init();

    if std::env::args().any(|a| a == "--calibrate") {
        return run_calibrate();
    }

    let mut gilrs = Gilrs::new().map_err(|e| anyhow::anyhow!("{}", e))?;
    info!("starting up ground_control");
    for (_id, gamepad) in gilrs.gamepads() {
        info!("{} is {:?}", gamepad.name(), gamepad.power_info());
    }

    loop {
        let (ip, port) = (libs::get_ip(), libs::get_port());
        let socket = match UdpSocket::bind("0.0.0.0:0") {
            Ok(s) => {
                info!("connected to {}:{}", ip, port);
                // short timeout so a missing telemetry reply doesn't stall the input loop
                #[cfg(feature = "telemetry")]
                s.set_read_timeout(Some(Duration::from_millis(20))).ok();
                s
            }
            Err(e) => {
                info!("connect failed: {e}, retrying in 200ms...");
                thread::sleep(Duration::from_millis(200));
                continue;
            }
        };

        let mut pkt = ControlPacket::new(0, 0.0, 0.0, 0.0, false);
        // only log the routine telemetry line when something actually changed
        #[cfg(feature = "telemetry")]
        let mut last_telemetry: Option<TelemetryPacket> = None;

        'inner: loop {
            // block up to 20ms waiting for input, then send current state regardless
            while let Some(Event { event, .. }) =
                gilrs.next_event_blocking(Some(Duration::from_millis(20)))
            // this 20ms loop drives the telemetry packets, they only get sent in response to
            // CONTROL packets, so if no input changes (no throttle/direction),
            // control packets still get sent every 20ms and we get telemetry data
            {
                match event {
                    EventType::AxisChanged(Axis::LeftStickY, value, _) => {
                        pkt.throttle = (value.max(0.0) * 100.0) as u8;
                        info!("throttle: {}", pkt.throttle);
                    }
                    EventType::AxisChanged(Axis::RightStickX, value, _) => {
                        pkt.roll = value;
                        info!("roll: {}", value);
                    }
                    EventType::AxisChanged(Axis::RightStickY, value, _) => {
                        pkt.pitch = value;
                        info!("pitch: {}", value);
                    }
                    EventType::AxisChanged(Axis::LeftStickX, val, _) => {
                        pkt.yaw = val;
                        info!("yaw: {}", val);
                    }
                    // start button toggles arm
                    EventType::ButtonPressed(Button::Start, _) => {
                        pkt.set_armed(!pkt.armed());
                        info!("armed: {}", pkt.armed());
                    }
                    EventType::Disconnected => {
                        info!("gamepad disconnected — disarming");
                        pkt.throttle = 0;
                        pkt.set_armed(false);
                        break 'inner;
                    }
                    EventType::Connected => info!("gamepad connected"),
                    _ => {}
                }
            }
            if let Err(e) = socket.send_to(&pkt.to_bytes(), (ip, port)) {
                info!("send error: {e}");
                break 'inner;
            }

            #[cfg(feature = "telemetry")]
            {
                let mut tbuf = [0u8; TELEMETRY_SIZE];
                match socket.recv_from(&mut tbuf) {
                    Ok((n, a)) if n == TELEMETRY_SIZE && a == SocketAddr::from((ip, port)) => {
                        if let Some(t) = TelemetryPacket::from_bytes(&tbuf) {
                            if last_telemetry != Some(t) {
                                #[cfg(not(feature = "telemetry-verbose"))]
                                let msg = format!(
                                    "telemetry: roll={:.1} pitch={:.1} yaw={:.1} armed={} failsafe={} fifo_overflow={} motors={:?}",
                                    t.roll,
                                    t.pitch,
                                    t.yaw,
                                    t.armed(),
                                    t.failsafe(),
                                    t.fifo_overflow(),
                                    t.motors
                                );
                                #[cfg(feature = "telemetry-verbose")]
                                let msg = format!(
                                    "telemetry: roll={:.1} pitch={:.1} yaw={:.1} armed={} failsafe={} fifo_overflow={} motors={:?} gyro={:?} torques={:?} gyro_bias={:?} dt={:.4} accel={:?}",
                                    t.roll,
                                    t.pitch,
                                    t.yaw,
                                    t.armed(),
                                    t.failsafe(),
                                    t.fifo_overflow(),
                                    t.motors,
                                    t.gyro,
                                    t.torques,
                                    t.gyro_bias,
                                    t.dt,
                                    t.accel
                                );

                                if t.fifo_overflow() {
                                    warn!("{msg}");
                                } else {
                                    info!("{msg}");
                                }
                            }
                            last_telemetry = Some(t);
                        }
                    }
                    Ok(_) => {}
                    Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
                    Err(e) => info!("telemetry recv error: {e}"),
                }
            }
        }
    }
}

// the firmware only speaks this protocol while it's running `--features calibrate`. any
// ControlPacket is treated by the firmware as "start calibrating", so send one and keep
// resending until we hear back, in case the first one is dropped.
fn run_calibrate() -> anyhow::Result<()> {
    let (ip, port) = (libs::get_ip(), libs::get_port());
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    socket.connect((ip, port))?;
    socket.set_read_timeout(Some(Duration::from_millis(500)))?;
    info!("connected to {ip}:{port} - waiting to start calibration");

    let start = ControlPacket::new(0, 0.0, 0.0, 0.0, false);
    let mut buf = [0u8; CALIBRATION_SIZE];
    let mut started = false;
    loop {
        if !started {
            socket.send(&start.to_bytes())?;
        }
        match socket.recv(&mut buf) {
            Ok(n) if n == CALIBRATION_SIZE => {
                started = true;
                let Some(mode) = CalibrationMode::from_bytes(&buf) else {
                    continue;
                };
                match mode {
                    CalibrationMode::Ended => {
                        info!("=== CALIBRATION COMPLETE - saved ===");
                        return Ok(());
                    }
                    CalibrationMode::Failed => {
                        error!("=== CALIBRATION FAILED - not saved ===");
                        return Ok(());
                    }
                    pose => warn!("place: {pose}"),
                }
            }
            Ok(_) => {}
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
            Err(e) => return Err(e.into()),
        }
    }
}
