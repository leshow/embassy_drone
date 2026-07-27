#[cfg(feature = "telemetry-verbose")]
use nalgebra::Vector3;

use crate::control::Flags;

pub const TELEMETRY_MAGIC: [u8; 4] = *b"TELM";

// 4 (magic) + 4+4+4 (roll/pitch/yaw f32 be) + 2+2+2+2 (motor duties u16 be) + 1 (flags)
#[cfg(not(feature = "telemetry-verbose"))]
pub const TELEMETRY_SIZE: usize = 25;
// base 25 + 4+4+4 (raw gyro rad/s f32 be) + 4+4+4 (roll/pitch/yaw torque f32 be)
// + 4+4+4 (gyro bias rad/s f32 be) + 4 (loop dt seconds f32 be) + 4+4+4 (accel g f32 be)
// - all telemetry-verbose only
#[cfg(feature = "telemetry-verbose")]
pub const TELEMETRY_SIZE: usize = 77;

/// telemetry packet sent from the ESP32 back to ground control over UDP, in reply to a
/// received control packet. serialized as big-endian; layout depends on the
/// `telemetry-verbose` feature (must match on both ends of the link).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TelemetryPacket {
    /// roll, in degrees
    pub roll: f32,
    /// pitch, in degrees
    pub pitch: f32,
    /// yaw, in degrees
    pub yaw: f32,
    /// motor duty cycles, in raw PWM hardware ticks: (front-left, front-right, rear-left, rear-right)
    pub motors: (u16, u16, u16, u16),
    /// raw gyro rates in rad/s: (x, y, z)
    #[cfg(feature = "telemetry-verbose")]
    pub gyro: Vector3<f32>,
    /// inner-loop PID output before motor mixing: (roll, pitch, yaw)
    #[cfg(feature = "telemetry-verbose")]
    pub torques: Vector3<f32>,
    /// current gyro zero-rate bias estimate in rad/s: (x, y, z) - always zero on the DMP
    /// path, which calibrates the gyro on-chip instead of via BiasTracker
    #[cfg(feature = "telemetry-verbose")]
    pub gyro_bias: Vector3<f32>,
    /// time since the previous control-loop tick, in seconds - lets ground_control see actual
    /// loop jitter, since the loop is interrupt-driven rather than fixed-rate
    #[cfg(feature = "telemetry-verbose")]
    pub dt: f32,
    /// accelerometer reading in g, bias + mount-trim corrected as fed to the fusion filter -
    /// always zero on the DMP path, which fuses on-chip from uncorrected raw readings
    #[cfg(feature = "telemetry-verbose")]
    pub accel: Vector3<f32>,
    /// armed + failsafe flags
    flags: Flags,
}

impl TelemetryPacket {
    #[cfg(not(feature = "telemetry-verbose"))]
    pub fn new(
        roll: f32,
        pitch: f32,
        yaw: f32,
        motors: (u16, u16, u16, u16),
        armed: bool,
        failsafe: bool,
    ) -> Self {
        let mut flags = Flags::new(0);
        flags.set_armed(armed);
        flags.set_failsafe(failsafe);
        Self {
            roll,
            pitch,
            yaw,
            motors,
            flags,
        }
    }

    #[cfg(feature = "telemetry-verbose")]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        roll: f32,
        pitch: f32,
        yaw: f32,
        motors: (u16, u16, u16, u16),
        armed: bool,
        failsafe: bool,
        gyro: Vector3<f32>,
        torques: Vector3<f32>,
        gyro_bias: Vector3<f32>,
        dt: f32,
        accel: Vector3<f32>,
    ) -> Self {
        let mut flags = Flags::new(0);
        flags.set_armed(armed);
        flags.set_failsafe(failsafe);
        Self {
            roll,
            pitch,
            yaw,
            motors,
            gyro,
            torques,
            gyro_bias,
            dt,
            accel,
            flags,
        }
    }

    /// marks the packet as having been produced right after a DMP FIFO overflow -
    /// set directly on the cached telemetry packet, doesn't require rebuilding one
    pub fn set_fifo_overflow(&mut self, b: bool) {
        self.flags.set_fifo_overflow(b);
    }

    pub fn to_bytes(&self) -> [u8; TELEMETRY_SIZE] {
        let mut buf = [0u8; TELEMETRY_SIZE];
        buf[0..4].copy_from_slice(&TELEMETRY_MAGIC);
        buf[4..8].copy_from_slice(&self.roll.to_be_bytes());
        buf[8..12].copy_from_slice(&self.pitch.to_be_bytes());
        buf[12..16].copy_from_slice(&self.yaw.to_be_bytes());
        buf[16..18].copy_from_slice(&self.motors.0.to_be_bytes());
        buf[18..20].copy_from_slice(&self.motors.1.to_be_bytes());
        buf[20..22].copy_from_slice(&self.motors.2.to_be_bytes());
        buf[22..24].copy_from_slice(&self.motors.3.to_be_bytes());
        buf[24] = Flags::to_bytes(&self.flags);
        #[cfg(feature = "telemetry-verbose")]
        {
            buf[25..29].copy_from_slice(&self.gyro.x.to_be_bytes());
            buf[29..33].copy_from_slice(&self.gyro.y.to_be_bytes());
            buf[33..37].copy_from_slice(&self.gyro.z.to_be_bytes());
            buf[37..41].copy_from_slice(&self.torques.x.to_be_bytes());
            buf[41..45].copy_from_slice(&self.torques.y.to_be_bytes());
            buf[45..49].copy_from_slice(&self.torques.z.to_be_bytes());
            buf[49..53].copy_from_slice(&self.gyro_bias.x.to_be_bytes());
            buf[53..57].copy_from_slice(&self.gyro_bias.y.to_be_bytes());
            buf[57..61].copy_from_slice(&self.gyro_bias.z.to_be_bytes());
            buf[61..65].copy_from_slice(&self.dt.to_be_bytes());
            buf[65..69].copy_from_slice(&self.accel.x.to_be_bytes());
            buf[69..73].copy_from_slice(&self.accel.y.to_be_bytes());
            buf[73..77].copy_from_slice(&self.accel.z.to_be_bytes());
        }
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < TELEMETRY_SIZE {
            return None;
        }
        if buf[0..4] != TELEMETRY_MAGIC {
            return None;
        }
        Some(Self {
            roll: f32::from_be_bytes(buf[4..8].try_into().ok()?),
            pitch: f32::from_be_bytes(buf[8..12].try_into().ok()?),
            yaw: f32::from_be_bytes(buf[12..16].try_into().ok()?),
            motors: (
                u16::from_be_bytes(buf[16..18].try_into().ok()?),
                u16::from_be_bytes(buf[18..20].try_into().ok()?),
                u16::from_be_bytes(buf[20..22].try_into().ok()?),
                u16::from_be_bytes(buf[22..24].try_into().ok()?),
            ),
            flags: Flags::from_bytes(buf[24]),
            #[cfg(feature = "telemetry-verbose")]
            gyro: Vector3::new(
                f32::from_be_bytes(buf[25..29].try_into().ok()?),
                f32::from_be_bytes(buf[29..33].try_into().ok()?),
                f32::from_be_bytes(buf[33..37].try_into().ok()?),
            ),
            #[cfg(feature = "telemetry-verbose")]
            torques: Vector3::new(
                f32::from_be_bytes(buf[37..41].try_into().ok()?),
                f32::from_be_bytes(buf[41..45].try_into().ok()?),
                f32::from_be_bytes(buf[45..49].try_into().ok()?),
            ),
            #[cfg(feature = "telemetry-verbose")]
            gyro_bias: Vector3::new(
                f32::from_be_bytes(buf[49..53].try_into().ok()?),
                f32::from_be_bytes(buf[53..57].try_into().ok()?),
                f32::from_be_bytes(buf[57..61].try_into().ok()?),
            ),
            #[cfg(feature = "telemetry-verbose")]
            dt: f32::from_be_bytes(buf[61..65].try_into().ok()?),
            #[cfg(feature = "telemetry-verbose")]
            accel: Vector3::new(
                f32::from_be_bytes(buf[65..69].try_into().ok()?),
                f32::from_be_bytes(buf[69..73].try_into().ok()?),
                f32::from_be_bytes(buf[73..77].try_into().ok()?),
            ),
        })
    }

    pub fn armed(&self) -> bool {
        self.flags.armed()
    }

    pub fn failsafe(&self) -> bool {
        self.flags.failsafe()
    }

    pub fn fifo_overflow(&self) -> bool {
        self.flags.fifo_overflow()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(not(feature = "telemetry-verbose"))]
    #[test]
    fn test_telemetry_packet_roundtrip() {
        let pkt = TelemetryPacket::new(12.5, -3.25, 180.0, (100, 200, 300, 400), true, false);
        let bytes = pkt.to_bytes();
        assert_eq!(TelemetryPacket::from_bytes(&bytes), Some(pkt));
    }

    #[cfg(feature = "telemetry-verbose")]
    #[test]
    fn test_telemetry_packet_roundtrip_verbose() {
        let pkt = TelemetryPacket::new(
            12.5,
            -3.25,
            180.0,
            (100, 200, 300, 400),
            true,
            false,
            Vector3::new(0.1, -0.2, 0.3),
            Vector3::new(0.4, -0.5, 0.6),
            Vector3::new(0.01, -0.02, 0.03),
            0.002,
            Vector3::new(0.0, 0.0, 1.0),
        );
        let bytes = pkt.to_bytes();
        assert_eq!(TelemetryPacket::from_bytes(&bytes), Some(pkt));
    }

    #[test]
    fn test_telemetry_packet_rejects_bad_magic() {
        #[cfg(not(feature = "telemetry-verbose"))]
        let pkt = TelemetryPacket::new(0.0, 0.0, 0.0, (0, 0, 0, 0), false, false);
        #[cfg(feature = "telemetry-verbose")]
        let pkt = TelemetryPacket::new(
            0.0,
            0.0,
            0.0,
            (0, 0, 0, 0),
            false,
            false,
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            0.0,
            Vector3::new(0.0, 0.0, 0.0),
        );

        let mut bytes = pkt.to_bytes();
        bytes[0] = b'X';
        assert_eq!(TelemetryPacket::from_bytes(&bytes), None);
    }

    #[test]
    fn test_telemetry_packet_rejects_short_buffer() {
        #[cfg(not(feature = "telemetry-verbose"))]
        let pkt = TelemetryPacket::new(0.0, 0.0, 0.0, (0, 0, 0, 0), false, false);
        #[cfg(feature = "telemetry-verbose")]
        let pkt = TelemetryPacket::new(
            0.0,
            0.0,
            0.0,
            (0, 0, 0, 0),
            false,
            false,
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            0.0,
            Vector3::new(0.0, 0.0, 0.0),
        );

        let bytes = pkt.to_bytes();
        assert_eq!(
            TelemetryPacket::from_bytes(&bytes[..TELEMETRY_SIZE - 1]),
            None
        );
    }

    #[test]
    fn test_telemetry_packet_armed_and_failsafe() {
        #[cfg(not(feature = "telemetry-verbose"))]
        let pkt = TelemetryPacket::new(0.0, 0.0, 0.0, (0, 0, 0, 0), true, true);
        #[cfg(feature = "telemetry-verbose")]
        let pkt = TelemetryPacket::new(
            0.0,
            0.0,
            0.0,
            (0, 0, 0, 0),
            true,
            true,
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            0.0,
            Vector3::new(0.0, 0.0, 0.0),
        );

        assert!(pkt.armed());
        assert!(pkt.failsafe());
        assert!(!pkt.fifo_overflow());
    }

    #[test]
    fn test_telemetry_packet_set_fifo_overflow() {
        #[cfg(not(feature = "telemetry-verbose"))]
        let mut pkt = TelemetryPacket::new(0.0, 0.0, 0.0, (0, 0, 0, 0), true, false);
        #[cfg(feature = "telemetry-verbose")]
        let mut pkt = TelemetryPacket::new(
            0.0,
            0.0,
            0.0,
            (0, 0, 0, 0),
            true,
            false,
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            0.0,
            Vector3::new(0.0, 0.0, 0.0),
        );

        assert!(!pkt.fifo_overflow());
        pkt.set_fifo_overflow(true);
        assert!(pkt.fifo_overflow());
        // doesn't disturb the other flags
        assert!(pkt.armed());
        assert!(!pkt.failsafe());

        let bytes = pkt.to_bytes();
        assert_eq!(TelemetryPacket::from_bytes(&bytes), Some(pkt));
    }
}
