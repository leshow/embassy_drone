#![no_std]

use core::{net::Ipv4Addr, str::FromStr};

pub const SSID_DEFAULT: &str = "esp-quad";
pub const GW_IP_DEFAULT: &str = "192.168.4.1";
pub const UDP_PORT_DEFAULT: u16 = 4444;
pub const UDP_PORT_ENV: Option<&'static str> = option_env!("UDP_PORT");
pub const GW_IP_ADDR_ENV: Option<&'static str> = option_env!("GATEWAY_IP");
// set AP_PASSWORD at build time; if unset the AP is open
pub const AP_PASSWORD: Option<&'static str> = option_env!("AP_PASSWORD");

pub fn get_ip() -> Ipv4Addr {
    Ipv4Addr::from_str(GW_IP_ADDR_ENV.unwrap_or(GW_IP_DEFAULT)).expect("invalid GATEWAY_IP")
}

pub fn get_port() -> u16 {
    UDP_PORT_ENV
        .map(|o| o.parse::<u16>())
        .transpose()
        .expect("invalid UDP_PORT")
        .unwrap_or(UDP_PORT_DEFAULT)
}

pub mod calibrate;
pub mod control;
pub mod flight;
pub mod level_correction;
pub mod mag_calibration;
pub mod mixer;
#[cfg(feature = "telemetry")]
pub mod telemetry;

#[cfg(test)]
mod tests {
    // generated this test so I could verify swapping dmp::Quaternion to nalgebra for simplification
    // of firmware code:
    // cross-checks icm20948's Quaternion::to_euler_angles against nalgebra's
    // UnitQuaternion::euler_angles for the same quaternion, confirming both use the same
    // roll/pitch/yaw (ZYX) convention. angles stay well clear of gimbal lock
    // (pitch = +/-90 degrees), where the two libraries make different tie-breaking choices.
    #[test]
    fn to_euler_angles_matches_nalgebra() {
        let cases = [
            (0.0_f32, 0.0_f32, 0.0_f32),
            (0.2, 0.0, 0.0),
            (0.0, 0.3, 0.0),
            (0.0, 0.0, 0.5),
            (0.1, 0.2, 0.3),
            (-0.4, 0.25, -0.6),
            (1.0, -0.5, 2.0),
        ];

        for (roll, pitch, yaw) in cases {
            let nq = nalgebra::UnitQuaternion::from_euler_angles(roll, pitch, yaw);
            let ours = icm20948::dmp::Quaternion::new(nq.w, nq.i, nq.j, nq.k);
            let euler = ours.to_euler_angles();

            assert!(
                (euler.roll - roll).abs() < 1e-4,
                "roll mismatch: got {} want {}",
                euler.roll,
                roll
            );
            assert!(
                (euler.pitch - pitch).abs() < 1e-4,
                "pitch mismatch: got {} want {}",
                euler.pitch,
                pitch
            );
            assert!(
                (euler.yaw - yaw).abs() < 1e-4,
                "yaw mismatch: got {} want {}",
                euler.yaw,
                yaw
            );

            // and matches nalgebra's own extraction directly
            let (nroll, npitch, nyaw) = nq.euler_angles();
            assert!((euler.roll - nroll).abs() < 1e-4);
            assert!((euler.pitch - npitch).abs() < 1e-4);
            assert!((euler.yaw - nyaw).abs() < 1e-4);
        }
    }
}
