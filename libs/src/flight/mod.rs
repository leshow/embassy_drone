pub mod filters;
pub mod fusion;
pub mod pid;
pub mod sensors;

use nalgebra::Vector3;

/// AccelBias
///
/// One-time accelerometer bias + scale correction, computed by the 6-orientation tumble
/// calibration (behind `--features calibrate`) and loaded from flash at boot - see
/// `calibration_storage`. Unlike gyro bias this isn't re-learned continuously; accel bias
/// mostly comes from IMU mounting tilt and silicon offset, both fixed for a given build.
/// Needed in all builds (not just non-DMP): `run_control`'s signature takes one unconditionally,
/// even though the DMP path leaves it unused (DMP fuses from uncorrected raw readings instead).
#[derive(Clone, Copy, Debug)]
pub struct AccelBias {
    pub bias: Vector3<f32>,
    pub scale: Vector3<f32>,
}

impl Default for AccelBias {
    fn default() -> Self {
        Self {
            bias: Vector3::zeros(),
            scale: Vector3::new(1.0, 1.0, 1.0),
        }
    }
}

impl AccelBias {
    // matches flix's apply step exactly: acc = (acc - accBias) / accScale
    pub fn apply(&self, accel: Vector3<f32>) -> Vector3<f32> {
        (accel - self.bias).component_div(&self.scale)
    }
}

pub const fn parse_u64(s: &str) -> u64 {
    // unfortunately parse is not a const fn
    let b = s.as_bytes();
    let mut n = 0u64;
    let mut i = 0;
    // no for loops in const either? damn.
    while i < b.len() {
        n = n * 10 + (b[i] - b'0') as u64;
        i += 1;
    }
    n
}
