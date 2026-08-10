use nalgebra::Vector3;

use crate::flight::fusion;

// IMU mounting tilt relative to the frame, measured at rest with the frame level (see the
// "fusion" debug log's roll/pitch) - per-axis accel bias/scale calibration can't correct a
// whole-body mounting rotation, only this can. Pitch isn't corrected: it sits within +/-0.1 deg
// of zero at rest across many samples, which is noise, not a real offset. Only applied on the
// software-fusion path - the DMP does its own on-chip fusion from uncorrected raw readings.
pub const MOUNT_TILT_ROLL_RAD: f32 = 1.15 * fusion::DEG_TO_RAD;

// cutoff for the gyro rate filter and the rate PIDs' D-term filter - matches flix's
// ratesFilter/RATES_D_LPF_ALPHA, both ~40 Hz. Bench testing (no motors spinning) couldn't
// surface the need for this - real prop/motor vibration lands well inside this band and was
// feeding straight into the rate loop unfiltered, confirmed by real-flight logs showing motor
// output saturating (0 <-> max) once real thrust builds up, despite a clean low-throttle ramp
pub const RATE_LPF_HZ: f32 = 40.0;

// same idea, applied to accel instead of gyro rate - matches betaflight's acceleration.c
// pt2Filter default (accLpfCutHz=25). a single-pole filter at 40 Hz still left real prop
// vibration corrupting most samples - see Lpf3::new_two_pole for the steeper two-stage filter
// this needs to hit 25 Hz cleanly
pub const ACCEL_LPF_HZ: f32 = 25.0;

// accel norm outside this band means the reading isn't (close to) pure gravity - real
// vibration or motion is mixed in, so its direction can't be trusted as a "down" reference
// this tick. matches betaflight's imuIsAccelerometerHealthy() (0.9g-1.1g) - confirmed on our
// own hardware tonight: clean readings sit within ~1% of 1g, corrupted ones swing far outside
// this band (measured as low as 0.145g, as high as 1.944g during real vibration)
pub const ACCEL_HEALTHY_MIN: f32 = 0.9;
pub const ACCEL_HEALTHY_MAX: f32 = 1.1;

// discrete low-pass filter gain for a given cutoff and this tick's dt - matches flix's
// LowPassFilter::setCutOffFrequency. Recomputed every tick since dt isn't fixed (interrupt
// driven loop, not a hard real-time scheduler)
pub fn lpf_alpha(cutoff_hz: f32, dt: f32) -> f32 {
    1.0 - libm::expf(-2.0 * core::f32::consts::PI * cutoff_hz * dt)
}

// per-stage cutoff correction for a two-pole (PT2) cascade - matches betaflight's
// CUTOFF_CORRECTION_PT2 (1/sqrt(2^(1/2)-1)). without this, cascading two PT1 stages at the same
pub const CUTOFF_CORRECTION_PT2: f32 = 1.553_774;

// low-pass filter over a Vector3 signal - shared by the gyro rate filter and the accel filter
// below. matches flix's ratesFilter / betaflight's pt1Filter for the single-pole case; accel
// needs steeper rolloff so it opts into a second cascaded stage - see new_two_pole
pub struct Lpf3 {
    state: Vector3<f32>,
    stage1: Vector3<f32>,
    two_pole: bool,
    initialized: bool,
    cutoff_hz: f32,
}

impl Lpf3 {
    pub fn new(cutoff_hz: f32) -> Self {
        Self {
            state: Vector3::zeros(),
            stage1: Vector3::zeros(),
            two_pole: false,
            initialized: false,
            cutoff_hz,
        }
    }

    // two cascaded PT1 stages sharing one gain - matches betaflight's pt2Filter, -40dB/decade
    // instead of -20dB/decade. cutoff_hz gets corrected (CUTOFF_CORRECTION_PT2) before computing
    // that gain so the cascade's actual -3dB point still lands at cutoff_hz, not higher
    pub fn new_two_pole(cutoff_hz: f32) -> Self {
        Self {
            two_pole: true,
            ..Self::new(cutoff_hz)
        }
    }

    pub fn update(&mut self, input: Vector3<f32>, dt: f32) -> Vector3<f32> {
        if !self.initialized {
            self.state = input;
            self.stage1 = input;
            self.initialized = true;
            return input;
        }
        if self.two_pole {
            let alpha = lpf_alpha(self.cutoff_hz * CUTOFF_CORRECTION_PT2, dt);
            self.stage1 += alpha * (input - self.stage1);
            self.state += alpha * (self.stage1 - self.state);
        } else {
            self.state += lpf_alpha(self.cutoff_hz, dt) * (input - self.state);
        }
        self.state
    }
}
