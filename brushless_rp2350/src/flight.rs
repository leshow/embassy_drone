use crate::{CtrlRx, motors::Motors};
use defmt::{debug, error, info};
use embassy_rp::{
    Peri,
    gpio::{Input, Pull},
    peripherals::PIN_6,
};
use embassy_time::{Duration, Instant};

use {defmt_rtt as _, panic_probe as _};

use libs::flight::{
    fusion::{FusionBuilder, RAD_TO_DEG},
    pid::Pid,
    sensors::ImuRead,
};

mod vals {
    use libs::flight::fusion;

    // max roll/pitch command from stick (+/- 25 deg)
    pub const MAX_TILT_RAD: f32 = 25.0 * fusion::DEG_TO_RAD;

    // outer loop p gains
    pub const ANGLE_P_ROLL_PITCH: f32 = 4.0;
    // and angle_p_yaw means the controller will eventually torque
    // quad to chase drift
    pub const ANGLE_P_YAW: f32 = 0.0;

    // inner loop - only P is live for the first test. D and I come in one at a time, in that
    // order, once the previous term is confirmed stable. INTEGRAL_LIMIT/LEAK below stay
    // inert while their Ki is 0 - both only ever shape the accumulator's value, and Ki=0
    // zeroes that value's contribution to the output regardless
    pub const RATE_KP_ROLL_PITCH: f32 = 0.05;
    pub const RATE_KI_ROLL_PITCH: f32 = 0.0; // was 0.3
    pub const RATE_KD_ROLL_PITCH: f32 = 0.0; // was 0.001

    pub const RATE_KP_YAW: f32 = 0.3; // flix 0.3 was set to 0.2
    pub const RATE_KI_YAW: f32 = 0.0; // was 0.05
    pub const RATE_KD_YAW: f32 = 0.0; // was 0.001

    pub const INTEGRAL_LIMIT: f32 = 0.3;
    // per-second decay on the rate pids' integral term - without this, any permanent small error
    // eventually winds the integral to integral_limit given enough time, no matter how small the error is. with
    // decay, a persistent error settles at an equilibrium of error/rate_integral_leak instead,
    // while a genuinely larger disturbance still gets proportionally more integral authority
    pub const RATE_INTEGRAL_LEAK: f32 = 1.0;

    // outer loop angle error deadband - a resting tilt under this is mounting tolerance subtracts the band rather than clamping to it, so there's
    // no discontinuity in commanded rate right at the edge. roll/pitch only: yaw can't wind up
    // (rate_ki_yaw = 0) and has its own heading-hold handling already
    pub const ANGLE_DEADBAND_RAD: f32 = 0.5 * fusion::DEG_TO_RAD;
}

mod util {

    pub(crate) fn deadband(x: f32, band: f32) -> f32 {
        if x > band {
            x - band
        } else if x < -band {
            x + band
        } else {
            0.0
        }
    }

    // normalises an angle to [-pi, pi] for the shortest-path yaw error
    pub(crate) fn wrap_angle(a: f32) -> f32 {
        use core::f32::consts::PI;
        // fmodf gives the same sign as the dividend, so shift into [0, 2pi) before subtracting back
        let r = libm::fmodf(a + PI, 2.0 * PI);
        (if r < 0.0 { r + 2.0 * PI } else { r }) - PI
    }

    // running min/max/avg of loop dt between periodic log lines - lets the actual loop rate be
    // checked without needing per-tick logging
    pub struct DtStats {
        pub min: f32,
        pub max: f32,
        pub sum: f32,
        pub n: u32,
    }

    impl DtStats {
        pub const fn new() -> Self {
            Self {
                min: f32::MAX,
                max: 0.0,
                sum: 0.0,
                n: 0,
            }
        }

        pub fn record(&mut self, dt: f32) {
            self.min = self.min.min(dt);
            self.max = self.max.max(dt);
            self.sum += dt;
            self.n += 1;
        }

        pub fn avg(&self) -> f32 {
            self.sum / self.n as f32
        }
    }
}

pub async fn run<'a, D>(
    mut sensor: crate::imu::Sensor<icm426xx::ICM42688<D, icm426xx::Ready>>,
    mut motors: Motors<'a>,
    int1: Peri<'static, PIN_6>,
    mut rx: CtrlRx<'a>,
) where
    D: embedded_hal_async::spi::SpiDevice,
    D::Error: defmt::Format,
{
    let mut log_count: u32 = 0;
    let mut int1 = Input::new(int1, Pull::None);
    let mut fusion = FusionBuilder::new().icm42688().madgwick().build();
    let mut last = Instant::now();
    let mut dt_stats = util::DtStats::new();

    // inner rate PIDs
    let mut roll_pid = Pid::new(
        vals::RATE_KP_ROLL_PITCH,
        vals::RATE_KI_ROLL_PITCH,
        vals::RATE_KD_ROLL_PITCH,
        vals::INTEGRAL_LIMIT,
        vals::RATE_INTEGRAL_LEAK,
    );
    let mut pitch_pid = Pid::new(
        vals::RATE_KP_ROLL_PITCH,
        vals::RATE_KI_ROLL_PITCH,
        vals::RATE_KD_ROLL_PITCH,
        vals::INTEGRAL_LIMIT,
        vals::RATE_INTEGRAL_LEAK,
    );
    let mut yaw_pid = Pid::new(
        vals::RATE_KP_YAW,
        vals::RATE_KI_YAW,
        vals::RATE_KD_YAW,
        vals::INTEGRAL_LIMIT,
        vals::RATE_INTEGRAL_LEAK,
    );

    let mut target_yaw: f32 = 0.0;
    let mut yaw_init = false;
    let mut last_armed = false;

    loop {
        int1.wait_for_high().await;

        // no LPF or gyro-bias tracking yet unlike esp32s3
        let (accel, gyro) = match sensor.read().await {
            Ok(sample) => sample,
            Err(e) => {
                error!("ICM42688 read_sample failed: {}", e);
                continue;
            }
        };

        let now = Instant::now();
        let dt = now.duration_since(last).as_micros() as f32 / 1_000_000.0;
        last = now;
        dt_stats.record(dt);

        let quat = fusion.update(dt, accel, gyro);
        let (actual_roll, actual_pitch, actual_yaw) = quat.euler_angles();

        log_count += 1;
        if log_count >= crate::LOG_EVERY_N {
            log_count = 0;
            debug!(
                "loop dt min: {} max: {} avg: {} | roll: {}\u{b0} pitch: {}\u{b0} yaw: {}\u{b0}",
                dt_stats.min,
                dt_stats.max,
                dt_stats.avg(),
                actual_roll * RAD_TO_DEG,
                actual_pitch * RAD_TO_DEG,
                actual_yaw * RAD_TO_DEG,
            );
            dt_stats = util::DtStats::new();
        }

        // latest control input, with a staleness check for failsafe (no pkt in last 500ms)
        let controls = rx.try_get();
        let fresh = controls.is_some_and(|(_, at)| at.elapsed() < Duration::from_millis(500));
        let armed = fresh && controls.is_some_and(|(c, _)| c.armed);

        if !last_armed && armed {
            info!("ARMED");
            roll_pid.reset();
            pitch_pid.reset();
            yaw_pid.reset();
            yaw_init = false;
        } else if last_armed && !armed {
            info!("DISARMED");
        }
        last_armed = armed;

        // fusion tracking above always runs regardless of arm state, so there's no cold-start
        // lag in the attitude estimate the moment it does arm
        let Some((ctrl, _)) = controls.filter(|_| armed) else {
            motors.turn_off().await;
            continue;
        };

        if !yaw_init {
            target_yaw = actual_yaw;
            yaw_init = true;
        }
        // near the ground, keep target tracking actual so handling the drone doesn't build a
        // large yaw error
        if ctrl.throttle < 0.15 {
            target_yaw = actual_yaw;
        }
        // heading hold: only update target when yaw stick is actually being pushed
        if ctrl.yaw.abs() >= 0.1 {
            target_yaw = actual_yaw;
        }
        // stick feedforward adds directly to yaw rate setpoint
        let yaw_ff = if ctrl.yaw.abs() >= 0.1 {
            -ctrl.yaw * core::f32::consts::PI // +/- pi rad/s
        } else {
            0.0
        };

        if ctrl.throttle < 0.05 {
            motors.turn_off().await;
            continue;
        }

        // outer angle-P loop: stick angle -> rate setpoint
        let target_roll = ctrl.roll * vals::MAX_TILT_RAD;
        let target_pitch = ctrl.pitch * vals::MAX_TILT_RAD;
        let roll_diff = util::deadband(
            util::wrap_angle(target_roll - actual_roll),
            vals::ANGLE_DEADBAND_RAD,
        );
        let pitch_diff = util::deadband(
            util::wrap_angle(target_pitch - actual_pitch),
            vals::ANGLE_DEADBAND_RAD,
        );
        let roll_rate_sp = vals::ANGLE_P_ROLL_PITCH * roll_diff;
        let pitch_rate_sp = vals::ANGLE_P_ROLL_PITCH * pitch_diff;
        let yaw_rate_sp = vals::ANGLE_P_YAW * util::wrap_angle(target_yaw - actual_yaw) + yaw_ff;

        // inner rate PID: rate setpoint vs actual gyro rate -> torque
        let roll_torque = roll_pid.update(roll_rate_sp - gyro.x, dt);
        let pitch_torque = pitch_pid.update(pitch_rate_sp - gyro.y, dt);
        let yaw_torque = yaw_pid.update(yaw_rate_sp - gyro.z, dt);

        // THROTTLE_CAP applied to the commanded throttle here, before mixing like betaflight's scale
        let capped_throttle = ctrl.throttle * (crate::THROTTLE_CAP as f32 / 100.0);
        let (fl, fr, rl, rr) =
            libs::mixer::mix_motors(capped_throttle, roll_torque, pitch_torque, yaw_torque);
        let duty = motors.set_motors(fl, fr, rl, rr).await;

        if log_count == 0 {
            debug!(
                "torques roll: {} pitch: {} yaw: {} | mix fl: {} fr: {} rl: {} rr: {} | duty fl: {} fr: {} rl: {} rr: {}",
                roll_torque,
                pitch_torque,
                yaw_torque,
                fl,
                fr,
                rl,
                rr,
                duty[0],
                duty[1],
                duty[2],
                duty[3],
            );
        }
    }
}
