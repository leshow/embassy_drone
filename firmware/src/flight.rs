use embassy_time::{Duration, Instant};
use esp_hal::gpio;
#[cfg(feature = "telemetry")]
use libs::telemetry::TelemetryPacket;
use nalgebra::{UnitQuaternion, Vector3};

use crate::sensors::ImuRead;
#[cfg(feature = "mag")]
use crate::sensors::ImuReadMag;
use crate::{Motors, Sensor20948, wifi};
#[cfg(feature = "mag")]
use libs::flight::fusion::MargFusion;
pub use libs::flight::AccelBias;
use libs::flight::{filters, fusion, fusion::ImuFusion, pid::Pid};

// publishes a telemetry snapshot for the wifi udp_task to reply with on the next control packet.
// called once per loop iteration, from whichever exit point (early continue or full mix) is
// actually taken, so telemetry is only ever built once per tick.
#[cfg(all(feature = "telemetry", not(feature = "telemetry-verbose")))]
fn publish_telemetry(
    euler: (f32, f32, f32),       // (roll, pitch, yaw) radians
    motors: (u16, u16, u16, u16), // fl fr rl rr
    armed: bool,
    failsafe: bool,
) {
    let roll_deg = euler.0 * fusion::RAD_TO_DEG;
    let pitch_deg = euler.1 * fusion::RAD_TO_DEG;
    let yaw_deg = euler.2 * fusion::RAD_TO_DEG;

    let pkt = TelemetryPacket::new(roll_deg, pitch_deg, yaw_deg, motors, armed, failsafe);
    wifi::TELEMETRY.lock(|c| c.set(Some((pkt, Instant::now()))));
}

// publishes a telemetry snapshot for the wifi udp_task to reply with on the next control packet.
// called once per loop iteration, from whichever exit point (early continue or full mix) is
// actually taken, so telemetry is only ever built once per tick.
#[cfg(feature = "telemetry-verbose")]
#[allow(clippy::too_many_arguments)]
fn publish_telemetry(
    euler: (f32, f32, f32),       // (roll, pitch, yaw) radians
    motors: (u16, u16, u16, u16), // fl fr rl rr
    armed: bool,
    failsafe: bool,
    gyro: Vector3<f32>,
    torques: Vector3<f32>,   // roll/pitch/yaw PID output before mixing
    gyro_bias: Vector3<f32>, // current BiasTracker estimate, zero on the DMP path
    dt: f32,                 // seconds since the previous loop tick
    accel: Vector3<f32>,     // bias + mount-trim corrected accel in g, zero on the DMP path
) {
    let roll_deg = euler.0 * fusion::RAD_TO_DEG;
    let pitch_deg = euler.1 * fusion::RAD_TO_DEG;
    let yaw_deg = euler.2 * fusion::RAD_TO_DEG;

    let pkt = TelemetryPacket::new(
        roll_deg, pitch_deg, yaw_deg, motors, armed, failsafe, gyro, torques, gyro_bias, dt, accel,
    );
    wifi::TELEMETRY.lock(|c| c.set(Some((pkt, Instant::now()))));
}

// same field set as publish_telemetry's verbose variant, logged locally over defmt trace
// instead of published over wifi - unconditional on the telemetry feature, so dt/attitude/etc
// stay visible with wifi telemetry compiled out entirely. free at the default DEFMT_LOG=info
// (see .cargo/config.toml) - trace! compiles down to a no-op unless DEFMT_LOG=trace is set
#[allow(clippy::too_many_arguments)]
fn trace_telemetry(
    euler: (f32, f32, f32),       // (roll, pitch, yaw) radians
    motors: (u16, u16, u16, u16), // fl fr rl rr
    armed: bool,
    failsafe: bool,
    gyro: Vector3<f32>,
    torques: Vector3<f32>,   // roll/pitch/yaw PID output before mixing
    gyro_bias: Vector3<f32>, // current BiasTracker estimate, zero on the DMP path
    dt: f32,                 // seconds since the previous loop tick
    accel: Vector3<f32>,     // bias + mount-trim corrected accel in g, zero on the DMP path
) {
    defmt::trace!(
        "telemetry roll={}° pitch={}° yaw={}° armed={} failsafe={} | motors fl={} fr={} rl={} rr={} | gyro x={} y={} z={} | torques roll={} pitch={} yaw={} | bias x={} y={} z={} | dt={} | accel x={} y={} z={}",
        euler.0 * fusion::RAD_TO_DEG,
        euler.1 * fusion::RAD_TO_DEG,
        euler.2 * fusion::RAD_TO_DEG,
        armed,
        failsafe,
        motors.0,
        motors.1,
        motors.2,
        motors.3,
        gyro.x,
        gyro.y,
        gyro.z,
        torques.x,
        torques.y,
        torques.z,
        gyro_bias.x,
        gyro_bias.y,
        gyro_bias.z,
        dt,
        accel.x,
        accel.y,
        accel.z,
    );
}

// max roll/pitch command from stick (+/- 25 deg)
const MAX_TILT_RAD: f32 = 25.0 * fusion::DEG_TO_RAD;

// outer loop P gains
const ANGLE_P_ROLL_PITCH: f32 = 4.0; // flix 6.0
// and angle_p_yaw means the controller will eventually torque
// quad to chase drift. we can enable it when we get mag working
const ANGLE_P_YAW: f32 = 0.0; // flix 3.0

// weak "assume roughly level" prior, applied every tick regardless of accel health - bounds
// roll/pitch drift during the stretches where the accel gate rejects the sample and nothing
// else is correcting the estimate (measured in some flight tests: gate rejects 53-81% of samples with
// motors spinning). matches flix's levelWeight - see libs::level_correction for the actual math
// and its tests. weight is a fixed fraction applied per TICK, not per second (same
// property flix's own filter turned out to have), so the real half-life depends on however fast
// the loop actually runs.
#[allow(dead_code)] // apply_level_prior call is temporarily commented out for a timing test
const LEVEL_PRIOR_WEIGHT: f32 = 0.0002;

// inner loop
const RATE_KP_ROLL_PITCH: f32 = 0.05; // flix 0.05
const RATE_KI_ROLL_PITCH: f32 = 0.3; // flix 0.2 was set to 0.1
const RATE_KD_ROLL_PITCH: f32 = 0.001; // flix 0.001

const RATE_KP_YAW: f32 = 0.3; // flix 0.3 was set to 0.2
const RATE_KI_YAW: f32 = 0.05;
const RATE_KD_YAW: f32 = 0.001;

const INTEGRAL_LIMIT: f32 = 0.3;
const BETA_FILTER: f32 = 0.1;
// per-second decay on the rate PIDs' integral term - without this, any permanent small error
// (mounting tolerance, an uneven resting surface - there's always something) eventually winds
// the integral to integral_limit given enough time, no matter how small the error is. With
// decay, a persistent error settles at an equilibrium of error/RATE_INTEGRAL_LEAK instead,
// while a genuinely larger disturbance still gets proportionally more integral authority
const RATE_INTEGRAL_LEAK: f32 = 1.0;

// outer loop angle error deadband - a resting tilt under this is mounting tolerance, not
// something worth actively fighting. subtracts the band rather than clamping to it, so there's
// no discontinuity in commanded rate right at the edge. roll/pitch only: yaw can't wind up
// (RATE_KI_YAW = 0) and has its own heading-hold handling already
const ANGLE_DEADBAND_RAD: f32 = 0.5 * fusion::DEG_TO_RAD;

fn deadband(x: f32, band: f32) -> f32 {
    if x > band {
        x - band
    } else if x < -band {
        x + band
    } else {
        0.0
    }
}

mod gyro_bias {
    use super::*;
    // gyro bias re-zeroing gain and debounce, matching flix's gyroBiasFilter/landedDelay
    const GYRO_BIAS_ALPHA: f32 = 0.001;
    const LANDED_DEBOUNCE: Duration = Duration::from_secs(2);
    const LANDED_ACCEL_TOLERANCE_G: f32 = 0.1; // +/- 10% of 1g
    // raw (not bias-corrected) gyro norm thresholds for "not actually rotating". Armed: has to
    // clear whatever residual bias might still be uncorrected (a few tens of deg/s at worst),
    // so it can keep learning while sitting still waiting to take off. Disarmed: much stricter,
    // so deliberate handling - picking it up, slowly tilting it to check response - doesn't get
    // mistaken for bias. Both stay well under real flight rates (typically hundreds of deg/s).
    const GYRO_STILL_THRESHOLD_ARMED_RAD_S: f32 = 0.5;
    const GYRO_STILL_THRESHOLD_DISARMED_RAD_S: f32 = 0.05;

    /// BiasTracker
    ///
    /// Continuously re-learns the gyro's zero-rate bias whenever the quad is actually
    /// motionless (flix's calibrateGyroOnce/landedDelay pattern, generalized to motion instead
    /// of a flat disarmed-only requirement - see `update`). "Still" means the accelerometer
    /// reads close to 1g and the raw gyro isn't reporting real rotation - armed or not, so a
    /// bad estimate can't stay stuck for an entire flight just because the quad is armed, but
    /// with a stricter bar while disarmed so handling it by hand doesn't get learned as bias.
    pub struct BiasTracker {
        bias: Vector3<f32>,
        landed_since: Option<Instant>,
    }

    impl BiasTracker {
        /// Starts from an already-known bias estimate (e.g. a quick boot-time average) instead
        /// of zero, so there's no window where a fully uncorrected bias reaches the control
        /// loop while this converges from scratch.
        pub fn new_seeded(bias: Vector3<f32>) -> Self {
            Self {
                bias,
                landed_since: None,
            }
        }

        // returns the current bias estimate - subtract this from raw gyro readings.
        // not called anywhere right now - see the comment at read_fusion's commented-out call
        // site. kept (not deleted) so re-enabling it is a one-line uncomment
        #[allow(dead_code)]
        pub fn update(
            &mut self,
            armed: bool,
            accel: Vector3<f32>,
            gyro: Vector3<f32>,
        ) -> Vector3<f32> {
            // motion-based rather than a flat "must be disarmed" - lets it keep learning while
            // armed and genuinely still (fixes it being stuck for the whole flight otherwise),
            // but demands much stronger stillness while disarmed so picking it up and tilting
            // it by hand isn't mistaken for bias
            let gyro_threshold = if armed {
                GYRO_STILL_THRESHOLD_ARMED_RAD_S
            } else {
                GYRO_STILL_THRESHOLD_DISARMED_RAD_S
            };
            let landed = (accel.norm() - 1.0).abs() < LANDED_ACCEL_TOLERANCE_G
                && gyro.norm() < gyro_threshold;
            if !landed {
                self.landed_since = None;
                return self.bias;
            }

            let landed_since = *self.landed_since.get_or_insert_with(Instant::now);
            if Instant::now().duration_since(landed_since) >= LANDED_DEBOUNCE {
                self.bias += GYRO_BIAS_ALPHA * (gyro - self.bias);
            }
            self.bias
        }

        // current bias estimate, without feeding it a new sample - used directly in read_fusion
        // now that continuous re-learning (update) is disabled, and still doubles as the
        // telemetry-verbose snapshot getter
        pub fn bias(&self) -> Vector3<f32> {
            self.bias
        }
    }

    // averages a batch of raw gyro samples right at startup and seeds a BiasTracker with it
    // Assumes the quad is sitting still once sampling starts, same assumption every
    // other flight-controller boot calibration makes. commented out: biastracker::update EMA
    //
    // waits for the same data-ready interrupt the main loop uses before each read
    // also tracks per-axis variance (Welford's online algorithm - single pass, no need to
    // buffer all 300 samples) and rejects the whole seed if it looks like the quad was moved during
    // sampling, matching betaflight's performGyroCalibration stddev-reject behavior. falls back to
    // an unseeded (zero) bias if every read failed or gets rejected
    pub(crate) async fn seed_gyro_bias(
        sensor: &mut Sensor20948<'_>,
        int_pin: &mut gpio::Input<'static>,
    ) -> gyro_bias::BiasTracker {
        // time to plug in the battery
        const STARTUP_DELAY_SECS: u64 = 3;
        embassy_time::Timer::after_secs(STARTUP_DELAY_SECS).await;

        const BOOT_SAMPLES: u32 = 300;
        // reject anything over this
        const STDDEV_REJECT_THRESHOLD_RAD_S: f32 = 0.05;

        let mut mean = Vector3::zeros();
        let mut m2 = Vector3::zeros();
        let mut successful: u32 = 0;
        for _ in 0..BOOT_SAMPLES {
            int_pin.wait_for_high().await;
            match sensor.read().await {
                Ok((_, g)) => {
                    successful += 1;
                    let delta = g - mean;
                    mean += delta / successful as f32;
                    let delta2 = g - mean;
                    m2 += delta.component_mul(&delta2);
                }
                Err(e) => defmt::error!("gyro bias seed read error: {}", defmt::Debug2Format(&e)),
            }
        }
        if successful == 0 {
            defmt::error!("gyro bias seeding got zero successful reads - starting from zero bias");
            return gyro_bias::BiasTracker::new_seeded(Vector3::zeros());
        }

        let variance = m2 / successful as f32;
        let stddev = Vector3::new(
            libm::sqrtf(variance.x),
            libm::sqrtf(variance.y),
            libm::sqrtf(variance.z),
        );
        if stddev.x > STDDEV_REJECT_THRESHOLD_RAD_S
            || stddev.y > STDDEV_REJECT_THRESHOLD_RAD_S
            || stddev.z > STDDEV_REJECT_THRESHOLD_RAD_S
        {
            defmt::error!(
                "gyro bias seed rejected - moved during sampling (stddev {} {} {} rad/s) - starting from zero bias",
                stddev.x,
                stddev.y,
                stddev.z
            );
            return gyro_bias::BiasTracker::new_seeded(Vector3::zeros());
        }

        defmt::info!("seeded gyro bias {}", defmt::Debug2Format(&mean));
        gyro_bias::BiasTracker::new_seeded(mean)
    }
}

// normalises an angle to [-pi, pi] for the shortest-path yaw error
fn wrap_angle(a: f32) -> f32 {
    use core::f32::consts::PI;
    // fmodf gives the same sign as the dividend, so shift into [0, 2pi) before subtracting back
    let r = libm::fmodf(a + PI, 2.0 * PI);
    (if r < 0.0 { r + 2.0 * PI } else { r }) - PI
}
// or:
// use num_traits::ops::euclid::Euclid;

// fn wrap_angle(a: f32) -> f32 {
//     use core::f32::consts::PI;
//     (a + PI).rem_euclid(2.0 * PI) - PI
// }

// running min/max/avg of loop dt between periodic log lines - see read_fusion's debug log.
// lets actual loop rate be checked without telemetry or trace logging (both DEFMT_LOG=info
// and DEFMT_LOG=debug print this; only trace_telemetry needs DEFMT_LOG=trace)
struct DtStats {
    min: f32,
    max: f32,
    sum: f32,
    n: u32,
}

impl DtStats {
    const fn new() -> Self {
        Self {
            min: f32::MAX,
            max: 0.0,
            sum: 0.0,
            n: 0,
        }
    }

    fn record(&mut self, dt: f32) {
        self.min = self.min.min(dt);
        self.max = self.max.max(dt);
        self.sum += dt;
        self.n += 1;
    }

    fn avg(&self) -> f32 {
        self.sum / self.n as f32
    }
}

// reads raw accel/gyro and fuses them into an orientation quaternion via whichever filter
// implements ImuFusion
#[allow(clippy::too_many_arguments)]
async fn read_fusion<F: ImuFusion>(
    sensor: &mut Sensor20948<'_>,
    filter: &mut F,
    gyro_bias: &mut gyro_bias::BiasTracker,
    rate_filter: &mut filters::Lpf3,
    accel_filter: &mut filters::Lpf3,
    accel_bias: &AccelBias,
    _armed: bool,
    dt: f32,
    log_counter: &mut u32,
    dt_stats: &mut DtStats,
    spi_stats: &mut DtStats,
    math_stats: &mut DtStats,
) -> Option<(UnitQuaternion<f32>, Vector3<f32>, Vector3<f32>)> {
    let read_start = Instant::now();
    let read_result = sensor.read().await;
    spi_stats.record(read_start.elapsed().as_micros() as f32 / 1_000_000.0);
    match read_result {
        Ok((accel, gyro)) => {
            let math_start = Instant::now();
            let accel = accel_bias.apply(accel);

            // THIS IS SPECIFIC TO MY CRAFT, it's mounted not perfectly so I'm compensating
            // undo the mounting tilt so a level frame reads as (0, 0, 0) - see MOUNT_TILT_ROLL_RAD
            let mount_trim =
                UnitQuaternion::from_euler_angles(filters::MOUNT_TILT_ROLL_RAD, 0.0, 0.0);
            // filtered here (before bias/mount-trim touch gyro at all) purely so the commented-out
            // BiasTracker call below sees the same accel signal the health gate further down does,
            // if it's ever re-enabled - see ACCEL_LPF_HZ
            let accel = accel_filter.update(mount_trim * accel, dt);

            // continuous in-flight bias re-learning is disabled for now - see the "is the gyro
            // bias tracker worth it" investigation. boot-time seeding (seed_gyro_bias) alone
            // covers it: measured bias is tiny (~0.003-0.007 rad/s, negligible against real
            // torque output), ongoing accel correction already compensates for roll/pitch drift
            // as a side effect, and betaflight does the same thing - gyroStartCalibration runs
            // once at boot and once on first arm, never continuously during flight. re-enable by
            // uncommenting if a real need for in-flight re-learning shows up (e.g. thermal drift
            // over a long flight) - accel above is already correctly filtered for when it does
            // let bias = gyro_bias.update(armed, accel, gyro);
            let bias = gyro_bias.bias();
            defmt::trace!(
                "gyro bias: {} {} {} | accel norm: {}",
                bias.x,
                bias.y,
                bias.z,
                accel.norm()
            );
            let gyro = gyro - bias;
            let gyro = mount_trim * gyro;

            // real prop/motor vibration under thrust - see RATE_LPF_HZ. same filtered rate
            // feeds both the fusion filter below and the inner rate loop (via the returned gyro)
            let gyro = rate_filter.update(gyro, dt);

            // outside ACCEL_HEALTHY_MIN..MAX, this sample isn't (close to) pure gravity - real
            // vibration or motion is mixed in, so its direction would pull the attitude estimate
            // the wrong way. feed the fusion filter a zero vector instead: both Madgwick and
            // Mahony's update_imu treat a non-normalizable accel as "no valid accel this tick"
            // and fall back to pure gyro integration, skipping the correction entirely rather
            // than trusting a distorted "down" - matches betaflight's imuIsAccelerometerHealthy()
            let accel_norm = accel.norm();
            let accel_for_fusion = if (filters::ACCEL_HEALTHY_MIN..=filters::ACCEL_HEALTHY_MAX)
                .contains(&accel_norm)
            {
                accel
            } else {
                Vector3::zeros()
            };

            let quat = filter.update_imu(dt, accel_for_fusion, gyro);
            // TEMPORARILY DISABLED - timing experiment to check whether this (four
            // transcendental calls: acos/atan2 x2, sin, cos) is what's driving math_stats.
            // restore once confirmed either way - see LEVEL_PRIOR_WEIGHT for what this does
            // let quat = libs::level_correction::apply_level_prior(quat, LEVEL_PRIOR_WEIGHT);
            math_stats.record(math_start.elapsed().as_micros() as f32 / 1_000_000.0);

            *log_counter += 1;
            dt_stats.record(dt);
            if *log_counter >= crate::LOG_EVERY_N {
                *log_counter = 0;
                let (roll, pitch, yaw) = quat.euler_angles();
                defmt::debug!(
                    "fusion w: {} x: {} y: {} z: {} | roll: {}° pitch: {}° yaw: {}° | dt min: {} max: {} avg: {} n: {} | spi read min: {} max: {} avg: {} | math min: {} max: {} avg: {}",
                    quat.w,
                    quat.i,
                    quat.j,
                    quat.k,
                    roll * fusion::RAD_TO_DEG,
                    pitch * fusion::RAD_TO_DEG,
                    yaw * fusion::RAD_TO_DEG,
                    dt_stats.min,
                    dt_stats.max,
                    dt_stats.avg(),
                    dt_stats.n,
                    spi_stats.min,
                    spi_stats.max,
                    spi_stats.avg(),
                    math_stats.min,
                    math_stats.max,
                    math_stats.avg(),
                );
                *dt_stats = DtStats::new();
                *spi_stats = DtStats::new();
                *math_stats = DtStats::new();
            }
            Some((quat, gyro, accel))
        }
        Err(e) => {
            defmt::error!("IMU read error: {}", defmt::Debug2Format(&e));
            None
        }
    }
}

// adds the magnetometer so yaw has an absolute reference
// instead of pure gyro integration. visualizer-only for now, not wired into run_control
#[cfg(feature = "mag")] // read_mag only exists on the MagEnabled driver
#[allow(dead_code)] // only called from run_fusion_visualizer, which needs the visualize feature
async fn read_fusion_marg<F: MargFusion>(
    sensor: &mut Sensor20948<'_>,
    filter: &mut F,
    dt: f32,
    log_counter: &mut u32,
) -> Option<(UnitQuaternion<f32>, Vector3<f32>)> {
    match sensor.read_mag().await {
        Ok((accel, gyro, mag)) => {
            let quat = filter.update(dt, accel, gyro, mag);

            *log_counter += 1;
            if *log_counter >= crate::LOG_EVERY_N {
                *log_counter = 0;
                let (roll, pitch, yaw) = quat.euler_angles();
                defmt::debug!(
                    "marg w: {} x: {} y: {} z: {} | roll: {}° pitch: {}° yaw: {}° | mag x: {} y: {} z: {}",
                    quat.w,
                    quat.i,
                    quat.j,
                    quat.k,
                    roll * fusion::RAD_TO_DEG,
                    pitch * fusion::RAD_TO_DEG,
                    yaw * fusion::RAD_TO_DEG,
                    mag.x,
                    mag.y,
                    mag.z,
                );
            }
            Some((quat, gyro))
        }
        Err(e) => {
            defmt::error!("IMU read error: {}", defmt::Debug2Format(&e));
            None
        }
    }
}

fn dur_since(last_instant: &mut Option<Instant>) -> f32 {
    let now = Instant::now();
    let dt = last_instant
        .map(|t| now.duration_since(t).as_micros() as f32 / 1_000_000.0)
        .unwrap_or(0.0);
    *last_instant = Some(now);

    dt
}
// control loop
pub async fn run_control(
    mut sensor: Sensor20948<'_>,
    mut int_pin: gpio::Input<'static>,
    motors: Motors<'_>,
    accel_bias: AccelBias,
) {
    let mut log_counter: u32 = 0;
    let mut fusion_filter = fusion::FusionBuilder::new()
        .icm20948()
        .madgwick()
        .beta(BETA_FILTER)
        .build();
    let mut gyro_bias = gyro_bias::seed_gyro_bias(&mut sensor, &mut int_pin).await;
    let mut rate_filter = filters::Lpf3::new(filters::RATE_LPF_HZ);
    let mut accel_filter = filters::Lpf3::new_two_pole(filters::ACCEL_LPF_HZ);
    let mut dt_stats = DtStats::new();
    let mut spi_stats = DtStats::new();
    // time from read_fusion's SPI read returning through the end of the Madgwick/filter math
    let mut math_stats = DtStats::new();
    // time actually spent inside int_pin.wait_for_high().await - from calling it to it
    // resolving, i.e. how long the task was genuinely suspended waiting for the data-ready
    // interrupt to fire
    let mut interrupt_wait_stats = DtStats::new();
    // time from the data-ready interrupt firing to the CONTROLS check finishing - isolates
    // interrupt/scheduler wake latency from actual work
    let mut wake_stats = DtStats::new();
    // time from read_fusion returning through euler_angles()+bias_snapshot - runs every tick
    // regardless of arm state, unlike pid_mix_stats below
    let mut euler_stats = DtStats::new();
    // time from the failsafe/packet check resolving through motors.set_motors() - the
    // outer/inner PID updates, motor mixing, and the PWM/LEDC hardware write. only recorded
    // once armed with a fresh packet and throttle above the idle cutoff
    let mut pid_mix_stats = DtStats::new();

    // inner rate PIDs
    let mut roll_pid = Pid::new(
        RATE_KP_ROLL_PITCH,
        RATE_KI_ROLL_PITCH,
        RATE_KD_ROLL_PITCH,
        INTEGRAL_LIMIT,
        RATE_INTEGRAL_LEAK,
    );
    let mut pitch_pid = Pid::new(
        RATE_KP_ROLL_PITCH,
        RATE_KI_ROLL_PITCH,
        RATE_KD_ROLL_PITCH,
        INTEGRAL_LIMIT,
        RATE_INTEGRAL_LEAK,
    );
    let mut yaw_pid = Pid::new(
        RATE_KP_YAW,
        RATE_KI_YAW,
        RATE_KD_YAW,
        INTEGRAL_LIMIT,
        RATE_INTEGRAL_LEAK,
    );

    let mut target_yaw: f32 = 0.0;
    let mut yaw_init = false;
    let mut last_armed = false;
    let mut last_instant: Option<Instant> = None;

    loop {
        let interrupt_wait_start = Instant::now();
        int_pin.wait_for_high().await;
        interrupt_wait_stats
            .record(interrupt_wait_start.elapsed().as_micros() as f32 / 1_000_000.0);
        let wake_start = Instant::now();

        let controls = wifi::CONTROLS.lock(|c| c.get());
        let fresh = controls.is_some_and(|(_, at)| at.elapsed() < Duration::from_millis(500));
        let armed = fresh && controls.is_some_and(|(p, _)| p.armed());
        wake_stats.record(wake_start.elapsed().as_micros() as f32 / 1_000_000.0);
        // printed here rather than after motors.set_motors() - disarmed ticks take an early
        // `continue` further down (controls.filter(|_| armed) is always None while disarmed)
        // and never reach that point, but wake latency is still meaningful to see regardless
        // of arm state
        if log_counter == 0 {
            defmt::debug!(
                "interrupt wait min: {} max: {} avg: {} | wake min: {} max: {} avg: {}",
                interrupt_wait_stats.min,
                interrupt_wait_stats.max,
                interrupt_wait_stats.avg(),
                wake_stats.min,
                wake_stats.max,
                wake_stats.avg(),
            );
            interrupt_wait_stats = DtStats::new();
            wake_stats = DtStats::new();
        }

        if !last_armed && armed {
            defmt::info!("ARMED");

            roll_pid.reset();
            pitch_pid.reset();
            yaw_pid.reset();
            yaw_init = false;
        } else if last_armed && !armed {
            defmt::info!("DISARMED");
        }
        last_armed = armed;

        if !armed {
            motors.turn_off();
        }

        let (quat, g, dt, accel) = {
            let dt = dur_since(&mut last_instant);
            let (quat, g, accel) = match read_fusion(
                &mut sensor,
                &mut fusion_filter,
                &mut gyro_bias,
                &mut rate_filter,
                &mut accel_filter,
                &accel_bias,
                armed,
                dt,
                &mut log_counter,
                &mut dt_stats,
                &mut spi_stats,
                &mut math_stats,
            )
            .await
            {
                Some(d) => d,
                None => continue,
            };
            (quat, g, dt, accel)
        };
        let attitude_start = Instant::now();

        let euler = quat.euler_angles();

        let bias_snapshot = gyro_bias.bias();
        // recorded (and printed) here rather than folded into pid_mix_stats - this segment
        // runs on every tick regardless of arm state, but pid_mix_stats' recording point is
        // past the failsafe/no-packet early `continue` below and so never reached while
        // disarmed, same issue wake_stats had
        euler_stats.record(attitude_start.elapsed().as_micros() as f32 / 1_000_000.0);
        if log_counter == 0 {
            defmt::debug!(
                "euler+bias min: {} max: {} avg: {}",
                euler_stats.min,
                euler_stats.max,
                euler_stats.avg(),
            );
            euler_stats = DtStats::new();
        }

        // failsafe: zero motors if no packet, packet is stale (>500 ms), or disarmed
        let pkt = match controls.filter(|_| armed) {
            Some((p, _)) => p,
            None => {
                #[cfg(all(feature = "telemetry", not(feature = "telemetry-verbose")))]
                publish_telemetry(euler, (0, 0, 0, 0), armed, !fresh);
                #[cfg(feature = "telemetry-verbose")]
                publish_telemetry(
                    euler,
                    (0, 0, 0, 0),
                    armed,
                    !fresh,
                    g,
                    Vector3::zeros(),
                    bias_snapshot,
                    dt,
                    accel,
                );
                trace_telemetry(
                    euler,
                    (0, 0, 0, 0),
                    armed,
                    !fresh,
                    g,
                    Vector3::zeros(),
                    bias_snapshot,
                    dt,
                    accel,
                );
                continue;
            }
        };
        let mix_start = Instant::now();

        // would be controlAttitude (flix)
        // DMP gives us the fused quaternion directly instead of running Mahony/Madgwick

        let actual_yaw = euler.2;
        let pkt_throttle = pkt.throttle as f32 / 100.0;

        // latch heading on first armed tick (flix: yawTarget initialised from attitude.getYaw())
        if !yaw_init {
            target_yaw = actual_yaw;
            yaw_init = true;
        }

        // while near the ground keep target tracking actual so handling the drone doesn't build a large error
        if pkt_throttle < 0.15 {
            target_yaw = actual_yaw;
        }

        // heading hold: only update target when yaw stick is active (matches flix interpretControls)
        if pkt.yaw.abs() >= 0.1 {
            target_yaw = actual_yaw;
        }
        // stick feedforward adds directly to yaw rate setpoint (matches flix ratesExtra)
        // deadzone mirrors heading hold threshold — prevents stick drift from killing FR at idle
        let yaw_ff = if pkt.yaw.abs() >= 0.1 {
            -pkt.yaw * core::f32::consts::PI // ±π rad/s
        } else {
            0.0
        };

        // like controlTorque / motor mixing (flix)
        if pkt_throttle < 0.05 {
            motors.turn_off();
            #[cfg(all(feature = "telemetry", not(feature = "telemetry-verbose")))]
            publish_telemetry(euler, (0, 0, 0, 0), armed, !fresh);
            #[cfg(feature = "telemetry-verbose")]
            publish_telemetry(
                euler,
                (0, 0, 0, 0),
                armed,
                !fresh,
                g,
                Vector3::zeros(),
                bias_snapshot,
                dt,
                accel,
            );
            trace_telemetry(
                euler,
                (0, 0, 0, 0),
                armed,
                !fresh,
                g,
                Vector3::zeros(),
                bias_snapshot,
                dt,
                accel,
            );
            continue;
        }

        // outer PID:
        // with quaternions:
        // build target attitude quaternion from stick angles (flix Quaternion::fromEuler)
        // let target_quat = UnitQuaternion::from_euler_angles(
        //     pkt.roll * MAX_TILT_RAD,
        //     pkt.pitch * MAX_TILT_RAD,
        //     target_yaw,
        // );
        // // up-vector cross product gives roll/pitch error (flix rotationVectorBetween)
        // // arg order matches flix: actual * target (swapped gives negated error vector)
        // let up = Vector3::z();
        // let att_err = quat
        //     .transform_vector(&up)
        //     .cross(&target_quat.transform_vector(&up)); // flix Vector::rotationVectorBetween - cross product of two up-vectors gives the attitude error
        // let roll_rate_sp = ANGLE_P_ROLL_PITCH * att_err.x;
        // let pitch_rate_sp = ANGLE_P_ROLL_PITCH * att_err.y;
        // let yaw_rate_sp = ANGLE_P_YAW * wrap_angle(target_yaw - actual_yaw) + yaw_ff;

        // alternative: roll/pitch error from decomposed Euler angles instead of the up-vector
        // cross product above. the cross product computes error from the full orientation
        // quaternion (all of w,x,y,z), so without a magnetometer, yaw drift leaks into roll/pitch
        // error - confirmed via tethered testing, where ~58deg of accumulated yaw drift produced
        // a full roll/pitch swap in the motor mix. euler_angles() decomposes each axis
        // independently, so yaw drift can't couple in - matches peterkrull/quad's
        // task_state_estimator.rs + task_attitude_controller.rs, which uses this approach.
        // to switch: comment out the att_err block above (through yaw_rate_sp) and uncomment this:
        let target_roll = pkt.roll * MAX_TILT_RAD; // pkt.roll/pitch are [-1..=1] so the multiplication caps 1 at MAX_TILT_RAD
        let target_pitch = pkt.pitch * MAX_TILT_RAD;
        let (actual_roll, actual_pitch, _) = euler;
        let roll_err = deadband(wrap_angle(target_roll - actual_roll), ANGLE_DEADBAND_RAD);
        let pitch_err = deadband(wrap_angle(target_pitch - actual_pitch), ANGLE_DEADBAND_RAD);
        let roll_rate_sp = ANGLE_P_ROLL_PITCH * roll_err;
        let pitch_rate_sp = ANGLE_P_ROLL_PITCH * pitch_err;
        let yaw_rate_sp = ANGLE_P_YAW * wrap_angle(target_yaw - actual_yaw) + yaw_ff;

        // inner PID:
        // like controlRates (flix)
        // calibrated_gyro replaces flix's raw gyro register reads; hardware DLPF replaces software LPF
        let roll_torque = roll_pid.update(roll_rate_sp - g.x, dt);
        let pitch_torque = pitch_pid.update(pitch_rate_sp - g.y, dt);
        let yaw_torque = yaw_pid.update(yaw_rate_sp - g.z, dt);

        // betaflight-style two-sided desaturation - see libs::mixer for the derivation, the
        // real crash it was built to fix (a real flight log showed a motor getting silently
        // floored while a big correction was active, escalating into full saturation on
        // multiple motors)
        let (fl, fr, rl, rr) =
            libs::mixer::mix_motors(pkt_throttle, roll_torque, pitch_torque, yaw_torque);

        let (dfl, dfr, drl, drr) = motors.set_motors(fl, fr, rl, rr);
        pid_mix_stats.record(mix_start.elapsed().as_micros() as f32 / 1_000_000.0);
        // only reached once armed with a fresh packet and throttle above the idle cutoff - see
        // the wake_stats print above for why this can't share that print site. log_counter was
        // just reset by read_fusion above if it hit LOG_EVERY_N - piggyback on that same
        // cadence rather than tracking a separate counter
        if log_counter == 0 {
            defmt::debug!(
                "pid+mix+motor min: {} max: {} avg: {}",
                pid_mix_stats.min,
                pid_mix_stats.max,
                pid_mix_stats.avg(),
            );
            pid_mix_stats = DtStats::new();
        }
        defmt::trace!(
            "torques roll={} pitch={} yaw={} | mix fl={} fr={} rl={} rr={} | duty fl={} fr={} rl={} rr={} | dt={}",
            roll_torque,
            pitch_torque,
            yaw_torque,
            fl,
            fr,
            rl,
            rr,
            dfl,
            dfr,
            drl,
            drr,
            dt,
        );

        #[cfg(all(feature = "telemetry", not(feature = "telemetry-verbose")))]
        publish_telemetry(
            euler,
            (dfl as u16, dfr as u16, drl as u16, drr as u16),
            armed,
            false,
        );
        #[cfg(feature = "telemetry-verbose")]
        publish_telemetry(
            euler,
            (dfl as u16, dfr as u16, drl as u16, drr as u16),
            armed,
            false,
            g,
            Vector3::new(roll_torque, pitch_torque, yaw_torque),
            bias_snapshot,
            dt,
            accel,
        );
        trace_telemetry(
            euler,
            (dfl as u16, dfr as u16, drl as u16, drr as u16),
            armed,
            false,
            g,
            Vector3::new(roll_torque, pitch_torque, yaw_torque),
            bias_snapshot,
            dt,
            accel,
        );
    }
}

// visualize-only loop: log orientation, no motor control, no WiFi
#[cfg(feature = "visualize")]
pub async fn run_fusion_visualizer(
    mut sensor: Sensor20948<'_>,
    mut int_pin: gpio::Input<'static>,
    accel_bias: AccelBias,
) {
    let mut log_counter: u32 = 0;
    let mut fusion_filter = fusion::FusionBuilder::new()
        .icm20948()
        // .mahony()
        .madgwick()
        .beta(BETA_FILTER)
        .build();
    let mut gyro_bias = gyro_bias::seed_gyro_bias(&mut sensor, &mut int_pin).await;
    let mut rate_filter = filters::Lpf3::new(filters::RATE_LPF_HZ);
    let mut accel_filter = filters::Lpf3::new_two_pole(filters::ACCEL_LPF_HZ);
    let mut dt_stats = DtStats::new();
    let mut spi_stats = DtStats::new();
    let mut math_stats = DtStats::new();
    let mut last_instant: Option<Instant> = None;
    loop {
        int_pin.wait_for_high().await;
        let dt = dur_since(&mut last_instant);

        read_fusion(
            &mut sensor,
            &mut fusion_filter,
            &mut gyro_bias,
            &mut rate_filter,
            &mut accel_filter,
            &accel_bias,
            false, // no arming concept in the visualizer, motors never spin
            dt,
            &mut log_counter,
            &mut dt_stats,
            &mut spi_stats,
            &mut math_stats,
        )
        .await;
        // read_fusion_marg(&mut sensor, &mut fusion_filter, dt, &mut log_counter).await;
    }
}
