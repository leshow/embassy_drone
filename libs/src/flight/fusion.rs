#![allow(dead_code)]
use core::{marker::PhantomData, time::Duration};

use nalgebra::{UnitQuaternion, Vector2, Vector3};
use uf_ahrs::{
    Ahrs as UfAhrs, Madgwick as UfMadgwick, MadgwickParams, Mahony as UfMahony, MahonyParams,
    Vqf as UfVqf, VqfParams,
};

pub const FLAT_DEG: f32 = 10.; // dead-zone around flat
pub const STEEP_DEG: f32 = 50.; // "both LEDs on" threshold
pub const RAD_TO_DEG: f32 = 180. / core::f32::consts::PI;
pub const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.;

const ALPHA_DEFAULT: f32 = 0.98; // complementary filter: trust gyro 98%, accel 2%
// Madgwick beta gain - lower than the library default (0.1, tuned for fast convergence) to cut
// how much thrust-induced accel noise bleeds into the attitude estimate mid-flight. Start here,
// halve toward ~0.015 if it still drifts; don't chase it toward ~0 or you trade accel noise for
// uncorrected gyro-integration drift instead
const BETA_DEFAULT: f32 = 0.1;
// Mahony gains matching Betaflight's armed defaults (imu_dcm_kp=2500/10000, imu_dcm_ki=0) rather
// than uf-ahrs's own crate defaults (0.74/0.0012)
// ki=0 means no gyro bias learning until explicitly enabled
const KP_DEFAULT: f32 = 0.25;
const KI_DEFAULT: f32 = 0.0;
// original uf-ahrs lib defaults:
// const KP_DEFAULT: f32 = 0.74; // Mahony kp
// const KI_DEFAULT: f32 = 0.0012; // Mahony ki
const SAMPLE_PERIOD_DEFAULT: f32 = 0.001; // 1000 Hz default

/// Sensor fusion
/// F - filter type
/// S - phantom type for sensor
///
/// (depends on how you mounted sensor)
/// ```
///         +X (forward)
///          ↑
///          |
/// -Y ------+------ +Y (right)
///          |
///          ↓
///         -X (back)
///
/// +Z points UP out of the chip surface
/// -Z points DOWN into the desk
/// ```
pub struct Fusion<S, F> {
    pub filter: F,
    _sensor: PhantomData<S>,
}

pub struct ICM20948; // 9DOF: accel + gyro + mag
pub struct MPU6050; // 6DOF: accel + gyro only

// Filter types — each carries its own runtime state
pub struct Complementary {
    pub angle_roll: f32,  // radians, rotation around X axis
    pub angle_pitch: f32, // radians, rotation around Y axis
    pub angle_yaw: f32, // radians, rotation around Z axis - only integrated by update_imu, since update derives yaw from the magnetometer instead
    pub alpha: f32,
}

pub struct Madgwick {
    pub inner: UfMadgwick,
    pub beta: f32,
}

pub struct Vqf {
    pub inner: UfVqf,
    pub params: VqfParams,
}

pub struct Mahony {
    pub inner: UfMahony,
    pub params: MahonyParams,
}

// Builder phantom types
pub struct NoSensor;
pub struct NoFilter;

// Builder
pub struct FusionBuilder<S, F> {
    // alpha only matters for complementary filter
    alpha: f32,
    // used in madgwick
    beta: f32,
    // used in mahony
    kp: f32,
    ki: f32,
    sample_period: f32,
    _sensor: PhantomData<S>,
    _filter: PhantomData<F>,
}

impl FusionBuilder<NoSensor, NoFilter> {
    pub fn new() -> Self {
        Self {
            alpha: ALPHA_DEFAULT,
            beta: BETA_DEFAULT,
            kp: KP_DEFAULT,
            ki: KI_DEFAULT,
            sample_period: SAMPLE_PERIOD_DEFAULT,
            _sensor: PhantomData,
            _filter: PhantomData,
        }
    }
}

impl Default for FusionBuilder<NoSensor, NoFilter> {
    fn default() -> Self {
        Self::new()
    }
}

// Sensor selection
impl<F> FusionBuilder<NoSensor, F> {
    pub fn icm20948(self) -> FusionBuilder<ICM20948, F> {
        FusionBuilder {
            alpha: self.alpha,
            beta: self.beta,
            kp: self.kp,
            ki: self.ki,
            sample_period: self.sample_period,
            _sensor: PhantomData,
            _filter: PhantomData,
        }
    }

    pub fn mpu6050(self) -> FusionBuilder<MPU6050, F> {
        FusionBuilder {
            alpha: self.alpha,
            beta: self.beta,
            kp: self.kp,
            ki: self.ki,
            sample_period: self.sample_period,
            _sensor: PhantomData,
            _filter: PhantomData,
        }
    }
}

// Filter selection
impl<S> FusionBuilder<S, NoFilter> {
    pub fn complementary(self) -> FusionBuilder<S, Complementary> {
        FusionBuilder {
            alpha: self.alpha,
            beta: self.beta,
            kp: self.kp,
            ki: self.ki,
            sample_period: self.sample_period,
            _sensor: PhantomData,
            _filter: PhantomData,
        }
    }

    pub fn madgwick(self) -> FusionBuilder<S, Madgwick> {
        FusionBuilder {
            alpha: self.alpha,
            beta: self.beta,
            kp: self.kp,
            ki: self.ki,
            sample_period: self.sample_period,
            _sensor: PhantomData,
            _filter: PhantomData,
        }
    }

    pub fn vqf(self) -> FusionBuilder<S, Vqf> {
        FusionBuilder {
            alpha: self.alpha,
            beta: self.beta,
            kp: self.kp,
            ki: self.ki,
            sample_period: self.sample_period,
            _sensor: PhantomData,
            _filter: PhantomData,
        }
    }

    pub fn mahony(self) -> FusionBuilder<S, Mahony> {
        FusionBuilder {
            alpha: self.alpha,
            beta: self.beta,
            kp: self.kp,
            ki: self.ki,
            sample_period: self.sample_period,
            _sensor: PhantomData,
            _filter: PhantomData,
        }
    }
}

// Alpha tuning — complementary filter only
impl<S> FusionBuilder<S, Complementary> {
    pub fn alpha(mut self, alpha: f32) -> Self {
        self.alpha = alpha;
        self
    }
}

// Madgwick tuning
impl<S> FusionBuilder<S, Madgwick> {
    pub fn beta(mut self, beta: f32) -> Self {
        self.beta = beta;
        self
    }

    pub fn sample_period(mut self, sample_period: f32) -> Self {
        self.sample_period = sample_period;
        self
    }
}

// Vqf tuning — only sample_period matters at build time; other params use defaults
impl<S> FusionBuilder<S, Vqf> {
    pub fn sample_period(mut self, sample_period: f32) -> Self {
        self.sample_period = sample_period;
        self
    }
}

/// Tuning methods for the Mahony complementary filter.
///
/// The Mahony filter corrects gyroscope integration using accelerometer (and
/// optionally magnetometer) feedback scaled by two gains:
///
/// - **`kp`** — proportional gain: how aggressively the filter pulls the
///   estimated orientation toward the gravity/field reference each step.
///   Higher values respond faster to disturbances but amplify vibration noise.
///   Typical range: `0.1` – `2.0`. Default: `0.25` (Betaflight's armed `imu_dcm_kp`).
///
/// - **`ki`** — integral gain: how quickly the filter accumulates a gyroscope
///   bias correction term. A non-zero `ki` lets the filter learn and cancel a
///   constant drift offset over time. Set to `0.0` to disable bias estimation.
///   Left off by default here since `BiasTracker` (flight.rs) already learns gyro
///   bias continuously - a second learner on the same signal would be redundant.
///   Typical range: `0.0` – `0.01`. Default: `0.0`.
///
/// # Example
/// ```rust
/// let mut fusion = FusionBuilder::new()
///     .icm20948()
///     .mahony()
///     .kp(1.0)    // more aggressive correction
///     .ki(0.005)  // moderate bias estimation
///     .build();
/// ```
impl<S> FusionBuilder<S, Mahony> {
    /// Sets the proportional gain `kp`.
    ///
    /// Controls how strongly accelerometer (and magnetometer) feedback corrects
    /// the orientation estimate each step. Higher values converge faster but are
    /// more sensitive to sensor noise and vibration. Default: `0.25`.
    pub fn kp(mut self, kp: f32) -> Self {
        self.kp = kp;
        self
    }

    /// Sets the integral gain `ki`.
    ///
    /// Drives a slowly-accumulating gyroscope bias correction. A non-zero value
    /// lets the filter remove a persistent drift offset learned over time.
    /// Set to `0.0` to disable bias estimation entirely. Default: `0.0`.
    pub fn ki(mut self, ki: f32) -> Self {
        self.ki = ki;
        self
    }

    /// Sets the nominal sample period in seconds.
    ///
    /// Used only as the initial `dt` stored in the filter at construction.
    /// Each call to `update` / `update_imu` rebuilds the filter with the
    /// actual measured `dt`, so this value mainly affects the very first step.
    /// Default: `0.001` (1 kHz).
    pub fn sample_period(mut self, sample_period: f32) -> Self {
        self.sample_period = sample_period;
        self
    }
}

// build() — only callable once both type params are concrete
impl FusionBuilder<ICM20948, Complementary> {
    pub fn build(self) -> Fusion<ICM20948, Complementary> {
        Fusion {
            filter: Complementary {
                angle_roll: 0.0,
                angle_pitch: 0.0,
                angle_yaw: 0.0,
                alpha: self.alpha,
            },
            _sensor: PhantomData,
        }
    }
}

impl FusionBuilder<MPU6050, Complementary> {
    pub fn build(self) -> Fusion<MPU6050, Complementary> {
        Fusion {
            filter: Complementary {
                angle_roll: 0.0,
                angle_pitch: 0.0,
                angle_yaw: 0.0,
                alpha: self.alpha,
            },
            _sensor: PhantomData,
        }
    }
}

impl FusionBuilder<ICM20948, Madgwick> {
    pub fn build(self) -> Fusion<ICM20948, Madgwick> {
        Fusion {
            filter: Madgwick {
                inner: UfMadgwick::new(
                    Duration::from_secs_f32(self.sample_period),
                    MadgwickParams { beta: self.beta },
                ),
                beta: self.beta,
            },
            _sensor: PhantomData,
        }
    }
}

impl FusionBuilder<MPU6050, Madgwick> {
    pub fn build(self) -> Fusion<MPU6050, Madgwick> {
        Fusion {
            filter: Madgwick {
                inner: UfMadgwick::new(
                    Duration::from_secs_f32(self.sample_period),
                    MadgwickParams { beta: self.beta },
                ),
                beta: self.beta,
            },
            _sensor: PhantomData,
        }
    }
}

impl FusionBuilder<ICM20948, Vqf> {
    pub fn build(self) -> Fusion<ICM20948, Vqf> {
        let params = VqfParams::default();
        Fusion {
            filter: Vqf {
                inner: UfVqf::new(Duration::from_secs_f32(self.sample_period), params.clone()),
                params,
            },
            _sensor: PhantomData,
        }
    }
}

impl FusionBuilder<MPU6050, Vqf> {
    pub fn build(self) -> Fusion<MPU6050, Vqf> {
        let params = VqfParams::default();
        Fusion {
            filter: Vqf {
                inner: UfVqf::new(Duration::from_secs_f32(self.sample_period), params.clone()),
                params,
            },
            _sensor: PhantomData,
        }
    }
}

impl FusionBuilder<ICM20948, Mahony> {
    pub fn build(self) -> Fusion<ICM20948, Mahony> {
        let params = MahonyParams {
            kp: self.kp,
            ki: self.ki,
        };
        Fusion {
            filter: Mahony {
                inner: UfMahony::new(Duration::from_secs_f32(self.sample_period), params),
                params,
            },
            _sensor: PhantomData,
        }
    }
}

impl FusionBuilder<MPU6050, Mahony> {
    pub fn build(self) -> Fusion<MPU6050, Mahony> {
        let params = MahonyParams {
            kp: self.kp,
            ki: self.ki,
        };
        Fusion {
            filter: Mahony {
                inner: UfMahony::new(Duration::from_secs_f32(self.sample_period), params),
                params,
            },
            _sensor: PhantomData,
        }
    }
}

impl Fusion<ICM20948, Complementary> {
    /// Returns the estimated orientation as a quaternion.
    pub fn update(
        &mut self,
        dt: f32,
        a: Vector3<f32>,
        g: Vector3<f32>,
        m: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        let acc_roll = libm::atan2f(a.y, libm::sqrtf(a.x * a.x + a.z * a.z));
        let acc_pitch = libm::atan2f(-a.x, libm::sqrtf(a.y * a.y + a.z * a.z));

        let this = &mut self.filter;
        (this.angle_roll, this.angle_pitch) = utils::complementary_filter(
            this.angle_roll,
            this.angle_pitch,
            g.x,
            g.y,
            dt,
            acc_roll,
            acc_pitch,
            this.alpha,
        );

        let (sr, cr) = (libm::sinf(this.angle_roll), libm::cosf(this.angle_roll));
        let (sp, cp) = (libm::sinf(this.angle_pitch), libm::cosf(this.angle_pitch));
        let mag_xc = m.x * cp + m.z * sp;
        let mag_yc = m.x * sp * sr + m.y * cr - m.z * cp * sr;
        let yaw_rad = libm::atan2f(-mag_yc, mag_xc);

        UnitQuaternion::from_euler_angles(this.angle_roll, this.angle_pitch, yaw_rad)
    }
}

impl Fusion<MPU6050, Complementary> {
    /// 6DOF complementary filter.
    /// `acc_angles` is the output of `mpu.get_acc_angles()`: [roll_rad, pitch_rad].
    /// No magnetometer — yaw is fixed at 0. Returns orientation as a quaternion.
    pub fn update(
        &mut self,
        dt: f32,
        acc_angles: Vector2<f32>,
        g: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        let this = &mut self.filter;
        (this.angle_roll, this.angle_pitch) = utils::complementary_filter(
            this.angle_roll,
            this.angle_pitch,
            g.x,
            g.y,
            dt,
            acc_angles[0],
            acc_angles[1],
            this.alpha,
        );

        UnitQuaternion::from_euler_angles(this.angle_roll, this.angle_pitch, 0.0)
    }
}

impl Fusion<ICM20948, Madgwick> {
    /// 9DOF Madgwick MARG via `uf-ahrs`.
    /// Rebuilds with the actual measured dt each call so gyro integration is correct.
    /// Returns orientation as a quaternion.
    pub fn update(
        &mut self,
        dt: f32,
        a: Vector3<f32>,
        g: Vector3<f32>,
        m: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        let dt = Duration::from_secs_f32(dt.max(0.0001));
        let current_q = self.filter.inner.orientation();
        self.filter.inner = UfMadgwick::new_with_orientation(
            dt,
            MadgwickParams {
                beta: self.filter.beta,
            },
            current_q,
        );
        self.filter.inner.update(g, a, m)
    }

    /// IMU-only mode: ignores magnetometer, yaw will drift but roll/pitch are clean.
    /// Returns orientation as a quaternion.
    pub fn update_imu(&mut self, dt: f32, a: Vector3<f32>, g: Vector3<f32>) -> UnitQuaternion<f32> {
        let dt = Duration::from_secs_f32(dt.max(0.0001));
        let current_q = self.filter.inner.orientation();
        self.filter.inner = UfMadgwick::new_with_orientation(
            dt,
            MadgwickParams {
                beta: self.filter.beta,
            },
            current_q,
        );
        self.filter.inner.update_imu(g, a)
    }
}

impl Fusion<MPU6050, Madgwick> {
    /// 6DOF Madgwick IMU-only via `uf-ahrs`. Yaw will drift.
    /// Returns orientation as a quaternion.
    pub fn update(&mut self, dt: f32, a: Vector3<f32>, g: Vector3<f32>) -> UnitQuaternion<f32> {
        let dt = Duration::from_secs_f32(dt.max(0.0001));
        let current_q = self.filter.inner.orientation();
        self.filter.inner = UfMadgwick::new_with_orientation(
            dt,
            MadgwickParams {
                beta: self.filter.beta,
            },
            current_q,
        );
        self.filter.inner.update_imu(g, a)
    }
}

impl Fusion<ICM20948, Vqf> {
    /// 9DOF VQF MARG via `uf-ahrs`.
    /// Rebuilds with the actual measured dt each call (trades bias-estimator history for correct timing).
    /// Returns orientation as a quaternion.
    pub fn update(
        &mut self,
        dt: f32,
        a: Vector3<f32>,
        g: Vector3<f32>,
        m: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        let dt = Duration::from_secs_f32(dt.max(0.0001));
        let current_q = self.filter.inner.orientation();
        self.filter.inner = UfVqf::new(dt, self.filter.params.clone());
        self.filter.inner.set_orientation(current_q);
        self.filter.inner.update(g, a, m)
    }

    /// IMU-only mode: ignores magnetometer, yaw will drift but roll/pitch are clean.
    /// Returns orientation as a quaternion.
    pub fn update_imu(&mut self, dt: f32, a: Vector3<f32>, g: Vector3<f32>) -> UnitQuaternion<f32> {
        let dt = Duration::from_secs_f32(dt.max(0.0001));
        let current_q = self.filter.inner.orientation();
        self.filter.inner = UfVqf::new(dt, self.filter.params.clone());
        self.filter.inner.set_orientation(current_q);
        self.filter.inner.update_imu(g, a)
    }
}

impl Fusion<MPU6050, Vqf> {
    /// 6DOF VQF IMU-only via `uf-ahrs`. Yaw will drift.
    /// Returns orientation as a quaternion.
    pub fn update(&mut self, dt: f32, a: Vector3<f32>, g: Vector3<f32>) -> UnitQuaternion<f32> {
        let dt = Duration::from_secs_f32(dt.max(0.0001));
        let current_q = self.filter.inner.orientation();
        self.filter.inner = UfVqf::new(dt, self.filter.params.clone());
        self.filter.inner.set_orientation(current_q);
        self.filter.inner.update_imu(g, a)
    }
}

impl Fusion<ICM20948, Complementary> {
    /// IMU-only mode: no magnetometer, yaw is pure gyro integration and will drift.
    /// Returns orientation as a quaternion.
    pub fn update_imu(&mut self, dt: f32, a: Vector3<f32>, g: Vector3<f32>) -> UnitQuaternion<f32> {
        let acc_roll = libm::atan2f(a.y, libm::sqrtf(a.x * a.x + a.z * a.z));
        let acc_pitch = libm::atan2f(-a.x, libm::sqrtf(a.y * a.y + a.z * a.z));

        let this = &mut self.filter;
        (this.angle_roll, this.angle_pitch) = utils::complementary_filter(
            this.angle_roll,
            this.angle_pitch,
            g.x,
            g.y,
            dt,
            acc_roll,
            acc_pitch,
            this.alpha,
        );
        this.angle_yaw += g.z * dt; // no absolute reference without a mag reading, so this drifts

        UnitQuaternion::from_euler_angles(this.angle_roll, this.angle_pitch, this.angle_yaw)
    }
}

impl Fusion<ICM20948, Mahony> {
    /// 9DOF Mahony MARG via `uf-ahrs`.
    /// Returns orientation as a quaternion.
    pub fn update(
        &mut self,
        dt: f32,
        a: Vector3<f32>,
        g: Vector3<f32>,
        m: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        let dt = Duration::from_secs_f32(dt.max(0.0001));
        let current_q = self.filter.inner.orientation();
        // new_with_orientation zeroes `bias`, so we've got to carry it through for the next tick
        let bias = self.filter.inner.bias;
        self.filter.inner = UfMahony::new_with_orientation(dt, self.filter.params, current_q);
        self.filter.inner.bias = bias;
        self.filter.inner.update(g, a, m)
    }

    /// IMU-only mode: no magnetometer, yaw will drift.
    /// Returns orientation as a quaternion.
    pub fn update_imu(&mut self, dt: f32, a: Vector3<f32>, g: Vector3<f32>) -> UnitQuaternion<f32> {
        let dt = Duration::from_secs_f32(dt.max(0.0001));
        let current_q = self.filter.inner.orientation();
        let bias = self.filter.inner.bias;
        self.filter.inner = UfMahony::new_with_orientation(dt, self.filter.params, current_q);
        self.filter.inner.bias = bias;
        self.filter.inner.update_imu(g, a)
    }
}

impl Fusion<MPU6050, Mahony> {
    /// 6DOF Mahony IMU-only via `uf-ahrs`. Yaw will drift.
    /// Returns orientation as a quaternion.
    pub fn update_imu(&mut self, dt: f32, a: Vector3<f32>, g: Vector3<f32>) -> UnitQuaternion<f32> {
        let dt = Duration::from_secs_f32(dt.max(0.0001));
        let current_q = self.filter.inner.orientation();
        let bias = self.filter.inner.bias;
        self.filter.inner = UfMahony::new_with_orientation(dt, self.filter.params, current_q);
        self.filter.inner.bias = bias;
        self.filter.inner.update_imu(g, a)
    }
}

// generalizes "fuse one accel+gyro sample into an orientation" across whichever filter
// FusionBuilder was configured with. Madgwick/Vqf/Mahony/Complementary all expose a matching
// IMU-only update_imu for the ICM20948, so read_fusion below can stay generic over the filter.
pub trait ImuFusion {
    fn update_imu(&mut self, dt: f32, a: Vector3<f32>, g: Vector3<f32>) -> UnitQuaternion<f32>;
}

impl ImuFusion for Fusion<ICM20948, Complementary> {
    fn update_imu(&mut self, dt: f32, a: Vector3<f32>, g: Vector3<f32>) -> UnitQuaternion<f32> {
        Self::update_imu(self, dt, a, g)
    }
}

impl ImuFusion for Fusion<ICM20948, Madgwick> {
    fn update_imu(&mut self, dt: f32, a: Vector3<f32>, g: Vector3<f32>) -> UnitQuaternion<f32> {
        Self::update_imu(self, dt, a, g)
    }
}

impl ImuFusion for Fusion<ICM20948, Vqf> {
    fn update_imu(&mut self, dt: f32, a: Vector3<f32>, g: Vector3<f32>) -> UnitQuaternion<f32> {
        Self::update_imu(self, dt, a, g)
    }
}

impl ImuFusion for Fusion<ICM20948, Mahony> {
    fn update_imu(&mut self, dt: f32, a: Vector3<f32>, g: Vector3<f32>) -> UnitQuaternion<f32> {
        Self::update_imu(self, dt, a, g)
    }
}

// full MARG update (accel + gyro + mag) - gives yaw an absolute reference instead of pure
// gyro integration. Madgwick/Vqf/Mahony all expose a matching update for the ICM20948
pub trait MargFusion {
    fn update(
        &mut self,
        dt: f32,
        a: Vector3<f32>,
        g: Vector3<f32>,
        m: Vector3<f32>,
    ) -> UnitQuaternion<f32>;
}

impl MargFusion for Fusion<ICM20948, Madgwick> {
    fn update(
        &mut self,
        dt: f32,
        a: Vector3<f32>,
        g: Vector3<f32>,
        m: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        Self::update(self, dt, a, g, m)
    }
}

impl MargFusion for Fusion<ICM20948, Vqf> {
    fn update(
        &mut self,
        dt: f32,
        a: Vector3<f32>,
        g: Vector3<f32>,
        m: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        Self::update(self, dt, a, g, m)
    }
}

impl MargFusion for Fusion<ICM20948, Mahony> {
    fn update(
        &mut self,
        dt: f32,
        a: Vector3<f32>,
        g: Vector3<f32>,
        m: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        Self::update(self, dt, a, g, m)
    }
}

pub mod utils {
    #[allow(clippy::too_many_arguments)]
    /// Core complementary filter step. Alpha is the gyro trust weight (e.g. 0.98).
    pub fn complementary_filter(
        angle_roll: f32,
        angle_pitch: f32,
        gyro_x: f32,
        gyro_y: f32,
        dt: f32,
        acc_roll: f32,
        acc_pitch: f32,
        alpha: f32,
    ) -> (f32, f32) {
        let angle_roll = alpha * (angle_roll + gyro_x * dt) + (1.0 - alpha) * acc_roll;
        let angle_pitch = alpha * (angle_pitch + gyro_y * dt) + (1.0 - alpha) * acc_pitch;
        (angle_roll, angle_pitch)
    }
}
