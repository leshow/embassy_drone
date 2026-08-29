#![allow(unused)]

use core::fmt::Debug;

use embedded_hal_async::i2c::I2c;
#[cfg(not(feature = "mag"))]
use icm20948_async::MagDisabled;
#[cfg(feature = "mag")]
use icm20948_async::MagEnabled;
use icm20948_async::{
    AccDlp, AccRange, AccUnit, GyrDlp, GyrRange, GyrUnit, Icm20948, IcmBuilder, SetupError,
    SpiDevice, Transport,
};
use mpu9250_async::{Mpu6050, Mpu6050Error};
use nalgebra::{Vector2, Vector3};

use libs::flight::sensors::{ImuRead, ImuReadMag};

pub struct Sensor<D> {
    driver: D,
}

// MPU6050 (6DOF, no mag) ---

impl<I: I2c> Sensor<Mpu6050<I>> {
    pub async fn init_mpu6050(i2c: I) -> Result<Self, Mpu6050Error<I::Error>> {
        let mut driver = Mpu6050::new(i2c);
        driver.init(&mut embassy_time::Delay).await?;
        defmt::info!("MPU6050 init OK");
        Ok(Self { driver })
    }
}

impl<I: I2c> ImuRead for Sensor<Mpu6050<I>> {
    type Error = Mpu6050Error<I::Error>;

    async fn read(&mut self) -> Result<(Vector3<f32>, Vector3<f32>), Self::Error> {
        let angles: Vector2<f32> = self.driver.get_acc_angles().await?;
        let gyro: Vector3<f32> = self.driver.get_gyro().await?;
        Ok((Vector3::new(angles.x, angles.y, 0.0), gyro))
    }
}

// ICM20948 (9DOF, with mag) ---

// divider=1 -> gyro ~550Hz (1100/2), accel ~562Hz (1125/2). gyro DLPF is Hz196: the real noise
// filtering is done downstream by the tunable, per-tick RATE_LPF_HZ software filter (see
// flight.rs), so the hardware DLPF just needs to be wide enough not to add its own phase lag
// on top - a narrower cutoff here would be redundant filtering that only costs delay.
#[cfg(feature = "mag")]
impl<SPI: embedded_hal_async::spi::SpiDevice> Sensor<Icm20948<SpiDevice<SPI>, MagEnabled>> {
    pub async fn init_icm20948(spi: SPI) -> Result<Self, SetupError<SPI::Error>> {
        let mut driver = IcmBuilder::new_spi(spi, embassy_time::Delay)
            .acc_range(AccRange::Gs4)
            .acc_dlp(AccDlp::Hz111)
            .acc_odr(0)
            .acc_unit(AccUnit::Gs)
            .gyr_range(GyrRange::Dps1000)
            .gyr_dlp(GyrDlp::Hz196)
            .gyr_odr(0)
            .gyr_unit(GyrUnit::Rps)
            .initialize_9dof()
            .await?;

        // raw data-ready interrupt on the INT pin, so the flight loop can stay
        // interrupt-driven instead of a fixed timer
        driver.enable_data_ready_interrupt().await?;
        // SPI-only: stop the shared SDA/SCL-vs-SDI/SCLK pins from being mistaken for I2C traffic
        driver.disable_i2c_interface().await?;

        defmt::info!("ICM20948 init OK");
        Ok(Self { driver })
    }
}

#[cfg(not(feature = "mag"))]
impl<SPI: embedded_hal_async::spi::SpiDevice> Sensor<Icm20948<SpiDevice<SPI>, MagDisabled>> {
    pub async fn init_icm20948(spi: SPI) -> Result<Self, SetupError<SPI::Error>> {
        let mut driver = IcmBuilder::new_spi(spi, embassy_time::Delay)
            .acc_range(AccRange::Gs4)
            .acc_dlp(AccDlp::Hz111)
            .acc_odr(0)
            .acc_unit(AccUnit::Gs)
            .gyr_range(GyrRange::Dps1000)
            .gyr_dlp(GyrDlp::Hz196)
            .gyr_odr(0)
            .gyr_unit(GyrUnit::Rps)
            .initialize_6dof()
            .await?;

        // raw data-ready interrupt on the INT pin, so the flight loop can stay
        // interrupt-driven instead of a fixed timer
        driver.enable_data_ready_interrupt().await?;
        // SPI-only: stop the shared SDA/SCL-vs-SDI/SCLK pins from being mistaken for I2C traffic
        driver.disable_i2c_interface().await?;

        defmt::info!("ICM20948 init OK");
        Ok(Self { driver })
    }
}

// read_6dof does a single combined burst read spanning accel+gyro+temp (contiguous registers) -
// works the same regardless of transport or whether mag is enabled, so this impl is generic
// over both
impl<T: Transport, MAG> ImuRead for Sensor<Icm20948<T, MAG>> {
    type Error = T::Error;

    async fn read(&mut self) -> Result<(Vector3<f32>, Vector3<f32>), Self::Error> {
        let data = self.driver.read_6dof().await?;
        Ok((Vector3::from(data.acc), Vector3::from(data.gyr)))
    }
}

#[cfg(feature = "calibrate")]
impl<T: Transport, MAG> Sensor<Icm20948<T, MAG>>
where
    T::Error: Debug,
{
    /// 6-orientation tumble calibration - flix's calibrateAccel/calibrateAccelOnce pattern.
    /// Place the frame in each of the 6 orientations in turn; each one's averaged reading
    /// updates a running per-axis min/max across all orientations seen so far. Whichever
    /// orientation puts a given axis at +1g and whichever puts it at -1g (not necessarily
    /// "level") together give that axis's bias (midpoint) and scale (half the swing) - no
    /// single orientation needs to be precisely level.
    pub async fn run_calibration(&mut self) -> Option<crate::flight::AccelBias> {
        use libs::calibrate::CalibrationMode;

        const SAMPLES: u32 = 1000;
        const POSES: [CalibrationMode; 6] = [
            CalibrationMode::Level,
            CalibrationMode::FrontUp,
            CalibrationMode::BackUp,
            CalibrationMode::RightSide,
            CalibrationMode::LeftSide,
            CalibrationMode::UpsideDown,
        ];

        // wait for ground_control to start
        defmt::info!("waiting for ground_control to start calibration");
        crate::wifi::calibrate::START.wait().await;
        let publisher = crate::wifi::calibrate::EVENTS
            .publisher()
            .expect("calibration publisher already taken");

        // most sensitive range, for the best resolution on small deviations from 1g
        if let Err(e) = self.driver.set_acc_range(AccRange::Gs2).await {
            defmt::error!(
                "failed to switch to +/-2g range for calibration: {}",
                defmt::Debug2Format(&e)
            );
        }

        let mut acc_max = Vector3::new(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY);
        let mut acc_min = Vector3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY);
        const POSE_HOLD_SECS: u64 = 15;

        for pose in POSES {
            defmt::info!("Place {} - {}s", pose.name(), POSE_HOLD_SECS);
            publisher.publish(pose).await;
            embassy_time::Timer::after_secs(POSE_HOLD_SECS).await;

            let mut sum = Vector3::zeros();
            let mut successful: u32 = 0;
            for _ in 0..SAMPLES {
                match self.driver.read_acc().await {
                    Ok(a) => {
                        sum += Vector3::from(a);
                        successful += 1;
                    }
                    Err(e) => defmt::error!(
                        "accel read error during calibration: {}",
                        defmt::Debug2Format(&e)
                    ),
                }
            }
            if successful == 0 {
                defmt::error!(
                    "calibration pose {} got zero successful reads, aborting calibration",
                    pose.name()
                );
                return None;
            }
            let avg = sum / successful as f32;

            if avg.x > acc_max.x {
                acc_max.x = avg.x;
            }
            if avg.y > acc_max.y {
                acc_max.y = avg.y;
            }
            if avg.z > acc_max.z {
                acc_max.z = avg.z;
            }
            if avg.x < acc_min.x {
                acc_min.x = avg.x;
            }
            if avg.y < acc_min.y {
                acc_min.y = avg.y;
            }
            if avg.z < acc_min.z {
                acc_min.z = avg.z;
            }

            let bias = (acc_max + acc_min) / 2.0;
            let scale = (acc_max - acc_min) / 2.0;
            defmt::info!(
                "running bias: {} {} {} scale: {} {} {}",
                bias.x,
                bias.y,
                bias.z,
                scale.x,
                scale.y,
                scale.z
            );
        }

        Some(crate::flight::AccelBias {
            bias: (acc_max + acc_min) / 2.0,
            scale: (acc_max - acc_min) / 2.0,
        })
    }
}

#[cfg(all(feature = "calibrate", feature = "mag"))]
impl<T: Transport> Sensor<Icm20948<T, MagEnabled>>
where
    T::Error: Debug,
{
    /// magnetometer hard-iron/soft-iron calibration - no fixed poses, just rotate through as
    /// many orientations as possible while it collects samples. see libs::mag_calibration for
    /// the actual fitting algorithm; this just feeds it live samples and reports progress.
    /// stops once the sample buffer's coverage (mean_distance) crosses MIN_MEAN_DISTANCE, or
    /// gives up after MAX_SAMPLES if coverage never gets there (e.g. barely moved).
    ///
    /// called right after run_calibration in the same session - doesn't wait on
    /// wifi::calibrate::START itself, that wait already happened for the accel pass
    pub async fn run_mag_calibration(&mut self) -> Option<MagBias> {
        const N: usize = 30; // matches peterkrull/mag-calibrator-rs's own real-hardware usage
        const MIN_MEAN_DISTANCE: f32 = 0.035; // matches peterkrull/mag-calibrator-rs's own usage
        const MAX_SAMPLES: u32 = 3000; // ~2.5min at the 50ms sample period below

        let publisher = crate::wifi::calibrate::EVENTS
            .publisher()
            .expect("calibration publisher already taken");
        defmt::info!("rotate the drone through as many orientations as possible");
        publisher
            .publish(libs::calibrate::CalibrationMode::MagRotate)
            .await;

        // TODO: verify this against real raw magnetometer magnitude on the bench and adjust -
        // picked from peterkrull/mag-calibrator-rs's own real-hardware value as a starting
        // point, not measured on our hardware yet. see mag_calibration's tests for why a
        // mismatched pre_scaler can make the fit ill-conditioned
        let mut cal = libs::mag_calibration::MagCalibrator::<N>::new().pre_scaler(200.0);

        for _ in 0..MAX_SAMPLES {
            match self.driver.read_mag().await {
                Ok(m) => {
                    cal.evaluate_sample_vec(Vector3::from(m));
                    if cal.get_mean_distance() > MIN_MEAN_DISTANCE
                        && let Some((bias, scale)) = cal.perform_calibration()
                    {
                        return Some(MagBias {
                            bias: bias.into(),
                            scale: scale.into(),
                        });
                    }
                }
                Err(e) => defmt::error!(
                    "mag read error during calibration: {}",
                    defmt::Debug2Format(&e)
                ),
            }
            embassy_time::Timer::after_millis(50).await;
        }
        defmt::error!("mag calibration timed out without reaching good sample coverage");
        None
    }
}

/// One-time magnetometer bias + scale correction, from the mag rotate calibration routine
/// (--features calibrate,mag) - see calibration_storage. Not yet consumed by the live fusion
/// loop (see docs/todo.md) - needs its own health gate first, same idea as accel's.
#[cfg(feature = "mag")]
#[derive(Clone, Copy, Debug)]
pub(crate) struct MagBias {
    pub bias: Vector3<f32>,
    pub scale: Vector3<f32>,
}

#[cfg(feature = "mag")]
impl Default for MagBias {
    fn default() -> Self {
        Self {
            bias: Vector3::zeros(),
            scale: Vector3::new(1.0, 1.0, 1.0),
        }
    }
}

#[cfg(feature = "mag")]
impl<T: Transport> ImuReadMag for Sensor<Icm20948<T, MagEnabled>> {
    async fn read_mag(
        &mut self,
    ) -> Result<(Vector3<f32>, Vector3<f32>, Vector3<f32>), Self::Error> {
        let data = self.driver.read_9dof().await?;
        Ok((
            Vector3::from(data.acc),
            Vector3::from(data.gyr),
            Vector3::from(data.mag),
        ))
    }
}
