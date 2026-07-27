#![allow(unused)]

use embedded_hal_async::i2c::I2c;
use icm20948::{
    AccelCalibration, AccelConfig, AccelDlpf, AccelFullScale, GyroCalibration, GyroConfig,
    GyroDlpf, GyroFullScale, I2cInterface, Icm20948Driver, MagConfig,
    interrupt::{InterruptConfig, InterruptPinConfig},
};
use mpu9250_async::{Mpu6050, Mpu6050Error};
use nalgebra::{Vector2, Vector3};

pub trait ImuRead {
    type Error;
    /// Returns (accel, gyro). For 6DOF sensors; accel is (roll_rad, pitch_rad, 0.0).
    async fn read(&mut self) -> Result<(Vector3<f32>, Vector3<f32>), Self::Error>;
}

pub trait ImuReadMag: ImuRead {
    /// Returns (accel, gyro, mag). For 9DOF sensors.
    async fn read_mag(&mut self)
    -> Result<(Vector3<f32>, Vector3<f32>, Vector3<f32>), Self::Error>;
}

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

impl<I: I2c> Sensor<Icm20948Driver<I2cInterface<I>>> {
    pub async fn init_icm20948(i2c: I) -> Result<Self, icm20948::Error<I::Error>> {
        let interface = I2cInterface::alternative(i2c);
        let mut driver = Icm20948Driver::new(interface);
        driver.verify_who_am_i().await?;
        driver.init(&mut embassy_time::Delay).await?;

        // DMP locks full scale (gyro=2000dps, accel=±4g) and ODR via dmp_configure;
        // only DLPF settings survive into DMP mode.
        // Non-DMP mode uses ±500dps/±4g with loop-rate-based ODR dividers.
        #[cfg(feature = "dmp")]
        {
            // accel 111Hz / gyro 51Hz DLPF — full_scale and sample_rate_div
            // are overwritten by dmp_configure; only DLPF survives into DMP mode
            driver
                .configure_accelerometer(AccelConfig {
                    full_scale: AccelFullScale::G4,
                    dlpf: AccelDlpf::Hz111,
                    dlpf_enable: true,
                    sample_rate_div: 0,
                })
                .await?;
            driver
                .configure_gyroscope(GyroConfig {
                    full_scale: GyroFullScale::Dps2000,
                    dlpf: GyroDlpf::Hz51,
                    dlpf_enable: true,
                    sample_rate_div: 0,
                })
                .await?;
        }
        #[cfg(not(feature = "dmp"))]
        {
            // divider=1 -> gyro ~550Hz (1100/2), accel ~562Hz (1125/2)
            driver
                .configure_accelerometer(AccelConfig {
                    full_scale: AccelFullScale::G4,
                    dlpf: AccelDlpf::Hz111,
                    dlpf_enable: true,
                    sample_rate_div: 1,
                })
                .await?;
            // 500deg/s gives finer resolution than 2000dps for stable hover corrections;
            driver
                .configure_gyroscope(GyroConfig {
                    full_scale: GyroFullScale::Dps1000,
                    dlpf: GyroDlpf::Hz197,
                    dlpf_enable: true,
                    sample_rate_div: 1,
                })
                .await?;

            // raw data-ready interrupt on the same INT pin the DMP path uses, so the
            // flight loop can stay interrupt-driven instead of a fixed timer
            driver
                .configure_interrupt_pin(&InterruptPinConfig {
                    active_low: false,
                    open_drain: false,
                    latch_enabled: true,
                    clear_on_any_read: true,
                })
                .await?;
            driver
                .configure_interrupts(&InterruptConfig {
                    raw_data_ready: true,
                    ..Default::default()
                })
                .await?;
            // driver
            //     .init_magnetometer(MagConfig::default(), &mut embassy_time::Delay)
            //     .await
            //     .inspect_err(|e| {
            //         defmt::error!(
            //             "error initializing magnetometer: {}",
            //             defmt::Debug2Format(e)
            //         );
            //     })?
        }

        #[cfg(feature = "dmp")]
        {
            use defmt::info;
            use embassy_time::Delay;
            use icm20948::dmp::DmpConfig;

            // set DMP hz cycle here
            let dmp_hz = 100;
            let mut int_cfg = InterruptConfig::data_ready_only();
            int_cfg.dmp = true;
            driver.configure_interrupts(&int_cfg).await.unwrap();

            info!("Loading DMP firmware and configuring...");
            driver.dmp_init(&mut Delay).await.unwrap();
            driver.dmp_init_magnetometer(&mut Delay).await.unwrap();
            // active-high push-pull — no pull resistor needed on the INT wire
            driver
                .configure_interrupt_pin(&InterruptPinConfig {
                    active_low: false,
                    open_drain: false,
                    latch_enabled: true,
                    clear_on_any_read: true,
                })
                .await?;
            driver
                .configure_interrupts(&InterruptConfig {
                    dmp: true,
                    ..Default::default()
                })
                .await?;

            let dmp_config = DmpConfig::six_axis()
                .with_calibrated_gyro()
                // .with_calibrated_mag()
                .with_sample_rate(dmp_hz);

            driver.dmp_configure(&dmp_config).await.unwrap();
            driver.reset_fifo().await.unwrap();
            driver.dmp_enable(true).await?;
            embassy_time::Timer::after(embassy_time::Duration::from_millis(10)).await;
            // clear any interrupt that fired before we started listening
            let _ = driver.read_interrupt_status().await;
            defmt::info!("ICM20948 DMP enabled {}Hz", dmp_hz);
        }

        defmt::info!("ICM20948 init OK");
        Ok(Self { driver })
    }

    pub async fn read_dmp(
        &mut self,
    ) -> Result<Option<icm20948::dmp::DmpData>, icm20948::Error<I::Error>> {
        self.driver.dmp_read_fifo().await
    }

    pub async fn reset_fifo(&mut self) -> Result<(), icm20948::Error<I::Error>> {
        self.driver.reset_fifo().await
    }
}

impl<I: I2c> ImuRead for Sensor<Icm20948Driver<I2cInterface<I>>> {
    type Error = icm20948::Error<I::Error>;

    async fn read(&mut self) -> Result<(Vector3<f32>, Vector3<f32>), Self::Error> {
        let acc = self.driver.read_accelerometer().await?;
        let gyro = self.driver.read_gyroscope_radians().await?;
        Ok((
            Vector3::new(acc.x, acc.y, acc.z),
            Vector3::new(gyro.x, gyro.y, gyro.z),
        ))
    }
}

#[cfg(feature = "calibrate")]
impl<I: I2c> Sensor<Icm20948Driver<I2cInterface<I>>> {
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
        if let Err(e) = self
            .driver
            .configure_accelerometer(AccelConfig {
                full_scale: AccelFullScale::G2,
                dlpf: AccelDlpf::Hz111,
                dlpf_enable: true,
                sample_rate_div: 1,
            })
            .await
        {
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
                match self.driver.read_accelerometer().await {
                    Ok(a) => {
                        sum += Vector3::new(a.x, a.y, a.z);
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

impl<I: I2c> ImuReadMag for Sensor<Icm20948Driver<I2cInterface<I>>> {
    async fn read_mag(
        &mut self,
    ) -> Result<(Vector3<f32>, Vector3<f32>, Vector3<f32>), Self::Error> {
        let acc = self.driver.read_accelerometer().await?;
        let gyro = self.driver.read_gyroscope_radians().await?;
        let mag = self.driver.read_magnetometer().await?;
        Ok((
            Vector3::new(acc.x, acc.y, acc.z),
            Vector3::new(gyro.x, gyro.y, gyro.z),
            Vector3::new(mag.x, mag.y, mag.z),
        ))
    }
}
