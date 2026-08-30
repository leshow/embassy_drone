use defmt::info;
use embassy_time::Delay;
use embedded_hal_async::spi::SpiDevice;
use icm426xx::{Config, ICM42688, OutputDataRate};
use nalgebra::Vector3;

use libs::flight::sensors::ImuRead;

pub struct Sensor<D> {
    driver: D,
}

impl<I> Sensor<ICM42688<I, icm426xx::Ready>>
where
    I: SpiDevice,
    I::Error: defmt::Format,
{
    /// init sensor
    pub async fn init(spi: I) -> Result<Self, icm426xx::Error<I::Error>> {
        // 1kHz read rate
        let imu_config = Config {
            rate: OutputDataRate::Hz1000,
            ..Default::default()
        };
        // initialize() soft-resets the sensor and checks WHO_AM_I internally (expects 0x47)
        let driver = ICM42688::new(spi).initialize(Delay, imu_config).await?;
        info!("ICM42688 init OK, WHO_AM_I matched");
        Ok(Self { driver })
    }

    /// reset fifo queue and dump output
    pub async fn reset_fifo(&mut self) -> Result<(), icm426xx::Error<I::Error>> {
        self.driver.reset_fifo().await.map_err(icm426xx::Error::Bus)
    }
}

impl<I: SpiDevice> ImuRead for Sensor<ICM42688<I, icm426xx::Ready>> {
    type Error = icm426xx::Error<I::Error>;

    async fn read(&mut self) -> Result<(Vector3<f32>, Vector3<f32>), Self::Error> {
        loop {
            // only loops on a "successful" empty read
            if let Some((data, _data_remaining)) = self.driver.read_sample().await?
                && let (Some(accel), Some(gyro)) = (data.accel, data.gyro)
            {
                return Ok((
                    Vector3::new(accel.0, accel.1, accel.2),
                    Vector3::new(gyro.0, gyro.1, gyro.2),
                ));
            }
        }
    }
}
