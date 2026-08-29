#![allow(async_fn_in_trait)]

use nalgebra::Vector3;

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
