//! # MPU9250 sensor driver.
//!
//! `embedded_hal_async` based driver with i2c access to MPU9250 (MPU6050 + AK8963)
//!
//! ### Misc
//! * [Register sheet](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf),
//! * [Data sheet](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6500-Datasheet2.pdf)
//!
//! To use this driver you must provide a concrete `embedded_hal_async` implementation.

#![no_std]

pub mod ak8963;
mod bits;
pub mod device;

use crate::device::*;
use embedded_hal_async::{delay::DelayNs, i2c::I2c};
use libm::{atan2f, sqrtf};
use nalgebra::{Vector2, Vector3};

/// PI, f32
pub const PI: f32 = core::f32::consts::PI;

/// PI / 180, for conversion to radians
pub const PI_180: f32 = PI / 180.0;

/// MPU9250 WHO_AM_I value
pub const MPU9250_WHOAMI: u8 = 0x71;
/// MPU9255 WHO_AM_I value
pub const MPU9255_WHOAMI: u8 = 0x73;
/// MPU6050 WHO_AM_I value
pub const MPU6050_WHOAMI: u8 = 0x68;

// MPU9250 uses 0x1D for accelerometer filtering
pub const ACCEL_CONFIG_2: u8 = 0x1D;

/// All possible errors in this crate
#[derive(Debug)]
pub enum Mpu6050Error<E> {
    /// I2C bus error
    I2c(E),
    /// Invalid chip ID was read
    InvalidChipId(u8),
}

/// All possible errors in this crate
#[derive(Debug)]
pub enum Mpu9250Error<E> {
    /// I2C bus error
    I2c(E),
    /// Invalid chip ID was read
    InvalidChipId(u8),
    /// Magnetometer data was not ready
    MagDataNotReady,
    /// Magnetometer sensor overflowed
    MagOverflow,
}

impl<E> From<Mpu6050Error<E>> for Mpu9250Error<E> {
    fn from(e: Mpu6050Error<E>) -> Self {
        match e {
            Mpu6050Error::I2c(e) => Mpu9250Error::I2c(e),
            Mpu6050Error::InvalidChipId(id) => Mpu9250Error::InvalidChipId(id),
        }
    }
}

/// Handles all operations on/with Mpu6050
pub struct Mpu6050<I2C> {
    pub i2c: I2C,
    pub slave_addr: u8,
    pub acc_sensitivity: f32,
    pub gyro_sensitivity: f32,
}

impl<I2C, E> Mpu6050<I2C>
where
    I2C: I2c<Error = E>,
{
    /// Side effect free constructor with default sensitivies, no calibration
    pub fn new(i2c: I2C) -> Self {
        Mpu6050 {
            i2c,
            slave_addr: DEFAULT_SLAVE_ADDR,
            acc_sensitivity: ACCEL_SENS[0],
            gyro_sensitivity: GYRO_SENS[0],
        }
    }

    /// custom sensitivity
    pub fn new_with_sens(i2c: I2C, arange: AccelRange, grange: GyroRange) -> Self {
        Mpu6050 {
            i2c,
            slave_addr: DEFAULT_SLAVE_ADDR,
            acc_sensitivity: arange.sensitivity(),
            gyro_sensitivity: grange.sensitivity(),
        }
    }

    /// Same as `new`, but the chip address can be specified (e.g. 0x69, if the A0 pin is pulled up)
    pub fn new_with_addr(i2c: I2C, slave_addr: u8) -> Self {
        Mpu6050 {
            i2c,
            slave_addr,
            acc_sensitivity: ACCEL_SENS[0],
            gyro_sensitivity: GYRO_SENS[0],
        }
    }

    /// Combination of `new_with_sens` and `new_with_addr`
    pub fn new_with_addr_and_sens(
        i2c: I2C,
        slave_addr: u8,
        arange: AccelRange,
        grange: GyroRange,
    ) -> Self {
        Mpu6050 {
            i2c,
            slave_addr,
            acc_sensitivity: arange.sensitivity(),
            gyro_sensitivity: grange.sensitivity(),
        }
    }

    /// Wakes MPU6050 with all sensors enabled (default)
    pub async fn wake<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Mpu6050Error<E>> {
        // MPU6050 has sleep enabled by default -> set bit 0 to wake
        // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001 (See Register Map )
        self.write_byte(PWR_MGMT_1::ADDR, 0x01).await?;
        delay.delay_ms(100u32).await;
        Ok(())
    }

    /// From Register map:
    /// "An  internal  8MHz  oscillator,  gyroscope based  clock,or  external  sources  can  be
    /// selected  as the MPU-60X0 clock source.
    /// When the internal 8 MHz oscillator or an external source is chosen as the clock source,
    /// the MPU-60X0 can operate in low power modes with the gyroscopes disabled. Upon power up,
    /// the MPU-60X0clock source defaults to the internal oscillator. However, it is highly
    /// recommended  that  the  device beconfigured  to  use  one  of  the  gyroscopes
    /// (or  an  external  clocksource) as the clock reference for improved stability.
    /// The clock source can be selected according to the following table...."
    pub async fn set_clock_source(&mut self, source: CLKSEL) -> Result<(), Mpu6050Error<E>> {
        self.write_bits(
            PWR_MGMT_1::ADDR,
            PWR_MGMT_1::CLKSEL.bit,
            PWR_MGMT_1::CLKSEL.length,
            source as u8,
        )
        .await
    }

    /// get current clock source
    pub async fn get_clock_source(&mut self) -> Result<CLKSEL, Mpu6050Error<E>> {
        let source = self
            .read_bits(
                PWR_MGMT_1::ADDR,
                PWR_MGMT_1::CLKSEL.bit,
                PWR_MGMT_1::CLKSEL.length,
            )
            .await?;
        Ok(CLKSEL::from(source))
    }

    /// Init wakes MPU6050 and verifies register addr, e.g. in i2c
    pub async fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Mpu6050Error<E>> {
        self.wake(delay).await?;
        self.verify().await?;
        self.set_accel_range(AccelRange::G2).await?;
        self.set_gyro_range(GyroRange::D250).await?;
        self.set_accel_hpf(ACCEL_HPF::_RESET).await?;
        Ok(())
    }

    /// Verifies the chip identity by reading the WHOAMI register and comparing
    /// against the known MPU6050 chip ID (0x68), not the I2C slave address.
    async fn verify(&mut self) -> Result<(), Mpu6050Error<E>> {
        let chip_id = self.read_byte(WHOAMI).await?;
        if chip_id != MPU6050_WHOAMI
            && chip_id != MPU9250_WHOAMI
            && chip_id != MPU9255_WHOAMI
            && chip_id != 0x70
        {
            return Err(Mpu6050Error::InvalidChipId(chip_id));
        }
        Ok(())
    }

    /// setup motion detection
    /// sources:
    /// * <https://github.com/kriswiner/MPU6050/blob/a7e0c8ba61a56c5326b2bcd64bc81ab72ee4616b/MPU6050IMU.ino#L486>
    /// * <https://arduino.stackexchange.com/a/48430>
    pub async fn setup_motion_detection(&mut self) -> Result<(), Mpu6050Error<E>> {
        self.write_byte(0x6B, 0x00).await?;
        // optional? self.write_byte(0x68, 0x07)?; // Reset all internal signal paths in the MPU-6050 by writing 0x07 to register 0x68;
        self.write_byte(INT_PIN_CFG::ADDR, 0x20).await?; //write register 0x37 to select how to use the interrupt pin. For an active high, push-pull signal that stays until register (decimal) 58 is read, write 0x20.
        self.write_byte(ACCEL_CONFIG::ADDR, 0x01).await?; //Write register 28 (==0x1C) to set the Digital High Pass Filter, bits 3:0. For example set it to 0x01 for 5Hz. (These 3 bits are grey in the data sheet, but they are used! Leaving them 0 means the filter always outputs 0.)
        self.write_byte(MOT_THR, 10).await?; //Write the desired Motion threshold to register 0x1F (For example, write decimal 20).
        self.write_byte(MOT_DUR, 40).await?; //Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate
        self.write_byte(0x69, 0x15).await?; //to register 0x69, write the motion detection decrement and a few other settings (for example write 0x15 to set both free-fall and motion decrements to 1 and accelerometer start-up delay to 5ms total by adding 1ms. )
        self.write_byte(INT_ENABLE::ADDR, 0x40).await?; //write register 0x38, bit 6 (0x40), to enable motion detection interrupt.
        Ok(())
    }

    /// get whether or not motion has been detected (INT_STATUS, MOT_INT)
    pub async fn get_motion_detected(&mut self) -> Result<bool, Mpu6050Error<E>> {
        Ok(self.read_bit(INT_STATUS::ADDR, INT_STATUS::MOT_INT).await? != 0)
    }

    /// set accel high pass filter mode
    pub async fn set_accel_hpf(&mut self, mode: ACCEL_HPF) -> Result<(), Mpu6050Error<E>> {
        self.write_bits(
            ACCEL_CONFIG::ADDR,
            ACCEL_CONFIG::ACCEL_HPF.bit,
            ACCEL_CONFIG::ACCEL_HPF.length,
            mode as u8,
        )
        .await
    }

    /// get accel high pass filter mode
    pub async fn get_accel_hpf(&mut self) -> Result<ACCEL_HPF, Mpu6050Error<E>> {
        let mode: u8 = self
            .read_bits(
                ACCEL_CONFIG::ADDR,
                ACCEL_CONFIG::ACCEL_HPF.bit,
                ACCEL_CONFIG::ACCEL_HPF.length,
            )
            .await?;

        Ok(ACCEL_HPF::from(mode))
    }

    /// Set gyro range, and update sensitivity accordingly
    pub async fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Mpu6050Error<E>> {
        self.write_bits(
            GYRO_CONFIG::ADDR,
            GYRO_CONFIG::FS_SEL.bit,
            GYRO_CONFIG::FS_SEL.length,
            range as u8,
        )
        .await?;

        self.gyro_sensitivity = range.sensitivity();
        Ok(())
    }

    /// get current gyro range
    pub async fn get_gyro_range(&mut self) -> Result<GyroRange, Mpu6050Error<E>> {
        let byte = self
            .read_bits(
                GYRO_CONFIG::ADDR,
                GYRO_CONFIG::FS_SEL.bit,
                GYRO_CONFIG::FS_SEL.length,
            )
            .await?;

        Ok(GyroRange::from(byte))
    }

    /// set accel range, and update sensitivy accordingly
    pub async fn set_accel_range(&mut self, range: AccelRange) -> Result<(), Mpu6050Error<E>> {
        self.write_bits(
            ACCEL_CONFIG::ADDR,
            ACCEL_CONFIG::FS_SEL.bit,
            ACCEL_CONFIG::FS_SEL.length,
            range as u8,
        )
        .await?;

        self.acc_sensitivity = range.sensitivity();
        Ok(())
    }

    /// get current accel_range
    pub async fn get_accel_range(&mut self) -> Result<AccelRange, Mpu6050Error<E>> {
        let byte = self
            .read_bits(
                ACCEL_CONFIG::ADDR,
                ACCEL_CONFIG::FS_SEL.bit,
                ACCEL_CONFIG::FS_SEL.length,
            )
            .await?;

        Ok(AccelRange::from(byte))
    }

    /// reset device
    pub async fn reset_device<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Mpu6050Error<E>> {
        self.write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::DEVICE_RESET, true)
            .await?;
        delay.delay_ms(100u32).await;
        // Note: Reset sets sleep to true! Section register map: resets PWR_MGMT to 0x40
        Ok(())
    }

    /// enable, disable i2c master interrupt
    pub async fn set_master_interrupt_enabled(
        &mut self,
        enable: bool,
    ) -> Result<(), Mpu6050Error<E>> {
        self.write_bit(INT_ENABLE::ADDR, INT_ENABLE::I2C_MST_INT_EN, enable)
            .await
    }

    /// get i2c master interrupt status
    pub async fn get_master_interrupt_enabled(&mut self) -> Result<bool, Mpu6050Error<E>> {
        Ok(self
            .read_bit(INT_ENABLE::ADDR, INT_ENABLE::I2C_MST_INT_EN)
            .await?
            != 0)
    }

    /// enable, disable bypass of sensor
    pub async fn set_bypass_enabled(&mut self, enable: bool) -> Result<(), Mpu6050Error<E>> {
        self.write_bit(INT_PIN_CFG::ADDR, INT_PIN_CFG::I2C_BYPASS_EN, enable)
            .await
    }

    /// get bypass status
    pub async fn get_bypass_enabled(&mut self) -> Result<bool, Mpu6050Error<E>> {
        Ok(self
            .read_bit(INT_PIN_CFG::ADDR, INT_PIN_CFG::I2C_BYPASS_EN)
            .await?
            != 0)
    }

    /// enable, disable sleep of sensor
    pub async fn set_sleep_enabled(&mut self, enable: bool) -> Result<(), Mpu6050Error<E>> {
        self.write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::SLEEP, enable)
            .await
    }

    /// get sleep status
    pub async fn get_sleep_enabled(&mut self) -> Result<bool, Mpu6050Error<E>> {
        Ok(self.read_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::SLEEP).await? != 0)
    }

    /// enable, disable temperature measurement of sensor
    /// TEMP_DIS actually saves "disabled status"
    /// 1 is disabled! -> enable=true : bit=!enable
    pub async fn set_temp_enabled(&mut self, enable: bool) -> Result<(), Mpu6050Error<E>> {
        self.write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::TEMP_DIS, !enable)
            .await
    }

    /// get temperature sensor status
    /// TEMP_DIS actually saves "disabled status"
    /// 1 is disabled! -> 1 == 0 : false, 0 == 0 : true
    pub async fn get_temp_enabled(&mut self) -> Result<bool, Mpu6050Error<E>> {
        Ok(self
            .read_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::TEMP_DIS)
            .await?
            == 0)
    }

    /// set accel x self test
    pub async fn set_accel_x_self_test(&mut self, enable: bool) -> Result<(), Mpu6050Error<E>> {
        self.write_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::XA_ST, enable)
            .await
    }

    /// get accel x self test
    pub async fn get_accel_x_self_test(&mut self) -> Result<bool, Mpu6050Error<E>> {
        Ok(self
            .read_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::XA_ST)
            .await?
            != 0)
    }

    /// set accel y self test
    pub async fn set_accel_y_self_test(&mut self, enable: bool) -> Result<(), Mpu6050Error<E>> {
        self.write_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::YA_ST, enable)
            .await
    }

    /// get accel y self test
    pub async fn get_accel_y_self_test(&mut self) -> Result<bool, Mpu6050Error<E>> {
        Ok(self
            .read_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::YA_ST)
            .await?
            != 0)
    }

    /// set accel z self test
    pub async fn set_accel_z_self_test(&mut self, enable: bool) -> Result<(), Mpu6050Error<E>> {
        self.write_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::ZA_ST, enable)
            .await
    }

    /// get accel z self test
    pub async fn get_accel_z_self_test(&mut self) -> Result<bool, Mpu6050Error<E>> {
        Ok(self
            .read_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::ZA_ST)
            .await?
            != 0)
    }

    /// Roll and pitch estimation from raw accelerometer readings
    /// NOTE: no yaw! no magnetometer present on MPU6050
    /// <https://www.nxp.com/docs/en/application-note/AN3461.pdf> equation 28, 29
    pub async fn get_acc_angles(&mut self) -> Result<Vector2<f32>, Mpu6050Error<E>> {
        let acc = self.get_acc().await?;

        Ok(Vector2::<f32>::new(
            atan2f(acc.y, libm::sqrtf(acc.x * acc.x + acc.z * acc.z)),
            atan2f(-acc.x, sqrtf(acc.y * acc.y + acc.z * acc.z)),
        ))
    }

    /// Converts 2 bytes number in 2 compliment
    fn read_word_2c(&self, byte: &[u8]) -> i32 {
        i16::from_be_bytes([byte[0], byte[1]]) as i32
    }

    /// Reads rotation (gyro/acc) from specified register
    async fn read_rot(&mut self, reg: u8) -> Result<Vector3<f32>, Mpu6050Error<E>> {
        let mut buf: [u8; 6] = [0; 6];
        self.read_bytes(reg, &mut buf).await?;

        Ok(Vector3::<f32>::new(
            self.read_word_2c(&buf[0..2]) as f32,
            self.read_word_2c(&buf[2..4]) as f32,
            self.read_word_2c(&buf[4..6]) as f32,
        ))
    }

    /// Accelerometer readings in g
    pub async fn get_acc(&mut self) -> Result<Vector3<f32>, Mpu6050Error<E>> {
        let mut acc = self.read_rot(ACC_REGX_H).await?;
        acc /= self.acc_sensitivity;

        Ok(acc)
    }

    /// Gyro readings in rad/s
    pub async fn get_gyro(&mut self) -> Result<Vector3<f32>, Mpu6050Error<E>> {
        let mut gyro = self.read_rot(GYRO_REGX_H).await?;

        gyro *= PI_180 / self.gyro_sensitivity;

        Ok(gyro)
    }

    /// Sensor Temp in degrees celcius
    pub async fn get_temp(&mut self) -> Result<f32, Mpu6050Error<E>> {
        let mut buf: [u8; 2] = [0; 2];
        self.read_bytes(TEMP_OUT_H, &mut buf).await?;
        let raw_temp = self.read_word_2c(&buf[0..2]) as f32;

        // According to revision 4.2
        Ok((raw_temp / TEMP_SENSITIVITY) + TEMP_OFFSET)
    }

    /// Writes byte to register
    pub async fn write_byte(&mut self, reg: u8, byte: u8) -> Result<(), Mpu6050Error<E>> {
        self.i2c
            .write(self.slave_addr, &[reg, byte])
            .await
            .map_err(Mpu6050Error::I2c)?;
        Ok(())
    }

    /// Enables bit n at register address reg
    pub async fn write_bit(
        &mut self,
        reg: u8,
        bit_n: u8,
        enable: bool,
    ) -> Result<(), Mpu6050Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte).await?;
        bits::set_bit(&mut byte[0], bit_n, enable);
        self.write_byte(reg, byte[0]).await
    }

    /// Write bits data at reg from start_bit to start_bit+length
    pub async fn write_bits(
        &mut self,
        reg: u8,
        start_bit: u8,
        length: u8,
        data: u8,
    ) -> Result<(), Mpu6050Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte).await?;
        bits::set_bits(&mut byte[0], start_bit, length, data);
        self.write_byte(reg, byte[0]).await
    }

    /// Read bit n from register
    async fn read_bit(&mut self, reg: u8, bit_n: u8) -> Result<u8, Mpu6050Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte).await?;
        Ok(bits::get_bit(byte[0], bit_n))
    }

    /// Read bits at register reg, starting with bit start_bit, until start_bit+length
    pub async fn read_bits(
        &mut self,
        reg: u8,
        start_bit: u8,
        length: u8,
    ) -> Result<u8, Mpu6050Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte).await?;
        Ok(bits::get_bits(byte[0], start_bit, length))
    }

    /// Reads byte from register
    pub async fn read_byte(&mut self, reg: u8) -> Result<u8, Mpu6050Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.i2c
            .write_read(self.slave_addr, &[reg], &mut byte)
            .await
            .map_err(Mpu6050Error::I2c)?;
        Ok(byte[0])
    }

    /// Reads series of bytes into buf from specified reg
    pub async fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Mpu6050Error<E>> {
        self.i2c
            .write_read(self.slave_addr, &[reg], buf)
            .await
            .map_err(Mpu6050Error::I2c)?;
        Ok(())
    }
}

/// MPU9250: MPU6050 + AK8963 magnetometer
///
/// Uses I2C bypass mode so the AK8963 is accessible directly on the main I2C bus.
pub struct Mpu9250<I2C, D> {
    mpu: Mpu6050<I2C>,
    ak: ak8963::Ak8963, // no I2C type param
    delay: D,
}

#[derive(Debug, Clone, Copy)]
pub struct SensorData {
    pub accel: Vector3<f32>, // g
    pub gyro: Vector3<f32>,  // rad/s
    pub mag: Vector3<f32>,   // µT
}

impl<I2C, E, D> Mpu9250<I2C, D>
where
    I2C: I2c<Error = E>,
{
    pub async fn new(i2c: I2C, delay: D) -> Result<Self, Mpu9250Error<E>>
    where
        D: DelayNs,
    {
        Self::new_with_addr_and_sens(
            i2c,
            DEFAULT_SLAVE_ADDR,
            AccelRange::G2,
            GyroRange::D250,
            delay,
        )
        .await
    }

    /// custom sensitivity
    pub async fn new_with_sens(
        mpu_bus: I2C,
        arange: AccelRange,
        grange: GyroRange,
        delay: D,
    ) -> Result<Self, Mpu9250Error<E>>
    where
        D: DelayNs,
    {
        Self::new_with_addr_and_sens(mpu_bus, DEFAULT_SLAVE_ADDR, arange, grange, delay).await
    }

    /// Same as `new`, but the chip address can be specified (e.g. 0x69, if the A0 pin is pulled up)
    pub async fn new_with_addr(
        mpu_bus: I2C,
        slave_addr: u8,
        delay: D,
    ) -> Result<Self, Mpu9250Error<E>>
    where
        D: DelayNs,
    {
        Self::new_with_addr_and_sens(mpu_bus, slave_addr, AccelRange::G2, GyroRange::D250, delay)
            .await
    }

    /// Combination of `new_with_sens` and `new_with_addr`
    pub async fn new_with_addr_and_sens(
        mpu_bus: I2C,
        slave_addr: u8,
        arange: AccelRange,
        grange: GyroRange,
        mut delay: D,
    ) -> Result<Self, Mpu9250Error<E>>
    where
        D: DelayNs,
    {
        let mut mpu = Mpu6050::new_with_addr_and_sens(mpu_bus, slave_addr, arange, grange);

        mpu.write_byte(PWR_MGMT_1::ADDR, 1 << PWR_MGMT_1::DEVICE_RESET)
            .await?;
        delay.delay_ms(100u32).await;

        mpu.wake(&mut delay).await?;

        let who = mpu.read_byte(WHOAMI).await?;
        // Also accept 0x70, a known WHO_AM_I value on clone GY-9250 boards
        if who != MPU9250_WHOAMI && who != MPU9255_WHOAMI && who != 0x70 {
            return Err(Mpu9250Error::InvalidChipId(who));
        }

        mpu.set_accel_range(arange).await?;
        mpu.set_gyro_range(grange).await?;
        mpu.set_accel_hpf(ACCEL_HPF::_RESET).await?;

        // Enable I2C bypass so AK8963 is visible on the main bus
        mpu.set_bypass_enabled(true).await?;
        delay.delay_ms(50u32).await;

        let ak = ak8963::Ak8963::init(&mut mpu.i2c, &mut delay).await?;

        Ok(Self { mpu, ak, delay })
    }

    pub async fn get_all(&mut self) -> Result<SensorData, Mpu9250Error<E>>
    where
        D: DelayNs,
    {
        Ok(SensorData {
            accel: self.get_acc().await?,
            gyro: self.get_gyro().await?,
            mag: self.get_mag().await?,
        })
    }

    /// Sensor temperature in °C (MPU-9250 specific coefficients).
    pub async fn get_temp(&mut self) -> Result<f32, Mpu9250Error<E>> {
        let mut buf: [u8; 2] = [0; 2];
        self.read_bytes(TEMP_OUT_H, &mut buf).await?;
        let raw = i16::from_be_bytes([buf[0], buf[1]]) as f32;
        // MPU-9250 datasheet: Temp = (RAW / 333.87) + 21.0
        Ok((raw / 333.87) + 21.0)
    }

    /// Magnetometer reading in µT. Delegates entirely to [`ak8963::Ak8963::read_mag`].
    /// Axis are remapped to match the accelerometer/gyroscope axis
    pub async fn get_mag(&mut self) -> Result<Vector3<f32>, Mpu9250Error<E>>
    where
        D: DelayNs,
    {
        let (mx, my, mz) = self.ak.read_mag(&mut self.mpu.i2c, &mut self.delay).await?;
        // Axis remap
        Ok(Vector3::new(my, mx, -mz))
    }

    /// Magnetometer reading in µT. Delegates entirely to [`ak8963::Ak8963::read_mag`].
    /// Axis are not remapped
    pub async fn get_mag_raw(&mut self) -> Result<(i16, i16, i16), Mpu9250Error<E>> {
        let mut buf = [0u8; 6];
        self.ak.read_raw(&mut self.mpu.i2c, &mut buf).await?;
        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);
        Ok((x, y, z))
    }

    pub async fn set_accel_dlpf(&mut self, dlpf_cfg: u8) -> Result<(), Mpu9250Error<E>> {
        // Bits [2:0] control the Digital Low Pass Filter (A_DLPFCFG)
        // Bit 3 controls accel_fchoice_b (0 to enable filter)
        self.mpu.write_byte(ACCEL_CONFIG_2, dlpf_cfg).await?;
        Ok(())
    }

    /// setup motion detection
    pub async fn setup_motion_detection(&mut self) -> Result<(), Mpu9250Error<E>> {
        // Write to MOT_DETECT_CTRL (0x69)
        // Bit 7: ACCEL_INTEL_EN (Enable Accel Intel hardware)
        // Bit 6: ACCEL_INTEL_MODE (1 = Compare current sample with previous sample)
        // 0xC0 is 11000000 in binary.
        self.mpu.write_byte(MOT_DETECT_CONTROL::ADDR, 0xC0).await?;

        // Set the Wake-On-Motion Threshold in WOM_THR (0x1F)
        // (You can adjust this threshold value, e.g., 0x14)
        self.mpu.write_byte(0x1F, 0x14).await?;

        // Enable the Wake-on-Motion Interrupt in INT_ENABLE (0x38)
        // Bit 6: WOM_EN
        self.mpu.write_byte(INT_ENABLE::ADDR, 0x40).await?;

        Ok(())
    }
}

impl<I2C, D> core::ops::Deref for Mpu9250<I2C, D> {
    type Target = Mpu6050<I2C>;

    fn deref(&self) -> &Self::Target {
        &self.mpu
    }
}

impl<I2C, D> core::ops::DerefMut for Mpu9250<I2C, D> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.mpu
    }
}
