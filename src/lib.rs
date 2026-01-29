#![no_std]

//! This module provides an asynchronous driver for the QMC5883P 3-Axis Magnetic Sensor using I2C.
//! # Hardware Details
//! * **I2C Address**: `0x2C`
//! * **Chip ID**: `0x80`
//! * **Register 0x29**: Used for Axis Sign definition (Undocumented in register map, but described in App Examples).

#[cfg(all(feature = "defmt", not(test)))]
use defmt::{error, info, trace, warn};

#[cfg(any(not(feature = "defmt"), test))]
mod logging_shim {

    macro_rules! nop {
        ($($tt:tt)*) => {};
    }
    pub(crate) use nop as error;
    pub(crate) use nop as info;
    pub(crate) use nop as trace;
    pub(crate) use nop as warn;
}

#[cfg(any(not(feature = "defmt"), test))]
use logging_shim::{error, info, trace, warn};

use embassy_time::Timer;
use embedded_hal_async::i2c::I2c;

#[cfg(feature = "magnitude")]
use micromath::F32Ext;

const I2C_ADDR: u8 = 0x2C; //
const QMC5883P_CHIP_ID: u8 = 0x80;

const REG_CHIP_ID: u8 = 0x00;
const REG_DATA_OUT_X_L: u8 = 0x01;
const REG_STATUS: u8 = 0x09;
const REG_CONTROL1: u8 = 0x0A;
const REG_CONTROL2: u8 = 0x0B;
const REG_AXIS_DEF: u8 = 0x29;

const RESET_VALUE: u8 = 0x80;
const AXIS_DEF_VALUE: u8 = 0x06;

const POLL_READY_LIMIT: usize = 50;

/// Operating mode of the sensor (Datasheet Sec 6.2).
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum Mode {
    /// Suspend Mode (Default): Very low power (~22uA).
    /// I2C bus is active, registers can be read/written, but no measurements are taken.
    Suspend = 0b00,
    /// Normal Mode: Periodic measurements based on the configured ODR (Output Data Rate).
    Normal = 0b01,
    /// Single Mode: Takes one measurement, updates registers, then automatically transitions to Suspend Mode.
    Single = 0b10,
    /// Continuous Mode: Continuous measurements at the maximum possible rate (up to 1.5kHz).
    /// Required for using the Self-Test function.
    Continuous = 0b11,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum OutputDataRate {
    Hz10 = 0b00,
    Hz50 = 0b01,
    Hz100 = 0b10,
    Hz200 = 0b11,
}

/// Over Sample Ratio (OSR1) - Digital Filter Bandwidth Control.
///
/// Larger OSR values lead to smaller filter bandwidth, less in-band noise,
/// but higher power consumption (Datasheet Sec 9.2.3).
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum OverSampleRatio1 {
    Ratio8 = 0b00,
    Ratio4 = 0b01,
    Ratio2 = 0b10,
    Ratio1 = 0b11,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum OverSampleRate {
    Rate1 = 0b00,
    Rate2 = 0b01,
    Rate4 = 0b10,
    Rate8 = 0b11,
}

/// Magnetic measurement range configuration.
///
/// Defines the full-scale range of the sensor and its corresponding sensitivity.
/// Sensitivity values are derived from the QMC5883P datasheet (Page 5).
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum Range {
    /// Range: ±2 Gauss (Sensitivity: 15000 LSB/Gauss)
    Gauss2 = 0b11,
    /// Range: ±8 Gauss (Sensitivity: 3750 LSB/Gauss)
    Gauss8 = 0b10,
    /// Range: ±12 Gauss (Sensitivity: 2500 LSB/Gauss)
    Gauss12 = 0b01,
    /// Range: ±30 Gauss (Sensitivity: 1000 LSB/Gauss)
    Gauss30 = 0b00,
}

impl Range {
    /// Returns the sensitivity in LSB per Gauss for this range.
    pub fn sensitivity(&self) -> f32 {
        match self {
            Range::Gauss2 => 15000.0,
            Range::Gauss8 => 3750.0,
            Range::Gauss12 => 2500.0,
            Range::Gauss30 => 1000.0,
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum SoftReset {
    Reset = 0b1,
    NoReset = 0b0,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum SelfTest {
    Enabled = 0b1,
    Disabled = 0b0,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum SetResetMode {
    SetAndResetOff = 0b11,
    SetOnly = 0b01,
    SetAndResetOn = 0b00,
}

/// Configuration structure for the QMC5883P sensor.
/// Use the builder pattern to create a configuration.
///
/// Example:
///
/// ```rust
/// use qmc5883p::{Range, Mode, Qmc5883PConfig, OutputDataRate, OverSampleRate, OverSampleRatio1 };
///
/// let config = Qmc5883PConfig::default()
///     .with_mode(Mode::Continuous)
///     .with_odr(OutputDataRate::Hz100)
///     .with_range(Range::Gauss8)
///     .with_osr1(OverSampleRatio1::Ratio4)
///     .with_osr2(OverSampleRate::Rate4);
///
/// ```
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy)]
pub struct Qmc5883PConfig {
    pub mode: Mode,
    pub odr: OutputDataRate,
    pub rng: Range,
    pub osr1: OverSampleRatio1,
    pub osr2: OverSampleRate,
    set_reset: SetResetMode,
    self_test: SelfTest,
}

impl Qmc5883PConfig {
    /// Refer to the default implementation for default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Chainable method to set the operating mode.
    pub fn with_mode(mut self, mode: Mode) -> Self {
        self.mode = mode;
        self
    }

    /// Chainable method to set the output data rate.
    pub fn with_odr(mut self, odr: OutputDataRate) -> Self {
        self.odr = odr;
        self
    }

    /// Chainable method to set the measurement range.
    pub fn with_range(mut self, rng: Range) -> Self {
        self.rng = rng;
        self
    }

    /// Chainable method to set the oversample ratio 1.
    pub fn with_osr1(mut self, osr1: OverSampleRatio1) -> Self {
        self.osr1 = osr1;
        self
    }

    /// Chainable method to set the oversample rate 2.
    pub fn with_osr2(mut self, osr2: OverSampleRate) -> Self {
        self.osr2 = osr2;
        self
    }

    pub fn to_control1_byte(self) -> u8 {
        let byte = (self.osr2 as u8) << 6
            | (self.osr1 as u8) << 4
            | (self.odr as u8) << 2
            | (self.mode as u8);
        trace!("Control1 Byte: 0b{:08b}", byte);
        byte
    }

    pub fn to_control2_byte(self) -> u8 {
        let byte = (self.self_test as u8) << 6 | (self.rng as u8) << 2 | (self.set_reset as u8);
        trace!("Control2 Byte: 0b{:08b}", byte);
        byte
    }
}

impl Default for Qmc5883PConfig {
    /// Start a new configuration with default values.
    fn default() -> Self {
        Self {
            mode: Mode::Continuous,
            odr: OutputDataRate::Hz200,
            rng: Range::Gauss30,
            osr1: OverSampleRatio1::Ratio1,
            osr2: OverSampleRate::Rate8,
            set_reset: SetResetMode::SetAndResetOn,
            self_test: SelfTest::Disabled,
        }
    }
}

pub struct Qmc5883p<I> {
    i2c: I,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum QmcError<E> {
    I2c(E),
    SelfTestFailed,
    DataNotReady,
    Overflow,
    WrongChipId(u8),
}

impl<E> From<E> for QmcError<E> {
    fn from(err: E) -> Self {
        QmcError::I2c(err)
    }
}

// TODO This will be somewhat re-written in 2.14.1 - Create modular magnetic sensor driver with builder config
// https://github.com/katkes/SmartParkingDetectionAndNavigation/issues/417
impl<I, E> Qmc5883p<I>
where
    I: I2c<Error = E>,
{
    pub fn new(i2c: I) -> Self {
        Self { i2c }
    }

    pub async fn init(&mut self, config: Qmc5883PConfig) -> Result<(), QmcError<E>> {
        trace!("Initializing QMC5883P Sensor...");
        self.check_id().await?;

        // Reset the sensor to defaults
        self.soft_reset().await?;

        // The datasheet requires this for Normal and Continuous Modes
        self.config_axis_sign().await?;

        self.apply_configuration(config).await?;

        Timer::after_millis(100).await;

        // Run self test
        if !self.self_test().await? {
            return Err(QmcError::SelfTestFailed);
        }
        Ok(())
    }

    /// Put the sensor into suspend mode to save power
    pub async fn deinit(&mut self) -> Result<(), QmcError<E>> {
        self.configure_control_register_1(0x00).await?;
        Ok(())
    }

    /// Applies the given configuration to the sensor by writing to the control registers.
    pub async fn apply_configuration(&mut self, config: Qmc5883PConfig) -> Result<(), QmcError<E>> {
        self.configure_control_register_1(config.to_control1_byte())
            .await?;
        self.configure_control_register_2(config.to_control2_byte())
            .await?;
        Ok(())
    }

    /// Checks if the sensor id matches the one from the datasheet.
    async fn check_id(&mut self) -> Result<(), QmcError<E>> {
        let mut id_buf = [0u8; 1];
        match self
            .i2c
            .write_read(I2C_ADDR, &[REG_CHIP_ID], &mut id_buf)
            .await
        {
            Ok(it) => it,
            Err(err) => {
                error!("I2C Error during Chip ID read");
                return Err(QmcError::I2c(err));
            }
        };
        trace!("Chip ID: 0x{:x}", id_buf[0]);

        if id_buf[0] != QMC5883P_CHIP_ID {
            warn!("Warning: Expected ID 0x80, got 0x{:x}", id_buf[0]);
            Err(QmcError::WrongChipId(id_buf[0]))
        } else {
            info!("QMC5883P Detected!");
            Ok(())
        }
    }

    /// Configure x, y and z axis sign by writing 0x06 to 29H register.
    async fn config_axis_sign(&mut self) -> Result<(), E> {
        self.i2c
            .write(I2C_ADDR, &[REG_AXIS_DEF, AXIS_DEF_VALUE])
            .await?;
        Ok(())
    }

    /// Soft Reset, resets all registers to default values.
    /// Write 0x80 to CTRL_2 (0x0B) to reset
    pub async fn soft_reset(&mut self) -> Result<(), E> {
        self.i2c
            .write(I2C_ADDR, &[REG_CONTROL2, RESET_VALUE])
            .await?;
        trace!("Sensor Reset Command Sent");

        // Wait a tiny bit for reset NOTE (if fails, try 100)
        Timer::after_millis(10).await;
        Ok(())
    }

    /// Configure Mode
    /// Register 0x0A (CTRL_1)
    ///
    /// Example:
    /// C3 -> 1100 0011 -> Continuous mode with OSR2 = 8
    async fn configure_control_register_1(&mut self, value: u8) -> Result<(), E> {
        self.i2c.write(I2C_ADDR, &[REG_CONTROL1, value]).await?;
        //
        // Wait for first sample (at 200Hz, 50ms is plenty)
        Timer::after_millis(50).await;
        trace!("Configured Control1 Register");
        Ok(())
    }

    /// Configure Control 2 (Range)
    ///
    /// Example:
    /// 0x00 = 30 Gauss Range, Set/Reset On
    async fn configure_control_register_2(&mut self, value: u8) -> Result<(), E> {
        self.i2c.write(I2C_ADDR, &[REG_CONTROL2, value]).await?;
        Timer::after_millis(20).await; // Wait for measurement
        Ok(())
    }

    /// Self-Test Procedure to verify QMC5883P works correctly.
    ///
    /// Uses the sequence defined in the QMC5883P datasheet to perform a self-test:
    ///
    /// - Write Register 29H by 0x06 (Define the sign for X Y and Z axis)
    /// - Write Register 0AH by 0x03 (set continuous mode)
    /// - Check status register 09H[0] ,”1” means ready
    /// - Read data Register 01H ~ 06H, recording as datax1/datay1/dataz1
    /// - Write Register 0BH by 0x40(enter self-test function)
    /// - Waiting 5 millisecond until measurement ends
    /// - Read data Register 01H ~ 06H, recording as datax2/datay2/dataz2
    /// - Calculate the delta (datax1-datax2), (datay2-datay1), (dataz2-dataz1)
    pub async fn self_test(&mut self) -> Result<bool, QmcError<E>> {
        info!("--- Starting Self-Test ---");

        self.config_axis_sign().await?;

        // Set Continuous Mode, BUT use 200Hz (0x1D) instead of 10Hz (0x03)
        // This ensures data is ready in ~5ms.
        self.configure_control_register_1(0x1D).await?;

        // Read Baseline
        let mut baseline = [0i16; 3];

        match self.poll_ready_and_check_overflow().await {
            Ok(it) => it,
            Err(err) => {
                error!("Self-Test Failed during baseline read");
                return Err(err);
            }
        };

        // Read Baseline
        let mut buf = [0u8; 6];
        self.i2c
            .write_read(I2C_ADDR, &[REG_DATA_OUT_X_L], &mut buf)
            .await?;
        baseline[0] = i16::from_le_bytes([buf[0], buf[1]]);
        baseline[1] = i16::from_le_bytes([buf[2], buf[3]]);
        baseline[2] = i16::from_le_bytes([buf[4], buf[5]]);

        // Enable Self-Test Current
        self.configure_control_register_2(0x40).await?;

        // Read Test Data
        self.i2c
            .write_read(I2C_ADDR, &[REG_DATA_OUT_X_L], &mut buf)
            .await?;
        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);

        // Restore Defaults (Disable Self Test, Range 30G)
        self.configure_control_register_2(0x00).await?;

        // Calculate Delta
        let delta_x = (x as i32 - baseline[0] as i32).abs();
        let delta_y = (y as i32 - baseline[1] as i32).abs();
        let delta_z = (z as i32 - baseline[2] as i32).abs();

        // Verify all axes moved significantly
        // 100 represents a significant change in magnetic field
        Ok(delta_x > 100 && delta_y > 100 && delta_z > 100)
    }

    /// Reads the X, Y, Z magnetic data from the sensor.
    pub async fn read_x_y_z(&mut self) -> Result<[i16; 3], QmcError<E>> {
        match self.poll_ready_and_check_overflow().await {
            Ok(it) => it,
            Err(err) => {
                error!("Self-Test Failed during baseline read");
                return Err(err);
            }
        };

        let mut buf = [0u8; 6];
        // Read 6 bytes starting from 0x01 (Data X LSB)
        self.i2c
            .write_read(I2C_ADDR, &[REG_DATA_OUT_X_L], &mut buf)
            .await?;

        trace!("Raw Mag Data: {:?}", buf);

        // Convert (Little Endian)
        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);

        Ok([x, y, z])
    }

    /// Reads the magnitude of the magnetic field vector.
    ///
    /// # Returns
    /// The magnitude in **raw LSB**. To convert to Gauss, divide this value by
    /// the sensitivity of your current `Range` setting, see [`Range::sensitivity`].
    ///
    /// * Range::Gauss2  -> / 15000.0
    /// * Range::Gauss8  -> / 3750.0
    /// * Range::Gauss12 -> / 2500.0
    /// * Range::Gauss30 -> / 1000.0
    #[cfg(feature = "magnitude")]
    pub async fn read_magnitude(&mut self) -> Result<f32, QmcError<E>> {
        let data = self.read_x_y_z().await?;
        let x = data[0] as f32;
        let y = data[1] as f32;
        let z = data[2] as f32;

        let magnitude = (x * x + y * y + z * z).sqrt();
        Ok(magnitude)
    }

    /// Polls the status register until the data is ready or timeout occurs after POLL_READY_LIMIT
    /// attempts. If when ready, the overflow bit is set, returns an Overflow error.
    pub async fn poll_ready_and_check_overflow(&mut self) -> Result<(), QmcError<E>> {
        // Poll for ready
        let mut status = [0u8; 1];
        for _ in 0..POLL_READY_LIMIT {
            self.i2c
                .write_read(I2C_ADDR, &[REG_STATUS], &mut status)
                .await?;
            if (status[0] & 0x01) != 0 {
                if (status[0] & 0x02) != 0 {
                    error!("Data Overflow Detected");
                    return Err(QmcError::Overflow);
                }
                return Ok(());
            }
            status.fill(0);
            Timer::after_millis(10).await;
        }

        error!("Poll Ready Timeout");
        Err(QmcError::DataNotReady)
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    extern crate std;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    #[tokio::test]
    async fn test_read_x_y_z() {
        //! Test reading X, Y, Z values from the sensor. We will mock the I2C transactions to
        //! return specific values.

        let expectations = [
            I2cTransaction::write_read(I2C_ADDR, std::vec![REG_STATUS], std::vec![0x01]),
            I2cTransaction::write_read(
                I2C_ADDR,
                std::vec![REG_DATA_OUT_X_L],
                std::vec![0x01, 0x02, 0x03, 0x04, 0x05, 0x06],
            ),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut sensor = Qmc5883p::new(i2c.clone());

        let res = sensor.read_x_y_z().await;
        assert!(res.is_ok(), "Read operation failed");
        let values = res.unwrap();

        assert!(values[0] == 0x0201, "Expected: {:#x}", values[0]);
        assert!(values[1] == 0x0403, "Expected: {:#x}", values[1]);
        assert!(values[2] == 0x0605, "Expected: {:#x}", values[2]);

        i2c.done();
    }

    #[tokio::test]
    async fn test_read_x_y_z_with_overflow() {
        //! Test that if the status register indicates overflow, the read_x_y_z function returns an
        //! error.

        let expectations = [I2cTransaction::write_read(
            I2C_ADDR,
            std::vec![REG_STATUS],
            std::vec![0x03],
        )];
        let mut i2c = I2cMock::new(&expectations);
        let mut sensor = Qmc5883p::new(i2c.clone());

        let res = sensor.read_x_y_z().await;
        assert_eq!(res, Err(QmcError::Overflow), "Overflow should be detected");

        i2c.done();
    }

    #[tokio::test]
    async fn test_overflow_detection() {
        //! Test overflow in isolation by calling the poll_ready_and_check_overflow function
        //! directly. We will mock the I2C transaction to return a status byte with the overflow
        //! bit set.

        let expectations = [I2cTransaction::write_read(
            I2C_ADDR,
            std::vec![REG_STATUS],
            std::vec![0x03],
        )];
        let mut i2c = I2cMock::new(&expectations);
        let mut sensor = Qmc5883p::new(i2c.clone());

        let res = sensor.poll_ready_and_check_overflow().await;

        assert!(
            res == Err(QmcError::Overflow),
            "Overflow should be detected"
        );
        i2c.done();
    }

    #[tokio::test]
    async fn test_soft_reset() {
        //! Test soft reset by verifying that the correct byte is written to the control register
        let expectations = [I2cTransaction::write(
            I2C_ADDR,
            std::vec![REG_CONTROL2, RESET_VALUE],
        )];
        let mut i2c = I2cMock::new(&expectations);
        let mut sensor = Qmc5883p::new(i2c.clone());

        let res = sensor.soft_reset().await;

        assert!(res.is_ok(), "Soft reset should return ok");

        i2c.done();
    }

    #[tokio::test]
    async fn test_config_axis_sign() {
        //! Test the config_axis_sign function by verifying that the correct byte is written to the
        //! axis definition register

        let expectations = [I2cTransaction::write(
            I2C_ADDR,
            std::vec![REG_AXIS_DEF, AXIS_DEF_VALUE],
        )];

        let mut i2c = I2cMock::new(&expectations);
        let mut sensor = Qmc5883p::new(i2c.clone());

        let res = sensor.config_axis_sign().await;

        assert!(res.is_ok(), "Soft reset should return ok");

        i2c.done();
    }

    #[tokio::test]
    async fn test_check_id() {
        //! Test the board returns the correct ID by mocking the I2C transaction to return the
        //! expected chip ID.

        let expectations = [I2cTransaction::write_read(
            I2C_ADDR,
            std::vec![REG_CHIP_ID],
            std::vec![QMC5883P_CHIP_ID],
        )];

        let mut i2c = I2cMock::new(&expectations);
        let mut sensor = Qmc5883p::new(i2c.clone());

        let res = sensor.check_id().await;
        assert!(res.is_ok(), "Correct ID should return ok");
        i2c.done();
    }

    #[tokio::test]
    async fn test_wrong_check_id() {
        //! Test that the wrong id is handled correctly by mocking the I2C transaction to return an
        //! incorrect chip ID and verifying that the function returns an error.

        let expectations = [I2cTransaction::write_read(
            I2C_ADDR,
            std::vec![REG_CHIP_ID],
            std::vec![QMC5883P_CHIP_ID + 1],
        )];

        let mut i2c = I2cMock::new(&expectations);
        let mut sensor = Qmc5883p::new(i2c.clone());

        let res = sensor.check_id().await;
        assert!(res.is_err(), "Wrong ID should return error");
        i2c.done();
    }

    #[tokio::test]
    async fn test_apply_configuration() {
        //! Simple apply configuration test to verify the correct bytes are written to the control
        //! registers.
        //!
        //! Control 1: OSR2(Rate8=11) << 6 | OSR1(Ratio4=01) << 4 | ODR(Hz100=10) << 2 | Mode(Cont=11)
        //! Binary: 11 01 10 11 -> 0xDB
        //!
        //! Control 2: Soft(0) | Self(0) | Rng(Gauss8=10) << 2 | SetReset(On=00)
        //! Binary: 0 0 10 00 -> 0x08

        let config = Qmc5883PConfig::default()
            .with_osr2(OverSampleRate::Rate8) // 0b11
            .with_osr1(OverSampleRatio1::Ratio4) // 0b01
            .with_odr(OutputDataRate::Hz100) // 0b10
            .with_mode(Mode::Continuous) // 0b11
            .with_range(Range::Gauss8); // 0b10 (shifted by 2)

        let expected_ctrl1 = 0xDB;
        let expected_ctrl2 = 0x08;

        let expectations = [
            // Expect Write to Control 1 (0x0A)
            I2cTransaction::write(I2C_ADDR, std::vec![REG_CONTROL1, expected_ctrl1]),
            // Expect Write to Control 2 (0x0B)
            I2cTransaction::write(I2C_ADDR, std::vec![REG_CONTROL2, expected_ctrl2]),
        ];

        let mut i2c = I2cMock::new(&expectations);
        let mut sensor = Qmc5883p::new(i2c.clone());

        let res = sensor.apply_configuration(config).await;
        assert!(res.is_ok());

        i2c.done();
    }

    #[tokio::test]
    async fn test_poll_timeout() {
        //! Simulate the sensor NEVER being ready.
        //! We expect POLL_READY_LIMIT (50) reads of the Status register returning 0x00.
        let mut expectations = std::vec::Vec::new();
        for _ in 0..POLL_READY_LIMIT {
            expectations.push(I2cTransaction::write_read(
                I2C_ADDR,
                std::vec![REG_STATUS],
                std::vec![0x00], // Bit 0 is 0 (Not Ready)
            ));
        }

        let mut i2c = I2cMock::new(&expectations);
        let mut sensor = Qmc5883p::new(i2c.clone());

        let res = sensor.poll_ready_and_check_overflow().await;
        assert_eq!(res, Err(QmcError::DataNotReady));

        i2c.done();
    }

    #[tokio::test]
    async fn test_self_test_success() {
        //! Test the sequence of operations in the self-test function by mocking the I2C
        //! transactions to return

        let expectations = [
            // Config Axis Sign
            I2cTransaction::write(I2C_ADDR, std::vec![REG_AXIS_DEF, AXIS_DEF_VALUE]),
            // Set 200Hz Mode (0x1D)
            I2cTransaction::write(I2C_ADDR, std::vec![REG_CONTROL1, 0x1D]),
            // Poll Ready (Status = 0x01)
            I2cTransaction::write_read(I2C_ADDR, std::vec![REG_STATUS], std::vec![0x01]),
            // Read Baseline (Let's say 100, 100, 100)
            I2cTransaction::write_read(
                I2C_ADDR,
                std::vec![REG_DATA_OUT_X_L],
                std::vec![100, 0, 100, 0, 100, 0],
            ),
            // Enable Self Test Current (0x40 to Control 2)
            I2cTransaction::write(I2C_ADDR, std::vec![REG_CONTROL2, 0x40]),
            // Read New Data (Must be > 100 delta. Let's say 300, 300, 300)
            I2cTransaction::write_read(
                I2C_ADDR,
                std::vec![REG_DATA_OUT_X_L],
                std::vec![44, 1, 44, 1, 44, 1],
            ), // 300 = 0x012C
            // Restore Defaults (0x00 to Control 2)
            I2cTransaction::write(I2C_ADDR, std::vec![REG_CONTROL2, 0x00]),
        ];

        let mut i2c = I2cMock::new(&expectations);
        let mut sensor = Qmc5883p::new(i2c.clone());

        let res = sensor.self_test().await;
        assert!(res.is_ok());
        assert!(res.unwrap(), "Self test should pass with sufficient delta");

        i2c.done();
    }

    #[cfg(feature = "magnitude")]
    #[tokio::test]
    async fn test_read_magnitude() {
        //! Test the magnitude calculation by mocking the I2C transactions to return specific X, Y,
        //! Z values and verifying that the magnitude is calculated correctly.

        let expectations = [
            I2cTransaction::write_read(I2C_ADDR, std::vec![REG_STATUS], std::vec![0x01]),
            I2cTransaction::write_read(
                I2C_ADDR,
                std::vec![REG_DATA_OUT_X_L],
                // X=3000, Y=4000, Z=0. (3-4-5 triangle)
                // 3000 = 0x0BB8, 4000 = 0x0FA0
                std::vec![0xB8, 0x0B, 0xA0, 0x0F, 0x00, 0x00],
            ),
        ];

        let mut i2c = I2cMock::new(&expectations);
        let mut sensor = Qmc5883p::new(i2c.clone());

        let res = sensor.read_magnitude().await;
        assert!(res.is_ok());

        // Sqrt(3000^2 + 4000^2) = 5000
        let mag = res.unwrap();
        assert!(mag > 4999.0 && mag < 5001.0, "Magnitude calculation failed");

        i2c.done();
    }

    #[tokio::test]
    async fn test_deinit_enters_suspend() {
        //! Simple deinit test to verify that the correct byte is written to the control register
        //! to enter suspend mode.
        let expectations = [I2cTransaction::write(
            I2C_ADDR,
            std::vec![REG_CONTROL1, 0x00],
        )];

        let mut i2c = I2cMock::new(&expectations);
        let mut sensor = Qmc5883p::new(i2c.clone());

        let res = sensor.deinit().await;

        assert!(res.is_ok(), "Deinit should succeed");
        i2c.done();
    }
}
