//! Generic `no_std` driver for the Bosch BMI323 IMU.
//!
//! This crate provides:
//!
//! - blocking and async drivers
//! - I2C and SPI transport support
//! - accelerometer and gyroscope configuration
//! - burst sample reads
//! - FIFO configuration and reads
//! - interrupt pin electrical configuration and interrupt routing
//! - feature-engine enable flow
//! - any-motion and no-motion configuration
//! - tap and orientation/flat configuration
//! - significant-motion and tilt configuration
//! - step detector and step counter support
//! - alternate accel/gyro configuration switching
//! - built-in accelerometer and gyroscope self-test
//!
//! The API is built on top of `embedded-hal` 1.0 and `embedded-hal-async` 1.0.
//! It does not own the external interrupt GPIO. This keeps the driver generic and
//! makes it easy to use with Embassy or with a platform-specific interrupt layer.
//!
//! # Driver variants
//!
//! - [`Bmi323`] is the blocking driver.
//! - [`Bmi323Async`] is the async driver.
//!
//! Both expose the same high-level BMI323 operations where practical.
//!
//! # Transport model
//!
//! The BMI323 uses 8-bit register addresses with 16-bit register payloads.
//! Reads include interface-specific dummy bytes, which this crate handles
//! internally for I2C and SPI.
//!
//! # Interrupt model
//!
//! BMI323 interrupt sources are routed to `INT1`, `INT2`, or I3C IBI inside the
//! sensor. The driver configures the sensor-side routing, but the external GPIO
//! line is managed by the application:
//!
//! - in blocking applications, poll the GPIO or an MCU interrupt flag yourself,
//!   then call [`Bmi323::read_interrupt_status`]
//! - in async applications, either wait on the GPIO yourself or use
//!   [`Bmi323Async::wait_for_interrupt`] with a pin implementing
//!   [`embedded_hal_async::digital::Wait`]
//!
//! # Feature engine note
//!
//! Advanced features such as any-motion and no-motion depend on the BMI323
//! feature engine. The datasheet requires the feature engine to be enabled
//! before sensors are re-enabled for these features. The helper methods in this
//! crate follow that model, but application code should still keep the order in
//! mind when building its configuration sequence.
//!
//! For motion-feature timing and threshold fields, prefer the conversion
//! helpers on [`AnyMotionConfig`] and [`NoMotionConfig`] instead of hand-coding
//! raw register values.
//!
//! The `report_mode` and `interrupt_hold` fields are written to a single shared
//! BMI323 register (`EXT_GEN_SET_1`). When multiple feature-engine blocks are
//! configured, the last `configure_*` call's values win for both fields. Use
//! the same values across all `configure_*` calls, or set them in the intended
//! final order.
//!
//! # Startup configuration
//!
//! [`Bmi323::init`] and [`Bmi323Async::init`] perform a soft reset so the
//! sensor starts from a known state. After that reset, this driver does not
//! assume the accelerometer or gyroscope are configured for your application.
//! In practice, you should call [`Bmi323::set_accel_config`] and
//! [`Bmi323::set_gyro_config`] or their async equivalents before relying on
//! accelerometer or gyroscope sample reads.
//!
//! The driver tracks local range fields initialized to `AccelRange::G8` and
//! `GyroRange::Dps2000`, but those are only fallback bookkeeping values until
//! you explicitly configure the sensor through the driver.
//!
//! # Example: blocking I2C
//!
//! ```no_run
//! use bmi323_driver::{
//!     AccelConfig, AccelMode, AccelRange, AverageSamples, Bandwidth, Bmi323,
//!     GyroConfig, GyroMode, GyroRange, I2C_ADDRESS_PRIMARY, OutputDataRate,
//! };
//! use embedded_hal::delay::DelayNs;
//! use embedded_hal::i2c::I2c;
//!
//! fn example<I2C, D>(i2c: I2C, delay: &mut D) -> Result<(), bmi323_driver::Error<I2C::Error>>
//! where
//!     I2C: I2c,
//!     D: DelayNs,
//! {
//!     let mut imu = Bmi323::new_i2c(i2c, I2C_ADDRESS_PRIMARY);
//!     let state = imu.init(delay)?;
//!     let _ = state;
//!
//!     imu.set_accel_config(AccelConfig {
//!         odr: OutputDataRate::Hz100,
//!         ..Default::default()
//!     })?;
//!
//!     imu.set_gyro_config(GyroConfig {
//!         odr: OutputDataRate::Hz100,
//!         ..Default::default()
//!     })?;
//!
//!     let sample = imu.read_imu_data()?;
//!     let accel_g = sample.accel.as_g(imu.accel_range());
//!     let gyro_dps = sample.gyro.as_dps(imu.gyro_range());
//!     let _ = (accel_g, gyro_dps);
//!     Ok(())
//! }
//! ```
//!
//! # Example: async interrupt-driven usage
//!
//! ```no_run
//! use bmi323_driver::{
//!     AccelConfig, AccelMode, AccelRange, ActiveLevel, AnyMotionConfig,
//!     AverageSamples, Bandwidth, Bmi323Async, EventReportMode,
//!     I2C_ADDRESS_PRIMARY, InterruptChannel, InterruptPinConfig,
//!     InterruptRoute, InterruptSource, MotionAxes, OutputDataRate, OutputMode,
//!     ReferenceUpdate,
//! };
//! use embedded_hal_async::delay::DelayNs;
//! use embedded_hal_async::digital::Wait;
//! use embedded_hal_async::i2c::I2c;
//!
//! async fn example<I2C, D, P>(
//!     i2c: I2C,
//!     delay: &mut D,
//!     int1_pin: &mut P,
//! ) -> Result<(), bmi323_driver::Error<I2C::Error>>
//! where
//!     I2C: I2c,
//!     D: DelayNs,
//!     P: Wait,
//! {
//!     let mut imu = Bmi323Async::new_i2c(i2c, I2C_ADDRESS_PRIMARY);
//!     imu.init(delay).await?;
//!     imu.enable_feature_engine().await?;
//!     imu.set_accel_config(AccelConfig {
//!         mode: AccelMode::HighPerformance,
//!         odr: OutputDataRate::Hz100,
//!         ..Default::default()
//!     }).await?;
//!     imu.configure_any_motion(AnyMotionConfig {
//!         axes: MotionAxes::XYZ,
//!         threshold: AnyMotionConfig::threshold_from_g(0.08),
//!         hysteresis: AnyMotionConfig::hysteresis_from_g(0.02),
//!         duration: 5, // 5 / 50 s = 100 ms above threshold before event
//!         wait_time: 1, // 1 / 50 s = 20 ms clear delay after slope drops
//!         reference_update: ReferenceUpdate::EverySample,
//!         report_mode: EventReportMode::AllEvents,
//!         interrupt_hold: 3, // 0.625 ms * 2^3 = 5 ms interrupt hold
//!     }).await?;
//!     imu.set_interrupt_latching(true).await?;
//!     imu.configure_interrupt_pin(
//!         InterruptChannel::Int1,
//!         InterruptPinConfig {
//!             active_level: ActiveLevel::High,
//!             output_mode: OutputMode::PushPull,
//!             enabled: true,
//!         },
//!     ).await?;
//!     imu.map_interrupt(InterruptSource::AnyMotion, InterruptRoute::Int1).await?;
//!
//!     let status = imu.wait_for_interrupt(int1_pin, InterruptChannel::Int1).await?;
//!     if status.any_motion() {
//!         let accel = imu.read_accel().await?;
//!         let _ = accel;
//!     }
//!     Ok(())
//! }
//! ```
#![no_std]

#[cfg(test)]
extern crate std;

mod async_driver;
mod blocking_driver;
mod driver;
mod registers;
mod transport;
mod types;

pub use driver::{Bmi323, Bmi323Async};
pub use transport::{
    AsyncAccess, AsyncI2cTransport, AsyncSpiTransport, SyncAccess, SyncI2cTransport,
    SyncSpiTransport,
};
pub use types::*;

#[cfg(test)]
mod tests;
