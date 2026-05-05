//! Generic `no_std` driver for the Bosch Sensortec BMI323 IMU.
//!
//! This crate provides:
//!
//! - blocking and async drivers (selected by Cargo feature)
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
//! # Blocking vs async
//!
//! Select exactly one Cargo feature:
//!
//! - `blocking` (default) — all [`Bmi323`] methods are regular synchronous `fn`.
//!   Uses `embedded-hal` 1.0 I2C/SPI/delay traits.
//! - `async` — all [`Bmi323`] methods are `async fn`.
//!   Uses `embedded-hal-async` 1.0 traits. Required for Embassy.
//!
//! Enabling both features simultaneously is a compile error.
//!
//! # Driver
//!
//! [`Bmi323`] exposes the same high-level BMI323 operations in both modes.
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
//!   [`Bmi323::wait_for_interrupt`] with a pin implementing
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
//! [`Bmi323::init`] performs a soft reset so the sensor starts from a known
//! state. After that reset, this driver does not assume the accelerometer or
//! gyroscope are configured for your application. In practice, you should call
//! [`Bmi323::set_accel_config`] and [`Bmi323::set_gyro_config`] before relying
//! on accelerometer or gyroscope sample reads.
//!
//! The driver tracks local range fields initialized to `AccelRange::G2` and
//! `GyroRange::Dps125` to match the BMI323 power-on reset defaults
//! (`ACC_CONF`/`GYR_CONF` = `0x0000`). These bookkeeping values only produce
//! correct physical conversions after you explicitly configure the sensor.
//!
//! # Example: blocking I2C
//!
//! Requires `features = ["blocking"]` (the default).
//!
//! ```no_run
//! # #[cfg(feature = "async")] fn main() {}
//! # #[cfg(not(feature = "async"))]
//! # fn main() {
//! use bmi323_driver::{AccelConfig, Bmi323, GyroConfig, I2C_ADDRESS_PRIMARY, OutputDataRate};
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
//! # }
//! ```
//!
//! # Example: async interrupt-driven usage
//!
//! Requires `features = ["async"]`.
//!
//! ```no_run
//! # #[cfg(not(feature = "async"))] fn main() {}
//! # #[cfg(feature = "async")]
//! # fn main() {
//! use bmi323_driver::{
//!     AccelConfig, AccelMode, ActiveLevel, AnyMotionConfig, Bmi323, EventReportMode,
//!     I2C_ADDRESS_PRIMARY, InterruptChannel, InterruptPinConfig, InterruptRoute,
//!     InterruptSource, MotionAxes, OutputDataRate, OutputMode, ReferenceUpdate,
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
//!     let mut imu = Bmi323::new_i2c(i2c, I2C_ADDRESS_PRIMARY);
//!     imu.init(delay).await?;
//!     imu.enable_feature_engine(delay).await?;
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
//! # }
//! ```
#![no_std]

#[cfg(all(feature = "blocking", feature = "async"))]
compile_error!("features \"blocking\" and \"async\" are mutually exclusive; \
    enable exactly one. If you added --features blocking, also pass \
    --no-default-features to suppress the default \"async\" feature.");

#[cfg(not(any(feature = "blocking", feature = "async")))]
compile_error!("one of features \"blocking\" or \"async\" must be enabled");

#[cfg(test)]
extern crate std;

mod driver;
mod driver_impl;
mod registers;
mod transport;
mod types;

pub use driver::Bmi323;
pub use transport::{Access, I2cTransport, SpiTransport, MAX_WORDS_PER_READ};
pub use types::*;

#[cfg(test)]
mod tests;
