# bmi323-driver

[![crates.io](https://img.shields.io/crates/v/bmi323-driver.svg)](https://crates.io/crates/bmi323-driver)
[![docs.rs](https://img.shields.io/docsrs/bmi323-driver)](https://docs.rs/bmi323-driver)
[![CI](https://github.com/snabb/bmi323-driver/actions/workflows/ci.yml/badge.svg)](https://github.com/snabb/bmi323-driver/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/snabb/bmi323-driver/graph/badge.svg)](https://codecov.io/gh/snabb/bmi323-driver)
[![license](https://img.shields.io/crates/l/bmi323-driver.svg)](https://github.com/snabb/bmi323-driver/blob/main/LICENSE)
[![rust-version](https://img.shields.io/badge/rust-1.85%2B-93450a.svg)](https://www.rust-lang.org/)

`bmi323-driver` is a `no_std` Rust driver for the Bosch Sensortec BMI323 6-DoF IMU.

The datasheet is available at Bosch Sensortec web site: https://www.bosch-sensortec.com/en/products/motion-sensors/imus/bmi323

This driver is designed to be transport-agnostic, small, and usable both
in generic embedded-hal applications and in async Embassy-based firmware.

`init()` performs a soft reset, so the driver starts from a known sensor state.
After `init()`, configure the accelerometer and gyroscope explicitly before
depending on sample reads. The driver does not promise application-ready accel
or gyro settings immediately after initialization.

The API is built on top of `embedded-hal` 1.0 and `embedded-hal-async` 1.0.
The driver does not own the external interrupt GPIO, which keeps it transport
agnostic and easy to integrate with Embassy or platform-specific interrupt
handling.

## Features

- `embedded-hal` 1.0 blocking API support
- `embedded-hal-async` 1.0 async API support
- I2C and SPI transports
- accelerometer and gyroscope configuration
- burst accel/gyro reads
- FIFO configuration and reads
- interrupt pin electrical configuration and interrupt routing
- feature-engine enable sequence
- any-motion and no-motion configuration
- tap and orientation/flat configuration
- significant-motion and tilt configuration
- step detector and step counter support
- alternate accel/gyro configuration switching
- built-in accelerometer and gyroscope self-test

## Feature flags

Async mode is the default when no feature flags are set. The `blocking` feature
opts into the synchronous `embedded-hal` API. Enabling both simultaneously
causes a compile error.

- `async`: no-op marker; async is the default behavior without `blocking`
- `blocking`: synchronous driver using `embedded-hal` traits
- `defmt`: derives `defmt::Format` for public value types

## Transport model

The BMI323 uses 8-bit register addresses with 16-bit register payloads. Reads
include interface-specific dummy bytes, which this crate handles internally for
both I2C and SPI.

## Interrupt model

BMI323 interrupt sources are routed to `INT1`, `INT2`, or I3C IBI inside the
sensor. The driver configures the sensor-side routing, but the external GPIO
line is managed by the application:

- in blocking applications, poll the GPIO or an MCU interrupt flag yourself,
  then call `read_interrupt_status()`
- in async applications, either wait on the GPIO yourself or use
  `wait_for_interrupt()` with a pin implementing
  `embedded_hal_async::digital::Wait`

## Feature engine notes

Advanced features such as any-motion and no-motion depend on the BMI323 feature
engine. The datasheet requires the feature engine to be enabled before sensors
are re-enabled for these features. The helper methods in this crate follow that
model, but application code should still keep the order in mind when building
its configuration sequence.

For motion-feature timing and threshold fields, prefer the conversion helpers
on `AnyMotionConfig` and `NoMotionConfig` instead of hand-coding raw register
values.

The `report_mode` and `interrupt_hold` fields are written to a single shared
BMI323 register (`EXT_GEN_SET_1`). When multiple feature-engine blocks are
configured, the last `configure_*` call's values win for both fields. Use the
same values across all `configure_*` calls, or set them in the intended final
order.

The driver tracks local range fields initialized to `AccelRange::G2` and
`GyroRange::Dps125` to match the BMI323 power-on reset defaults
(`ACC_CONF`/`GYR_CONF = 0x0000`).

### Blocking I2C example

```rust,no_run
# #[cfg(not(feature = "blocking"))] fn main() {}
# #[cfg(feature = "blocking")]
# fn main() {
use bmi323_driver::{
    AccelConfig, AccelRange, Bmi323, GyroConfig, GyroRange, I2C_ADDRESS_PRIMARY,
    OutputDataRate,
};
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

fn example<I2C, D>(i2c: I2C, delay: &mut D) -> Result<(), bmi323_driver::Error<I2C::Error>>
where
    I2C: I2c,
    D: DelayNs,
{
    let mut imu = Bmi323::new_i2c(i2c, I2C_ADDRESS_PRIMARY);
    let state = imu.init(delay)?;
    let _ = state;

    imu.set_accel_config(AccelConfig {
        odr: OutputDataRate::Hz100,
        ..Default::default()
    })?;

    imu.set_gyro_config(GyroConfig {
        odr: OutputDataRate::Hz100,
        ..Default::default()
    })?;

    let sample = imu.read_imu_data()?;
    let accel_g = sample.accel.as_g(imu.accel_range());
    let gyro_dps = sample.gyro.as_dps(imu.gyro_range());
    let _ = (accel_g, gyro_dps);
    Ok(())
}
# }
```

### Async interrupt-driven advanced example

```rust,no_run
# #[cfg(feature = "blocking")] fn main() {}
# #[cfg(not(feature = "blocking"))]
# fn main() {
use bmi323_driver::{
    AccelConfig, ActiveLevel, AnyMotionConfig, Bmi323, EventReportMode,
    I2C_ADDRESS_PRIMARY, InterruptChannel, InterruptPinConfig, InterruptRoute,
    InterruptSource, MotionAxes, OutputDataRate, OutputMode, ReferenceUpdate,
};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;

async fn example<I2C, D, P>(
    i2c: I2C,
    delay: &mut D,
    int1_pin: &mut P,
) -> Result<(), bmi323_driver::Error<I2C::Error>>
where
    I2C: I2c,
    D: DelayNs,
    P: Wait,
{
    let mut imu = Bmi323::new_i2c(i2c, I2C_ADDRESS_PRIMARY);
    imu.init(delay).await?;
    imu.enable_feature_engine(delay).await?;
    imu.set_accel_config(AccelConfig {
        mode: bmi323_driver::AccelMode::HighPerformance,
        odr: OutputDataRate::Hz100,
        ..Default::default()
    })
    .await?;
    imu.configure_any_motion(AnyMotionConfig {
        axes: MotionAxes::XYZ,
        threshold: AnyMotionConfig::threshold_from_g(0.08),
        hysteresis: AnyMotionConfig::hysteresis_from_g(0.02),
        duration: 5, // 5 / 50 s = 100 ms above threshold before event
        wait_time: 1, // 1 / 50 s = 20 ms clear delay after slope drops
        reference_update: ReferenceUpdate::EverySample,
        report_mode: EventReportMode::AllEvents,
        interrupt_hold: 3, // 0.625 ms * 2^3 = 5 ms interrupt hold
    })
    .await?;
    imu.set_interrupt_latching(true).await?;
    imu.configure_interrupt_pin(
        InterruptChannel::Int1,
        InterruptPinConfig {
            active_level: ActiveLevel::High,
            output_mode: OutputMode::PushPull,
            enabled: true,
        },
    )
    .await?;
    imu.map_interrupt(InterruptSource::AnyMotion, InterruptRoute::Int1)
        .await?;

    let status = imu
        .wait_for_interrupt(int1_pin, InterruptChannel::Int1)
        .await?;
    if status.any_motion() {
        let accel = imu.read_accel().await?;
        let _ = accel;
    }
    Ok(())
}
# }
```

## Repository examples

- `examples/blocking_i2c_basic.rs`
  Blocking I2C configuration and sample reads.
- `examples/async_i2c_interrupt.rs`
  Generic async interrupt-driven setup using `embedded-hal-async`.
- `examples/async_i2c_no_motion.rs`
  Generic async no-motion detection setup using `embedded-hal-async`.
- `examples/async_i2c_tap.rs`
  Generic async tap-detection setup using `embedded-hal-async`.
- `examples/async_i2c_orientation.rs`
  Generic async orientation-detection setup using `embedded-hal-async`.
- `examples/async_i2c_flat.rs`
  Generic async flat-detection setup using `embedded-hal-async`.
- `examples/async_i2c_significant_motion.rs`
  Generic async significant-motion detection setup using `embedded-hal-async`.
- `examples/async_i2c_tilt.rs`
  Generic async tilt-detection setup using `embedded-hal-async`.
- `examples/async_i2c_step_counter.rs`
  Generic async step-detector and step-counter setup using `embedded-hal-async`.
- `examples/async_i2c_alt_config.rs`
  Generic async alternate accel-configuration switching using any-motion and no-motion.
- `examples/async_i2c_self_test.rs`
  Generic async built-in accelerometer and gyroscope self-test.

Hardware-specific [STM32G030F6](https://www.st.com/en/microcontrollers-microprocessors/stm32g030f6.html) [Embassy](https://embassy.dev/) examples are in the separate
sub-crate [embassy-stm32g030f6-examples](./embassy-stm32g030f6-examples).

### STM32 Motion Detection Example

This screenshot shows the `embassy-stm32g030f6-examples/src/bin/motion.rs`
example running on an STM32G030F6 and printing detected accelerometer samples
after BMI323 motion interrupts fire.

![STM32 BMI323 motion detection example run](./assets/motion-detection-example-stm32g030f6.png)

## License

MIT. See [LICENSE](./LICENSE).
