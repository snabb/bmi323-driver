# bmi323-driver Embassy STM32G030F6 examples

This crate contains the [STM32G030F6](https://www.st.com/en/microcontrollers-microprocessors/stm32g030f6.html)
hardware-specific [Embassy](https://embassy.dev/) examples for the main `bmi323-driver` library.

It keeps the STM32-specific dependencies, target configuration, and `probe-rs`
runner setup out of the main driver crate.

The examples assume:

- target: `thumbv6m-none-eabi`
- chip: `STM32G030F6` in `probe-rs`

These examples were tested with the following hardware:
- [7Semi STM32G030F6P6 Anchor STM32 mini Development Board](https://7semi.com/7semi-stm32g030f6p6-anchor-stm32-mini-development-board/)
- [7Semi BMI323 IMU Breakout Board – 6DOF Accelerometer + Gyroscope with Qwiic Support (SPI/I2C)](https://7semi.com/7semi-bmi323-imu-sensor-breakout-board-6dof-qwiic-spi-i2c/)

Wiring used by the examples:

- `PA11` -> BMI323 `I2C2_SCL`
- `PA12` -> BMI323 `I2C2_SDA`
- `PB3` -> BMI323 `INT1`

It should be relatively easy to modify the examples to work with other Embassy supported MCUs or different pin configurations.

Available binaries:

- `motion`: arms BMI323 any-motion detection and logs when motion is detected
- `no_motion`: arms BMI323 no-motion detection and logs when a stationary event is detected
- `stream`: continuously logs accelerometer and gyroscope output
- `self_test`: runs the built-in accelerometer and gyroscope self-tests and logs the detailed result
- `tap`: arms BMI323 tap detection and logs when a tap is detected
- `orientation`: arms BMI323 orientation detection and logs when orientation changes
- `flat`: arms BMI323 flat detection and logs when the sensor becomes flat
- `significant_motion`: arms BMI323 significant-motion detection and logs when a significant-motion event is detected
- `tilt`: arms BMI323 tilt detection and logs when a tilt event is detected
- `steps`: arms BMI323 step-detector and step-counter interrupts and logs total steps
- `alt_config`: switches the accelerometer between low-power user config and high-performance alternate config based on any-motion and no-motion

The examples use the timer queue integrated into the current
`embassy-executor`. No `embassy-time` generic queue feature is enabled.

## probe-rs

From this directory:

```bash
rustup target add thumbv6m-none-eabi
cargo install probe-rs-tools --locked
```

Run examples:

```bash
cargo run-motion
cargo run-no-motion
cargo run-stream
cargo run-self-test
cargo run-tap
cargo run-orientation
cargo run-flat
cargo run-significant-motion
cargo run-tilt
cargo run-steps
cargo run-alt-config
```

Build examples without flashing:

```bash
cargo build-motion
cargo build-no-motion
cargo build-stream
cargo build-self-test
cargo build-tap
cargo build-orientation
cargo build-flat
cargo build-significant-motion
cargo build-tilt
cargo build-steps
cargo build-alt-config
```

If you only want to flash and not attach a runner session:

```bash
cargo build-motion
probe-rs download --chip STM32G030F6 target/thumbv6m-none-eabi/debug/motion
```

Recommended checks before flashing:

1. Connect SWD:
   - `SWDIO`
   - `SWCLK`
   - `GND`
   - target power
2. Confirm probe-rs sees the probe:
   `probe-rs list`

You need to update at least [`Cargo.toml`] and [`.cargo/config.toml`](./.cargo/config.toml) to use some different STM32 MCU.
