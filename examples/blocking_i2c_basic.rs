use bmi323_driver::{
    AccelConfig, ActiveLevel, AnyMotionConfig, Bmi323, EventReportMode, GyroConfig,
    I2C_ADDRESS_PRIMARY, InterruptChannel, InterruptPinConfig, InterruptRoute, InterruptSource,
    MotionAxes, NoMotionConfig, OutputDataRate, OutputMode, ReferenceUpdate,
};
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

#[allow(dead_code)]
fn blocking_example<I2C, D>(i2c: I2C, delay: &mut D) -> Result<(), bmi323_driver::Error<I2C::Error>>
where
    I2C: I2c,
    D: DelayNs,
{
    let mut imu = Bmi323::new_i2c(i2c, I2C_ADDRESS_PRIMARY);

    imu.init(delay)?;

    imu.enable_feature_engine()?;

    imu.configure_any_motion(AnyMotionConfig {
        axes: MotionAxes::XYZ,
        threshold: AnyMotionConfig::threshold_from_g(0.20),
        hysteresis: AnyMotionConfig::hysteresis_from_g(0.02),
        duration: 5,  // 5 / 50 s = 100 ms above threshold before event
        wait_time: 1, // 1 / 50 s = 20 ms clear delay after slope drops
        reference_update: ReferenceUpdate::EverySample,
        report_mode: EventReportMode::AllEvents,
        interrupt_hold: 3, // 0.625 ms * 2^3 = 5 ms interrupt hold
    })?;

    imu.configure_no_motion(NoMotionConfig {
        axes: MotionAxes::XYZ,
        threshold: NoMotionConfig::threshold_from_g(0.02),
        hysteresis: NoMotionConfig::hysteresis_from_g(0.01),
        duration: 50, // 50 / 50 s = 1.0 s below threshold before event
        wait_time: 1, // 1 / 50 s = 20 ms clear delay after slope stays low
        reference_update: ReferenceUpdate::EverySample,
        report_mode: EventReportMode::AllEvents,
        interrupt_hold: 3, // 0.625 ms * 2^3 = 5 ms interrupt hold
    })?;

    imu.set_accel_config(AccelConfig {
        odr: OutputDataRate::Hz100,
        ..Default::default()
    })?;

    imu.set_gyro_config(GyroConfig {
        odr: OutputDataRate::Hz100,
        ..Default::default()
    })?;

    imu.set_interrupt_latching(true)?;
    imu.configure_interrupt_pin(
        InterruptChannel::Int1,
        InterruptPinConfig {
            active_level: ActiveLevel::High,
            output_mode: OutputMode::PushPull,
            enabled: true,
        },
    )?;

    imu.map_interrupt(InterruptSource::AnyMotion, InterruptRoute::Int1)?;
    imu.map_interrupt(InterruptSource::NoMotion, InterruptRoute::Int1)?;
    imu.map_interrupt(InterruptSource::AccelDataReady, InterruptRoute::Int1)?;

    let sample = imu.read_imu_data().unwrap();
    let accel_g = sample.accel.as_g(imu.accel_range());
    let gyro_dps = sample.gyro.as_dps(imu.gyro_range());

    let _ = (accel_g, gyro_dps);
    let _ = delay;

    Ok(())
}

fn main() {}
