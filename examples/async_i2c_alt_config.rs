use bmi323_driver::{
    ActiveLevel, AltAccelSwitchProfile, AltConfigSwitchSource, Bmi323Async, EventReportMode,
    I2C_ADDRESS_PRIMARY, InterruptChannel, InterruptPinConfig, InterruptRoute, InterruptSource,
    MotionAxes, NoMotionConfig, OutputDataRate, OutputMode, ReferenceUpdate,
};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;

#[allow(dead_code)]
async fn embassy_style_alt_config_task<I2C, INT, D>(
    i2c: I2C,
    int1: &mut INT,
    delay: &mut D,
) -> Result<(), bmi323_driver::Error<I2C::Error>>
where
    I2C: I2c,
    INT: Wait,
    D: DelayNs,
{
    let mut imu = Bmi323Async::new_i2c(i2c, I2C_ADDRESS_PRIMARY);

    imu.init(delay).await?;
    imu.enable_feature_engine().await?;

    let profile = AltAccelSwitchProfile::low_power_to_high_performance(
        OutputDataRate::Hz25,
        OutputDataRate::Hz200,
        AltConfigSwitchSource::AnyMotion,
        AltConfigSwitchSource::NoMotion,
    );

    imu.set_accel_config(profile.user_accel).await?;
    imu.set_alt_accel_config(profile.alternate_accel).await?;

    imu.configure_any_motion(bmi323_driver::AnyMotionConfig {
        axes: MotionAxes::XYZ,
        threshold: bmi323_driver::AnyMotionConfig::threshold_from_g(0.08),
        hysteresis: bmi323_driver::AnyMotionConfig::hysteresis_from_g(0.02),
        duration: 5,  // 5 / 50 s = 100 ms above threshold before event
        wait_time: 1, // 1 / 50 s = 20 ms clear delay after slope drops
        reference_update: ReferenceUpdate::EverySample,
        report_mode: EventReportMode::AllEvents,
        interrupt_hold: 3, // 0.625 ms * 2^3 = 5 ms interrupt hold
    })
    .await?;

    imu.configure_no_motion(NoMotionConfig {
        axes: MotionAxes::XYZ,
        threshold: NoMotionConfig::threshold_from_g(0.03),
        hysteresis: NoMotionConfig::hysteresis_from_g(0.01),
        duration: 100, // 100 / 50 s = 2.0 s below threshold before event
        wait_time: 1,  // 1 / 50 s = 20 ms clear delay after slope stays low
        reference_update: ReferenceUpdate::EverySample,
        report_mode: EventReportMode::AllEvents,
        interrupt_hold: 3, // 0.625 ms * 2^3 = 5 ms interrupt hold
    })
    .await?;

    imu.configure_alt_config_control(profile.control).await?;

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
    imu.map_interrupt(InterruptSource::NoMotion, InterruptRoute::Int1)
        .await?;

    loop {
        let status = imu.wait_for_interrupt(int1, InterruptChannel::Int1).await?;
        let alt_status = imu.alt_status().await?;
        let accel = imu.read_accel().await?;
        let _ = (status, alt_status, accel);
    }
}

fn main() {}
