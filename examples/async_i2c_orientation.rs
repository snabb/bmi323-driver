use bmi323_driver::{
    AccelConfig, ActiveLevel, Bmi323Async, FeatureBlockingMode, I2C_ADDRESS_PRIMARY,
    InterruptChannel, InterruptPinConfig, InterruptRoute, InterruptSource, OrientationConfig,
    OrientationMode, OutputDataRate, OutputMode,
};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;

#[allow(dead_code)]
async fn embassy_style_orientation_task<I2C, INT, D>(
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

    imu.set_accel_config(AccelConfig {
        odr: OutputDataRate::Hz100,
        ..Default::default()
    })
    .await?;

    imu.configure_orientation(OrientationConfig {
        upside_down_enabled: true,
        mode: OrientationMode::Symmetrical,
        blocking: FeatureBlockingMode::AccelOver1p5gOrFullSlope,
        theta: 38,    // nonlinear orientation-angle field, about 38 deg
        hold_time: 5, // 5 / 50 s = 100 ms in the new orientation before event
        slope_threshold: OrientationConfig::slope_threshold_from_g(0.40),
        hysteresis: OrientationConfig::hysteresis_from_g(0.06),
        report_mode: bmi323_driver::EventReportMode::AllEvents,
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

    imu.map_interrupt(InterruptSource::Orientation, InterruptRoute::Int1)
        .await?;

    loop {
        let status = imu.wait_for_interrupt(int1, InterruptChannel::Int1).await?;
        if status.orientation() {
            let accel = imu.read_accel().await?;
            let _ = accel;
        }
    }
}

fn main() {}
