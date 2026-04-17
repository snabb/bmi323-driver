use bmi323_driver::{
    AccelConfig, ActiveLevel, Bmi323Async, FeatureBlockingMode, FlatConfig, I2C_ADDRESS_PRIMARY,
    InterruptChannel, InterruptPinConfig, InterruptRoute, InterruptSource, OutputDataRate,
    OutputMode,
};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;

#[allow(dead_code)]
async fn embassy_style_flat_task<I2C, INT, D>(
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

    imu.configure_flat(FlatConfig {
        theta: 20, // nonlinear flat-angle field, about 29 deg
        blocking: FeatureBlockingMode::AccelOver1p5gOrHalfSlope,
        hold_time: 32, // 32 / 50 s = 640 ms in flat position before event
        slope_threshold: FlatConfig::slope_threshold_from_g(0.40),
        hysteresis: 9, // nonlinear angular hysteresis field, close to datasheet default
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

    imu.map_interrupt(InterruptSource::Flat, InterruptRoute::Int1)
        .await?;

    loop {
        let status = imu.wait_for_interrupt(int1, InterruptChannel::Int1).await?;
        if status.flat() {
            let accel = imu.read_accel().await?;
            let _ = accel;
        }
    }
}

fn main() {}
