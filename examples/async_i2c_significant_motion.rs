use bmi323_driver::{
    AccelConfig, ActiveLevel, Bmi323Async, EventReportMode, I2C_ADDRESS_PRIMARY, InterruptChannel,
    InterruptPinConfig, InterruptRoute, InterruptSource, OutputDataRate, OutputMode,
    SignificantMotionConfig,
};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;

#[allow(dead_code)]
async fn embassy_style_significant_motion_task<I2C, INT, D>(
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

    imu.configure_significant_motion(SignificantMotionConfig {
        block_size: 250, // 250 / 50 s = 5.0 s segment size
        peak_to_peak_min: SignificantMotionConfig::peak_to_peak_from_g(0.08),
        mean_crossing_rate_min: 17, // 17 crossings/s lower bound
        peak_to_peak_max: SignificantMotionConfig::peak_to_peak_from_g(1.0),
        mean_crossing_rate_max: 17, // 17 crossings/s upper bound
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

    imu.map_interrupt(InterruptSource::SignificantMotion, InterruptRoute::Int1)
        .await?;

    loop {
        let status = imu.wait_for_interrupt(int1, InterruptChannel::Int1).await?;

        if status.significant_motion() {
            let accel = imu.read_accel().await?;
            let _ = accel;
        }
    }
}

fn main() {}
