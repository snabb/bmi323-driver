use bmi323_driver::{
    AccelConfig, ActiveLevel, Bmi323Async, EventReportMode, I2C_ADDRESS_PRIMARY, InterruptChannel,
    InterruptPinConfig, InterruptRoute, InterruptSource, OutputDataRate, OutputMode, TiltConfig,
};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;

#[allow(dead_code)]
async fn embassy_style_tilt_task<I2C, INT, D>(
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

    imu.configure_tilt(TiltConfig {
        segment_size: 100,     // 100 / 50 s = 2.0 s reference-vector averaging duration
        min_tilt_angle: 210,   // raw nonlinear min-tilt-angle field
        beta_acc_mean: 0xF069, // raw beta_acc_mean field, close to datasheet default
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

    imu.map_interrupt(InterruptSource::Tilt, InterruptRoute::Int1)
        .await?;

    loop {
        let status = imu.wait_for_interrupt(int1, InterruptChannel::Int1).await?;

        if status.tilt() {
            let accel = imu.read_accel().await?;
            let _ = accel;
        }
    }
}

fn main() {}
