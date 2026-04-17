use bmi323_driver::{
    AccelConfig, ActiveLevel, Bmi323Async, I2C_ADDRESS_PRIMARY, InterruptChannel,
    InterruptPinConfig, InterruptRoute, InterruptSource, OutputDataRate, OutputMode,
    StepCounterConfig,
};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;

#[allow(dead_code)]
async fn embassy_style_step_task<I2C, INT, D>(
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

    imu.set_step_detector_enabled(true).await?;
    imu.set_step_counter_enabled(true).await?;
    imu.configure_step_counter(StepCounterConfig {
        watermark_level: 10, // raise StepCounter interrupt every 10 newly counted steps
        reset_counter: true, // reset the accumulated count at startup
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

    imu.map_interrupt(InterruptSource::StepDetector, InterruptRoute::Int1)
        .await?;
    imu.map_interrupt(InterruptSource::StepCounter, InterruptRoute::Int1)
        .await?;

    loop {
        let status = imu.wait_for_interrupt(int1, InterruptChannel::Int1).await?;

        if status.step_detector() {
            let steps = imu.read_step_count().await?;
            let _ = steps;
        }

        if status.step_counter() {
            let steps = imu.read_step_count().await?;
            let _ = steps;
        }
    }
}

fn main() {}
