use bmi323_driver::{
    AccelConfig, ActiveLevel, Bmi323Async, I2C_ADDRESS_PRIMARY, InterruptChannel,
    InterruptPinConfig, InterruptRoute, InterruptSource, OutputDataRate, OutputMode, TapAxis,
    TapConfig, TapDetectionMode, TapReportingMode,
};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;

#[allow(dead_code)]
async fn embassy_style_tap_task<I2C, INT, D>(
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

    imu.configure_tap(TapConfig {
        axis: TapAxis::Z,
        reporting_mode: TapReportingMode::Confirmed,
        max_peaks_for_tap: 6,
        mode: TapDetectionMode::Normal,
        single_tap_enabled: true,
        double_tap_enabled: false,
        triple_tap_enabled: false,
        tap_peak_threshold: TapConfig::tap_peak_threshold_from_g(0.09),
        max_gesture_duration: 16, // 16 / 25 s = 640 ms confirmation window
        max_duration_between_peaks: 4, // 4 / 200 s = 20 ms peak-to-peak limit
        tap_shock_settling_duration: 6, // 6 / 200 s = 30 ms shock settling
        min_quiet_duration_between_taps: 8, // 8 / 200 s = 40 ms quiet time between taps
        quiet_time_after_gesture: 6, // 6 / 25 s = 240 ms quiet time after gesture
        report_mode: bmi323_driver::EventReportMode::AllEvents,
        interrupt_hold: 4, // 0.625 ms * 2^4 = 10 ms interrupt hold
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

    imu.map_interrupt(InterruptSource::Tap, InterruptRoute::Int1)
        .await?;

    loop {
        let status = imu.wait_for_interrupt(int1, InterruptChannel::Int1).await?;
        if status.tap() {
            let accel = imu.read_accel().await?;
            let _ = accel;
        }
    }
}

fn main() {}
