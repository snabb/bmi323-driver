use bmi323_driver::{Bmi323Async, I2C_ADDRESS_PRIMARY, SelfTestSelection};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

#[allow(dead_code)]
async fn embassy_style_self_test_task<I2C, D>(
    i2c: I2C,
    delay: &mut D,
) -> Result<(), bmi323_driver::Error<I2C::Error>>
where
    I2C: I2c,
    D: DelayNs,
{
    let mut imu = Bmi323Async::new_i2c(i2c, I2C_ADDRESS_PRIMARY);

    imu.init(delay).await?;

    let result = imu.run_self_test(delay, SelfTestSelection::Both).await?;

    let _ = result.passed;
    let _ = result.accelerometer_ok();
    let _ = result.gyroscope_ok();
    let _ = result.sample_rate_error;
    let _ = result.error_status;
    let _ = result.detail;

    Ok(())
}

fn main() {}
