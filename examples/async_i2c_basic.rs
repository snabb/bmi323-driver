use bmi323_driver::{AccelConfig, Bmi323, GyroConfig, I2C_ADDRESS_PRIMARY, OutputDataRate};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

#[allow(dead_code)]
async fn async_example<I2C, D>(
    i2c: I2C,
    delay: &mut D,
) -> Result<(), bmi323_driver::Error<I2C::Error>>
where
    I2C: I2c,
    D: DelayNs,
{
    let mut imu = Bmi323::new_i2c(i2c, I2C_ADDRESS_PRIMARY);

    imu.init(delay).await?;

    imu.set_accel_config(AccelConfig {
        odr: OutputDataRate::Hz100,
        ..Default::default()
    })
    .await?;

    imu.set_gyro_config(GyroConfig {
        odr: OutputDataRate::Hz100,
        ..Default::default()
    })
    .await?;

    let sample = imu.read_imu_data().await?;
    let accel_g = sample.accel.as_g(imu.accel_range());
    let gyro_dps = sample.gyro.as_dps(imu.gyro_range());

    let _ = (accel_g, gyro_dps);

    Ok(())
}

fn main() {}
