use bmi323_driver::{AccelConfig, Bmi323, GyroConfig, OutputDataRate, I2C_ADDRESS_PRIMARY};
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

    imu.set_accel_config(AccelConfig {
        odr: OutputDataRate::Hz100,
        ..Default::default()
    })?;

    imu.set_gyro_config(GyroConfig {
        odr: OutputDataRate::Hz100,
        ..Default::default()
    })?;

    let sample = imu.read_imu_data()?;
    let accel_g = sample.accel.as_g(imu.accel_range());
    let gyro_dps = sample.gyro.as_dps(imu.gyro_range());

    let _ = (accel_g, gyro_dps);

    Ok(())
}

fn main() {}
