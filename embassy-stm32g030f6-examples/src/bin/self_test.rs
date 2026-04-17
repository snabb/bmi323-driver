#![no_std]
#![no_main]

use bmi323_driver::{Bmi323Async, I2C_ADDRESS_PRIMARY, SelfTestSelection};
use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma;
use embassy_stm32::i2c::{self, Config as I2cConfig, I2c};
use embassy_stm32::peripherals;
use embassy_stm32::time::Hertz;
use embassy_time::Delay;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    I2C2 => i2c::EventInterruptHandler<peripherals::I2C2>, i2c::ErrorInterruptHandler<peripherals::I2C2>;
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2_3 => dma::InterruptHandler<peripherals::DMA1_CH2>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = Hertz(400_000);
    i2c_config.scl_pullup = false;
    i2c_config.sda_pullup = false;

    let i2c = I2c::new(
        p.I2C2, p.PA11, p.PA12, p.DMA1_CH1, p.DMA1_CH2, Irqs, i2c_config,
    );

    let mut delay = Delay;
    let mut imu = Bmi323Async::new_i2c(i2c, I2C_ADDRESS_PRIMARY);

    if let Err(err) = imu.init(&mut delay).await {
        error!("BMI323 init failed: {:?}", err);
        loop {
            cortex_m::asm::wfi();
        }
    }

    match imu.run_self_test(&mut delay, SelfTestSelection::Both).await {
        Ok(result) => {
            info!(
                "self-test passed={} accel_ok={} gyro_ok={} sample_rate_err={} error_status={=u8}",
                result.passed,
                result.accelerometer_ok(),
                result.gyroscope_ok(),
                result.sample_rate_error,
                result.error_status
            );
            info!(
                "self-test detail acc=({=bool}, {=bool}, {=bool}) gyro=({=bool}, {=bool}, {=bool}) drive={=bool}",
                result.detail.acc_sens_x_ok(),
                result.detail.acc_sens_y_ok(),
                result.detail.acc_sens_z_ok(),
                result.detail.gyr_sens_x_ok(),
                result.detail.gyr_sens_y_ok(),
                result.detail.gyr_sens_z_ok(),
                result.detail.gyr_drive_ok()
            );
        }
        Err(err) => error!("BMI323 self-test failed: {:?}", err),
    }

    loop {
        cortex_m::asm::wfi();
    }
}
