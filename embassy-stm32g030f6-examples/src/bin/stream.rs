#![no_std]
#![no_main]

use bmi323_driver::{AccelConfig, Bmi323Async, GyroConfig, I2C_ADDRESS_PRIMARY, OutputDataRate};
use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma;
use embassy_stm32::i2c::{self, Config as I2cConfig, I2c};
use embassy_stm32::peripherals;
use embassy_stm32::time::Hertz;
use embassy_time::{Delay, Timer};
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

    if let Err(err) = imu
        .set_accel_config(AccelConfig {
            odr: OutputDataRate::Hz100,
            ..Default::default()
        })
        .await
    {
        error!("BMI323 accel config failed: {:?}", err);
        loop {
            cortex_m::asm::wfi();
        }
    }

    if let Err(err) = imu
        .set_gyro_config(GyroConfig {
            odr: OutputDataRate::Hz100,
            ..Default::default()
        })
        .await
    {
        error!("BMI323 gyro config failed: {:?}", err);
        loop {
            cortex_m::asm::wfi();
        }
    }

    info!("BMI323 streaming started");

    loop {
        match imu.read_imu_data().await {
            Ok(sample) => {
                let [ax, ay, az] = sample.accel.as_g(imu.accel_range());
                let [gx, gy, gz] = sample.gyro.as_dps(imu.gyro_range());
                info!(
                    "accel_g=({=f32}, {=f32}, {=f32}) gyro_dps=({=f32}, {=f32}, {=f32})",
                    ax, ay, az, gx, gy, gz
                );
            }
            Err(err) => error!("BMI323 stream read failed: {:?}", err),
        }

        Timer::after_millis(200).await;
    }
}
