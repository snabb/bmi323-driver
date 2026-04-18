#![no_std]
#![no_main]

use bmi323_driver::{
    AccelConfig, ActiveLevel, Bmi323Async, EventReportMode, InterruptChannel,
    I2C_ADDRESS_PRIMARY, InterruptPinConfig, InterruptRoute, InterruptSource,
    OutputDataRate, OutputMode, TiltConfig,
};
use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::Pull;
use embassy_stm32::i2c::{self, Config as I2cConfig, I2c};
use embassy_stm32::interrupt::typelevel;
use embassy_stm32::peripherals;
use embassy_stm32::time::Hertz;
use embassy_time::Delay;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    I2C2 => i2c::EventInterruptHandler<peripherals::I2C2>, i2c::ErrorInterruptHandler<peripherals::I2C2>;
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2_3 => dma::InterruptHandler<peripherals::DMA1_CH2>;
    EXTI2_3 => exti::InterruptHandler<typelevel::EXTI2_3>;
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

    let mut int1 = ExtiInput::new(p.PB3, p.EXTI3, Pull::Down, Irqs);
    let mut delay = Delay;
    let mut imu = Bmi323Async::new_i2c(i2c, I2C_ADDRESS_PRIMARY);

    if let Err(err) = imu.init(&mut delay).await {
        error!("BMI323 init failed: {:?}", err);
        loop {
            cortex_m::asm::wfi();
        }
    }

    if let Err(err) = imu.enable_feature_engine().await {
        error!("BMI323 feature engine enable failed: {:?}", err);
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
        .configure_tilt(TiltConfig {
            segment_size: 100,     // 100 / 50 s = 2.0 s reference-vector averaging duration
            min_tilt_angle: 210,   // raw nonlinear min-tilt-angle field
            beta_acc_mean: 0xF069, // raw beta_acc_mean field, close to datasheet default
            report_mode: EventReportMode::AllEvents,
            interrupt_hold: 3, // 0.625 ms * 2^3 = 5 ms interrupt hold
        })
        .await
    {
        error!("BMI323 tilt config failed: {:?}", err);
        loop {
            cortex_m::asm::wfi();
        }
    }

    if let Err(err) = imu.set_interrupt_latching(true).await {
        error!("BMI323 interrupt latch config failed: {:?}", err);
        loop {
            cortex_m::asm::wfi();
        }
    }

    if let Err(err) = imu
        .configure_interrupt_pin(
            InterruptChannel::Int1,
            InterruptPinConfig {
                active_level: ActiveLevel::High,
                output_mode: OutputMode::PushPull,
                enabled: true,
            },
        )
        .await
    {
        error!("BMI323 INT1 pin config failed: {:?}", err);
        loop {
            cortex_m::asm::wfi();
        }
    }

    if let Err(err) = imu
        .map_interrupt(InterruptSource::Tilt, InterruptRoute::Int1)
        .await
    {
        error!("BMI323 tilt routing failed: {:?}", err);
        loop {
            cortex_m::asm::wfi();
        }
    }

    info!("BMI323 tilt detection armed");

    loop {
        match imu
            .wait_for_interrupt(&mut int1, InterruptChannel::Int1)
            .await
        {
            Ok(status) if status.tilt() => match imu.read_accel().await {
                Ok(accel) => {
                    let [x, y, z] = accel.as_g(imu.accel_range());
                    info!("tilt detected accel_g=({=f32}, {=f32}, {=f32})", x, y, z);
                }
                Err(err) => error!("BMI323 accel read failed: {:?}", err),
            },
            Ok(status) => info!("unexpected INT1 status raw=0x{=u16:04x}", status.0),
            Err(err) => error!("BMI323 interrupt handling failed: {:?}", err),
        }
    }
}
