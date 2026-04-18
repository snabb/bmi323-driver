#![no_std]
#![no_main]

use bmi323_driver::{
    ActiveLevel, AltAccelSwitchProfile, AltConfigSwitchSource, Bmi323Async,
    EventReportMode, I2C_ADDRESS_PRIMARY, InterruptChannel, InterruptPinConfig,
    InterruptRoute, InterruptSource, MotionAxes, NoMotionConfig, OutputDataRate,
    OutputMode, ReferenceUpdate,
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

    let profile = AltAccelSwitchProfile::low_power_to_high_performance(
        OutputDataRate::Hz25,
        OutputDataRate::Hz200,
        AltConfigSwitchSource::AnyMotion,
        AltConfigSwitchSource::NoMotion,
    );

    if let Err(err) = imu.set_accel_config(profile.user_accel).await {
        error!("BMI323 user accel config failed: {:?}", err);
        loop {
            cortex_m::asm::wfi();
        }
    }

    if let Err(err) = imu.set_alt_accel_config(profile.alternate_accel).await {
        error!("BMI323 alternate accel config failed: {:?}", err);
        loop {
            cortex_m::asm::wfi();
        }
    }

    if let Err(err) = imu
        .configure_any_motion(bmi323_driver::AnyMotionConfig {
            axes: MotionAxes::XYZ,
            threshold: bmi323_driver::AnyMotionConfig::threshold_from_g(0.08),
            hysteresis: bmi323_driver::AnyMotionConfig::hysteresis_from_g(0.02),
            duration: 5,  // 5 / 50 s = 100 ms above threshold before event
            wait_time: 1, // 1 / 50 s = 20 ms clear delay after slope drops
            reference_update: ReferenceUpdate::EverySample,
            report_mode: EventReportMode::AllEvents,
            interrupt_hold: 3, // 0.625 ms * 2^3 = 5 ms interrupt hold
        })
        .await
    {
        error!("BMI323 any-motion config failed: {:?}", err);
        loop {
            cortex_m::asm::wfi();
        }
    }

    if let Err(err) = imu
        .configure_no_motion(NoMotionConfig {
            axes: MotionAxes::XYZ,
            threshold: NoMotionConfig::threshold_from_g(0.03),
            hysteresis: NoMotionConfig::hysteresis_from_g(0.01),
            duration: 100, // 100 / 50 s = 2.0 s below threshold before event
            wait_time: 1,  // 1 / 50 s = 20 ms clear delay after slope stays low
            reference_update: ReferenceUpdate::EverySample,
            report_mode: EventReportMode::AllEvents,
            interrupt_hold: 3, // 0.625 ms * 2^3 = 5 ms interrupt hold
        })
        .await
    {
        error!("BMI323 no-motion config failed: {:?}", err);
        loop {
            cortex_m::asm::wfi();
        }
    }

    if let Err(err) = imu.configure_alt_config_control(profile.control).await {
        error!("BMI323 alternate control config failed: {:?}", err);
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
        .map_interrupt(InterruptSource::AnyMotion, InterruptRoute::Int1)
        .await
    {
        error!("BMI323 any-motion routing failed: {:?}", err);
        loop {
            cortex_m::asm::wfi();
        }
    }

    if let Err(err) = imu
        .map_interrupt(InterruptSource::NoMotion, InterruptRoute::Int1)
        .await
    {
        error!("BMI323 no-motion routing failed: {:?}", err);
        loop {
            cortex_m::asm::wfi();
        }
    }

    info!("BMI323 alternate accel switching armed");

    loop {
        match imu
            .wait_for_interrupt(&mut int1, InterruptChannel::Int1)
            .await
        {
            Ok(status) => match imu.alt_status().await {
                Ok(alt_status) => match imu.read_accel().await {
                    Ok(accel) => {
                        let [x, y, z] = accel.as_g(imu.accel_range());
                        info!(
                            "status any_motion={} no_motion={} alt_accel={} accel_g=({=f32}, {=f32}, {=f32})",
                            status.any_motion(),
                            status.no_motion(),
                            alt_status.accel_uses_alternate(),
                            x,
                            y,
                            z
                        );
                    }
                    Err(err) => error!("BMI323 accel read failed: {:?}", err),
                },
                Err(err) => error!("BMI323 alt status read failed: {:?}", err),
            },
            Err(err) => error!("BMI323 interrupt handling failed: {:?}", err),
        }
    }
}
