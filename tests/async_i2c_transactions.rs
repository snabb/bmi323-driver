use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

use bmi323_driver::{
    AccelConfig, AccelMode, AccelRange, ActiveLevel, AltAccelConfig, AltConfigControl,
    AltConfigSwitchSource, AltGyroConfig, AnyMotionConfig, AverageSamples, Bandwidth, Bmi323Async,
    EventReportMode, FeatureBlockingMode, FifoConfig, FlatConfig, GyroConfig, GyroMode, GyroRange,
    InterruptChannel, InterruptPinConfig, InterruptRoute, InterruptSource, MotionAxes,
    NoMotionConfig, OrientationConfig, OrientationMode, OutputDataRate, OutputMode, ReferenceUpdate,
    SelfTestSelection, SignificantMotionConfig, StepCounterConfig, TapAxis, TapConfig,
    TapDetectionMode, TapReportingMode, TiltConfig,
};
use embedded_hal_mock::eh1::delay::{CheckedDelay, NoopDelay, Transaction as DelayTransaction};
use embedded_hal_mock::eh1::digital::{
    Mock as PinMock, State as PinState, Transaction as PinTransaction,
};
use embedded_hal::i2c::ErrorKind as I2cErrorKind;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

const ADDR: u8 = 0x68;

const CHIP_ID: u8 = 0x00;
const ERR_REG: u8 = 0x01;
const STATUS: u8 = 0x02;
const ACC_DATA_X: u8 = 0x03;
const GYR_DATA_X: u8 = 0x06;
const TEMP_DATA: u8 = 0x09;
const SENSOR_TIME_0: u8 = 0x0A;
const INT_STATUS_INT1: u8 = 0x0D;
const INT_STATUS_INT2: u8 = 0x0E;
const INT_STATUS_IBI: u8 = 0x0F;
const FEATURE_IO0: u8 = 0x10;
const FEATURE_IO1: u8 = 0x11;
const FEATURE_IO2: u8 = 0x12;
const FEATURE_IO3: u8 = 0x13;
const FEATURE_IO_STATUS: u8 = 0x14;
const FIFO_FILL_LEVEL: u8 = 0x15;
const FIFO_DATA: u8 = 0x16;
const ACC_CONF: u8 = 0x20;
const GYR_CONF: u8 = 0x21;
const ALT_ACC_CONF: u8 = 0x28;
const ALT_GYR_CONF: u8 = 0x29;
const ALT_CONF: u8 = 0x2A;
const ALT_STATUS: u8 = 0x2B;
const FIFO_WATERMARK: u8 = 0x35;
const FIFO_CONF: u8 = 0x36;
const FIFO_CTRL: u8 = 0x37;
const IO_INT_CTRL: u8 = 0x38;
const INT_CONF: u8 = 0x39;
const INT_MAP1: u8 = 0x3A;
const FEATURE_CTRL: u8 = 0x40;
const FEATURE_DATA_ADDR: u8 = 0x41;
const FEATURE_DATA_TX: u8 = 0x42;
const CMD: u8 = 0x7E;

const EXT_GEN_SET_1: u16 = 0x02;
const EXT_ANYMO_1: u16 = 0x05;
const EXT_ANYMO_2: u16 = 0x06;
const EXT_ANYMO_3: u16 = 0x07;
const EXT_NOMO_1: u16 = 0x08;
const EXT_NOMO_2: u16 = 0x09;
const EXT_NOMO_3: u16 = 0x0A;
const EXT_FLAT_1: u16 = 0x0B;
const EXT_FLAT_2: u16 = 0x0C;
const EXT_SIGMO_1: u16 = 0x0D;
const EXT_SIGMO_2: u16 = 0x0E;
const EXT_SIGMO_3: u16 = 0x0F;
const EXT_SC_1: u16 = 0x10;
const EXT_ORIENT_1: u16 = 0x1C;
const EXT_ORIENT_2: u16 = 0x1D;
const EXT_TAP_1: u16 = 0x1E;
const EXT_TAP_2: u16 = 0x1F;
const EXT_TAP_3: u16 = 0x20;
const EXT_TILT_1: u16 = 0x21;
const EXT_TILT_2: u16 = 0x22;
const EXT_ALT_CONFIG_CHG: u16 = 0x23;
const EXT_ST_RESULT: u16 = 0x24;
const EXT_ST_SELECT: u16 = 0x25;

const BMI323_CHIP_ID: u16 = 0x0043;
const SOFT_RESET: u16 = 0xDEAF;

fn block_on<F: Future>(future: F) -> F::Output {
    fn raw_waker() -> RawWaker {
        fn clone(_: *const ()) -> RawWaker {
            raw_waker()
        }
        fn wake(_: *const ()) {}
        fn wake_by_ref(_: *const ()) {}
        fn drop(_: *const ()) {}

        RawWaker::new(
            core::ptr::null(),
            &RawWakerVTable::new(clone, wake, wake_by_ref, drop),
        )
    }

    // The mocked HAL operations complete immediately, so a no-op waker is
    // sufficient for driving these test futures.
    let waker = unsafe { Waker::from_raw(raw_waker()) };
    let mut context = Context::from_waker(&waker);
    let mut future = pin!(future);

    loop {
        match Future::poll(future.as_mut(), &mut context) {
            Poll::Ready(output) => return output,
            Poll::Pending => core::hint::spin_loop(),
        }
    }
}

fn write_word(reg: u8, word: u16) -> I2cTransaction {
    let [lo, hi] = word.to_le_bytes();
    I2cTransaction::write(ADDR, vec![reg, lo, hi])
}

fn read_word(reg: u8, word: u16) -> I2cTransaction {
    let [lo, hi] = word.to_le_bytes();
    I2cTransaction::write_read(ADDR, vec![reg], vec![0, 0, lo, hi])
}

fn read_words(reg: u8, words: &[u16]) -> I2cTransaction {
    let mut response = vec![0, 0];
    for word in words {
        let [lo, hi] = word.to_le_bytes();
        response.push(lo);
        response.push(hi);
    }
    I2cTransaction::write_read(ADDR, vec![reg], response)
}

#[test]
fn async_i2c_init_resets_sensor_and_reads_device_state() {
    let expectations = [
        write_word(CMD, SOFT_RESET),
        read_word(CHIP_ID, BMI323_CHIP_ID),
        read_word(ERR_REG, 0x0000),
        read_word(STATUS, 0x00E1),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);
    let mut delay = CheckedDelay::new(&[DelayTransaction::async_delay_ms(2)]);

    let state = block_on(imu.init(&mut delay)).unwrap();

    assert_eq!(state.chip_id, BMI323_CHIP_ID as u8);
    assert!(state.status.por_detected());
    assert!(state.status.drdy_temp());
    assert!(state.status.drdy_gyro());
    assert!(state.status.drdy_accel());
    assert!(!state.error.fatal());

    delay.done();
    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_set_sensor_configs_writes_expected_words_and_tracks_ranges() {
    let accel = AccelConfig {
        mode: AccelMode::HighPerformance,
        average: AverageSamples::Avg4,
        bandwidth: Bandwidth::OdrOver4,
        range: AccelRange::G16,
        odr: OutputDataRate::Hz200,
    };
    let gyro = GyroConfig {
        mode: GyroMode::LowPower,
        average: AverageSamples::Avg8,
        bandwidth: Bandwidth::OdrOver2,
        range: GyroRange::Dps500,
        odr: OutputDataRate::Hz100,
    };
    let expectations = [
        write_word(ACC_CONF, accel.to_word()),
        write_word(GYR_CONF, gyro.to_word()),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.set_accel_config(accel)).unwrap();
    block_on(imu.set_gyro_config(gyro)).unwrap();

    assert_eq!(imu.accel_range(), AccelRange::G16);
    assert_eq!(imu.gyro_range(), GyroRange::Dps500);

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_enable_feature_engine_uses_expected_register_sequence() {
    let expectations = [
        write_word(ACC_CONF, 0x0000),
        write_word(GYR_CONF, 0x0000),
        write_word(FEATURE_IO2, 0x012C),
        write_word(FEATURE_IO_STATUS, 0x0001),
        write_word(FEATURE_CTRL, 0x0001),
        read_word(FEATURE_IO1, 0x0001),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.enable_feature_engine()).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_configure_any_motion_and_interrupt_routing_programs_expected_registers() {
    let config = AnyMotionConfig {
        axes: MotionAxes {
            x: true,
            y: true,
            z: false,
        },
        threshold: 0x007B,
        hysteresis: 0x0009,
        duration: 50,
        wait_time: 2,
        reference_update: ReferenceUpdate::EverySample,
        report_mode: EventReportMode::AllEvents,
        interrupt_hold: 3,
    };
    let expectations = [
        write_word(FEATURE_DATA_ADDR, EXT_GEN_SET_1),
        read_word(FEATURE_DATA_TX, 0x0000),
        write_word(FEATURE_DATA_ADDR, EXT_GEN_SET_1),
        write_word(FEATURE_DATA_TX, 0x0006),
        write_word(FEATURE_DATA_ADDR, EXT_ANYMO_1),
        write_word(FEATURE_DATA_TX, 0x107B),
        write_word(FEATURE_DATA_ADDR, EXT_ANYMO_2),
        write_word(FEATURE_DATA_TX, 0x0009),
        write_word(FEATURE_DATA_ADDR, EXT_ANYMO_3),
        write_word(FEATURE_DATA_TX, 0x4032),
        read_word(FEATURE_IO0, 0x8000),
        write_word(FEATURE_IO0, 0x8018),
        write_word(FEATURE_IO_STATUS, 0x0001),
        read_word(IO_INT_CTRL, 0x0000),
        write_word(IO_INT_CTRL, 0x0007),
        read_word(INT_MAP1, 0x0000),
        write_word(INT_MAP1, 0x0004),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.configure_any_motion(config)).unwrap();
    block_on(imu.configure_interrupt_pin(
        InterruptChannel::Int1,
        InterruptPinConfig {
            active_level: ActiveLevel::High,
            output_mode: OutputMode::OpenDrain,
            enabled: true,
        },
    ))
    .unwrap();
    block_on(imu.map_interrupt(InterruptSource::AnyMotion, InterruptRoute::Int1)).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_configure_no_motion_programs_expected_registers() {
    let config = NoMotionConfig {
        axes: MotionAxes {
            x: false,
            y: true,
            z: true,
        },
        threshold: 0x0020,
        hysteresis: 0x0004,
        duration: 100,
        wait_time: 7,
        reference_update: ReferenceUpdate::OnDetection,
        report_mode: EventReportMode::FirstEventOnly,
        interrupt_hold: 20,
    };
    let expectations = [
        write_word(FEATURE_DATA_ADDR, EXT_GEN_SET_1),
        read_word(FEATURE_DATA_TX, 0x0000),
        write_word(FEATURE_DATA_ADDR, EXT_GEN_SET_1),
        write_word(FEATURE_DATA_TX, 0x001B),
        write_word(FEATURE_DATA_ADDR, EXT_NOMO_1),
        write_word(FEATURE_DATA_TX, 0x0020),
        write_word(FEATURE_DATA_ADDR, EXT_NOMO_2),
        write_word(FEATURE_DATA_TX, 0x0004),
        write_word(FEATURE_DATA_ADDR, EXT_NOMO_3),
        write_word(FEATURE_DATA_TX, 0xE064),
        read_word(FEATURE_IO0, 0xFFF8),
        write_word(FEATURE_IO0, 0xFFFE),
        write_word(FEATURE_IO_STATUS, 0x0001),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.configure_no_motion(config)).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_read_paths_decode_burst_and_feature_words_correctly() {
    let expectations = [
        read_words(
            ACC_DATA_X,
            &[0x0001, 0xFFFE, 0x7FFF, 0x8000, 0x0032, 0xFF9C],
        ),
        read_words(SENSOR_TIME_0, &[0x5678, 0x1234]),
        read_word(FEATURE_IO2, 0xCDEF),
        read_word(FEATURE_IO3, 0x89AB),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    let imu_data = block_on(imu.read_imu_data()).unwrap();
    let sensor_time = block_on(imu.read_sensor_time()).unwrap();
    let step_count = block_on(imu.read_step_count()).unwrap();

    assert_eq!(imu_data.accel.x, 1);
    assert_eq!(imu_data.accel.y, -2);
    assert_eq!(imu_data.accel.z, i16::MAX);
    assert_eq!(imu_data.gyro.x, i16::MIN);
    assert_eq!(imu_data.gyro.y, 50);
    assert_eq!(imu_data.gyro.z, -100);
    assert_eq!(sensor_time, 0x1234_5678);
    assert_eq!(step_count, 0x89AB_CDEF);

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_configure_alt_config_control_writes_sensor_and_feature_words() {
    let control = AltConfigControl {
        accel_enabled: true,
        gyro_enabled: false,
        reset_on_user_config_write: true,
        switch_to_alternate: AltConfigSwitchSource::AnyMotion,
        switch_to_user: AltConfigSwitchSource::NoMotion,
    };
    let expectations = [
        write_word(ALT_CONF, 0x0101),
        write_word(FEATURE_DATA_ADDR, EXT_ALT_CONFIG_CHG),
        write_word(FEATURE_DATA_TX, 0x0012),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.configure_alt_config_control(control)).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_self_test_uses_expected_sequence_and_restores_configuration() {
    let expectations = [
        read_word(ACC_CONF, 0x4127),
        read_word(GYR_CONF, 0x4047),
        read_word(ALT_ACC_CONF, 0x7208),
        read_word(ALT_GYR_CONF, 0x4108),
        write_word(FEATURE_DATA_ADDR, EXT_ST_SELECT),
        read_word(FEATURE_DATA_TX, 0x0003),
        write_word(ACC_CONF, 0x0000),
        write_word(GYR_CONF, 0x0000),
        write_word(FEATURE_IO2, 0x012C),
        write_word(FEATURE_IO_STATUS, 0x0001),
        write_word(FEATURE_CTRL, 0x0001),
        read_word(FEATURE_IO1, 0x0001),
        write_word(ACC_CONF, 0x7029),
        write_word(ALT_ACC_CONF, 0x0000),
        write_word(ALT_GYR_CONF, 0x0000),
        write_word(FEATURE_DATA_ADDR, EXT_ST_SELECT),
        write_word(FEATURE_DATA_TX, SelfTestSelection::Gyroscope.to_word()),
        write_word(CMD, 0x0100),
        read_word(FEATURE_IO1, 0x0055),
        write_word(FEATURE_DATA_ADDR, EXT_ST_RESULT),
        read_word(FEATURE_DATA_TX, 0x0078),
        write_word(ACC_CONF, 0x4127),
        write_word(GYR_CONF, 0x4047),
        write_word(ALT_ACC_CONF, 0x7208),
        write_word(ALT_GYR_CONF, 0x4108),
        write_word(FEATURE_DATA_ADDR, EXT_ST_SELECT),
        write_word(FEATURE_DATA_TX, 0x0003),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);
    let mut delay = CheckedDelay::new(&[DelayTransaction::async_delay_ms(10)]);

    let result = block_on(imu.run_self_test(&mut delay, SelfTestSelection::Gyroscope)).unwrap();

    assert!(result.passed);
    assert!(!result.sample_rate_error);
    assert_eq!(result.error_status, 5);
    assert!(result.gyroscope_ok());

    delay.done();
    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_configure_flat_programs_expected_registers() {
    let config = FlatConfig {
        theta: 5,
        blocking: FeatureBlockingMode::AccelOver1p5g,
        hold_time: 10,
        slope_threshold: 3,
        hysteresis: 2,
        report_mode: EventReportMode::AllEvents,
        interrupt_hold: 0,
    };
    let expectations = [
        write_word(FEATURE_DATA_ADDR, EXT_GEN_SET_1),
        read_word(FEATURE_DATA_TX, 0x0000),
        write_word(FEATURE_DATA_ADDR, EXT_GEN_SET_1),
        write_word(FEATURE_DATA_TX, 0x0000),
        write_word(FEATURE_DATA_ADDR, EXT_FLAT_1),
        write_word(FEATURE_DATA_TX, 0x0A45),
        write_word(FEATURE_DATA_ADDR, EXT_FLAT_2),
        write_word(FEATURE_DATA_TX, 0x0203),
        read_word(FEATURE_IO0, 0x0000),
        write_word(FEATURE_IO0, 0x0040),
        write_word(FEATURE_IO_STATUS, 0x0001),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.configure_flat(config)).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_configure_orientation_programs_expected_registers() {
    let config = OrientationConfig {
        upside_down_enabled: true,
        mode: OrientationMode::Symmetrical,
        blocking: FeatureBlockingMode::AccelOver1p5gOrHalfSlope,
        theta: 16,
        hold_time: 3,
        slope_threshold: 8,
        hysteresis: 5,
        report_mode: EventReportMode::FirstEventOnly,
        interrupt_hold: 2,
    };
    let expectations = [
        write_word(FEATURE_DATA_ADDR, EXT_GEN_SET_1),
        read_word(FEATURE_DATA_TX, 0x0000),
        write_word(FEATURE_DATA_ADDR, EXT_GEN_SET_1),
        write_word(FEATURE_DATA_TX, 0x0005),
        write_word(FEATURE_DATA_ADDR, EXT_ORIENT_1),
        write_word(FEATURE_DATA_TX, 0x1A11),
        write_word(FEATURE_DATA_ADDR, EXT_ORIENT_2),
        write_word(FEATURE_DATA_TX, 0x0508),
        read_word(FEATURE_IO0, 0x0000),
        write_word(FEATURE_IO0, 0x0080),
        write_word(FEATURE_IO_STATUS, 0x0001),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.configure_orientation(config)).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_configure_tap_programs_expected_registers() {
    let config = TapConfig {
        axis: TapAxis::Z,
        reporting_mode: TapReportingMode::Confirmed,
        max_peaks_for_tap: 4,
        mode: TapDetectionMode::Normal,
        single_tap_enabled: true,
        double_tap_enabled: true,
        triple_tap_enabled: false,
        tap_peak_threshold: 0x0100,
        max_gesture_duration: 8,
        max_duration_between_peaks: 3,
        tap_shock_settling_duration: 2,
        min_quiet_duration_between_taps: 1,
        quiet_time_after_gesture: 5,
        report_mode: EventReportMode::AllEvents,
        interrupt_hold: 1,
    };
    let expectations = [
        write_word(FEATURE_DATA_ADDR, EXT_GEN_SET_1),
        read_word(FEATURE_DATA_TX, 0x0000),
        write_word(FEATURE_DATA_ADDR, EXT_GEN_SET_1),
        write_word(FEATURE_DATA_TX, 0x0002),
        write_word(FEATURE_DATA_ADDR, EXT_TAP_1),
        write_word(FEATURE_DATA_TX, 0x0066),
        write_word(FEATURE_DATA_ADDR, EXT_TAP_2),
        write_word(FEATURE_DATA_TX, 0x2100),
        write_word(FEATURE_DATA_ADDR, EXT_TAP_3),
        write_word(FEATURE_DATA_TX, 0x5123),
        read_word(FEATURE_IO0, 0x0000),
        write_word(FEATURE_IO0, 0x3000),
        write_word(FEATURE_IO_STATUS, 0x0001),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.configure_tap(config)).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_configure_significant_motion_programs_expected_registers() {
    let config = SignificantMotionConfig {
        block_size: 250,
        peak_to_peak_min: 256,
        mean_crossing_rate_min: 10,
        peak_to_peak_max: 512,
        mean_crossing_rate_max: 20,
        report_mode: EventReportMode::AllEvents,
        interrupt_hold: 0,
    };
    let expectations = [
        write_word(FEATURE_DATA_ADDR, EXT_GEN_SET_1),
        read_word(FEATURE_DATA_TX, 0x0000),
        write_word(FEATURE_DATA_ADDR, EXT_GEN_SET_1),
        write_word(FEATURE_DATA_TX, 0x0000),
        write_word(FEATURE_DATA_ADDR, EXT_SIGMO_1),
        write_word(FEATURE_DATA_TX, 0x00FA),
        write_word(FEATURE_DATA_ADDR, EXT_SIGMO_2),
        write_word(FEATURE_DATA_TX, 0x2900),
        write_word(FEATURE_DATA_ADDR, EXT_SIGMO_3),
        write_word(FEATURE_DATA_TX, 0x5200),
        read_word(FEATURE_IO0, 0x0000),
        write_word(FEATURE_IO0, 0x0400),
        write_word(FEATURE_IO_STATUS, 0x0001),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.configure_significant_motion(config)).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_configure_tilt_programs_expected_registers() {
    let config = TiltConfig {
        segment_size: 100,
        min_tilt_angle: 0xC0,
        beta_acc_mean: 0x00AB,
        report_mode: EventReportMode::FirstEventOnly,
        interrupt_hold: 0,
    };
    let expectations = [
        write_word(FEATURE_DATA_ADDR, EXT_GEN_SET_1),
        read_word(FEATURE_DATA_TX, 0x0000),
        write_word(FEATURE_DATA_ADDR, EXT_GEN_SET_1),
        write_word(FEATURE_DATA_TX, 0x0001),
        write_word(FEATURE_DATA_ADDR, EXT_TILT_1),
        write_word(FEATURE_DATA_TX, 0xC064),
        write_word(FEATURE_DATA_ADDR, EXT_TILT_2),
        write_word(FEATURE_DATA_TX, 0x00AB),
        read_word(FEATURE_IO0, 0x0000),
        write_word(FEATURE_IO0, 0x0800),
        write_word(FEATURE_IO_STATUS, 0x0001),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.configure_tilt(config)).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_step_detector_and_counter_enable_programs_expected_bits() {
    let expectations = [
        read_word(FEATURE_IO0, 0x0000),
        write_word(FEATURE_IO0, 0x0100),
        write_word(FEATURE_IO_STATUS, 0x0001),
        read_word(FEATURE_IO0, 0x0100),
        write_word(FEATURE_IO0, 0x0300),
        write_word(FEATURE_IO_STATUS, 0x0001),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.set_step_detector_enabled(true)).unwrap();
    block_on(imu.set_step_counter_enabled(true)).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_configure_step_counter_writes_expected_feature_word() {
    let config = StepCounterConfig {
        watermark_level: 100,
        reset_counter: false,
    };
    let expectations = [
        write_word(FEATURE_DATA_ADDR, EXT_SC_1),
        write_word(FEATURE_DATA_TX, 0x0064),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.configure_step_counter(config)).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_reset_step_counter_sets_reset_bit() {
    let expectations = [
        write_word(FEATURE_DATA_ADDR, EXT_SC_1),
        write_word(FEATURE_DATA_TX, 0x0400),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.reset_step_counter()).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_set_interrupt_latching_writes_int_conf() {
    let expectations = [write_word(INT_CONF, 0x0001), write_word(INT_CONF, 0x0000)];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.set_interrupt_latching(true)).unwrap();
    block_on(imu.set_interrupt_latching(false)).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_read_interrupt_status_reads_expected_registers() {
    let expectations = [
        read_word(INT_STATUS_INT1, 0x2000),
        read_word(INT_STATUS_INT2, 0x1000),
        read_word(INT_STATUS_IBI, 0x0040),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    let s1 = block_on(imu.read_interrupt_status(InterruptChannel::Int1)).unwrap();
    let s2 = block_on(imu.read_interrupt_status(InterruptChannel::Int2)).unwrap();
    let s3 = block_on(imu.read_interrupt_status(InterruptChannel::Ibi)).unwrap();

    assert!(s1.accel_data_ready());
    assert!(!s1.gyro_data_ready());
    assert!(s2.gyro_data_ready());
    assert!(!s2.accel_data_ready());
    assert!(s3.significant_motion());
    assert!(!s3.accel_data_ready());

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_set_fifo_config_writes_watermark_and_conf() {
    let config = FifoConfig {
        stop_on_full: true,
        include_time: false,
        include_accel: true,
        include_gyro: true,
        include_temperature: false,
    };
    let expectations = [
        write_word(FIFO_WATERMARK, 0x002A),
        write_word(FIFO_CONF, 0x0601),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.set_fifo_config(config, 42)).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_fifo_fill_level_masks_upper_bits() {
    let expectations = [read_word(FIFO_FILL_LEVEL, 0x8ABC)];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    let level = block_on(imu.fifo_fill_level()).unwrap();

    assert_eq!(level, 0x02BC);

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_flush_fifo_writes_fifo_ctrl() {
    let expectations = [write_word(FIFO_CTRL, 0x0001)];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.flush_fifo()).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_read_fifo_words_reads_from_fifo_data_register() {
    let expectations = [read_words(FIFO_DATA, &[0x1234, 0x5678])];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    let mut words = [0u16; 2];
    block_on(imu.read_fifo_words(&mut words)).unwrap();

    assert_eq!(words[0], 0x1234);
    assert_eq!(words[1], 0x5678);

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_read_temperature_celsius_converts_raw_correctly() {
    let expectations = [read_word(TEMP_DATA, 0x0200)];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    let temp = block_on(imu.read_temperature_celsius()).unwrap();

    assert!((temp - 24.0).abs() < 0.001);

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_read_accel_and_gyro_read_from_separate_registers() {
    let expectations = [
        read_words(ACC_DATA_X, &[100, 0xFFC8, 1]),
        read_words(GYR_DATA_X, &[50, 0xFF9C, 3]),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    let accel = block_on(imu.read_accel()).unwrap();
    let gyro = block_on(imu.read_gyro()).unwrap();

    assert_eq!(accel.x, 100);
    assert_eq!(accel.y, -56);
    assert_eq!(accel.z, 1);
    assert_eq!(gyro.x, 50);
    assert_eq!(gyro.y, -100);
    assert_eq!(gyro.z, 3);

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_init_returns_invalid_chip_id_error() {
    let expectations = [
        write_word(CMD, SOFT_RESET),
        read_word(CHIP_ID, 0x00AB), // wrong chip ID
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);
    let mut delay = NoopDelay::new();

    let result = block_on(imu.init(&mut delay));

    assert!(matches!(result, Err(bmi323_driver::Error::InvalidChipId(0xAB))));

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_init_returns_fatal_error_when_fatal_bit_set() {
    let expectations = [
        write_word(CMD, SOFT_RESET),
        read_word(CHIP_ID, BMI323_CHIP_ID),
        read_word(ERR_REG, 0x0001), // fatal bit set
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);
    let mut delay = NoopDelay::new();

    let result = block_on(imu.init(&mut delay));

    assert!(matches!(result, Err(bmi323_driver::Error::FatalError)));

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_configure_interrupt_pin_int2_writes_upper_byte_bits() {
    // Int2 shift=8: active_high=bit8, push_pull=bit9(0), enabled=bit10 → 0x0500
    let expectations = [
        read_word(IO_INT_CTRL, 0x0000),
        write_word(IO_INT_CTRL, 0x0500),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.configure_interrupt_pin(
        InterruptChannel::Int2,
        InterruptPinConfig {
            active_level: ActiveLevel::High,
            output_mode: OutputMode::PushPull,
            enabled: true,
        },
    ))
    .unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_configure_interrupt_pin_ibi_reads_register_then_returns_without_writing() {
    let expectations = [read_word(IO_INT_CTRL, 0x0000)];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.configure_interrupt_pin(
        InterruptChannel::Ibi,
        InterruptPinConfig {
            active_level: ActiveLevel::High,
            output_mode: OutputMode::PushPull,
            enabled: true,
        },
    ))
    .unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_configure_any_motion_on_detection_sets_bit12_to_zero() {
    let config = AnyMotionConfig {
        axes: MotionAxes {
            x: true,
            y: false,
            z: false,
        },
        threshold: 0x0064,
        hysteresis: 0x0000,
        duration: 0,
        wait_time: 0,
        reference_update: ReferenceUpdate::OnDetection,
        report_mode: EventReportMode::AllEvents,
        interrupt_hold: 0,
    };
    let expectations = [
        write_word(FEATURE_DATA_ADDR, EXT_GEN_SET_1),
        read_word(FEATURE_DATA_TX, 0x0000),
        write_word(FEATURE_DATA_ADDR, EXT_GEN_SET_1),
        write_word(FEATURE_DATA_TX, 0x0000),
        write_word(FEATURE_DATA_ADDR, EXT_ANYMO_1),
        write_word(FEATURE_DATA_TX, 0x0064), // OnDetection: bit 12 = 0
        write_word(FEATURE_DATA_ADDR, EXT_ANYMO_2),
        write_word(FEATURE_DATA_TX, 0x0000),
        write_word(FEATURE_DATA_ADDR, EXT_ANYMO_3),
        write_word(FEATURE_DATA_TX, 0x0000),
        read_word(FEATURE_IO0, 0x0000),
        write_word(FEATURE_IO0, 0x0008),
        write_word(FEATURE_IO_STATUS, 0x0001),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.configure_any_motion(config)).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_configure_no_motion_every_sample_sets_bit12_to_one() {
    let config = NoMotionConfig {
        axes: MotionAxes {
            x: true,
            y: false,
            z: false,
        },
        threshold: 0x0040,
        hysteresis: 0x0000,
        duration: 0,
        wait_time: 0,
        reference_update: ReferenceUpdate::EverySample,
        report_mode: EventReportMode::AllEvents,
        interrupt_hold: 0,
    };
    let expectations = [
        write_word(FEATURE_DATA_ADDR, EXT_GEN_SET_1),
        read_word(FEATURE_DATA_TX, 0x0000),
        write_word(FEATURE_DATA_ADDR, EXT_GEN_SET_1),
        write_word(FEATURE_DATA_TX, 0x0000),
        write_word(FEATURE_DATA_ADDR, EXT_NOMO_1),
        write_word(FEATURE_DATA_TX, 0x1040), // EverySample: bit 12 = 1
        write_word(FEATURE_DATA_ADDR, EXT_NOMO_2),
        write_word(FEATURE_DATA_TX, 0x0000),
        write_word(FEATURE_DATA_ADDR, EXT_NOMO_3),
        write_word(FEATURE_DATA_TX, 0x0000),
        read_word(FEATURE_IO0, 0x0000),
        write_word(FEATURE_IO0, 0x0001),
        write_word(FEATURE_IO_STATUS, 0x0001),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.configure_no_motion(config)).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_disabling_feature_engine_bits_clears_expected_bits() {
    let expectations = [
        read_word(FEATURE_IO0, 0x0040),
        write_word(FEATURE_IO0, 0x0000),
        write_word(FEATURE_IO_STATUS, 0x0001),
        read_word(FEATURE_IO0, 0x0080),
        write_word(FEATURE_IO0, 0x0000),
        write_word(FEATURE_IO_STATUS, 0x0001),
        read_word(FEATURE_IO0, 0x0400),
        write_word(FEATURE_IO0, 0x0000),
        write_word(FEATURE_IO_STATUS, 0x0001),
        read_word(FEATURE_IO0, 0x0800),
        write_word(FEATURE_IO0, 0x0000),
        write_word(FEATURE_IO_STATUS, 0x0001),
        read_word(FEATURE_IO0, 0x0100),
        write_word(FEATURE_IO0, 0x0000),
        write_word(FEATURE_IO_STATUS, 0x0001),
        read_word(FEATURE_IO0, 0x0200),
        write_word(FEATURE_IO0, 0x0000),
        write_word(FEATURE_IO_STATUS, 0x0001),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.set_flat_enabled(false)).unwrap();
    block_on(imu.set_orientation_enabled(false)).unwrap();
    block_on(imu.set_significant_motion_enabled(false)).unwrap();
    block_on(imu.set_tilt_enabled(false)).unwrap();
    block_on(imu.set_step_detector_enabled(false)).unwrap();
    block_on(imu.set_step_counter_enabled(false)).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_set_alt_accel_config_writes_alt_acc_conf() {
    let config = AltAccelConfig {
        mode: AccelMode::HighPerformance,
        average: AverageSamples::Avg4,
        odr: OutputDataRate::Hz100,
    };
    let expectations = [write_word(ALT_ACC_CONF, config.to_word())];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.set_alt_accel_config(config)).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_set_alt_gyro_config_writes_alt_gyr_conf() {
    let config = AltGyroConfig {
        mode: GyroMode::Normal,
        average: AverageSamples::Avg1,
        odr: OutputDataRate::Hz100,
    };
    let expectations = [write_word(ALT_GYR_CONF, config.to_word())];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    block_on(imu.set_alt_gyro_config(config)).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_alt_status_reads_alt_status_register() {
    let expectations = [read_word(ALT_STATUS, 0x0011)];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    let status = block_on(imu.alt_status()).unwrap();

    assert!(status.accel_uses_alternate());
    assert!(status.gyro_uses_alternate());

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_run_self_test_returns_restore_error_when_restore_configuration_fails() {
    let expectations = [
        read_word(ACC_CONF, 0x4127),
        read_word(GYR_CONF, 0x4047),
        read_word(ALT_ACC_CONF, 0x7208),
        read_word(ALT_GYR_CONF, 0x4108),
        write_word(FEATURE_DATA_ADDR, EXT_ST_SELECT),
        read_word(FEATURE_DATA_TX, 0x0003),
        write_word(ACC_CONF, 0x0000),
        write_word(GYR_CONF, 0x0000),
        write_word(FEATURE_IO2, 0x012C),
        write_word(FEATURE_IO_STATUS, 0x0001),
        write_word(FEATURE_CTRL, 0x0001),
        read_word(FEATURE_IO1, 0x0001),
        write_word(ACC_CONF, 0x7029),
        write_word(ALT_ACC_CONF, 0x0000),
        write_word(ALT_GYR_CONF, 0x0000),
        write_word(FEATURE_DATA_ADDR, EXT_ST_SELECT),
        write_word(FEATURE_DATA_TX, SelfTestSelection::Gyroscope.to_word()),
        write_word(CMD, 0x0100),
        read_word(FEATURE_IO1, 0x0055),
        write_word(FEATURE_DATA_ADDR, EXT_ST_RESULT),
        read_word(FEATURE_DATA_TX, 0x0078),
        // restore fails: first write (ACC_CONF restore) returns an error
        I2cTransaction::write(ADDR, vec![ACC_CONF, 0x27, 0x41])
            .with_error(I2cErrorKind::Other),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);
    let mut delay = CheckedDelay::new(&[DelayTransaction::async_delay_ms(10)]);

    let result = block_on(imu.run_self_test(&mut delay, SelfTestSelection::Gyroscope));

    assert!(matches!(result, Err(bmi323_driver::Error::Bus(_))));
    delay.done();
    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_enable_feature_engine_returns_error_when_never_ready() {
    let mut expectations = vec![
        write_word(ACC_CONF, 0x0000),
        write_word(GYR_CONF, 0x0000),
        write_word(FEATURE_IO2, 0x012C),
        write_word(FEATURE_IO_STATUS, 0x0001),
        write_word(FEATURE_CTRL, 0x0001),
    ];
    for _ in 0..32 {
        expectations.push(read_word(FEATURE_IO1, 0x0000));
    }
    expectations.push(read_word(FEATURE_IO1, 0x0002));
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);

    let result = block_on(imu.enable_feature_engine());

    assert!(matches!(
        result,
        Err(bmi323_driver::Error::FeatureEngineNotReady(2))
    ));
    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_run_self_test_accelerometer_selection_times_out_when_feature_never_signals() {
    let mut expectations = vec![
        read_word(ACC_CONF, 0x4127),
        read_word(GYR_CONF, 0x4047),
        read_word(ALT_ACC_CONF, 0x7208),
        read_word(ALT_GYR_CONF, 0x4108),
        write_word(FEATURE_DATA_ADDR, EXT_ST_SELECT),
        read_word(FEATURE_DATA_TX, 0x0003),
        // enable_feature_engine: success on first read
        write_word(ACC_CONF, 0x0000),
        write_word(GYR_CONF, 0x0000),
        write_word(FEATURE_IO2, 0x012C),
        write_word(FEATURE_IO_STATUS, 0x0001),
        write_word(FEATURE_CTRL, 0x0001),
        read_word(FEATURE_IO1, 0x0001),
        // Accelerometer selection: tests_gyroscope() == false → no ACC_CONF write
        write_word(ALT_ACC_CONF, 0x0000),
        write_word(ALT_GYR_CONF, 0x0000),
        write_word(FEATURE_DATA_ADDR, EXT_ST_SELECT),
        write_word(FEATURE_DATA_TX, 0x0001), // Accelerometer.to_word()
        write_word(CMD, 0x0100),             // SELF_TEST
    ];
    // 50 poll iterations: bit 4 never set
    let mut delay_transactions = vec![];
    for _ in 0..50 {
        delay_transactions.push(DelayTransaction::async_delay_ms(10));
        expectations.push(read_word(FEATURE_IO1, 0x0000));
    }
    // restore_self_test_configuration
    expectations.push(write_word(ACC_CONF, 0x4127));
    expectations.push(write_word(GYR_CONF, 0x4047));
    expectations.push(write_word(ALT_ACC_CONF, 0x7208));
    expectations.push(write_word(ALT_GYR_CONF, 0x4108));
    expectations.push(write_word(FEATURE_DATA_ADDR, EXT_ST_SELECT));
    expectations.push(write_word(FEATURE_DATA_TX, 0x0003));

    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);
    let mut delay = CheckedDelay::new(&delay_transactions);

    let result = block_on(imu.run_self_test(&mut delay, SelfTestSelection::Accelerometer));

    assert!(matches!(result, Err(bmi323_driver::Error::SelfTestTimeout)));
    delay.done();
    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn async_i2c_wait_for_interrupt_waits_for_high_then_reads_status() {
    let i2c_expectations = [read_word(INT_STATUS_INT1, 0x0002)]; // any_motion bit set
    let pin_expectations = [PinTransaction::wait_for_state(PinState::High)];
    let i2c = I2cMock::new(&i2c_expectations);
    let mut imu = Bmi323Async::new_i2c(i2c, ADDR);
    let mut pin = PinMock::new(&pin_expectations);

    let status = block_on(imu.wait_for_interrupt(&mut pin, InterruptChannel::Int1)).unwrap();

    assert!(status.any_motion());
    pin.done();
    let mut i2c = imu.destroy();
    i2c.done();
}
