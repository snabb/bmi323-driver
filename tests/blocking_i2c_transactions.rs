use bmi323_driver::{
    AccelConfig, AccelMode, AccelRange, ActiveLevel, AltConfigControl, AltConfigSwitchSource,
    AnyMotionConfig, AverageSamples, Bandwidth, Bmi323, EventReportMode, GyroConfig, GyroMode,
    GyroRange, InterruptChannel, InterruptPinConfig, InterruptRoute, InterruptSource, MotionAxes,
    NoMotionConfig, OutputDataRate, OutputMode, ReferenceUpdate, SelfTestSelection,
};
use embedded_hal::delay::DelayNs;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

const ADDR: u8 = 0x68;

const CHIP_ID: u8 = 0x00;
const ERR_REG: u8 = 0x01;
const STATUS: u8 = 0x02;
const ACC_DATA_X: u8 = 0x03;
const SENSOR_TIME_0: u8 = 0x0A;
const FEATURE_IO0: u8 = 0x10;
const FEATURE_IO1: u8 = 0x11;
const FEATURE_IO2: u8 = 0x12;
const FEATURE_IO3: u8 = 0x13;
const FEATURE_IO_STATUS: u8 = 0x14;
const ACC_CONF: u8 = 0x20;
const GYR_CONF: u8 = 0x21;
const ALT_ACC_CONF: u8 = 0x28;
const ALT_GYR_CONF: u8 = 0x29;
const ALT_CONF: u8 = 0x2A;
const IO_INT_CTRL: u8 = 0x38;
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
const EXT_ALT_CONFIG_CHG: u16 = 0x23;
const EXT_ST_RESULT: u16 = 0x24;
const EXT_ST_SELECT: u16 = 0x25;

const BMI323_CHIP_ID: u16 = 0x0043;
const SOFT_RESET: u16 = 0xDEAF;

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

#[derive(Default, Debug)]
struct TestDelay {
    ms_calls: Vec<u32>,
    us_calls: Vec<u32>,
    ns_calls: Vec<u32>,
}

impl DelayNs for TestDelay {
    fn delay_ns(&mut self, ns: u32) {
        self.ns_calls.push(ns);
    }

    fn delay_us(&mut self, us: u32) {
        self.us_calls.push(us);
    }

    fn delay_ms(&mut self, ms: u32) {
        self.ms_calls.push(ms);
    }
}

#[test]
fn init_resets_sensor_and_reads_device_state() {
    let expectations = [
        write_word(CMD, SOFT_RESET),
        read_word(CHIP_ID, BMI323_CHIP_ID),
        read_word(ERR_REG, 0x0000),
        read_word(STATUS, 0x00E1),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323::new_i2c(i2c, ADDR);
    let mut delay = TestDelay::default();

    let state = imu.init(&mut delay).unwrap();

    assert_eq!(state.chip_id, BMI323_CHIP_ID as u8);
    assert!(state.status.por_detected());
    assert!(state.status.drdy_temp());
    assert!(state.status.drdy_gyro());
    assert!(state.status.drdy_accel());
    assert!(!state.error.fatal());
    assert_eq!(delay.ms_calls, vec![2]);
    assert!(delay.us_calls.is_empty());
    assert!(delay.ns_calls.is_empty());

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn set_sensor_configs_writes_expected_words_and_tracks_ranges() {
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
    let mut imu = Bmi323::new_i2c(i2c, ADDR);

    imu.set_accel_config(accel).unwrap();
    imu.set_gyro_config(gyro).unwrap();

    assert_eq!(imu.accel_range(), AccelRange::G16);
    assert_eq!(imu.gyro_range(), GyroRange::Dps500);

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn enable_feature_engine_uses_expected_register_sequence() {
    let expectations = [
        write_word(ACC_CONF, 0x0000),
        write_word(GYR_CONF, 0x0000),
        write_word(FEATURE_IO2, 0x012C),
        write_word(FEATURE_IO_STATUS, 0x0001),
        write_word(FEATURE_CTRL, 0x0001),
        read_word(FEATURE_IO1, 0x0001),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Bmi323::new_i2c(i2c, ADDR);

    imu.enable_feature_engine().unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn configure_any_motion_and_interrupt_routing_programs_expected_registers() {
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
    let mut imu = Bmi323::new_i2c(i2c, ADDR);

    imu.configure_any_motion(config).unwrap();
    imu.configure_interrupt_pin(
        InterruptChannel::Int1,
        InterruptPinConfig {
            active_level: ActiveLevel::High,
            output_mode: OutputMode::OpenDrain,
            enabled: true,
        },
    )
    .unwrap();
    imu.map_interrupt(InterruptSource::AnyMotion, InterruptRoute::Int1)
        .unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn configure_no_motion_programs_expected_registers() {
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
    let mut imu = Bmi323::new_i2c(i2c, ADDR);

    imu.configure_no_motion(config).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn read_paths_decode_burst_and_feature_words_correctly() {
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
    let mut imu = Bmi323::new_i2c(i2c, ADDR);

    let imu_data = imu.read_imu_data().unwrap();
    let sensor_time = imu.read_sensor_time().unwrap();
    let step_count = imu.read_step_count().unwrap();

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
fn configure_alt_config_control_writes_sensor_and_feature_words() {
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
    let mut imu = Bmi323::new_i2c(i2c, ADDR);

    imu.configure_alt_config_control(control).unwrap();

    let mut i2c = imu.destroy();
    i2c.done();
}

#[test]
fn run_self_test_uses_expected_i2c_sequence_and_restores_configuration() {
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
    let mut imu = Bmi323::new_i2c(i2c, ADDR);
    let mut delay = TestDelay::default();

    let result = imu
        .run_self_test(&mut delay, SelfTestSelection::Gyroscope)
        .unwrap();

    assert!(result.passed);
    assert!(!result.sample_rate_error);
    assert_eq!(result.error_status, 5);
    assert!(result.gyroscope_ok());
    assert!(delay.ms_calls.iter().any(|&ms| ms == 10));

    let mut i2c = imu.destroy();
    i2c.done();
}
