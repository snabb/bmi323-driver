use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

use bmi323_driver::{Bmi323Async, SelfTestSelection};
use embedded_hal_mock::eh1::delay::{CheckedDelay, Transaction as DelayTransaction};
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

const ADDR: u8 = 0x68;

const CHIP_ID: u8 = 0x00;
const ERR_REG: u8 = 0x01;
const STATUS: u8 = 0x02;
const ACC_DATA_X: u8 = 0x03;
const SENSOR_TIME_0: u8 = 0x0A;
const FEATURE_IO1: u8 = 0x11;
const FEATURE_IO2: u8 = 0x12;
const FEATURE_IO3: u8 = 0x13;
const FEATURE_IO_STATUS: u8 = 0x14;
const ACC_CONF: u8 = 0x20;
const GYR_CONF: u8 = 0x21;
const ALT_ACC_CONF: u8 = 0x28;
const ALT_GYR_CONF: u8 = 0x29;
const FEATURE_CTRL: u8 = 0x40;
const FEATURE_DATA_ADDR: u8 = 0x41;
const FEATURE_DATA_TX: u8 = 0x42;
const CMD: u8 = 0x7E;

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
