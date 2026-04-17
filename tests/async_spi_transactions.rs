use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

use bmi323_driver::{
    AccelConfig, AccelMode, AccelRange, Bmi323Async, GyroConfig, GyroMode, GyroRange,
    OutputDataRate, SelfTestSelection,
};
use embedded_hal_mock::eh1::delay::{CheckedDelay, Transaction as DelayTransaction};
use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};

const CHIP_ID: u8 = 0x00;
const ERR_REG: u8 = 0x01;
const STATUS: u8 = 0x02;
const ACC_DATA_X: u8 = 0x03;
const FEATURE_IO1: u8 = 0x11;
const FEATURE_IO2: u8 = 0x12;
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

fn spi_write(payload: &[u8]) -> Vec<SpiTransaction<u8>> {
    vec![
        SpiTransaction::transaction_start(),
        SpiTransaction::write_vec(payload.to_vec()),
        SpiTransaction::transaction_end(),
    ]
}

fn spi_read_word(reg: u8, word: u16) -> Vec<SpiTransaction<u8>> {
    let [lo, hi] = word.to_le_bytes();
    vec![
        SpiTransaction::transaction_start(),
        SpiTransaction::write(0x80 | reg),
        SpiTransaction::read_vec(vec![0, lo, hi]),
        SpiTransaction::transaction_end(),
    ]
}

fn spi_read_words(reg: u8, words: &[u16]) -> Vec<SpiTransaction<u8>> {
    let mut response = vec![0];
    for word in words {
        let [lo, hi] = word.to_le_bytes();
        response.push(lo);
        response.push(hi);
    }
    vec![
        SpiTransaction::transaction_start(),
        SpiTransaction::write(0x80 | reg),
        SpiTransaction::read_vec(response),
        SpiTransaction::transaction_end(),
    ]
}

#[test]
fn async_spi_init_performs_dummy_chip_id_read_then_reads_device_state() {
    let mut expectations = Vec::new();
    expectations.extend(spi_write(&[CMD, 0xAF, 0xDE]));
    expectations.extend(spi_read_word(CHIP_ID, BMI323_CHIP_ID));
    expectations.extend(spi_read_word(CHIP_ID, BMI323_CHIP_ID));
    expectations.extend(spi_read_word(ERR_REG, 0x0000));
    expectations.extend(spi_read_word(STATUS, 0x00E1));

    let spi = SpiMock::new(&expectations);
    let mut imu = Bmi323Async::new_spi(spi);
    let mut delay = CheckedDelay::new(&[
        DelayTransaction::async_delay_ms(2),
        DelayTransaction::async_delay_us(250),
    ]);

    let state = block_on(imu.init(&mut delay)).unwrap();

    assert_eq!(state.chip_id, BMI323_CHIP_ID as u8);
    assert!(state.status.por_detected());
    assert!(state.status.drdy_temp());
    assert!(state.status.drdy_gyro());
    assert!(state.status.drdy_accel());
    assert!(!state.error.fatal());

    delay.done();
    let mut spi = imu.destroy();
    spi.done();
}

#[test]
fn async_spi_config_and_burst_read_use_expected_payload_format() {
    let accel = AccelConfig {
        mode: AccelMode::HighPerformance,
        range: AccelRange::G4,
        odr: OutputDataRate::Hz400,
        ..Default::default()
    };
    let gyro = GyroConfig {
        mode: GyroMode::Normal,
        range: GyroRange::Dps125,
        odr: OutputDataRate::Hz200,
        ..Default::default()
    };

    let mut expectations = Vec::new();
    expectations.extend(spi_write(&[
        ACC_CONF,
        accel.to_word() as u8,
        (accel.to_word() >> 8) as u8,
    ]));
    expectations.extend(spi_write(&[
        GYR_CONF,
        gyro.to_word() as u8,
        (gyro.to_word() >> 8) as u8,
    ]));
    expectations.extend(spi_read_words(
        ACC_DATA_X,
        &[0x1234, 0xFEDC, 0x8001, 0x0002, 0x7FFF, 0xFF00],
    ));

    let spi = SpiMock::new(&expectations);
    let mut imu = Bmi323Async::new_spi(spi);

    block_on(imu.set_accel_config(accel)).unwrap();
    block_on(imu.set_gyro_config(gyro)).unwrap();
    let sample = block_on(imu.read_imu_data()).unwrap();

    assert_eq!(sample.accel.x, 0x1234);
    assert_eq!(sample.accel.y, -292);
    assert_eq!(sample.accel.z, -32767);
    assert_eq!(sample.gyro.x, 2);
    assert_eq!(sample.gyro.y, 32767);
    assert_eq!(sample.gyro.z, -256);

    let mut spi = imu.destroy();
    spi.done();
}

#[test]
fn async_spi_self_test_uses_expected_sequence_and_restores_configuration() {
    let mut expectations = Vec::new();
    expectations.extend(spi_read_word(ACC_CONF, 0x4127));
    expectations.extend(spi_read_word(GYR_CONF, 0x4047));
    expectations.extend(spi_read_word(ALT_ACC_CONF, 0x7208));
    expectations.extend(spi_read_word(ALT_GYR_CONF, 0x4108));
    expectations.extend(spi_write(&[FEATURE_DATA_ADDR, EXT_ST_SELECT as u8, 0x00]));
    expectations.extend(spi_read_word(FEATURE_DATA_TX, 0x0003));
    expectations.extend(spi_write(&[ACC_CONF, 0x00, 0x00]));
    expectations.extend(spi_write(&[GYR_CONF, 0x00, 0x00]));
    expectations.extend(spi_write(&[FEATURE_IO2, 0x2C, 0x01]));
    expectations.extend(spi_write(&[FEATURE_IO_STATUS, 0x01, 0x00]));
    expectations.extend(spi_write(&[FEATURE_CTRL, 0x01, 0x00]));
    expectations.extend(spi_read_word(FEATURE_IO1, 0x0001));
    expectations.extend(spi_write(&[ACC_CONF, 0x29, 0x70]));
    expectations.extend(spi_write(&[ALT_ACC_CONF, 0x00, 0x00]));
    expectations.extend(spi_write(&[ALT_GYR_CONF, 0x00, 0x00]));
    expectations.extend(spi_write(&[FEATURE_DATA_ADDR, EXT_ST_SELECT as u8, 0x00]));
    expectations.extend(spi_write(&[FEATURE_DATA_TX, 0x02, 0x00]));
    expectations.extend(spi_write(&[CMD, 0x00, 0x01]));
    expectations.extend(spi_read_word(FEATURE_IO1, 0x0055));
    expectations.extend(spi_write(&[FEATURE_DATA_ADDR, EXT_ST_RESULT as u8, 0x00]));
    expectations.extend(spi_read_word(FEATURE_DATA_TX, 0x0078));
    expectations.extend(spi_write(&[ACC_CONF, 0x27, 0x41]));
    expectations.extend(spi_write(&[GYR_CONF, 0x47, 0x40]));
    expectations.extend(spi_write(&[ALT_ACC_CONF, 0x08, 0x72]));
    expectations.extend(spi_write(&[ALT_GYR_CONF, 0x08, 0x41]));
    expectations.extend(spi_write(&[FEATURE_DATA_ADDR, EXT_ST_SELECT as u8, 0x00]));
    expectations.extend(spi_write(&[FEATURE_DATA_TX, 0x03, 0x00]));

    let spi = SpiMock::new(&expectations);
    let mut imu = Bmi323Async::new_spi(spi);
    let mut delay = CheckedDelay::new(&[DelayTransaction::async_delay_ms(10)]);

    let result = block_on(imu.run_self_test(&mut delay, SelfTestSelection::Gyroscope)).unwrap();

    assert!(result.passed);
    assert!(!result.sample_rate_error);
    assert_eq!(result.error_status, 5);
    assert!(result.gyroscope_ok());

    delay.done();
    let mut spi = imu.destroy();
    spi.done();
}
