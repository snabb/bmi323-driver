// Compiled twice via Cargo.toml [[test]] entries — once in async mode (default, no feature flag)
// and once with `blocking` feature. The `run!` macro dispatches each driver call to the right form.

#[cfg(not(feature = "blocking"))]
use core::future::Future;
#[cfg(not(feature = "blocking"))]
use core::pin::pin;
#[cfg(not(feature = "blocking"))]
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

use bmi323_driver::{
    AccelConfig, AccelMode, AccelRange, AnyMotionConfig, AverageSamples, Bandwidth, Bmi323,
    EventReportMode, GyroConfig, GyroMode, GyroRange, MotionAxes, OutputDataRate, ReferenceUpdate,
    SelfTestSelection,
};
use embedded_hal_mock::eh1::delay::{CheckedDelay, Transaction as DelayTransaction};
use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};

const CHIP_ID: u8 = 0x00;
const ERR_REG: u8 = 0x01;
const STATUS: u8 = 0x02;
const ACC_DATA_X: u8 = 0x03;
const FEATURE_IO0: u8 = 0x10;
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

const EXT_GEN_SET_1: u16 = 0x02;
const EXT_ANYMO_1: u16 = 0x05;
const EXT_ANYMO_2: u16 = 0x06;
const EXT_ANYMO_3: u16 = 0x07;
const EXT_ST_RESULT: u16 = 0x24;
const EXT_ST_SELECT: u16 = 0x25;

const BMI323_CHIP_ID: u16 = 0x0043;

/// Minimal executor for driving futures in these unit tests; the mock HAL
/// operations always complete immediately so a no-op waker is sufficient.
#[cfg(not(feature = "blocking"))]
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

/// Drive `expr` to completion: evaluates directly in blocking mode,
/// or runs through `block_on` in async mode.
macro_rules! run {
    ($expr:expr) => {{
        #[cfg(feature = "blocking")]
        let __result = $expr;
        #[cfg(not(feature = "blocking"))]
        let __result = block_on($expr);
        __result
    }};
}

/// Build a `DelayTransaction` for the active feature mode.
macro_rules! delay_tx {
    (ms, $n:expr) => {{
        #[cfg(feature = "blocking")]
        let __tx = DelayTransaction::delay_ms($n);
        #[cfg(not(feature = "blocking"))]
        let __tx = DelayTransaction::async_delay_ms($n);
        __tx
    }};
    (us, $n:expr) => {{
        #[cfg(feature = "blocking")]
        let __tx = DelayTransaction::delay_us($n);
        #[cfg(not(feature = "blocking"))]
        let __tx = DelayTransaction::async_delay_us($n);
        __tx
    }};
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
fn spi_init_performs_dummy_chip_id_read_then_reads_device_state() {
    let mut expectations = Vec::new();
    expectations.extend(spi_write(&[CMD, 0xAF, 0xDE]));
    expectations.extend(spi_read_word(CHIP_ID, BMI323_CHIP_ID));
    expectations.extend(spi_read_word(CHIP_ID, BMI323_CHIP_ID));
    expectations.extend(spi_read_word(ERR_REG, 0x0000));
    expectations.extend(spi_read_word(STATUS, 0x00E1));

    let spi = SpiMock::new(&expectations);
    let mut imu = Bmi323::new_spi(spi);
    // SPI init: 2ms after soft-reset + 250µs after the dummy chip-id read
    let mut delay = CheckedDelay::new(&[delay_tx!(ms, 2), delay_tx!(us, 250)]);

    let state = run!(imu.init(&mut delay)).unwrap();

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
fn spi_config_writes_use_expected_payload_format() {
    let accel = AccelConfig {
        mode: AccelMode::HighPerformance,
        average: AverageSamples::Avg2,
        bandwidth: Bandwidth::OdrOver4,
        range: AccelRange::G4,
        odr: OutputDataRate::Hz400,
    };
    let gyro = GyroConfig {
        mode: GyroMode::Normal,
        average: AverageSamples::Avg4,
        bandwidth: Bandwidth::OdrOver2,
        range: GyroRange::Dps125,
        odr: OutputDataRate::Hz200,
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

    let spi = SpiMock::new(&expectations);
    let mut imu = Bmi323::new_spi(spi);

    run!(imu.set_accel_config(accel)).unwrap();
    run!(imu.set_gyro_config(gyro)).unwrap();

    assert_eq!(imu.accel_range(), AccelRange::G4);
    assert_eq!(imu.gyro_range(), GyroRange::Dps125);

    let mut spi = imu.destroy();
    spi.done();
}

#[test]
fn spi_burst_read_decodes_dummy_byte_framing_correctly() {
    let mut expectations = Vec::new();
    expectations.extend(spi_read_words(
        ACC_DATA_X,
        &[0x1234, 0xFEDC, 0x8001, 0x0002, 0x7FFF, 0xFF00],
    ));

    let spi = SpiMock::new(&expectations);
    let mut imu = Bmi323::new_spi(spi);

    let sample = run!(imu.read_imu_data()).unwrap();

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
fn spi_feature_engine_enable_uses_expected_transaction_sequence() {
    let mut expectations = Vec::new();
    expectations.extend(spi_write(&[ACC_CONF, 0x00, 0x00]));
    expectations.extend(spi_write(&[GYR_CONF, 0x00, 0x00]));
    expectations.extend(spi_write(&[FEATURE_IO2, 0x2C, 0x01]));
    expectations.extend(spi_write(&[FEATURE_IO_STATUS, 0x01, 0x00]));
    expectations.extend(spi_write(&[FEATURE_CTRL, 0x01, 0x00]));
    // Returns NO_ERROR (0x05) on first poll.
    expectations.extend(spi_read_word(FEATURE_IO1, 0x0005));

    let spi = SpiMock::new(&expectations);
    let mut imu = Bmi323::new_spi(spi);
    // The polling loop delays 200 µs before each read; returns on first poll.
    let mut delay = CheckedDelay::new(&[delay_tx!(us, 200)]);

    run!(imu.enable_feature_engine(&mut delay)).unwrap();

    delay.done();
    let mut spi = imu.destroy();
    spi.done();
}

#[test]
fn spi_any_motion_configuration_uses_expected_feature_transactions() {
    let config = AnyMotionConfig {
        axes: MotionAxes::XYZ,
        threshold: 0x0028,
        hysteresis: 0x0002,
        duration: 5,
        wait_time: 1,
        reference_update: ReferenceUpdate::OnDetection,
        report_mode: EventReportMode::FirstEventOnly,
        interrupt_hold: 4,
    };

    let mut expectations = Vec::new();
    expectations.extend(spi_write(&[FEATURE_DATA_ADDR, EXT_GEN_SET_1 as u8, 0x00]));
    expectations.extend(spi_read_word(FEATURE_DATA_TX, 0x0020));
    expectations.extend(spi_write(&[FEATURE_DATA_ADDR, EXT_GEN_SET_1 as u8, 0x00]));
    expectations.extend(spi_write(&[FEATURE_DATA_TX, 0x09, 0x00]));
    expectations.extend(spi_write(&[FEATURE_DATA_ADDR, EXT_ANYMO_1 as u8, 0x00]));
    expectations.extend(spi_write(&[FEATURE_DATA_TX, 0x28, 0x00]));
    expectations.extend(spi_write(&[FEATURE_DATA_ADDR, EXT_ANYMO_2 as u8, 0x00]));
    expectations.extend(spi_write(&[FEATURE_DATA_TX, 0x02, 0x00]));
    expectations.extend(spi_write(&[FEATURE_DATA_ADDR, EXT_ANYMO_3 as u8, 0x00]));
    expectations.extend(spi_write(&[FEATURE_DATA_TX, 0x05, 0x20]));
    expectations.extend(spi_read_word(FEATURE_IO0, 0x0000));
    expectations.extend(spi_write(&[FEATURE_IO0, 0x38, 0x00]));
    expectations.extend(spi_write(&[FEATURE_IO_STATUS, 0x01, 0x00]));

    let spi = SpiMock::new(&expectations);
    let mut imu = Bmi323::new_spi(spi);

    run!(imu.configure_any_motion(config)).unwrap();

    let mut spi = imu.destroy();
    spi.done();
}

#[test]
fn spi_self_test_uses_expected_sequence() {
    let mut expectations = Vec::new();
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
    expectations.extend(spi_write(&[FEATURE_DATA_TX, 0x03, 0x00]));
    expectations.extend(spi_write(&[CMD, 0x00, 0x01]));
    // Self-test result ready on first poll → no 10ms poll delay.
    expectations.extend(spi_read_word(FEATURE_IO1, 0x0055));
    expectations.extend(spi_write(&[FEATURE_DATA_ADDR, EXT_ST_RESULT as u8, 0x00]));
    expectations.extend(spi_read_word(FEATURE_DATA_TX, 0x007F));

    let spi = SpiMock::new(&expectations);
    let mut imu = Bmi323::new_spi(spi);
    // enable_feature_engine: 1 poll → 1×200µs. Self-test ready on first poll → no ms delay.
    let mut delay = CheckedDelay::new(&[delay_tx!(us, 200)]);

    let result = run!(imu.run_self_test(&mut delay, SelfTestSelection::Both)).unwrap();

    assert!(result.passed);
    assert!(result.accelerometer_ok());
    assert!(result.gyroscope_ok());

    delay.done();
    let mut spi = imu.destroy();
    spi.done();
}
