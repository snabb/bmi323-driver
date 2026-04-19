use crate::registers::{INT_MAP1, INT_MAP2, interrupt_map_location};
use crate::*;

#[test]
fn accel_config_matches_datasheet_example() {
    let config = AccelConfig {
        mode: AccelMode::LowPower,
        average: AverageSamples::Avg2,
        bandwidth: Bandwidth::OdrOver2,
        range: AccelRange::G8,
        odr: OutputDataRate::Hz50,
    };
    assert_eq!(config.to_word(), 0x3127);
}

#[test]
fn gyro_config_matches_datasheet_example() {
    let config = GyroConfig {
        mode: GyroMode::Normal,
        average: AverageSamples::Avg1,
        bandwidth: Bandwidth::OdrOver2,
        range: GyroRange::Dps2000,
        odr: OutputDataRate::Hz800,
    };
    assert_eq!(config.to_word(), 0x404B);
}

#[test]
fn interrupt_map_location_is_stable() {
    assert_eq!(
        interrupt_map_location(InterruptSource::AnyMotion),
        (INT_MAP1, 2)
    );
    assert_eq!(
        interrupt_map_location(InterruptSource::AccelDataReady),
        (INT_MAP2, 10)
    );
}

#[test]
fn step_counter_config_encodes_watermark_and_reset() {
    let config = StepCounterConfig {
        watermark_level: 37,
        reset_counter: true,
    };
    assert_eq!(config.to_word(), 37 | (1 << 10));
}

#[test]
fn step_counter_config_with_watermark_saturates() {
    assert_eq!(
        StepCounterConfig::with_watermark(2048).watermark_level,
        1023
    );
}

#[test]
fn tap_duration_helpers_match_datasheet_scaling() {
    assert_eq!(TapConfig::max_gesture_duration_from_seconds(0.64), 16);
    assert!((TapConfig::short_duration_to_seconds(6) - 0.03).abs() < 0.001);
}

#[test]
fn significant_motion_helpers_match_datasheet_scaling() {
    assert_eq!(SignificantMotionConfig::block_size_from_seconds(5.0), 250);
    assert!(
        (SignificantMotionConfig::peak_to_peak_to_g(SignificantMotionConfig::peak_to_peak_from_g(
            0.5
        )) - 0.5)
            .abs()
            < 0.01
    );
}

#[test]
fn tilt_segment_size_helper_matches_datasheet_scaling() {
    assert_eq!(TiltConfig::segment_size_from_seconds(2.0), 100);
    assert!((TiltConfig::segment_size_to_seconds(100) - 2.0).abs() < 0.001);
}

#[test]
fn alt_accel_config_encodes_expected_fields() {
    let config = AltAccelConfig {
        mode: AccelMode::HighPerformance,
        average: AverageSamples::Avg4,
        odr: OutputDataRate::Hz100,
    };
    assert_eq!(config.to_word(), 0x7208);
}

#[test]
fn alt_status_decodes_active_flags() {
    let status = AltStatus(0x0011);
    assert!(status.accel_uses_alternate());
    assert!(status.gyro_uses_alternate());
}

#[test]
fn alt_accel_switch_profile_builds_expected_defaults() {
    let profile = AltAccelSwitchProfile::low_power_to_high_performance(
        OutputDataRate::Hz25,
        OutputDataRate::Hz200,
        AltConfigSwitchSource::AnyMotion,
        AltConfigSwitchSource::NoMotion,
    );
    assert_eq!(profile.user_accel.mode, AccelMode::LowPower);
    assert_eq!(profile.user_accel.average, AverageSamples::Avg2);
    assert_eq!(profile.alternate_accel.mode, AccelMode::HighPerformance);
    assert!(profile.control.accel_enabled);
    assert!(!profile.control.gyro_enabled);
}

#[test]
fn self_test_selection_encodes_expected_bits() {
    assert_eq!(SelfTestSelection::Accelerometer.to_word(), 0x0001);
    assert_eq!(SelfTestSelection::Gyroscope.to_word(), 0x0002);
    assert_eq!(SelfTestSelection::Both.to_word(), 0x0003);
}

#[test]
fn self_test_detail_decodes_expected_checks() {
    let detail = SelfTestDetail(0x007F);
    assert!(detail.accelerometer_ok());
    assert!(detail.gyroscope_ok());
}

// Datasheet p.89: G2 → 16.38 LSB/mg = 2.0/32768.0 g/LSB
#[test]
fn accel_range_scale_matches_datasheet() {
    let tol = 1e-9_f32;
    assert!((AccelRange::G2.scale_g_per_lsb() - 2.0 / 32768.0).abs() < tol);
    assert!((AccelRange::G4.scale_g_per_lsb() - 4.0 / 32768.0).abs() < tol);
    assert!((AccelRange::G8.scale_g_per_lsb() - 8.0 / 32768.0).abs() < tol);
    assert!((AccelRange::G16.scale_g_per_lsb() - 16.0 / 32768.0).abs() < tol);
}

// Datasheet p.91: Dps125 → 262.144 LSB/°/s = 125.0/32768.0 dps/LSB
#[test]
fn gyro_range_scale_matches_datasheet() {
    let tol = 1e-9_f32;
    assert!((GyroRange::Dps125.scale_dps_per_lsb() - 125.0 / 32768.0).abs() < tol);
    assert!((GyroRange::Dps250.scale_dps_per_lsb() - 250.0 / 32768.0).abs() < tol);
    assert!((GyroRange::Dps500.scale_dps_per_lsb() - 500.0 / 32768.0).abs() < tol);
    assert!((GyroRange::Dps1000.scale_dps_per_lsb() - 1000.0 / 32768.0).abs() < tol);
    assert!((GyroRange::Dps2000.scale_dps_per_lsb() - 2000.0 / 32768.0).abs() < tol);
}

#[test]
fn axis_data_as_g_applies_accel_scale() {
    // 32767 LSB with G2 range ≈ +2.0 g (within 1 LSB)
    let data = AxisData {
        x: 32767,
        y: -32768,
        z: 16384,
    };
    let g = data.as_g(AccelRange::G2);
    let scale = 2.0_f32 / 32768.0;
    assert!((g[0] - 32767.0 * scale).abs() < 1e-6);
    assert!((g[1] - (-32768.0) * scale).abs() < 1e-6);
    assert!((g[2] - 16384.0 * scale).abs() < 1e-6);
}

#[test]
fn axis_data_as_dps_applies_gyro_scale() {
    let data = AxisData {
        x: 32767,
        y: -32768,
        z: 16384,
    };
    let dps = data.as_dps(GyroRange::Dps2000);
    let scale = 2000.0_f32 / 32768.0;
    assert!((dps[0] - 32767.0 * scale).abs() < 1e-6);
    assert!((dps[1] - (-32768.0) * scale).abs() < 1e-6);
    assert!((dps[2] - 16384.0 * scale).abs() < 1e-6);
}

// AnyMotion/NoMotion: threshold & hysteresis raw/512 g, duration & wait_time raw/50 s
#[test]
fn any_motion_threshold_scaling_matches_datasheet() {
    // 1 g → 512 raw
    assert_eq!(AnyMotionConfig::threshold_from_g(1.0), 512);
    assert!((AnyMotionConfig::threshold_to_g(512) - 1.0).abs() < 1e-4);
    // clamp at 4095
    assert_eq!(AnyMotionConfig::threshold_from_g(100.0), 4095);
    assert!((AnyMotionConfig::threshold_to_g(0) - 0.0).abs() < 1e-6);
}

#[test]
fn any_motion_hysteresis_scaling_matches_datasheet() {
    assert_eq!(AnyMotionConfig::hysteresis_from_g(0.5), 256);
    assert!((AnyMotionConfig::hysteresis_to_g(256) - 0.5).abs() < 1e-4);
    // clamp at 1023
    assert_eq!(AnyMotionConfig::hysteresis_from_g(100.0), 1023);
}

#[test]
fn any_motion_duration_scaling_matches_datasheet() {
    // 1 second → 50 raw
    assert_eq!(AnyMotionConfig::duration_from_seconds(1.0), 50);
    assert!((AnyMotionConfig::duration_to_seconds(50) - 1.0).abs() < 1e-4);
    // clamp at 8191
    assert_eq!(AnyMotionConfig::duration_from_seconds(10000.0), 8191);
}

#[test]
fn any_motion_wait_time_scaling_matches_datasheet() {
    // 0.14 s → 50*0.14=7 raw (max 3-bit value)
    assert_eq!(AnyMotionConfig::wait_time_from_seconds(0.14), 7);
    assert!((AnyMotionConfig::wait_time_to_seconds(4) - 4.0 / 50.0).abs() < 1e-4);
    // clamp at 7
    assert_eq!(AnyMotionConfig::wait_time_from_seconds(1000.0), 7);
}

// interrupt_hold: 0.625 ms * 2^n, smallest n so duration >= millis, max n=13
#[test]
fn any_motion_interrupt_hold_scaling_matches_datasheet() {
    // 0.625 ms → n=0
    assert_eq!(AnyMotionConfig::interrupt_hold_from_millis(0.625), 0);
    assert!((AnyMotionConfig::interrupt_hold_to_millis(0) - 0.625).abs() < 1e-4);
    // 1.25 ms → n=1
    assert_eq!(AnyMotionConfig::interrupt_hold_from_millis(1.25), 1);
    assert!((AnyMotionConfig::interrupt_hold_to_millis(1) - 1.25).abs() < 1e-4);
    // 10 ms: 0.625*2^4=10.0 → n=4
    assert_eq!(AnyMotionConfig::interrupt_hold_from_millis(10.0), 4);
    assert!((AnyMotionConfig::interrupt_hold_to_millis(4) - 10.0).abs() < 1e-4);
    // clamp at 13
    assert_eq!(AnyMotionConfig::interrupt_hold_from_millis(99999.0), 13);
    assert!((AnyMotionConfig::interrupt_hold_to_millis(13) - 0.625 * 8192.0).abs() < 0.01);
}

// NoMotion delegates to AnyMotion helpers — spot-check one each
#[test]
fn no_motion_helpers_delegate_correctly() {
    assert_eq!(NoMotionConfig::threshold_from_g(1.0), 512);
    assert!((NoMotionConfig::threshold_to_g(512) - 1.0).abs() < 1e-4);
    assert_eq!(NoMotionConfig::hysteresis_from_g(0.5), 256);
    assert_eq!(NoMotionConfig::duration_from_seconds(1.0), 50);
    assert!((NoMotionConfig::duration_to_seconds(50) - 1.0).abs() < 1e-4);
    assert_eq!(NoMotionConfig::wait_time_from_seconds(0.14), 7);
    assert_eq!(NoMotionConfig::interrupt_hold_from_millis(10.0), 4);
}

// FlatConfig: hold_time raw/50 s, slope_threshold raw/512 g
#[test]
fn flat_config_hold_time_scaling_matches_datasheet() {
    assert_eq!(FlatConfig::hold_time_from_seconds(1.0), 50);
    assert!((FlatConfig::hold_time_to_seconds(50) - 1.0).abs() < 1e-4);
    // clamp at 255
    assert_eq!(FlatConfig::hold_time_from_seconds(1000.0), 255);
    assert!((FlatConfig::hold_time_to_seconds(0) - 0.0).abs() < 1e-6);
}

#[test]
fn flat_config_slope_threshold_scaling_matches_datasheet() {
    // 0.25 g → 128 raw
    assert_eq!(FlatConfig::slope_threshold_from_g(0.25), 128);
    assert!((FlatConfig::slope_threshold_to_g(128) - 0.25).abs() < 1e-4);
    // clamp at 255
    assert_eq!(FlatConfig::slope_threshold_from_g(100.0), 255);
}

// OrientationConfig: hold_time raw/50 s (5-bit, max 31), hysteresis raw/512 g
#[test]
fn orientation_config_hold_time_scaling_matches_datasheet() {
    assert_eq!(OrientationConfig::hold_time_from_seconds(0.5), 25);
    assert!((OrientationConfig::hold_time_to_seconds(25) - 0.5).abs() < 1e-4);
    // clamp at 31 (5-bit field)
    assert_eq!(OrientationConfig::hold_time_from_seconds(1000.0), 31);
}

#[test]
fn orientation_config_hysteresis_scaling_matches_datasheet() {
    // 0.25 g → 128 raw
    assert_eq!(OrientationConfig::hysteresis_from_g(0.25), 128);
    assert!((OrientationConfig::hysteresis_to_g(128) - 0.25).abs() < 1e-4);
    // clamp at 255
    assert_eq!(OrientationConfig::hysteresis_from_g(100.0), 255);
}

// TapConfig: tap_peak_threshold raw/512 g, quiet_time_after_gesture raw/25 s
#[test]
fn tap_config_peak_threshold_scaling_matches_datasheet() {
    assert_eq!(TapConfig::tap_peak_threshold_from_g(1.0), 512);
    assert!((TapConfig::tap_peak_threshold_to_g(512) - 1.0).abs() < 1e-4);
    // clamp at 1023
    assert_eq!(TapConfig::tap_peak_threshold_from_g(100.0), 1023);
}

#[test]
fn tap_config_quiet_time_scaling_matches_datasheet() {
    // 0.4 s → 0.4*25=10 raw
    assert_eq!(TapConfig::quiet_time_after_gesture_from_seconds(0.4), 10);
    assert!((TapConfig::quiet_time_after_gesture_to_seconds(10) - 0.4).abs() < 1e-4);
    // clamp at 15 (4-bit field)
    assert_eq!(TapConfig::quiet_time_after_gesture_from_seconds(1000.0), 15);
}

// AltGyroConfig::to_word encodes mode<<12 | average<<8 | odr
#[test]
fn alt_gyro_config_encodes_expected_fields() {
    let config = AltGyroConfig {
        mode: GyroMode::Normal,
        average: AverageSamples::Avg1,
        odr: OutputDataRate::Hz100,
    };
    // Normal=4, Avg1=0, Hz100=8 → 0x4008
    assert_eq!(config.to_word(), 0x4008);
}

// SignificantMotion: skip block_size (tested above), verify peak_to_peak with 0-boundary
#[test]
fn significant_motion_peak_to_peak_zero_roundtrip() {
    assert_eq!(SignificantMotionConfig::peak_to_peak_from_g(0.0), 0);
    assert!((SignificantMotionConfig::peak_to_peak_to_g(0) - 0.0).abs() < 1e-6);
}

// TiltConfig: spot-check zero boundary
#[test]
fn tilt_segment_size_zero_boundary() {
    assert_eq!(TiltConfig::segment_size_from_seconds(0.0), 0);
    assert!((TiltConfig::segment_size_to_seconds(0) - 0.0).abs() < 1e-6);
}

#[test]
fn error_word_conf_error_flags_decode_correctly() {
    let accel_err = ErrorWord(1 << 5);
    assert!(accel_err.accel_conf_error());
    assert!(!accel_err.gyro_conf_error());
    let gyro_err = ErrorWord(1 << 6);
    assert!(!gyro_err.accel_conf_error());
    assert!(gyro_err.gyro_conf_error());
    assert!(!ErrorWord(0).accel_conf_error());
    assert!(!ErrorWord(0).gyro_conf_error());
}

#[test]
fn interrupt_status_decodes_all_bits() {
    assert!(InterruptStatus(1 << 0).no_motion());
    assert!(InterruptStatus(1 << 1).any_motion());
    assert!(InterruptStatus(1 << 2).flat());
    assert!(InterruptStatus(1 << 3).orientation());
    assert!(InterruptStatus(1 << 4).step_detector());
    assert!(InterruptStatus(1 << 5).step_counter());
    assert!(InterruptStatus(1 << 7).tilt());
    assert!(InterruptStatus(1 << 8).tap());
    assert!(InterruptStatus(1 << 10).feature_status());
    assert!(InterruptStatus(1 << 11).temp_data_ready());
    assert!(InterruptStatus(1 << 14).fifo_watermark());
    assert!(InterruptStatus(1 << 15).fifo_full());
    let none = InterruptStatus(0);
    assert!(!none.no_motion());
    assert!(!none.any_motion());
    assert!(!none.flat());
    assert!(!none.tilt());
    assert!(!none.tap());
    assert!(!none.fifo_full());
}

#[test]
fn self_test_result_skips_unchecked_component() {
    // Gyroscope-only selection: accelerometer_ok() returns true even if detail bits are 0
    let gyro_only = SelfTestResult {
        selection: SelfTestSelection::Gyroscope,
        passed: false,
        sample_rate_error: false,
        error_status: 0,
        detail: SelfTestDetail(0),
    };
    assert!(gyro_only.accelerometer_ok());
    assert!(!gyro_only.gyroscope_ok());

    // Accelerometer-only selection: gyroscope_ok() returns true even if detail bits are 0
    let accel_only = SelfTestResult {
        selection: SelfTestSelection::Accelerometer,
        passed: false,
        sample_rate_error: false,
        error_status: 0,
        detail: SelfTestDetail(0),
    };
    assert!(!accel_only.accelerometer_ok());
    assert!(accel_only.gyroscope_ok());
}

#[test]
fn interrupt_map_location_covers_remaining_sources() {
    assert_eq!(
        interrupt_map_location(InterruptSource::NoMotion),
        (INT_MAP1, 0)
    );
    assert_eq!(interrupt_map_location(InterruptSource::Flat), (INT_MAP1, 4));
    assert_eq!(
        interrupt_map_location(InterruptSource::Orientation),
        (INT_MAP1, 6)
    );
    assert_eq!(
        interrupt_map_location(InterruptSource::StepDetector),
        (INT_MAP1, 8)
    );
    assert_eq!(
        interrupt_map_location(InterruptSource::StepCounter),
        (INT_MAP1, 10)
    );
    assert_eq!(
        interrupt_map_location(InterruptSource::SignificantMotion),
        (INT_MAP1, 12)
    );
    assert_eq!(
        interrupt_map_location(InterruptSource::Tilt),
        (INT_MAP1, 14)
    );
    assert_eq!(interrupt_map_location(InterruptSource::Tap), (INT_MAP2, 0));
    assert_eq!(
        interrupt_map_location(InterruptSource::I3cSync),
        (INT_MAP2, 2)
    );
    assert_eq!(
        interrupt_map_location(InterruptSource::FeatureStatus),
        (INT_MAP2, 4)
    );
    assert_eq!(
        interrupt_map_location(InterruptSource::TempDataReady),
        (INT_MAP2, 6)
    );
    assert_eq!(
        interrupt_map_location(InterruptSource::GyroDataReady),
        (INT_MAP2, 8)
    );
    assert_eq!(
        interrupt_map_location(InterruptSource::FifoWatermark),
        (INT_MAP2, 12)
    );
    assert_eq!(
        interrupt_map_location(InterruptSource::FifoFull),
        (INT_MAP2, 14)
    );
}

#[test]
fn no_motion_additional_helpers_delegate_correctly() {
    assert!((NoMotionConfig::hysteresis_to_g(256) - 0.5).abs() < 1e-4);
    assert!((NoMotionConfig::wait_time_to_seconds(4) - 4.0 / 50.0).abs() < 1e-4);
    assert_eq!(NoMotionConfig::interrupt_hold_from_millis(10.0), 4);
    assert!((NoMotionConfig::interrupt_hold_to_millis(4) - 10.0).abs() < 1e-4);
}

#[test]
fn flat_config_interrupt_hold_delegates_correctly() {
    assert_eq!(FlatConfig::interrupt_hold_from_millis(10.0), 4);
    assert!((FlatConfig::interrupt_hold_to_millis(4) - 10.0).abs() < 1e-4);
}

#[test]
fn orientation_slope_threshold_and_interrupt_hold_delegates_correctly() {
    assert_eq!(OrientationConfig::slope_threshold_from_g(0.25), 128);
    assert!((OrientationConfig::slope_threshold_to_g(128) - 0.25).abs() < 1e-4);
    assert_eq!(OrientationConfig::interrupt_hold_from_millis(10.0), 4);
    assert!((OrientationConfig::interrupt_hold_to_millis(4) - 10.0).abs() < 1e-4);
}

#[test]
fn tap_gesture_duration_and_short_duration_helpers() {
    // max_gesture_duration: raw / 25 s
    assert!((TapConfig::max_gesture_duration_to_seconds(16) - 0.64).abs() < 1e-4);
    // short_duration: raw * 200 → raw from seconds
    assert_eq!(TapConfig::short_duration_from_seconds(0.03), 6);
    assert_eq!(TapConfig::interrupt_hold_from_millis(10.0), 4);
    assert!((TapConfig::interrupt_hold_to_millis(4) - 10.0).abs() < 1e-4);
}

#[test]
fn significant_motion_block_size_to_seconds_and_interrupt_hold() {
    assert!((SignificantMotionConfig::block_size_to_seconds(250) - 5.0).abs() < 1e-4);
    assert_eq!(SignificantMotionConfig::interrupt_hold_from_millis(10.0), 4);
    assert!((SignificantMotionConfig::interrupt_hold_to_millis(4) - 10.0).abs() < 1e-4);
}

#[test]
fn tilt_interrupt_hold_delegates_correctly() {
    assert_eq!(TiltConfig::interrupt_hold_from_millis(10.0), 4);
    assert!((TiltConfig::interrupt_hold_to_millis(4) - 10.0).abs() < 1e-4);
}
