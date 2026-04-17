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
