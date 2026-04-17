use crate::{AxisData, InterruptSource};

pub(crate) const CHIP_ID: u8 = 0x00;
pub(crate) const ERR_REG: u8 = 0x01;
pub(crate) const STATUS: u8 = 0x02;
pub(crate) const ACC_DATA_X: u8 = 0x03;
pub(crate) const GYR_DATA_X: u8 = 0x06;
pub(crate) const TEMP_DATA: u8 = 0x09;
pub(crate) const SENSOR_TIME_0: u8 = 0x0A;
pub(crate) const FEATURE_IO0: u8 = 0x10;
pub(crate) const FEATURE_IO1: u8 = 0x11;
pub(crate) const FEATURE_IO2: u8 = 0x12;
pub(crate) const FEATURE_IO3: u8 = 0x13;
pub(crate) const FEATURE_IO_STATUS: u8 = 0x14;
pub(crate) const FIFO_FILL_LEVEL: u8 = 0x15;
pub(crate) const FIFO_DATA: u8 = 0x16;
pub(crate) const ACC_CONF: u8 = 0x20;
pub(crate) const GYR_CONF: u8 = 0x21;
pub(crate) const ALT_ACC_CONF: u8 = 0x28;
pub(crate) const ALT_GYR_CONF: u8 = 0x29;
pub(crate) const ALT_CONF: u8 = 0x2A;
pub(crate) const ALT_STATUS: u8 = 0x2B;
pub(crate) const FIFO_WATERMARK: u8 = 0x35;
pub(crate) const FIFO_CONF: u8 = 0x36;
pub(crate) const FIFO_CTRL: u8 = 0x37;
pub(crate) const IO_INT_CTRL: u8 = 0x38;
pub(crate) const INT_CONF: u8 = 0x39;
pub(crate) const INT_MAP1: u8 = 0x3A;
pub(crate) const INT_MAP2: u8 = 0x3B;
pub(crate) const FEATURE_CTRL: u8 = 0x40;
pub(crate) const FEATURE_DATA_ADDR: u8 = 0x41;
pub(crate) const FEATURE_DATA_TX: u8 = 0x42;
pub(crate) const CMD: u8 = 0x7E;

pub(crate) const EXT_GEN_SET_1: u16 = 0x02;
pub(crate) const EXT_ANYMO_1: u16 = 0x05;
pub(crate) const EXT_ANYMO_2: u16 = 0x06;
pub(crate) const EXT_ANYMO_3: u16 = 0x07;
pub(crate) const EXT_NOMO_1: u16 = 0x08;
pub(crate) const EXT_NOMO_2: u16 = 0x09;
pub(crate) const EXT_NOMO_3: u16 = 0x0A;
pub(crate) const EXT_FLAT_1: u16 = 0x0B;
pub(crate) const EXT_FLAT_2: u16 = 0x0C;
pub(crate) const EXT_SIGMO_1: u16 = 0x0D;
pub(crate) const EXT_SIGMO_2: u16 = 0x0E;
pub(crate) const EXT_SIGMO_3: u16 = 0x0F;
pub(crate) const EXT_SC_1: u16 = 0x10;
pub(crate) const EXT_ORIENT_1: u16 = 0x1C;
pub(crate) const EXT_ORIENT_2: u16 = 0x1D;
pub(crate) const EXT_TAP_1: u16 = 0x1E;
pub(crate) const EXT_TAP_2: u16 = 0x1F;
pub(crate) const EXT_TAP_3: u16 = 0x20;
pub(crate) const EXT_TILT_1: u16 = 0x21;
pub(crate) const EXT_TILT_2: u16 = 0x22;
pub(crate) const EXT_ALT_CONFIG_CHG: u16 = 0x23;
pub(crate) const EXT_ST_RESULT: u16 = 0x24;
pub(crate) const EXT_ST_SELECT: u16 = 0x25;

pub(crate) const BMI323_CHIP_ID: u8 = 0x43;
pub(crate) const SOFT_RESET: u16 = 0xDEAF;
pub(crate) const SELF_TEST: u16 = 0x0100;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum TransportKind {
    I2c,
    Spi,
}

pub(crate) const fn words_to_axis(words: [u16; 3]) -> AxisData {
    AxisData {
        x: words[0] as i16,
        y: words[1] as i16,
        z: words[2] as i16,
    }
}

pub(crate) const fn interrupt_map_location(source: InterruptSource) -> (u8, u8) {
    match source {
        InterruptSource::NoMotion => (INT_MAP1, 0),
        InterruptSource::AnyMotion => (INT_MAP1, 2),
        InterruptSource::Flat => (INT_MAP1, 4),
        InterruptSource::Orientation => (INT_MAP1, 6),
        InterruptSource::StepDetector => (INT_MAP1, 8),
        InterruptSource::StepCounter => (INT_MAP1, 10),
        InterruptSource::SignificantMotion => (INT_MAP1, 12),
        InterruptSource::Tilt => (INT_MAP1, 14),
        InterruptSource::Tap => (INT_MAP2, 0),
        InterruptSource::I3cSync => (INT_MAP2, 2),
        InterruptSource::FeatureStatus => (INT_MAP2, 4),
        InterruptSource::TempDataReady => (INT_MAP2, 6),
        InterruptSource::GyroDataReady => (INT_MAP2, 8),
        InterruptSource::AccelDataReady => (INT_MAP2, 10),
        InterruptSource::FifoWatermark => (INT_MAP2, 12),
        InterruptSource::FifoFull => (INT_MAP2, 14),
    }
}
