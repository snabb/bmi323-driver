use crate::{AxisData, InterruptSource};

// §6.1.2 Register Map Details
pub(crate) const CHIP_ID: u8 = 0x00;             // Register (0x00) chip_id
pub(crate) const ERR_REG: u8 = 0x01;             // Register (0x01) err_reg
pub(crate) const STATUS: u8 = 0x02;              // Register (0x02) status
pub(crate) const ACC_DATA_X: u8 = 0x03;          // Register (0x03) acc_data_x — burst-reads y (0x04) and z (0x05)
pub(crate) const GYR_DATA_X: u8 = 0x06;          // Register (0x06) gyr_data_x — burst-reads y (0x07) and z (0x08)
pub(crate) const TEMP_DATA: u8 = 0x09;           // Register (0x09) temp_data
pub(crate) const SENSOR_TIME_0: u8 = 0x0A;       // Register (0x0A) sensor_time_0
pub(crate) const INT_STATUS_INT1: u8 = 0x0D;     // Register (0x0D) int_status_int1
pub(crate) const INT_STATUS_INT2: u8 = 0x0E;     // Register (0x0E) int_status_int2
pub(crate) const INT_STATUS_IBI: u8 = 0x0F;      // Register (0x0F) int_status_ibi
pub(crate) const FEATURE_IO0: u8 = 0x10;         // Register (0x10) feature_io0
pub(crate) const FEATURE_IO1: u8 = 0x11;         // Register (0x11) feature_io1
pub(crate) const FEATURE_IO2: u8 = 0x12;         // Register (0x12) feature_io2
pub(crate) const FEATURE_IO3: u8 = 0x13;         // Register (0x13) feature_io3
pub(crate) const FEATURE_IO_STATUS: u8 = 0x14;   // Register (0x14) feature_io_status
pub(crate) const FIFO_FILL_LEVEL: u8 = 0x15;     // Register (0x15) fifo_fill_level
pub(crate) const FIFO_DATA: u8 = 0x16;           // Register (0x16) fifo_data
pub(crate) const ACC_CONF: u8 = 0x20;            // Register (0x20) acc_conf
pub(crate) const GYR_CONF: u8 = 0x21;            // Register (0x21) gyr_conf
pub(crate) const ALT_ACC_CONF: u8 = 0x28;        // Register (0x28) alt_acc_conf
pub(crate) const ALT_GYR_CONF: u8 = 0x29;        // Register (0x29) alt_gyr_conf
pub(crate) const ALT_CONF: u8 = 0x2A;            // Register (0x2A) alt_conf
pub(crate) const ALT_STATUS: u8 = 0x2B;          // Register (0x2B) alt_status
pub(crate) const FIFO_WATERMARK: u8 = 0x35;      // Register (0x35) fifo_watermark
pub(crate) const FIFO_CONF: u8 = 0x36;           // Register (0x36) fifo_conf
pub(crate) const FIFO_CTRL: u8 = 0x37;           // Register (0x37) fifo_ctrl
pub(crate) const IO_INT_CTRL: u8 = 0x38;         // Register (0x38) io_int_ctrl
pub(crate) const INT_CONF: u8 = 0x39;            // Register (0x39) int_conf
pub(crate) const INT_MAP1: u8 = 0x3A;            // Register (0x3A) int_map1
pub(crate) const INT_MAP2: u8 = 0x3B;            // Register (0x3B) int_map2
pub(crate) const FEATURE_CTRL: u8 = 0x40;        // Register (0x40) feature_ctrl
pub(crate) const FEATURE_DATA_ADDR: u8 = 0x41;   // Register (0x41) feature_data_addr
pub(crate) const FEATURE_DATA_TX: u8 = 0x42;     // Register (0x42) feature_data_tx
pub(crate) const CMD: u8 = 0x7E;                 // Register (0x7E) cmd

// §6.2.2 Extended Register Map Details
pub(crate) const EXT_GEN_SET_1: u16 = 0x02;      // Register (0x02) gen_set_1
pub(crate) const EXT_ANYMO_1: u16 = 0x05;        // Register (0x05) anymo_1
pub(crate) const EXT_ANYMO_2: u16 = 0x06;        // Register (0x06) anymo_2
pub(crate) const EXT_ANYMO_3: u16 = 0x07;        // Register (0x07) anymo_3
pub(crate) const EXT_NOMO_1: u16 = 0x08;         // Register (0x08) nomo_1
pub(crate) const EXT_NOMO_2: u16 = 0x09;         // Register (0x09) nomo_2
pub(crate) const EXT_NOMO_3: u16 = 0x0A;         // Register (0x0A) nomo_3
pub(crate) const EXT_FLAT_1: u16 = 0x0B;         // Register (0x0B) flat_1
pub(crate) const EXT_FLAT_2: u16 = 0x0C;         // Register (0x0C) flat_2
pub(crate) const EXT_SIGMO_1: u16 = 0x0D;        // Register (0x0D) sigmo_1
pub(crate) const EXT_SIGMO_2: u16 = 0x0E;        // Register (0x0E) sigmo_2
pub(crate) const EXT_SIGMO_3: u16 = 0x0F;        // Register (0x0F) sigmo_3
pub(crate) const EXT_SC_1: u16 = 0x10;           // Register (0x10) sc_1
pub(crate) const EXT_ORIENT_1: u16 = 0x1C;       // Register (0x1C) orient_1
pub(crate) const EXT_ORIENT_2: u16 = 0x1D;       // Register (0x1D) orient_2
pub(crate) const EXT_TAP_1: u16 = 0x1E;          // Register (0x1E) tap_1
pub(crate) const EXT_TAP_2: u16 = 0x1F;          // Register (0x1F) tap_2
pub(crate) const EXT_TAP_3: u16 = 0x20;          // Register (0x20) tap_3
pub(crate) const EXT_TILT_1: u16 = 0x21;         // Register (0x21) tilt_1
pub(crate) const EXT_TILT_2: u16 = 0x22;         // Register (0x22) tilt_2
pub(crate) const EXT_ALT_CONFIG_CHG: u16 = 0x23; // Register (0x23) alt_config_chg
pub(crate) const EXT_ST_RESULT: u16 = 0x24;      // Register (0x24) st_result
pub(crate) const EXT_ST_SELECT: u16 = 0x25;      // Register (0x25) st_select

/// Expected value read from the `CHIP_ID` register (§6.1.2, Register (0x00) chip_id).
pub(crate) const BMI323_CHIP_ID: u8 = 0x43;
/// Soft-reset opcode written to CMD (§6.1.2, Register (0x7E) cmd; §5.17).
pub(crate) const SOFT_RESET: u16 = 0xDEAF;
/// Self-test trigger opcode written to CMD (§6.1.2, Register (0x7E) cmd; §5.14).
pub(crate) const SELF_TEST: u16 = 0x0100;

/// Feature-engine configuration payload written to FEATURE_IO2 during init
/// (§6.1.2, Register (0x12) feature_io2; §5.8.1).
pub(crate) const FEATURE_ENGINE_CONFIG: u16 = 0x012C;
/// Sync trigger written to FEATURE_IO_STATUS after FEATURE_IO2 is set
/// (§6.1.2, Register (0x14) feature_io_status; §5.8.1).
pub(crate) const FEATURE_IO_STATUS_SYNC: u16 = 0x0001;
/// Feature-engine enable bit written to FEATURE_CTRL
/// (§6.1.2, Register (0x40) feature_ctrl, `engine_en` bit 0).
pub(crate) const FEATURE_CTRL_ENABLE: u16 = 0x0001;

/// `FEATURE_IO1` `error_status` value indicating the feature engine finished
/// initializing successfully (§6.1.2, Register (0x11) feature_io1).
pub(crate) const FEATURE_ENGINE_STATUS_INIT_OK: u8 = 0x01;
/// `FEATURE_IO1` `error_status` value indicating the feature engine is active
/// with no errors — the normal running state (§6.1.2, Register (0x11) feature_io1).
pub(crate) const FEATURE_ENGINE_STATUS_NO_ERROR: u8 = 0x05;

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
        InterruptSource::ErrorStatus => (INT_MAP2, 4),
        InterruptSource::TempDataReady => (INT_MAP2, 6),
        InterruptSource::GyroDataReady => (INT_MAP2, 8),
        InterruptSource::AccelDataReady => (INT_MAP2, 10),
        InterruptSource::FifoWatermark => (INT_MAP2, 12),
        InterruptSource::FifoFull => (INT_MAP2, 14),
    }
}
