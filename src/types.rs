/// Driver error type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// Transport-level bus error returned by the underlying HAL.
    Bus(E),
    /// The chip ID register did not contain the expected BMI323 identifier.
    InvalidChipId(u8),
    /// The BMI323 reported a fatal internal error.
    FatalError,
    /// The feature engine did not become ready after the expected init sequence.
    FeatureEngineNotReady(u8),
    /// The BMI323 self-test did not complete within the expected timeout.
    SelfTestTimeout,
}

/// Primary 7-bit BMI323 I2C address.
///
/// Use this when the sensor address-selection pin is strapped for the default
/// address.
pub const I2C_ADDRESS_PRIMARY: u8 = 0x68;

/// Alternate 7-bit BMI323 I2C address.
///
/// Use this when the sensor address-selection pin is strapped for the
/// alternate address.
pub const I2C_ADDRESS_ALTERNATE: u8 = 0x69;

/// Decoded contents of the BMI323 `STATUS` register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StatusWord(pub u16);

impl StatusWord {
    /// Returns true if the sensor reports a power-on reset condition.
    pub const fn por_detected(self) -> bool {
        self.0 & 0x0001 != 0
    }

    /// Returns true if a new temperature sample is available.
    pub const fn drdy_temp(self) -> bool {
        self.0 & (1 << 5) != 0
    }

    /// Returns true if a new gyroscope sample is available.
    pub const fn drdy_gyro(self) -> bool {
        self.0 & (1 << 6) != 0
    }

    /// Returns true if a new accelerometer sample is available.
    pub const fn drdy_accel(self) -> bool {
        self.0 & (1 << 7) != 0
    }
}

/// Decoded contents of the BMI323 `ERR_REG` register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ErrorWord(pub u16);

impl ErrorWord {
    /// Returns true if the BMI323 reports a fatal internal error.
    pub const fn fatal(self) -> bool {
        self.0 & 0x0001 != 0
    }

    /// Returns true if the current accelerometer configuration is invalid.
    pub const fn accel_conf_error(self) -> bool {
        self.0 & (1 << 5) != 0
    }

    /// Returns true if the current gyroscope configuration is invalid.
    pub const fn gyro_conf_error(self) -> bool {
        self.0 & (1 << 6) != 0
    }
}

/// Decoded interrupt status for `INT1`, `INT2`, or I3C IBI.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterruptStatus(pub u16);

impl InterruptStatus {
    /// Returns true if no-motion is asserted.
    pub const fn no_motion(self) -> bool {
        self.0 & (1 << 0) != 0
    }
    /// Returns true if any-motion is asserted.
    pub const fn any_motion(self) -> bool {
        self.0 & (1 << 1) != 0
    }
    /// Returns true if flat detection is asserted.
    pub const fn flat(self) -> bool {
        self.0 & (1 << 2) != 0
    }
    /// Returns true if orientation change is asserted.
    pub const fn orientation(self) -> bool {
        self.0 & (1 << 3) != 0
    }
    /// Returns true if the step detector event is asserted.
    pub const fn step_detector(self) -> bool {
        self.0 & (1 << 4) != 0
    }
    /// Returns true if the step counter event is asserted.
    pub const fn step_counter(self) -> bool {
        self.0 & (1 << 5) != 0
    }
    /// Returns true if significant motion is asserted.
    pub const fn significant_motion(self) -> bool {
        self.0 & (1 << 6) != 0
    }
    /// Returns true if tilt is asserted.
    pub const fn tilt(self) -> bool {
        self.0 & (1 << 7) != 0
    }
    /// Returns true if tap detection is asserted.
    pub const fn tap(self) -> bool {
        self.0 & (1 << 8) != 0
    }
    /// Returns true if the feature-engine status interrupt is asserted.
    pub const fn feature_status(self) -> bool {
        self.0 & (1 << 10) != 0
    }
    /// Returns true if temperature data-ready is asserted.
    pub const fn temp_data_ready(self) -> bool {
        self.0 & (1 << 11) != 0
    }
    /// Returns true if gyroscope data-ready is asserted.
    pub const fn gyro_data_ready(self) -> bool {
        self.0 & (1 << 12) != 0
    }
    /// Returns true if accelerometer data-ready is asserted.
    pub const fn accel_data_ready(self) -> bool {
        self.0 & (1 << 13) != 0
    }
    /// Returns true if FIFO watermark is asserted.
    pub const fn fifo_watermark(self) -> bool {
        self.0 & (1 << 14) != 0
    }
    /// Returns true if FIFO full is asserted.
    pub const fn fifo_full(self) -> bool {
        self.0 & (1 << 15) != 0
    }
}

/// Self-test selection written to the BMI323 `st_select` extended register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SelfTestSelection {
    /// Run accelerometer self-test only.
    Accelerometer,
    /// Run gyroscope self-test only.
    Gyroscope,
    /// Run both accelerometer and gyroscope self-tests.
    Both,
}

impl SelfTestSelection {
    /// Encode the selection into the BMI323 `st_select` bit layout.
    pub const fn to_word(self) -> u16 {
        match self {
            Self::Accelerometer => 0x0001,
            Self::Gyroscope => 0x0002,
            Self::Both => 0x0003,
        }
    }

    /// Returns true if the accelerometer self-test is enabled.
    pub const fn tests_accelerometer(self) -> bool {
        matches!(self, Self::Accelerometer | Self::Both)
    }

    /// Returns true if the gyroscope self-test is enabled.
    pub const fn tests_gyroscope(self) -> bool {
        matches!(self, Self::Gyroscope | Self::Both)
    }
}

/// Detailed self-test result bits read from the BMI323 `st_result` extended
/// register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SelfTestDetail(pub u16);

impl SelfTestDetail {
    /// Returns true if the accelerometer X-axis self-test passed.
    pub const fn acc_sens_x_ok(self) -> bool {
        self.0 & (1 << 0) != 0
    }

    /// Returns true if the accelerometer Y-axis self-test passed.
    pub const fn acc_sens_y_ok(self) -> bool {
        self.0 & (1 << 1) != 0
    }

    /// Returns true if the accelerometer Z-axis self-test passed.
    pub const fn acc_sens_z_ok(self) -> bool {
        self.0 & (1 << 2) != 0
    }

    /// Returns true if the gyroscope X-axis self-test passed.
    pub const fn gyr_sens_x_ok(self) -> bool {
        self.0 & (1 << 3) != 0
    }

    /// Returns true if the gyroscope Y-axis self-test passed.
    pub const fn gyr_sens_y_ok(self) -> bool {
        self.0 & (1 << 4) != 0
    }

    /// Returns true if the gyroscope Z-axis self-test passed.
    pub const fn gyr_sens_z_ok(self) -> bool {
        self.0 & (1 << 5) != 0
    }

    /// Returns true if the gyroscope drive self-test passed.
    pub const fn gyr_drive_ok(self) -> bool {
        self.0 & (1 << 6) != 0
    }

    /// Returns true when all requested accelerometer axes passed.
    pub const fn accelerometer_ok(self) -> bool {
        self.acc_sens_x_ok() && self.acc_sens_y_ok() && self.acc_sens_z_ok()
    }

    /// Returns true when all requested gyroscope checks passed.
    pub const fn gyroscope_ok(self) -> bool {
        self.gyr_sens_x_ok() && self.gyr_sens_y_ok() && self.gyr_sens_z_ok() && self.gyr_drive_ok()
    }
}

/// Result of a completed BMI323 self-test run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SelfTestResult {
    /// Which sensor blocks were included in the self-test run.
    pub selection: SelfTestSelection,
    /// Overall pass/fail bit reported by `FEATURE_IO1.st_result`.
    pub passed: bool,
    /// Indicates that the required accelerometer sample rate precondition was
    /// not met.
    pub sample_rate_error: bool,
    /// Low-level feature-engine status code from `FEATURE_IO1.error_status`.
    ///
    /// On the BMI323 this field uses non-obvious encodings. In particular,
    /// `0x5` is the normal "no error" state after the feature engine is active.
    pub error_status: u8,
    /// Per-axis and gyroscope-drive detailed result bits from `st_result`.
    pub detail: SelfTestDetail,
}

impl SelfTestResult {
    /// Returns true if the requested accelerometer self-test portion passed.
    pub const fn accelerometer_ok(self) -> bool {
        if self.selection.tests_accelerometer() {
            self.detail.accelerometer_ok()
        } else {
            true
        }
    }

    /// Returns true if the requested gyroscope self-test portion passed.
    pub const fn gyroscope_ok(self) -> bool {
        if self.selection.tests_gyroscope() {
            self.detail.gyroscope_ok()
        } else {
            true
        }
    }
}

/// Raw 3-axis sample from the accelerometer or gyroscope.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AxisData {
    /// X-axis sample in raw sensor counts.
    pub x: i16,
    /// Y-axis sample in raw sensor counts.
    pub y: i16,
    /// Z-axis sample in raw sensor counts.
    pub z: i16,
}

impl AxisData {
    /// Convert a raw accelerometer sample into `g` for the selected range.
    pub fn as_g(self, range: AccelRange) -> [f32; 3] {
        let scale = range.scale_g_per_lsb();
        [
            self.x as f32 * scale,
            self.y as f32 * scale,
            self.z as f32 * scale,
        ]
    }

    /// Convert a raw gyroscope sample into `deg/s` for the selected range.
    pub fn as_dps(self, range: GyroRange) -> [f32; 3] {
        let scale = range.scale_dps_per_lsb();
        [
            self.x as f32 * scale,
            self.y as f32 * scale,
            self.z as f32 * scale,
        ]
    }
}

/// Combined accelerometer and gyroscope sample read in one burst.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ImuData {
    /// Accelerometer sample.
    pub accel: AxisData,
    /// Gyroscope sample.
    pub gyro: AxisData,
}

/// Result returned by [`Bmi323::init`](crate::Bmi323::init) and
/// [`Bmi323Async::init`](crate::Bmi323Async::init).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceState {
    /// Value read from the `CHIP_ID` register.
    pub chip_id: u8,
    /// Snapshot of the `STATUS` register taken during init.
    pub status: StatusWord,
    /// Snapshot of the `ERR_REG` register taken during init.
    pub error: ErrorWord,
}

/// Output data rate selection used by accel and gyro configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OutputDataRate {
    Hz0_78125 = 0x1,
    Hz1_5625 = 0x2,
    Hz3_125 = 0x3,
    Hz6_25 = 0x4,
    Hz12_5 = 0x5,
    Hz25 = 0x6,
    Hz50 = 0x7,
    Hz100 = 0x8,
    Hz200 = 0x9,
    Hz400 = 0xA,
    Hz800 = 0xB,
    Hz1600 = 0xC,
    Hz3200 = 0xD,
    Hz6400 = 0xE,
}

/// Low-pass filter bandwidth relative to output data rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Bandwidth {
    OdrOver2 = 0,
    OdrOver4 = 1,
}

/// Sample averaging depth used in supported sensor modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AverageSamples {
    Avg1 = 0,
    Avg2 = 1,
    Avg4 = 2,
    Avg8 = 3,
    Avg16 = 4,
    Avg32 = 5,
    Avg64 = 6,
}

/// Accelerometer operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelMode {
    Disabled = 0,
    LowPower = 3,
    Normal = 4,
    HighPerformance = 7,
}

/// Gyroscope operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GyroMode {
    Disabled = 0,
    DriveEnabledOnly = 1,
    LowPower = 3,
    Normal = 4,
    HighPerformance = 7,
}

/// Accelerometer full-scale measurement range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelRange {
    G2 = 0,
    G4 = 1,
    G8 = 2,
    G16 = 3,
}

impl AccelRange {
    /// Scale factor in `g/LSB` for raw accelerometer samples.
    pub const fn scale_g_per_lsb(self) -> f32 {
        match self {
            Self::G2 => 2.0 / 32768.0,
            Self::G4 => 4.0 / 32768.0,
            Self::G8 => 8.0 / 32768.0,
            Self::G16 => 16.0 / 32768.0,
        }
    }
}

/// Gyroscope full-scale measurement range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GyroRange {
    Dps125 = 0,
    Dps250 = 1,
    Dps500 = 2,
    Dps1000 = 3,
    Dps2000 = 4,
}

impl GyroRange {
    /// Scale factor in `deg/s per LSB` for raw gyroscope samples.
    pub const fn scale_dps_per_lsb(self) -> f32 {
        match self {
            Self::Dps125 => 125.0 / 32768.0,
            Self::Dps250 => 250.0 / 32768.0,
            Self::Dps500 => 500.0 / 32768.0,
            Self::Dps1000 => 1000.0 / 32768.0,
            Self::Dps2000 => 2000.0 / 32768.0,
        }
    }
}

/// High-level accelerometer configuration written to `ACC_CONF`.
///
/// `Default::default()` yields:
/// - mode: [`AccelMode::Normal`]
/// - average: [`AverageSamples::Avg1`]
/// - bandwidth: [`Bandwidth::OdrOver2`]
/// - range: [`AccelRange::G8`]
/// - odr: [`OutputDataRate::Hz50`]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AccelConfig {
    /// Sensor operating mode.
    pub mode: AccelMode,
    /// Sample averaging depth.
    pub average: AverageSamples,
    /// Low-pass filter bandwidth.
    pub bandwidth: Bandwidth,
    /// Full-scale range.
    pub range: AccelRange,
    /// Output data rate.
    pub odr: OutputDataRate,
}

impl Default for AccelConfig {
    fn default() -> Self {
        Self {
            mode: AccelMode::Normal,
            average: AverageSamples::Avg1,
            bandwidth: Bandwidth::OdrOver2,
            range: AccelRange::G8,
            odr: OutputDataRate::Hz50,
        }
    }
}

impl AccelConfig {
    /// Encode the configuration into the BMI323 register bit layout.
    pub const fn to_word(self) -> u16 {
        ((self.mode as u16) << 12)
            | ((self.average as u16) << 8)
            | ((self.bandwidth as u16) << 7)
            | ((self.range as u16) << 4)
            | self.odr as u16
    }
}

/// High-level gyroscope configuration written to `GYR_CONF`.
///
/// `Default::default()` yields:
/// - mode: [`GyroMode::Normal`]
/// - average: [`AverageSamples::Avg1`]
/// - bandwidth: [`Bandwidth::OdrOver2`]
/// - range: [`GyroRange::Dps2000`]
/// - odr: [`OutputDataRate::Hz50`]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GyroConfig {
    /// Sensor operating mode.
    pub mode: GyroMode,
    /// Sample averaging depth.
    pub average: AverageSamples,
    /// Low-pass filter bandwidth.
    pub bandwidth: Bandwidth,
    /// Full-scale range.
    pub range: GyroRange,
    /// Output data rate.
    pub odr: OutputDataRate,
}

impl Default for GyroConfig {
    fn default() -> Self {
        Self {
            mode: GyroMode::Normal,
            average: AverageSamples::Avg1,
            bandwidth: Bandwidth::OdrOver2,
            range: GyroRange::Dps2000,
            odr: OutputDataRate::Hz50,
        }
    }
}

impl GyroConfig {
    /// Encode the configuration into the BMI323 register bit layout.
    pub const fn to_word(self) -> u16 {
        ((self.mode as u16) << 12)
            | ((self.average as u16) << 8)
            | ((self.bandwidth as u16) << 7)
            | ((self.range as u16) << 4)
            | self.odr as u16
    }
}

/// FIFO enable and behavior configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoConfig {
    /// Stop writing new data once the FIFO is full.
    pub stop_on_full: bool,
    /// Include sensor time words in FIFO output.
    pub include_time: bool,
    /// Include accelerometer samples in FIFO output.
    pub include_accel: bool,
    /// Include gyroscope samples in FIFO output.
    pub include_gyro: bool,
    /// Include temperature samples in FIFO output.
    pub include_temperature: bool,
}

impl FifoConfig {
    /// Encode the configuration into the BMI323 register bit layout.
    pub const fn to_word(self) -> u16 {
        (self.stop_on_full as u16)
            | ((self.include_time as u16) << 8)
            | ((self.include_accel as u16) << 9)
            | ((self.include_gyro as u16) << 10)
            | ((self.include_temperature as u16) << 11)
    }
}

/// Interrupt output channel inside the BMI323.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptChannel {
    /// Route or query the `INT1` output.
    Int1,
    /// Route or query the `INT2` output.
    Int2,
    /// Route or query the I3C in-band interrupt channel.
    Ibi,
}

/// Mapping destination for an interrupt source.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptRoute {
    /// Do not route the interrupt source anywhere.
    Disabled = 0,
    /// Route the source to `INT1`.
    Int1 = 1,
    /// Route the source to `INT2`.
    Int2 = 2,
    /// Route the source to I3C in-band interrupt.
    Ibi = 3,
}

impl From<InterruptRoute> for u16 {
    fn from(value: InterruptRoute) -> Self {
        value as u16
    }
}

/// Electrical active level for an interrupt output pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ActiveLevel {
    /// Active-low signaling.
    Low,
    /// Active-high signaling.
    High,
}

/// Electrical driver mode for an interrupt output pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OutputMode {
    /// Push-pull output driver.
    PushPull,
    /// Open-drain output driver.
    OpenDrain,
}

/// Electrical configuration for `INT1` or `INT2`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterruptPinConfig {
    /// Active signaling level.
    pub active_level: ActiveLevel,
    /// Output driver type.
    pub output_mode: OutputMode,
    /// Whether the selected output is enabled.
    pub enabled: bool,
}

/// Interrupt source that can be mapped to an output channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptSource {
    /// No-motion feature interrupt.
    NoMotion,
    /// Any-motion feature interrupt.
    AnyMotion,
    /// Flat detection interrupt.
    Flat,
    /// Orientation change interrupt.
    Orientation,
    /// Step detector interrupt.
    StepDetector,
    /// Step counter interrupt.
    StepCounter,
    /// Significant motion interrupt.
    SignificantMotion,
    /// Tilt interrupt.
    Tilt,
    /// Tap interrupt.
    Tap,
    /// I3C synchronization interrupt.
    I3cSync,
    /// Feature-engine status interrupt.
    FeatureStatus,
    /// Temperature data-ready interrupt.
    TempDataReady,
    /// Gyroscope data-ready interrupt.
    GyroDataReady,
    /// Accelerometer data-ready interrupt.
    AccelDataReady,
    /// FIFO watermark interrupt.
    FifoWatermark,
    /// FIFO full interrupt.
    FifoFull,
}

/// Event reporting policy for supported feature-engine motion detectors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EventReportMode {
    /// Report every qualifying event.
    AllEvents,
    /// Report only the first event until cleared internally by the sensor.
    FirstEventOnly,
}

/// Configuration for the BMI323 feature-engine step counter block.
///
/// The counter itself accumulates the total detected step count inside the
/// sensor. This configuration controls two separate behaviors:
///
/// - `watermark_level`: generates a step-counter interrupt whenever the number
///   of newly counted steps since the last step-counter event reaches the
///   programmed value
/// - `reset_counter`: requests a reset of the accumulated internal step count
///
/// A `watermark_level` of `0` disables the step-counter interrupt source.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StepCounterConfig {
    /// Step-counter interrupt watermark.
    ///
    /// Unit: steps
    /// Scaling: 1 LSB = 1 step event increment
    /// Range: `0 ..= 1023`
    ///
    /// When this is nonzero, the BMI323 raises the `StepCounter` interrupt
    /// each time this many additional steps have been accumulated since the
    /// last step-counter event. A value of `0` disables the step-counter
    /// interrupt.
    pub watermark_level: u16,
    /// Reset request for the accumulated step count.
    ///
    /// When set to `true`, the driver writes the feature-engine reset bit for
    /// the step counter. This is useful when re-arming an application that
    /// wants a fresh accumulated count after initialization or after reading a
    /// previous session's result.
    pub reset_counter: bool,
}

impl StepCounterConfig {
    /// Create a configuration with the step-counter interrupt disabled.
    pub const fn disabled() -> Self {
        Self {
            watermark_level: 0,
            reset_counter: false,
        }
    }

    /// Create a configuration that raises an interrupt every `steps` steps.
    ///
    /// Values above `1023` are saturated to the BMI323 register field width.
    pub const fn with_watermark(steps: u16) -> Self {
        Self {
            watermark_level: if steps > 1023 { 1023 } else { steps },
            reset_counter: false,
        }
    }

    /// Encode the configuration into the BMI323 `sc_1` feature word.
    pub const fn to_word(self) -> u16 {
        (self.watermark_level & 0x03FF) | ((self.reset_counter as u16) << 10)
    }
}

/// Shared blocking behavior used by flat and orientation detection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FeatureBlockingMode {
    /// Do not block state changes during large movements.
    Disabled = 0,
    /// Block when acceleration on any axis exceeds `1.5g`.
    AccelOver1p5g = 1,
    /// Block when acceleration exceeds `1.5g` or slope exceeds half of the
    /// configured slope threshold.
    AccelOver1p5gOrHalfSlope = 2,
    /// Block when acceleration exceeds `1.5g` or slope exceeds the configured
    /// slope threshold.
    AccelOver1p5gOrFullSlope = 3,
}

/// Configuration mode for orientation spread between portrait and landscape.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OrientationMode {
    /// Symmetrical spread for portrait and landscape orientations.
    Symmetrical = 0,
    /// Larger landscape area than portrait area.
    LandscapeWide = 1,
    /// Larger portrait area than landscape area.
    PortraitWide = 2,
}

/// Dominant accelerometer axis used for tap detection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TapAxis {
    /// Detect taps along the X axis.
    X = 0,
    /// Detect taps along the Y axis.
    Y = 1,
    /// Detect taps along the Z axis.
    Z = 2,
}

/// Reporting policy for tap gesture confirmation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TapReportingMode {
    /// Report the gesture as soon as it is detected.
    Immediate = 0,
    /// Delay reporting until the configured gesture timeout confirms it.
    Confirmed = 1,
}

/// Detection profile for tap recognition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TapDetectionMode {
    /// Most sensitive profile. Useful for stable devices.
    Sensitive = 0,
    /// Balanced default profile.
    Normal = 1,
    /// More robust profile with reduced false detections in noisy scenarios.
    Robust = 2,
}

/// Configuration for the BMI323 flat-detection feature.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FlatConfig {
    /// Maximum allowed tilt angle for the device to be considered flat.
    ///
    /// Unit: degrees
    /// Encoding: nonlinear
    /// Field width: 6 bits
    /// Valid raw range: `0 ..= 63`
    ///
    /// The datasheet interpretation is `(tan(theta)^2) * 64`.
    pub theta: u8,
    /// Blocking behavior to suppress flat-state changes during large movement.
    pub blocking: FeatureBlockingMode,
    /// Minimum time the device must remain flat before the event is asserted.
    ///
    /// Unit: seconds
    /// Scaling: `raw / 50`
    /// Range: `0 ..= 255`, corresponding to approximately `0.0s ..= 5.10s`
    pub hold_time: u8,
    /// Minimum acceleration slope that blocks flat-status changes during large
    /// movement.
    ///
    /// Unit: `g`
    /// Scaling: `raw / 512`
    /// Range: `0 ..= 255`, corresponding to approximately `0.0g ..= 0.498g`
    pub slope_threshold: u8,
    /// Angular hysteresis for flat detection.
    ///
    /// Unit: degrees
    /// Encoding: nonlinear
    /// Field width: 8 bits
    ///
    /// The datasheet expresses this field relative to the configured
    /// `theta`. The raw value can be programmed directly when precise control
    /// is needed.
    pub hysteresis: u8,
    /// Event reporting behavior shared across feature-engine interrupts.
    pub report_mode: EventReportMode,
    /// Shared feature-engine interrupt hold-time exponent.
    ///
    /// Effective hold time in non-latched mode:
    /// `0.625ms * 2^interrupt_hold`
    pub interrupt_hold: u8,
}

impl FlatConfig {
    /// Convert a hold time in seconds to the BMI323 field encoding.
    pub fn hold_time_from_seconds(seconds: f32) -> u8 {
        (seconds * 50.0).clamp(0.0, 255.0) as u8
    }

    /// Convert a raw hold-time field value back to seconds.
    pub fn hold_time_to_seconds(raw: u8) -> f32 {
        raw as f32 / 50.0
    }

    /// Convert a slope threshold in `g` to the BMI323 field encoding.
    pub fn slope_threshold_from_g(g: f32) -> u8 {
        (g * 512.0).clamp(0.0, 255.0) as u8
    }

    /// Convert a raw slope-threshold field value back to `g`.
    pub fn slope_threshold_to_g(raw: u8) -> f32 {
        raw as f32 / 512.0
    }

    /// Convert an interrupt hold time in milliseconds to the BMI323 field
    /// encoding.
    pub fn interrupt_hold_from_millis(millis: f32) -> u8 {
        AnyMotionConfig::interrupt_hold_from_millis(millis)
    }

    /// Convert a raw interrupt-hold field value back to milliseconds.
    pub fn interrupt_hold_to_millis(raw: u8) -> f32 {
        AnyMotionConfig::interrupt_hold_to_millis(raw)
    }
}

/// Configuration for the BMI323 orientation-detection feature.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OrientationConfig {
    /// Whether upside-down orientation detection is enabled.
    pub upside_down_enabled: bool,
    /// Portrait/landscape spread mode.
    pub mode: OrientationMode,
    /// Blocking behavior to suppress orientation changes during large movement.
    pub blocking: FeatureBlockingMode,
    /// Maximum allowed tilt angle for orientation classification.
    ///
    /// Unit: degrees
    /// Encoding: nonlinear
    /// Field width: 6 bits
    ///
    /// The datasheet interpretation is `(tan(theta)^2) * 64`.
    pub theta: u8,
    /// Minimum time the device must remain in the new orientation before an
    /// event is asserted.
    ///
    /// Unit: seconds
    /// Scaling: `raw / 50`
    /// Range: `0 ..= 31`, corresponding to approximately `0.0s ..= 0.62s`
    pub hold_time: u8,
    /// Minimum slope between consecutive acceleration samples used for blocking
    /// orientation changes during large movement.
    ///
    /// Unit: `g`
    /// Scaling: `raw / 512`
    /// Range: `0 ..= 255`, corresponding to approximately `0.0g ..= 0.498g`
    pub slope_threshold: u8,
    /// Hysteresis of acceleration for orientation change detection.
    ///
    /// Unit: `g`
    /// Scaling: `raw / 512`
    /// Range: `0 ..= 255`, corresponding to approximately `0.0g ..= 0.498g`
    pub hysteresis: u8,
    /// Event reporting behavior shared across feature-engine interrupts.
    pub report_mode: EventReportMode,
    /// Shared feature-engine interrupt hold-time exponent.
    pub interrupt_hold: u8,
}

impl OrientationConfig {
    /// Convert a hold time in seconds to the BMI323 field encoding.
    pub fn hold_time_from_seconds(seconds: f32) -> u8 {
        (seconds * 50.0).clamp(0.0, 31.0) as u8
    }

    /// Convert a raw hold-time field value back to seconds.
    pub fn hold_time_to_seconds(raw: u8) -> f32 {
        f32::from(raw & 0x1F) / 50.0
    }

    /// Convert a slope threshold in `g` to the BMI323 field encoding.
    pub fn slope_threshold_from_g(g: f32) -> u8 {
        FlatConfig::slope_threshold_from_g(g)
    }

    /// Convert a raw slope-threshold field value back to `g`.
    pub fn slope_threshold_to_g(raw: u8) -> f32 {
        FlatConfig::slope_threshold_to_g(raw)
    }

    /// Convert a hysteresis in `g` to the BMI323 field encoding.
    pub fn hysteresis_from_g(g: f32) -> u8 {
        (g * 512.0).clamp(0.0, 255.0) as u8
    }

    /// Convert a raw hysteresis field value back to `g`.
    pub fn hysteresis_to_g(raw: u8) -> f32 {
        raw as f32 / 512.0
    }

    /// Convert an interrupt hold time in milliseconds to the BMI323 field
    /// encoding.
    pub fn interrupt_hold_from_millis(millis: f32) -> u8 {
        AnyMotionConfig::interrupt_hold_from_millis(millis)
    }

    /// Convert a raw interrupt-hold field value back to milliseconds.
    pub fn interrupt_hold_to_millis(raw: u8) -> f32 {
        AnyMotionConfig::interrupt_hold_to_millis(raw)
    }
}

/// Configuration for the BMI323 tap-detection feature.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TapConfig {
    /// Dominant axis used for tap detection.
    pub axis: TapAxis,
    /// Whether tap gestures should be reported immediately or only after
    /// timeout-based confirmation.
    pub reporting_mode: TapReportingMode,
    /// Maximum number of threshold crossings expected around a tap.
    ///
    /// Range: `0 ..= 7`
    pub max_peaks_for_tap: u8,
    /// Tap detection profile.
    pub mode: TapDetectionMode,
    /// Whether single-tap reporting is enabled.
    pub single_tap_enabled: bool,
    /// Whether double-tap reporting is enabled.
    pub double_tap_enabled: bool,
    /// Whether triple-tap reporting is enabled.
    pub triple_tap_enabled: bool,
    /// Minimum peak threshold caused by the tap.
    ///
    /// Unit: `g`
    /// Scaling: `raw / 512`
    /// Range: `0 ..= 1023`, corresponding to approximately `0.0g ..= 1.998g`
    pub tap_peak_threshold: u16,
    /// Maximum duration from the first tap until the second and/or third tap is
    /// expected.
    ///
    /// Unit: seconds
    /// Scaling: `raw / 25`
    /// Range: `0 ..= 63`, corresponding to approximately `0.0s ..= 2.52s`
    pub max_gesture_duration: u8,
    /// Maximum duration between positive and negative peaks belonging to a tap.
    ///
    /// Unit: seconds
    /// Scaling: `raw / 200`
    /// Range: `0 ..= 15`, corresponding to approximately `0.0s ..= 0.075s`
    pub max_duration_between_peaks: u8,
    /// Maximum duration for which the tap impact is observed.
    ///
    /// Unit: seconds
    /// Scaling: `raw / 200`
    /// Range: `0 ..= 15`, corresponding to approximately `0.0s ..= 0.075s`
    pub tap_shock_settling_duration: u8,
    /// Minimum quiet duration between consecutive tap impacts.
    ///
    /// Unit: seconds
    /// Scaling: `raw / 200`
    /// Range: `0 ..= 15`, corresponding to approximately `0.0s ..= 0.075s`
    pub min_quiet_duration_between_taps: u8,
    /// Minimum quiet duration between two gestures.
    ///
    /// Unit: seconds
    /// Scaling: `raw / 25`
    /// Range: `0 ..= 15`, corresponding to approximately `0.0s ..= 0.6s`
    pub quiet_time_after_gesture: u8,
    /// Event reporting behavior shared across feature-engine interrupts.
    pub report_mode: EventReportMode,
    /// Shared feature-engine interrupt hold-time exponent.
    ///
    /// Note: the BMI323 datasheet warns that tap detection must not use the
    /// 5ms hold-time setting unless Bosch's enhanced flexibility configuration
    /// has been applied externally.
    pub interrupt_hold: u8,
}

impl TapConfig {
    /// Convert a tap threshold in `g` to the BMI323 field encoding.
    pub fn tap_peak_threshold_from_g(g: f32) -> u16 {
        (g * 512.0).clamp(0.0, 1023.0) as u16
    }

    /// Convert a raw tap-threshold field value back to `g`.
    pub fn tap_peak_threshold_to_g(raw: u16) -> f32 {
        (raw & 0x03FF) as f32 / 512.0
    }

    /// Convert a gesture duration in seconds to the BMI323 field encoding.
    pub fn max_gesture_duration_from_seconds(seconds: f32) -> u8 {
        (seconds * 25.0).clamp(0.0, 63.0) as u8
    }

    /// Convert a raw gesture-duration field value back to seconds.
    pub fn max_gesture_duration_to_seconds(raw: u8) -> f32 {
        f32::from(raw & 0x3F) / 25.0
    }

    /// Convert a short tap timing parameter in seconds to the BMI323 field
    /// encoding used by `max_duration_between_peaks`,
    /// `tap_shock_settling_duration`, and `min_quiet_duration_between_taps`.
    pub fn short_duration_from_seconds(seconds: f32) -> u8 {
        (seconds * 200.0).clamp(0.0, 15.0) as u8
    }

    /// Convert a raw short-duration field value back to seconds.
    pub fn short_duration_to_seconds(raw: u8) -> f32 {
        f32::from(raw & 0x0F) / 200.0
    }

    /// Convert a post-gesture quiet time in seconds to the BMI323 field
    /// encoding.
    pub fn quiet_time_after_gesture_from_seconds(seconds: f32) -> u8 {
        (seconds * 25.0).clamp(0.0, 15.0) as u8
    }

    /// Convert a raw post-gesture quiet-time field value back to seconds.
    pub fn quiet_time_after_gesture_to_seconds(raw: u8) -> f32 {
        f32::from(raw & 0x0F) / 25.0
    }

    /// Convert an interrupt hold time in milliseconds to the BMI323 field
    /// encoding.
    pub fn interrupt_hold_from_millis(millis: f32) -> u8 {
        AnyMotionConfig::interrupt_hold_from_millis(millis)
    }

    /// Convert a raw interrupt-hold field value back to milliseconds.
    pub fn interrupt_hold_to_millis(raw: u8) -> f32 {
        AnyMotionConfig::interrupt_hold_to_millis(raw)
    }
}

/// Configuration for the BMI323 significant-motion feature.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SignificantMotionConfig {
    /// Segment size used by the significant-motion detector.
    ///
    /// Unit: seconds
    /// Scaling: `raw / 50`
    /// Range: `0 ..= 65535`, corresponding to approximately
    /// `0.0s ..= 1310.7s`
    pub block_size: u16,
    /// Minimum peak-to-peak acceleration magnitude.
    ///
    /// Unit: `g`
    /// Scaling: `raw / 512`
    /// Range: `0 ..= 1023`, corresponding to approximately
    /// `0.0g ..= 1.998g`
    pub peak_to_peak_min: u16,
    /// Minimum mean-crossing rate in acceleration magnitude.
    ///
    /// Unit: crossings per second
    /// Range: `0 ..= 63`
    pub mean_crossing_rate_min: u8,
    /// Maximum peak-to-peak acceleration magnitude.
    ///
    /// Unit: `g`
    /// Scaling: `raw / 512`
    /// Range: `0 ..= 1023`, corresponding to approximately
    /// `0.0g ..= 1.998g`
    pub peak_to_peak_max: u16,
    /// Maximum mean-crossing rate in acceleration magnitude.
    ///
    /// Unit: crossings per second
    /// Range: `0 ..= 63`
    pub mean_crossing_rate_max: u8,
    /// Event reporting behavior shared across feature-engine interrupts.
    pub report_mode: EventReportMode,
    /// Shared feature-engine interrupt hold-time exponent.
    pub interrupt_hold: u8,
}

impl SignificantMotionConfig {
    /// Convert a segment size in seconds to the BMI323 field encoding.
    pub fn block_size_from_seconds(seconds: f32) -> u16 {
        (seconds * 50.0).clamp(0.0, 65535.0) as u16
    }

    /// Convert a raw segment-size field value back to seconds.
    pub fn block_size_to_seconds(raw: u16) -> f32 {
        raw as f32 / 50.0
    }

    /// Convert a peak-to-peak acceleration magnitude in `g` to the BMI323
    /// field encoding.
    pub fn peak_to_peak_from_g(g: f32) -> u16 {
        (g * 512.0).clamp(0.0, 1023.0) as u16
    }

    /// Convert a raw peak-to-peak field value back to `g`.
    pub fn peak_to_peak_to_g(raw: u16) -> f32 {
        (raw & 0x03FF) as f32 / 512.0
    }

    /// Convert an interrupt hold time in milliseconds to the BMI323 field
    /// encoding.
    pub fn interrupt_hold_from_millis(millis: f32) -> u8 {
        AnyMotionConfig::interrupt_hold_from_millis(millis)
    }

    /// Convert a raw interrupt-hold field value back to milliseconds.
    pub fn interrupt_hold_to_millis(raw: u8) -> f32 {
        AnyMotionConfig::interrupt_hold_to_millis(raw)
    }
}

/// Configuration for the BMI323 tilt-detection feature.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TiltConfig {
    /// Averaging duration of the acceleration reference vector.
    ///
    /// Unit: seconds
    /// Scaling: `raw / 50`
    /// Range: `0 ..= 255`, corresponding to approximately `0.0s ..= 5.10s`
    pub segment_size: u8,
    /// Minimum tilt angle raw field.
    ///
    /// Unit: degrees
    /// Encoding: nonlinear
    /// Field width: 8 bits
    ///
    /// The datasheet interpretation is `cos(angle) * 256`. The raw value is
    /// exposed directly to avoid adding a floating-point math dependency in
    /// this `no_std` crate.
    pub min_tilt_angle: u8,
    /// Exponential smoothing coefficient for the low-pass mean of the
    /// acceleration vector.
    ///
    /// This is the raw `beta_acc_mean` field from the datasheet.
    pub beta_acc_mean: u16,
    /// Event reporting behavior shared across feature-engine interrupts.
    pub report_mode: EventReportMode,
    /// Shared feature-engine interrupt hold-time exponent.
    pub interrupt_hold: u8,
}

impl TiltConfig {
    /// Convert a reference-vector averaging duration in seconds to the BMI323
    /// field encoding.
    pub fn segment_size_from_seconds(seconds: f32) -> u8 {
        (seconds * 50.0).clamp(0.0, 255.0) as u8
    }

    /// Convert a raw segment-size field value back to seconds.
    pub fn segment_size_to_seconds(raw: u8) -> f32 {
        raw as f32 / 50.0
    }

    /// Convert an interrupt hold time in milliseconds to the BMI323 field
    /// encoding.
    pub fn interrupt_hold_from_millis(millis: f32) -> u8 {
        AnyMotionConfig::interrupt_hold_from_millis(millis)
    }

    /// Convert a raw interrupt-hold field value back to milliseconds.
    pub fn interrupt_hold_to_millis(raw: u8) -> f32 {
        AnyMotionConfig::interrupt_hold_to_millis(raw)
    }
}

/// Reduced accelerometer configuration used by the BMI323 alternate mode.
///
/// Unlike [`AccelConfig`], the alternate configuration does not contain
/// bandwidth or range fields. The hardware only exposes mode, averaging, and
/// output data rate in `ALT_ACC_CONF`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AltAccelConfig {
    /// Sensor operating mode while the alternate configuration is active.
    pub mode: AccelMode,
    /// Sample averaging depth used in the alternate configuration.
    pub average: AverageSamples,
    /// Alternate output data rate.
    pub odr: OutputDataRate,
}

impl AltAccelConfig {
    /// Encode the alternate configuration into the BMI323 register bit layout.
    pub const fn to_word(self) -> u16 {
        ((self.mode as u16) << 12) | ((self.average as u16) << 8) | self.odr as u16
    }
}

/// Reduced gyroscope configuration used by the BMI323 alternate mode.
///
/// Unlike [`GyroConfig`], the alternate configuration does not contain
/// bandwidth or range fields. The hardware only exposes mode, averaging, and
/// output data rate in `ALT_GYR_CONF`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AltGyroConfig {
    /// Sensor operating mode while the alternate configuration is active.
    pub mode: GyroMode,
    /// Sample averaging depth used in the alternate configuration.
    pub average: AverageSamples,
    /// Alternate output data rate.
    pub odr: OutputDataRate,
}

impl AltGyroConfig {
    /// Encode the alternate configuration into the BMI323 register bit layout.
    pub const fn to_word(self) -> u16 {
        ((self.mode as u16) << 12) | ((self.average as u16) << 8) | self.odr as u16
    }
}

/// Supported feature-engine sources that can switch between user and alternate
/// sensor configurations.
///
/// The BMI323 datasheet describes these as switch sources `A..I`. This API
/// names them by their corresponding feature interrupts instead.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AltConfigSwitchSource {
    /// Disable automatic switching for this direction.
    None = 0,
    /// Source `A`: no-motion.
    NoMotion = 1,
    /// Source `B`: any-motion.
    AnyMotion = 2,
    /// Source `C`: flat.
    Flat = 3,
    /// Source `D`: orientation.
    Orientation = 4,
    /// Source `E`: step detector.
    StepDetector = 5,
    /// Source `F`: step-counter watermark.
    StepCounter = 6,
    /// Source `G`: significant motion.
    SignificantMotion = 7,
    /// Source `H`: tilt.
    Tilt = 8,
    /// Source `I`: tap.
    Tap = 9,
}

/// Automatic switching policy between user and alternate accel/gyro
/// configurations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AltConfigControl {
    /// Whether the accelerometer may switch to its alternate configuration.
    pub accel_enabled: bool,
    /// Whether the gyroscope may switch to its alternate configuration.
    pub gyro_enabled: bool,
    /// If enabled, any later write to `ACC_CONF` or `GYR_CONF` immediately
    /// returns the affected sensor to its user configuration.
    pub reset_on_user_config_write: bool,
    /// Feature-engine source that switches sensors into the alternate
    /// configuration.
    pub switch_to_alternate: AltConfigSwitchSource,
    /// Feature-engine source that switches sensors back to the user
    /// configuration.
    pub switch_to_user: AltConfigSwitchSource,
}

/// Current active accel/gyro configuration selection reported by the BMI323.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AltStatus(pub u16);

impl AltStatus {
    /// Returns true when the accelerometer is currently using `ALT_ACC_CONF`.
    pub const fn accel_uses_alternate(self) -> bool {
        self.0 & 0x0001 != 0
    }

    /// Returns true when the gyroscope is currently using `ALT_GYR_CONF`.
    pub const fn gyro_uses_alternate(self) -> bool {
        self.0 & (1 << 4) != 0
    }
}

/// Convenience bundle for a common accel-only alternate-configuration setup.
///
/// This represents a practical battery-oriented pattern:
/// - a low-power user accelerometer configuration for normal operation
/// - a high-performance alternate accelerometer configuration for higher-fidelity
///   sampling after a feature-engine event
/// - an [`AltConfigControl`] value that enables accel switching and keeps gyro
///   on the user configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AltAccelSwitchProfile {
    /// User accelerometer configuration written to `ACC_CONF`.
    pub user_accel: AccelConfig,
    /// Alternate accelerometer configuration written to `ALT_ACC_CONF`.
    pub alternate_accel: AltAccelConfig,
    /// Matching alternate-configuration control policy.
    pub control: AltConfigControl,
}

impl AltAccelSwitchProfile {
    /// Build a recommended accel-only low-power to high-performance switching
    /// profile.
    ///
    /// This helper keeps:
    /// - accelerometer range at [`AccelRange::G8`]
    /// - user accel mode at [`AccelMode::LowPower`] with `Avg2`
    /// - alternate accel mode at [`AccelMode::HighPerformance`] with `Avg1`
    /// - gyro alternate switching disabled
    /// - `reset_on_user_config_write` enabled
    ///
    /// The caller still needs to configure the feature-engine event that will
    /// generate the chosen switch sources, for example any-motion or
    /// no-motion.
    pub const fn low_power_to_high_performance(
        user_odr: OutputDataRate,
        alternate_odr: OutputDataRate,
        switch_to_alternate: AltConfigSwitchSource,
        switch_to_user: AltConfigSwitchSource,
    ) -> Self {
        Self {
            user_accel: AccelConfig {
                mode: AccelMode::LowPower,
                average: AverageSamples::Avg2,
                bandwidth: Bandwidth::OdrOver2,
                range: AccelRange::G8,
                odr: user_odr,
            },
            alternate_accel: AltAccelConfig {
                mode: AccelMode::HighPerformance,
                average: AverageSamples::Avg1,
                odr: alternate_odr,
            },
            control: AltConfigControl {
                accel_enabled: true,
                gyro_enabled: false,
                reset_on_user_config_write: true,
                switch_to_alternate,
                switch_to_user,
            },
        }
    }
}

/// Reference update policy for supported motion detectors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReferenceUpdate {
    /// Update the internal reference only when an event is detected.
    OnDetection,
    /// Continuously update the internal reference.
    EverySample,
}

/// Per-axis enable mask for motion features.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotionAxes {
    /// Enable X-axis contribution.
    pub x: bool,
    /// Enable Y-axis contribution.
    pub y: bool,
    /// Enable Z-axis contribution.
    pub z: bool,
}

impl MotionAxes {
    /// Convenience constant enabling X, Y, and Z axes.
    pub const XYZ: Self = Self {
        x: true,
        y: true,
        z: true,
    };
}

/// Configuration for the BMI323 any-motion feature.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnyMotionConfig {
    /// Axis selection for detection.
    pub axes: MotionAxes,
    /// Minimum acceleration slope for motion detection.
    ///
    /// Unit: `g`
    /// Scaling: `raw / 512`
    /// Range: `0 ..= 4095`, corresponding to approximately `0.0g ..= 7.998g`
    pub threshold: u16,
    /// Hysteresis for the acceleration slope comparator.
    ///
    /// Unit: `g`
    /// Scaling: `raw / 512`
    /// Range: `0 ..= 1023`, corresponding to approximately `0.0g ..= 1.998g`
    pub hysteresis: u16,
    /// Minimum duration for which the slope must stay above `threshold`.
    ///
    /// Unit: seconds
    /// Scaling: `raw / 50`
    /// Range: `0 ..= 8191`, corresponding to approximately `0.0s ..= 163.82s`
    pub duration: u16,
    /// Wait time before the event is cleared after the slope drops below
    /// `threshold`.
    ///
    /// Unit: seconds
    /// Scaling: `raw / 50`
    /// Range: `0 ..= 7`, corresponding to `0.00s ..= 0.14s` in `20ms` steps
    pub wait_time: u8,
    /// Reference update policy.
    pub reference_update: ReferenceUpdate,
    /// Event reporting behavior.
    pub report_mode: EventReportMode,
    /// Interrupt hold-time exponent used by the feature engine.
    ///
    /// Effective hold time in non-latched mode:
    /// `0.625ms * 2^interrupt_hold`
    ///
    /// Valid raw range is `0 ..= 13`. Larger values are clamped to `13` by the
    /// driver before programming the register. This setting is only applicable
    /// to non-latched feature-engine interrupts.
    pub interrupt_hold: u8,
}

impl AnyMotionConfig {
    /// Convert a physical threshold in `g` to the BMI323 field encoding.
    pub fn threshold_from_g(g: f32) -> u16 {
        (g * 512.0).clamp(0.0, 4095.0) as u16
    }

    /// Convert a raw threshold field value back to `g`.
    pub fn threshold_to_g(raw: u16) -> f32 {
        (raw & 0x0FFF) as f32 / 512.0
    }

    /// Convert a physical hysteresis in `g` to the BMI323 field encoding.
    pub fn hysteresis_from_g(g: f32) -> u16 {
        (g * 512.0).clamp(0.0, 1023.0) as u16
    }

    /// Convert a raw hysteresis field value back to `g`.
    pub fn hysteresis_to_g(raw: u16) -> f32 {
        (raw & 0x03FF) as f32 / 512.0
    }

    /// Convert a duration in seconds to the BMI323 field encoding.
    ///
    /// The datasheet specifies a `1/50s` step size.
    pub fn duration_from_seconds(seconds: f32) -> u16 {
        (seconds * 50.0).clamp(0.0, 8191.0) as u16
    }

    /// Convert a raw duration field value back to seconds.
    pub fn duration_to_seconds(raw: u16) -> f32 {
        (raw & 0x1FFF) as f32 / 50.0
    }

    /// Convert a wait time in seconds to the BMI323 field encoding.
    ///
    /// The datasheet specifies a `1/50s` step size and a 3-bit field.
    pub fn wait_time_from_seconds(seconds: f32) -> u8 {
        (seconds * 50.0).clamp(0.0, 7.0) as u8
    }

    /// Convert a raw wait-time field value back to seconds.
    pub fn wait_time_to_seconds(raw: u8) -> f32 {
        (raw & 0x07) as f32 / 50.0
    }

    /// Convert an interrupt hold time in milliseconds to the BMI323 field
    /// encoding.
    ///
    /// The encoded hold time is `0.625ms * 2^n`. This helper returns the
    /// smallest `n` that is greater than or equal to the requested duration and
    /// saturates at `13`.
    pub fn interrupt_hold_from_millis(millis: f32) -> u8 {
        let mut encoded = 0u8;
        let mut hold_ms = 0.625f32;
        while hold_ms < millis && encoded < 13 {
            encoded += 1;
            hold_ms *= 2.0;
        }
        encoded
    }

    /// Convert a raw interrupt-hold field value back to milliseconds.
    pub fn interrupt_hold_to_millis(raw: u8) -> f32 {
        let mut hold_ms = 0.625f32;
        let mut encoded = 0u8;
        while encoded < raw.min(13) {
            hold_ms *= 2.0;
            encoded += 1;
        }
        hold_ms
    }
}

/// Configuration for the BMI323 no-motion feature.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NoMotionConfig {
    /// Axis selection for detection.
    pub axes: MotionAxes,
    /// Maximum acceleration slope allowed for stationary detection.
    ///
    /// Unit: `g`
    /// Scaling: `raw / 512`
    /// Range: `0 ..= 4095`, corresponding to approximately `0.0g ..= 7.998g`
    pub threshold: u16,
    /// Hysteresis for the acceleration slope comparator.
    ///
    /// Unit: `g`
    /// Scaling: `raw / 512`
    /// Range: `0 ..= 1023`, corresponding to approximately `0.0g ..= 1.998g`
    pub hysteresis: u16,
    /// Minimum duration for which the slope must stay below `threshold`.
    ///
    /// Unit: seconds
    /// Scaling: `raw / 50`
    /// Range: `0 ..= 8191`, corresponding to approximately `0.0s ..= 163.82s`
    pub duration: u16,
    /// Wait time before the event is cleared after the slope remains below
    /// `threshold`.
    ///
    /// Unit: seconds
    /// Scaling: `raw / 50`
    /// Range: `0 ..= 7`, corresponding to `0.00s ..= 0.14s` in `20ms` steps
    pub wait_time: u8,
    /// Reference update policy.
    pub reference_update: ReferenceUpdate,
    /// Event reporting behavior.
    pub report_mode: EventReportMode,
    /// Interrupt hold-time exponent used by the feature engine.
    ///
    /// Effective hold time in non-latched mode:
    /// `0.625ms * 2^interrupt_hold`
    ///
    /// Valid raw range is `0 ..= 13`. Larger values are clamped to `13` by the
    /// driver before programming the register. This setting is only applicable
    /// to non-latched feature-engine interrupts.
    pub interrupt_hold: u8,
}

impl NoMotionConfig {
    /// Convert a physical threshold in `g` to the BMI323 field encoding.
    pub fn threshold_from_g(g: f32) -> u16 {
        (g * 512.0).clamp(0.0, 4095.0) as u16
    }

    /// Convert a raw threshold field value back to `g`.
    pub fn threshold_to_g(raw: u16) -> f32 {
        (raw & 0x0FFF) as f32 / 512.0
    }

    /// Convert a physical hysteresis in `g` to the BMI323 field encoding.
    pub fn hysteresis_from_g(g: f32) -> u16 {
        (g * 512.0).clamp(0.0, 1023.0) as u16
    }

    /// Convert a raw hysteresis field value back to `g`.
    pub fn hysteresis_to_g(raw: u16) -> f32 {
        (raw & 0x03FF) as f32 / 512.0
    }

    /// Convert a duration in seconds to the BMI323 field encoding.
    ///
    /// The datasheet specifies a `1/50s` step size.
    pub fn duration_from_seconds(seconds: f32) -> u16 {
        (seconds * 50.0).clamp(0.0, 8191.0) as u16
    }

    /// Convert a raw duration field value back to seconds.
    pub fn duration_to_seconds(raw: u16) -> f32 {
        (raw & 0x1FFF) as f32 / 50.0
    }

    /// Convert a wait time in seconds to the BMI323 field encoding.
    ///
    /// The datasheet specifies a `1/50s` step size and a 3-bit field.
    pub fn wait_time_from_seconds(seconds: f32) -> u8 {
        (seconds * 50.0).clamp(0.0, 7.0) as u8
    }

    /// Convert a raw wait-time field value back to seconds.
    pub fn wait_time_to_seconds(raw: u8) -> f32 {
        (raw & 0x07) as f32 / 50.0
    }

    /// Convert an interrupt hold time in milliseconds to the BMI323 field
    /// encoding.
    ///
    /// The encoded hold time is `0.625ms * 2^n`. This helper returns the
    /// smallest `n` that is greater than or equal to the requested duration and
    /// saturates at `13`.
    pub fn interrupt_hold_from_millis(millis: f32) -> u8 {
        AnyMotionConfig::interrupt_hold_from_millis(millis)
    }

    /// Convert a raw interrupt-hold field value back to milliseconds.
    pub fn interrupt_hold_to_millis(raw: u8) -> f32 {
        AnyMotionConfig::interrupt_hold_to_millis(raw)
    }
}
