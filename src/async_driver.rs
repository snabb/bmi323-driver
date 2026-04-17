use embedded_hal_async::delay::DelayNs as AsyncDelayNs;
use embedded_hal_async::digital::Wait;

use crate::registers::{
    ACC_CONF, ACC_DATA_X, ALT_ACC_CONF, ALT_CONF, ALT_GYR_CONF, ALT_STATUS, BMI323_CHIP_ID,
    CHIP_ID, CMD, ERR_REG, EXT_ALT_CONFIG_CHG, EXT_ANYMO_1, EXT_ANYMO_2, EXT_ANYMO_3, EXT_FLAT_1,
    EXT_FLAT_2, EXT_GEN_SET_1, EXT_NOMO_1, EXT_NOMO_2, EXT_NOMO_3, EXT_ORIENT_1, EXT_ORIENT_2,
    EXT_SC_1, EXT_SIGMO_1, EXT_SIGMO_2, EXT_SIGMO_3, EXT_ST_RESULT, EXT_ST_SELECT, EXT_TAP_1,
    EXT_TAP_2, EXT_TAP_3, EXT_TILT_1, EXT_TILT_2, FEATURE_CTRL, FEATURE_DATA_ADDR, FEATURE_DATA_TX,
    FEATURE_IO_STATUS, FEATURE_IO0, FEATURE_IO1, FEATURE_IO2, FEATURE_IO3, FIFO_CONF, FIFO_CTRL,
    FIFO_DATA, FIFO_FILL_LEVEL, FIFO_WATERMARK, GYR_CONF, GYR_DATA_X, INT_CONF, IO_INT_CTRL,
    SELF_TEST, SENSOR_TIME_0, SOFT_RESET, STATUS, TEMP_DATA, TransportKind, interrupt_map_location,
    words_to_axis,
};
use crate::{
    AccelConfig, ActiveLevel, AltAccelConfig, AltConfigControl, AltGyroConfig, AltStatus,
    AnyMotionConfig, AsyncAccess, AxisData, Bmi323Async, DeviceState, Error, ErrorWord,
    EventReportMode, FifoConfig, FlatConfig, GyroConfig, ImuData, InterruptChannel,
    InterruptPinConfig, InterruptRoute, InterruptSource, InterruptStatus, NoMotionConfig,
    OrientationConfig, OutputDataRate, OutputMode, ReferenceUpdate, SelfTestDetail, SelfTestResult,
    SelfTestSelection, SignificantMotionConfig, StatusWord, StepCounterConfig, TapConfig,
    TiltConfig,
};

impl<T> Bmi323Async<T>
where
    Self: AsyncAccess,
{
    /// Reset the sensor and verify communication, chip ID, and error state.
    ///
    /// This method performs a soft reset on every call so startup begins from a
    /// known device state. After `init`, configure the accelerometer and
    /// gyroscope explicitly with
    /// [`set_accel_config`](Self::set_accel_config) and
    /// [`set_gyro_config`](Self::set_gyro_config) before depending on sample
    /// reads in application code.
    pub async fn init<D: AsyncDelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<DeviceState, Error<<Self as AsyncAccess>::BusError>> {
        // Reset on every init so the driver starts from a known device state
        // even if the MCU rebooted while the sensor stayed powered.
        self.soft_reset(delay).await?;

        if matches!(self.kind, TransportKind::Spi) {
            self.read_word(CHIP_ID).await.map_err(Error::Bus)?;
            delay.delay_us(250).await;
        }

        let chip_id = self.read_word(CHIP_ID).await.map_err(Error::Bus)? as u8;
        if chip_id != BMI323_CHIP_ID {
            return Err(Error::InvalidChipId(chip_id));
        }

        let error = self.error_word().await?;
        if error.fatal() {
            return Err(Error::FatalError);
        }

        Ok(DeviceState {
            chip_id,
            status: self.status_word().await?,
            error,
        })
    }

    /// Issue the BMI323 soft-reset command and wait for restart completion.
    pub async fn soft_reset<D: AsyncDelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.write_word(CMD, SOFT_RESET).await.map_err(Error::Bus)?;
        delay.delay_ms(2).await;
        Ok(())
    }

    /// Run the BMI323 built-in self-test and return the detailed result.
    ///
    /// For gyroscope self-test, the BMI323 requires the accelerometer to be in
    /// high-performance mode with an output data rate between `12.5 Hz` and
    /// `200 Hz`. This method enforces that prerequisite temporarily when needed
    /// and restores the previous accel/gyro and alternate-configuration
    /// registers afterwards.
    ///
    /// The returned [`SelfTestResult::error_status`] comes from
    /// `FEATURE_IO1.error_status`. On the BMI323, `0x5` is the normal
    /// "no error" value after the feature engine is active.
    pub async fn run_self_test<D: AsyncDelayNs>(
        &mut self,
        delay: &mut D,
        selection: SelfTestSelection,
    ) -> Result<SelfTestResult, Error<<Self as AsyncAccess>::BusError>> {
        let saved_acc_conf = self.read_word(ACC_CONF).await.map_err(Error::Bus)?;
        let saved_gyr_conf = self.read_word(GYR_CONF).await.map_err(Error::Bus)?;
        let saved_alt_acc_conf = self.read_word(ALT_ACC_CONF).await.map_err(Error::Bus)?;
        let saved_alt_gyr_conf = self.read_word(ALT_GYR_CONF).await.map_err(Error::Bus)?;
        let saved_st_select = self.read_feature_word(EXT_ST_SELECT).await?;
        let saved_accel_range = self.accel_range;
        let saved_gyro_range = self.gyro_range;

        let result = self.run_self_test_inner(delay, selection).await;
        let restore = self
            .restore_self_test_configuration(
                saved_acc_conf,
                saved_gyr_conf,
                saved_alt_acc_conf,
                saved_alt_gyr_conf,
                saved_st_select,
            )
            .await;

        self.accel_range = saved_accel_range;
        self.gyro_range = saved_gyro_range;

        match (result, restore) {
            (Err(err), _) => Err(err),
            (Ok(_), Err(err)) => Err(err),
            (Ok(report), Ok(())) => Ok(report),
        }
    }

    /// Read and decode the `STATUS` register.
    pub async fn status_word(
        &mut self,
    ) -> Result<StatusWord, Error<<Self as AsyncAccess>::BusError>> {
        self.read_word(STATUS)
            .await
            .map(StatusWord)
            .map_err(Error::Bus)
    }

    /// Read and decode the `ERR_REG` register.
    pub async fn error_word(
        &mut self,
    ) -> Result<ErrorWord, Error<<Self as AsyncAccess>::BusError>> {
        self.read_word(ERR_REG)
            .await
            .map(ErrorWord)
            .map_err(Error::Bus)
    }

    /// Configure the accelerometer and remember the selected range locally.
    ///
    /// Call this after [`init`](Self::init) before relying on accelerometer or
    /// combined IMU sample reads.
    pub async fn set_accel_config(
        &mut self,
        config: AccelConfig,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.write_word(ACC_CONF, config.to_word())
            .await
            .map_err(Error::Bus)?;
        self.accel_range = config.range;
        Ok(())
    }

    /// Configure the gyroscope and remember the selected range locally.
    ///
    /// Call this after [`init`](Self::init) before relying on gyroscope or
    /// combined IMU sample reads.
    pub async fn set_gyro_config(
        &mut self,
        config: GyroConfig,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.write_word(GYR_CONF, config.to_word())
            .await
            .map_err(Error::Bus)?;
        self.gyro_range = config.range;
        Ok(())
    }

    /// Return the last accelerometer range configured through this driver.
    ///
    /// Before any explicit accelerometer configuration, this returns the
    /// driver's local startup value of `AccelRange::G8`.
    pub fn accel_range(&self) -> crate::AccelRange {
        self.accel_range
    }

    /// Return the last gyroscope range configured through this driver.
    ///
    /// Before any explicit gyroscope configuration, this returns the driver's
    /// local startup value of `GyroRange::Dps2000`.
    pub fn gyro_range(&self) -> crate::GyroRange {
        self.gyro_range
    }

    /// Read a single accelerometer sample.
    ///
    /// For predictable results, configure the accelerometer first with
    /// [`set_accel_config`](Self::set_accel_config).
    pub async fn read_accel(&mut self) -> Result<AxisData, Error<<Self as AsyncAccess>::BusError>> {
        let mut words = [0u16; 3];
        self.read_words(ACC_DATA_X, &mut words)
            .await
            .map_err(Error::Bus)?;
        Ok(words_to_axis(words))
    }

    /// Read a single gyroscope sample.
    ///
    /// For predictable results, configure the gyroscope first with
    /// [`set_gyro_config`](Self::set_gyro_config).
    pub async fn read_gyro(&mut self) -> Result<AxisData, Error<<Self as AsyncAccess>::BusError>> {
        let mut words = [0u16; 3];
        self.read_words(GYR_DATA_X, &mut words)
            .await
            .map_err(Error::Bus)?;
        Ok(words_to_axis(words))
    }

    /// Read accelerometer and gyroscope data in one burst transaction.
    ///
    /// For predictable results, configure both sensors first with
    /// [`set_accel_config`](Self::set_accel_config) and
    /// [`set_gyro_config`](Self::set_gyro_config).
    pub async fn read_imu_data(
        &mut self,
    ) -> Result<ImuData, Error<<Self as AsyncAccess>::BusError>> {
        let mut words = [0u16; 6];
        self.read_words(ACC_DATA_X, &mut words)
            .await
            .map_err(Error::Bus)?;
        Ok(ImuData {
            accel: words_to_axis([words[0], words[1], words[2]]),
            gyro: words_to_axis([words[3], words[4], words[5]]),
        })
    }

    /// Read the temperature sensor and convert it to degrees Celsius.
    pub async fn read_temperature_celsius(
        &mut self,
    ) -> Result<f32, Error<<Self as AsyncAccess>::BusError>> {
        let raw = self.read_word(TEMP_DATA).await.map_err(Error::Bus)? as i16;
        Ok(raw as f32 / 512.0 + 23.0)
    }

    /// Read the 24-bit sensor time counter.
    pub async fn read_sensor_time(
        &mut self,
    ) -> Result<u32, Error<<Self as AsyncAccess>::BusError>> {
        let mut words = [0u16; 2];
        self.read_words(SENSOR_TIME_0, &mut words)
            .await
            .map_err(Error::Bus)?;
        Ok(words[0] as u32 | ((words[1] as u32) << 16))
    }

    /// Configure the electrical behavior of `INT1` or `INT2`.
    ///
    /// Passing [`InterruptChannel::Ibi`] is a no-op: I3C in-band interrupts
    /// have no pin electrical configuration in the BMI323.
    pub async fn configure_interrupt_pin(
        &mut self,
        channel: InterruptChannel,
        config: InterruptPinConfig,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        let mut word = self.read_word(IO_INT_CTRL).await.map_err(Error::Bus)?;
        let shift = match channel {
            InterruptChannel::Int1 => 0,
            InterruptChannel::Int2 => 8,
            InterruptChannel::Ibi => return Ok(()),
        };
        word &= !(0b111 << shift);
        word |= ((matches!(config.active_level, ActiveLevel::High) as u16) << shift)
            | ((matches!(config.output_mode, OutputMode::OpenDrain) as u16) << (shift + 1))
            | ((config.enabled as u16) << (shift + 2));
        self.write_word(IO_INT_CTRL, word).await.map_err(Error::Bus)
    }

    /// Enable or disable latched interrupt behavior inside the sensor.
    pub async fn set_interrupt_latching(
        &mut self,
        latched: bool,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.write_word(INT_CONF, latched as u16)
            .await
            .map_err(Error::Bus)
    }

    /// Route an interrupt source to `INT1`, `INT2`, or I3C IBI.
    pub async fn map_interrupt(
        &mut self,
        source: InterruptSource,
        route: InterruptRoute,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        let (register, shift) = interrupt_map_location(source);
        let mut word = self.read_word(register).await.map_err(Error::Bus)?;
        word &= !(0b11 << shift);
        word |= u16::from(route) << shift;
        self.write_word(register, word).await.map_err(Error::Bus)
    }

    /// Read the interrupt status register for the selected output channel.
    pub async fn read_interrupt_status(
        &mut self,
        channel: InterruptChannel,
    ) -> Result<InterruptStatus, Error<<Self as AsyncAccess>::BusError>> {
        let reg = match channel {
            InterruptChannel::Int1 => 0x0D,
            InterruptChannel::Int2 => 0x0E,
            InterruptChannel::Ibi => 0x0F,
        };
        self.read_word(reg)
            .await
            .map(InterruptStatus)
            .map_err(Error::Bus)
    }

    /// Wait for an external GPIO interrupt line to assert, then read status.
    ///
    /// Any error from `pin.wait_for_high()` is silently discarded. The
    /// interrupt status register is read regardless, so if the pin wait fails
    /// the status read still proceeds. If you need to handle GPIO errors, call
    /// `pin.wait_for_high()` yourself and then call
    /// [`read_interrupt_status`](Self::read_interrupt_status) directly.
    pub async fn wait_for_interrupt<P: Wait>(
        &mut self,
        pin: &mut P,
        channel: InterruptChannel,
    ) -> Result<InterruptStatus, Error<<Self as AsyncAccess>::BusError>> {
        pin.wait_for_high().await.ok();
        self.read_interrupt_status(channel).await
    }

    /// Configure FIFO contents and watermark level.
    pub async fn set_fifo_config(
        &mut self,
        config: FifoConfig,
        watermark_words: u16,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.write_word(FIFO_WATERMARK, watermark_words & 0x03FF)
            .await
            .map_err(Error::Bus)?;
        self.write_word(FIFO_CONF, config.to_word())
            .await
            .map_err(Error::Bus)
    }

    /// Read the current FIFO fill level in 16-bit words.
    pub async fn fifo_fill_level(&mut self) -> Result<u16, Error<<Self as AsyncAccess>::BusError>> {
        Ok(self.read_word(FIFO_FILL_LEVEL).await.map_err(Error::Bus)? & 0x07FF)
    }

    /// Flush all currently buffered FIFO contents.
    pub async fn flush_fifo(&mut self) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.write_word(FIFO_CTRL, 0x0001).await.map_err(Error::Bus)
    }

    /// Read raw FIFO words into the provided output slice.
    ///
    /// `words.len()` must not exceed 64 due to the fixed internal transfer
    /// buffer. Use [`fifo_fill_level`](Self::fifo_fill_level) first and split
    /// larger reads into chunks of at most 64 words.
    ///
    /// # Panics
    ///
    /// Panics if `words.len() > 64`.
    pub async fn read_fifo_words(
        &mut self,
        words: &mut [u16],
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        assert!(
            words.len() <= 64,
            "read_fifo_words: words.len() must not exceed 64"
        );
        self.read_words(FIFO_DATA, words).await.map_err(Error::Bus)
    }

    /// Enable the BMI323 feature engine required by advanced motion features.
    ///
    /// This method temporarily disables the accelerometer and gyroscope by
    /// writing to `ACC_CONF` and `GYR_CONF` as part of the enable sequence.
    /// After calling this method, re-configure the sensors with
    /// [`set_accel_config`](Self::set_accel_config) and
    /// [`set_gyro_config`](Self::set_gyro_config) before reading samples.
    pub async fn enable_feature_engine(
        &mut self,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.write_word(ACC_CONF, 0).await.map_err(Error::Bus)?;
        self.write_word(GYR_CONF, 0).await.map_err(Error::Bus)?;
        self.write_word(FEATURE_IO2, 0x012C)
            .await
            .map_err(Error::Bus)?;
        self.write_word(FEATURE_IO_STATUS, 0x0001)
            .await
            .map_err(Error::Bus)?;
        self.write_word(FEATURE_CTRL, 0x0001)
            .await
            .map_err(Error::Bus)?;

        for _ in 0..32 {
            let io1 = self.read_word(FEATURE_IO1).await.map_err(Error::Bus)?;
            let status = (io1 & 0x000F) as u8;
            if status == 0x1 || status == 0x5 {
                return Ok(());
            }
        }
        let io1 = self.read_word(FEATURE_IO1).await.map_err(Error::Bus)?;
        Err(Error::FeatureEngineNotReady((io1 & 0x000F) as u8))
    }

    /// Configure the BMI323 any-motion feature-engine block.
    pub async fn configure_any_motion(
        &mut self,
        config: AnyMotionConfig,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.apply_common_feature_settings(config.report_mode, config.interrupt_hold)
            .await?;
        self.write_feature_word(
            EXT_ANYMO_1,
            (config.threshold & 0x0FFF)
                | (match config.reference_update {
                    ReferenceUpdate::OnDetection => 0,
                    ReferenceUpdate::EverySample => 1,
                } << 12),
        )
        .await?;
        self.write_feature_word(EXT_ANYMO_2, config.hysteresis & 0x03FF)
            .await?;
        self.write_feature_word(
            EXT_ANYMO_3,
            (config.duration & 0x1FFF) | (((config.wait_time as u16) & 0x07) << 13),
        )
        .await?;
        self.modify_word(FEATURE_IO0, |mut word| {
            word &= !(0b111 << 3);
            word |= (config.axes.x as u16) << 3;
            word |= (config.axes.y as u16) << 4;
            word |= (config.axes.z as u16) << 5;
            word
        })
        .await
    }

    /// Configure the BMI323 no-motion feature-engine block.
    pub async fn configure_no_motion(
        &mut self,
        config: NoMotionConfig,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.apply_common_feature_settings(config.report_mode, config.interrupt_hold)
            .await?;
        self.write_feature_word(
            EXT_NOMO_1,
            (config.threshold & 0x0FFF)
                | (match config.reference_update {
                    ReferenceUpdate::OnDetection => 0,
                    ReferenceUpdate::EverySample => 1,
                } << 12),
        )
        .await?;
        self.write_feature_word(EXT_NOMO_2, config.hysteresis & 0x03FF)
            .await?;
        self.write_feature_word(
            EXT_NOMO_3,
            (config.duration & 0x1FFF) | (((config.wait_time as u16) & 0x07) << 13),
        )
        .await?;
        self.modify_word(FEATURE_IO0, |mut word| {
            word &= !0b111;
            word |= config.axes.x as u16;
            word |= (config.axes.y as u16) << 1;
            word |= (config.axes.z as u16) << 2;
            word
        })
        .await
    }

    /// Configure the BMI323 flat-detection feature-engine block and enable the
    /// flat detector.
    pub async fn configure_flat(
        &mut self,
        config: FlatConfig,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.apply_common_feature_settings(config.report_mode, config.interrupt_hold)
            .await?;
        self.write_feature_word(
            EXT_FLAT_1,
            u16::from(config.theta & 0x3F)
                | ((config.blocking as u16) << 6)
                | (u16::from(config.hold_time) << 8),
        )
        .await?;
        self.write_feature_word(
            EXT_FLAT_2,
            u16::from(config.slope_threshold) | (u16::from(config.hysteresis) << 8),
        )
        .await?;
        self.set_flat_enabled(true).await
    }

    /// Enable or disable the flat-detection feature.
    pub async fn set_flat_enabled(
        &mut self,
        enabled: bool,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.modify_word(FEATURE_IO0, |mut word| {
            if enabled {
                word |= 1 << 6;
            } else {
                word &= !(1 << 6);
            }
            word
        })
        .await
    }

    /// Configure the BMI323 orientation-detection feature-engine block and
    /// enable the orientation detector.
    pub async fn configure_orientation(
        &mut self,
        config: OrientationConfig,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.apply_common_feature_settings(config.report_mode, config.interrupt_hold)
            .await?;
        self.write_feature_word(
            EXT_ORIENT_1,
            (config.upside_down_enabled as u16)
                | ((config.mode as u16) << 1)
                | ((config.blocking as u16) << 3)
                | ((u16::from(config.theta & 0x3F)) << 5)
                | ((u16::from(config.hold_time & 0x1F)) << 11),
        )
        .await?;
        self.write_feature_word(
            EXT_ORIENT_2,
            u16::from(config.slope_threshold) | (u16::from(config.hysteresis) << 8),
        )
        .await?;
        self.set_orientation_enabled(true).await
    }

    /// Enable or disable the orientation-detection feature.
    pub async fn set_orientation_enabled(
        &mut self,
        enabled: bool,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.modify_word(FEATURE_IO0, |mut word| {
            if enabled {
                word |= 1 << 7;
            } else {
                word &= !(1 << 7);
            }
            word
        })
        .await
    }

    /// Configure the BMI323 tap-detection feature-engine block.
    ///
    /// The interrupt status register exposes a single tap bit, even when
    /// single-, double-, and triple-tap detection are enabled together.
    pub async fn configure_tap(
        &mut self,
        config: TapConfig,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.apply_common_feature_settings(config.report_mode, config.interrupt_hold)
            .await?;
        self.write_feature_word(
            EXT_TAP_1,
            (config.axis as u16)
                | ((config.reporting_mode as u16) << 2)
                | ((u16::from(config.max_peaks_for_tap & 0x07)) << 3)
                | ((config.mode as u16) << 6),
        )
        .await?;
        self.write_feature_word(
            EXT_TAP_2,
            (config.tap_peak_threshold & 0x03FF)
                | ((u16::from(config.max_gesture_duration & 0x3F)) << 10),
        )
        .await?;
        self.write_feature_word(
            EXT_TAP_3,
            u16::from(config.max_duration_between_peaks & 0x0F)
                | (u16::from(config.tap_shock_settling_duration & 0x0F) << 4)
                | (u16::from(config.min_quiet_duration_between_taps & 0x0F) << 8)
                | (u16::from(config.quiet_time_after_gesture & 0x0F) << 12),
        )
        .await?;
        self.modify_word(FEATURE_IO0, |mut word| {
            word &= !((1 << 12) | (1 << 13) | (1 << 14));
            word |= (config.single_tap_enabled as u16) << 12;
            word |= (config.double_tap_enabled as u16) << 13;
            word |= (config.triple_tap_enabled as u16) << 14;
            word
        })
        .await
    }

    /// Configure the BMI323 significant-motion feature-engine block and enable
    /// the significant-motion detector.
    pub async fn configure_significant_motion(
        &mut self,
        config: SignificantMotionConfig,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.apply_common_feature_settings(config.report_mode, config.interrupt_hold)
            .await?;
        self.write_feature_word(EXT_SIGMO_1, config.block_size)
            .await?;
        self.write_feature_word(
            EXT_SIGMO_2,
            (config.peak_to_peak_min & 0x03FF)
                | ((u16::from(config.mean_crossing_rate_min & 0x3F)) << 10),
        )
        .await?;
        self.write_feature_word(
            EXT_SIGMO_3,
            (config.peak_to_peak_max & 0x03FF)
                | ((u16::from(config.mean_crossing_rate_max & 0x3F)) << 10),
        )
        .await?;
        self.set_significant_motion_enabled(true).await
    }

    /// Enable or disable the significant-motion feature.
    pub async fn set_significant_motion_enabled(
        &mut self,
        enabled: bool,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.modify_word(FEATURE_IO0, |mut word| {
            if enabled {
                word |= 1 << 10;
            } else {
                word &= !(1 << 10);
            }
            word
        })
        .await
    }

    /// Configure the BMI323 tilt-detection feature-engine block and enable the
    /// tilt detector.
    pub async fn configure_tilt(
        &mut self,
        config: TiltConfig,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.apply_common_feature_settings(config.report_mode, config.interrupt_hold)
            .await?;
        self.write_feature_word(
            EXT_TILT_1,
            u16::from(config.segment_size) | (u16::from(config.min_tilt_angle) << 8),
        )
        .await?;
        self.write_feature_word(EXT_TILT_2, config.beta_acc_mean)
            .await?;
        self.set_tilt_enabled(true).await
    }

    /// Enable or disable the tilt-detection feature.
    pub async fn set_tilt_enabled(
        &mut self,
        enabled: bool,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.modify_word(FEATURE_IO0, |mut word| {
            if enabled {
                word |= 1 << 11;
            } else {
                word &= !(1 << 11);
            }
            word
        })
        .await
    }

    /// Enable or disable the feature-engine step detector.
    ///
    /// When enabled, the BMI323 can assert the
    /// [`InterruptSource::StepDetector`] interrupt source after the feature
    /// engine has been enabled with [`enable_feature_engine`](Self::enable_feature_engine).
    pub async fn set_step_detector_enabled(
        &mut self,
        enabled: bool,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.modify_word(FEATURE_IO0, |mut word| {
            if enabled {
                word |= 1 << 8;
            } else {
                word &= !(1 << 8);
            }
            word
        })
        .await
    }

    /// Enable or disable the feature-engine step counter.
    ///
    /// This controls the BMI323 step-count accumulation path and the
    /// [`InterruptSource::StepCounter`] interrupt source. The feature engine
    /// must already be enabled with [`enable_feature_engine`](Self::enable_feature_engine).
    pub async fn set_step_counter_enabled(
        &mut self,
        enabled: bool,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.modify_word(FEATURE_IO0, |mut word| {
            if enabled {
                word |= 1 << 9;
            } else {
                word &= !(1 << 9);
            }
            word
        })
        .await
    }

    /// Configure the BMI323 step-counter watermark and optional reset request.
    ///
    /// A watermark of `0` disables the step-counter interrupt source while
    /// still allowing the accumulated step count to be read.
    pub async fn configure_step_counter(
        &mut self,
        config: StepCounterConfig,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.write_feature_word(EXT_SC_1, config.to_word()).await
    }

    /// Request a reset of the accumulated step count.
    pub async fn reset_step_counter(
        &mut self,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.configure_step_counter(StepCounterConfig {
            reset_counter: true,
            ..StepCounterConfig::disabled()
        })
        .await
    }

    /// Read the 32-bit accumulated step count from the feature engine.
    pub async fn read_step_count(&mut self) -> Result<u32, Error<<Self as AsyncAccess>::BusError>> {
        let low = self.read_word(FEATURE_IO2).await.map_err(Error::Bus)? as u32;
        let high = self.read_word(FEATURE_IO3).await.map_err(Error::Bus)? as u32;
        Ok(low | (high << 16))
    }

    /// Program the alternate accelerometer configuration in `ALT_ACC_CONF`.
    pub async fn set_alt_accel_config(
        &mut self,
        config: AltAccelConfig,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.write_word(ALT_ACC_CONF, config.to_word())
            .await
            .map_err(Error::Bus)
    }

    /// Program the alternate gyroscope configuration in `ALT_GYR_CONF`.
    pub async fn set_alt_gyro_config(
        &mut self,
        config: AltGyroConfig,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.write_word(ALT_GYR_CONF, config.to_word())
            .await
            .map_err(Error::Bus)
    }

    /// Configure automatic switching between user and alternate sensor
    /// configurations.
    pub async fn configure_alt_config_control(
        &mut self,
        config: AltConfigControl,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.write_word(
            ALT_CONF,
            (config.accel_enabled as u16)
                | ((config.gyro_enabled as u16) << 4)
                | ((config.reset_on_user_config_write as u16) << 8),
        )
        .await
        .map_err(Error::Bus)?;
        self.write_feature_word(
            EXT_ALT_CONFIG_CHG,
            (config.switch_to_alternate as u16) | ((config.switch_to_user as u16) << 4),
        )
        .await
    }

    /// Read which configuration set is currently active for accel and gyro.
    pub async fn alt_status(
        &mut self,
    ) -> Result<AltStatus, Error<<Self as AsyncAccess>::BusError>> {
        self.read_word(ALT_STATUS)
            .await
            .map(AltStatus)
            .map_err(Error::Bus)
    }

    async fn modify_word<F>(
        &mut self,
        reg: u8,
        f: F,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>>
    where
        F: FnOnce(u16) -> u16,
    {
        let value = self.read_word(reg).await.map_err(Error::Bus)?;
        self.write_word(reg, f(value)).await.map_err(Error::Bus)?;
        if reg == FEATURE_IO0 {
            self.write_word(FEATURE_IO_STATUS, 1)
                .await
                .map_err(Error::Bus)?;
        }
        Ok(())
    }

    async fn run_self_test_inner<D: AsyncDelayNs>(
        &mut self,
        delay: &mut D,
        selection: SelfTestSelection,
    ) -> Result<SelfTestResult, Error<<Self as AsyncAccess>::BusError>> {
        self.enable_feature_engine().await?;

        if selection.tests_gyroscope() {
            self.write_word(
                ACC_CONF,
                AccelConfig {
                    mode: crate::AccelMode::HighPerformance,
                    odr: OutputDataRate::Hz200,
                    ..Default::default()
                }
                .to_word(),
            )
            .await
            .map_err(Error::Bus)?;
        }

        self.write_word(ALT_ACC_CONF, 0).await.map_err(Error::Bus)?;
        self.write_word(ALT_GYR_CONF, 0).await.map_err(Error::Bus)?;
        self.write_feature_word(EXT_ST_SELECT, selection.to_word())
            .await?;
        self.write_word(CMD, SELF_TEST).await.map_err(Error::Bus)?;

        for _ in 0..50 {
            delay.delay_ms(10).await;
            let feature_io1 = self.read_word(FEATURE_IO1).await.map_err(Error::Bus)?;
            if feature_io1 & (1 << 4) != 0 {
                let detail = SelfTestDetail(self.read_feature_word(EXT_ST_RESULT).await?);
                return Ok(SelfTestResult {
                    selection,
                    passed: feature_io1 & (1 << 6) != 0,
                    sample_rate_error: feature_io1 & (1 << 7) != 0,
                    error_status: (feature_io1 & 0x000F) as u8,
                    detail,
                });
            }
        }

        Err(Error::SelfTestTimeout)
    }

    async fn restore_self_test_configuration(
        &mut self,
        acc_conf: u16,
        gyr_conf: u16,
        alt_acc_conf: u16,
        alt_gyr_conf: u16,
        st_select: u16,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.write_word(ACC_CONF, acc_conf)
            .await
            .map_err(Error::Bus)?;
        self.write_word(GYR_CONF, gyr_conf)
            .await
            .map_err(Error::Bus)?;
        self.write_word(ALT_ACC_CONF, alt_acc_conf)
            .await
            .map_err(Error::Bus)?;
        self.write_word(ALT_GYR_CONF, alt_gyr_conf)
            .await
            .map_err(Error::Bus)?;
        self.write_feature_word(EXT_ST_SELECT, st_select).await
    }

    async fn write_feature_word(
        &mut self,
        ext_addr: u16,
        value: u16,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.write_word(FEATURE_DATA_ADDR, ext_addr)
            .await
            .map_err(Error::Bus)?;
        self.write_word(FEATURE_DATA_TX, value)
            .await
            .map_err(Error::Bus)
    }

    async fn read_feature_word(
        &mut self,
        ext_addr: u16,
    ) -> Result<u16, Error<<Self as AsyncAccess>::BusError>> {
        self.write_word(FEATURE_DATA_ADDR, ext_addr)
            .await
            .map_err(Error::Bus)?;
        self.read_word(FEATURE_DATA_TX).await.map_err(Error::Bus)
    }

    async fn modify_feature_word<F>(
        &mut self,
        ext_addr: u16,
        f: F,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>>
    where
        F: FnOnce(u16) -> u16,
    {
        let word = self.read_feature_word(ext_addr).await?;
        self.write_feature_word(ext_addr, f(word)).await
    }

    /// Write `report_mode` and `interrupt_hold` into `EXT_GEN_SET_1`.
    ///
    /// `EXT_GEN_SET_1` is a single shared register used by all
    /// feature-engine blocks. Every call to a `configure_*` method overwrites
    /// the same two fields, so the last call's values take effect across all
    /// features. If your application uses multiple feature-engine blocks,
    /// configure them all with consistent `report_mode` and `interrupt_hold`
    /// values, or be aware that later calls overwrite earlier ones.
    async fn apply_common_feature_settings(
        &mut self,
        report_mode: EventReportMode,
        interrupt_hold: u8,
    ) -> Result<(), Error<<Self as AsyncAccess>::BusError>> {
        self.modify_feature_word(EXT_GEN_SET_1, |word| {
            let mut updated = word & !0x003F;
            updated |= match report_mode {
                EventReportMode::AllEvents => 0,
                EventReportMode::FirstEventOnly => 1,
            };
            updated |= ((interrupt_hold.min(13) as u16) & 0x0F) << 1;
            updated
        })
        .await
    }
}
