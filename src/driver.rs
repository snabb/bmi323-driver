use core::fmt;

#[cfg(feature = "blocking")]
use embedded_hal::i2c::I2c as HalI2c;
#[cfg(feature = "blocking")]
use embedded_hal::spi::SpiDevice as HalSpiDevice;
#[cfg(not(feature = "blocking"))]
use embedded_hal_async::i2c::I2c as HalI2c;
#[cfg(not(feature = "blocking"))]
use embedded_hal_async::spi::SpiDevice as HalSpiDevice;

use crate::registers::TransportKind;
use crate::types::Error;
use crate::{AccelRange, GyroRange, I2cTransport, SpiTransport};

/// BMI323 driver.
///
/// Async mode (the default) exposes all methods as `async fn`. Add
/// `features = ["blocking"]` (with `default-features = false`) to get the
/// synchronous embedded-hal API instead. Create with [`Bmi323::new_i2c`] or
/// [`Bmi323::new_spi`].
pub struct Bmi323<T> {
    pub(crate) transport: T,
    pub(crate) kind: TransportKind,
    pub(crate) accel_range: AccelRange,
    pub(crate) gyro_range: GyroRange,
}

impl<T> fmt::Debug for Bmi323<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Bmi323")
            .field("kind", &self.kind)
            .field("accel_range", &self.accel_range)
            .field("gyro_range", &self.gyro_range)
            .finish_non_exhaustive()
    }
}

impl<I2C: HalI2c> Bmi323<I2cTransport<I2C>> {
    /// Create a BMI323 driver over an I2C bus.
    ///
    /// `address` is the 7-bit BMI323 I2C address selected by hardware.
    pub fn new_i2c(i2c: I2C, address: u8) -> Self {
        Self {
            transport: I2cTransport { bus: i2c, address },
            kind: TransportKind::I2c,
            accel_range: AccelRange::G2,
            gyro_range: GyroRange::Dps125,
        }
    }

    /// Consume the driver and return ownership of the underlying I2C bus.
    pub fn destroy(self) -> I2C {
        self.transport.bus
    }
}

impl<SPI: HalSpiDevice<u8>> Bmi323<SpiTransport<SPI>> {
    /// Create a BMI323 driver over an SPI device.
    pub fn new_spi(spi: SPI) -> Self {
        Self {
            transport: SpiTransport { bus: spi },
            kind: TransportKind::Spi,
            accel_range: AccelRange::G2,
            gyro_range: GyroRange::Dps125,
        }
    }

    /// Consume the driver and return ownership of the underlying SPI device.
    pub fn destroy(self) -> SPI {
        self.transport.bus
    }
}

/// Convert a raw TEMP_DATA register value to degrees Celsius.
///
/// Returns `Err(Error::InvalidTemperature)` when the raw value is 0x8000,
/// the BMI323 "invalid temperature" sentinel
/// (§6.1.2, Register (0x09) temp_data; §5.6.3).
///
/// The conversion formula `T = raw / 512 + 23` is defined in §5.6.3.
pub(crate) fn temperature_raw_to_celsius<E>(raw: i16) -> Result<f32, Error<E>> {
    if raw == i16::MIN {
        return Err(Error::InvalidTemperature);
    }
    Ok(raw as f32 / 512.0 + 23.0)
}
