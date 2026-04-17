use core::marker::PhantomData;

use embedded_hal::i2c::I2c;
use embedded_hal::spi::SpiDevice;
use embedded_hal_async::i2c::I2c as AsyncI2c;
use embedded_hal_async::spi::SpiDevice as AsyncSpiDevice;

use crate::registers::TransportKind;
use crate::{
    AccelRange, AsyncI2cTransport, AsyncSpiTransport, GyroRange, SyncI2cTransport, SyncSpiTransport,
};

/// Blocking BMI323 driver.
///
/// Create this with [`Bmi323::new_i2c`] or [`Bmi323::new_spi`].
pub struct Bmi323<T> {
    pub(crate) transport: T,
    pub(crate) kind: TransportKind,
    pub(crate) accel_range: AccelRange,
    pub(crate) gyro_range: GyroRange,
}

/// Async BMI323 driver.
///
/// Create this with [`Bmi323Async::new_i2c`] or [`Bmi323Async::new_spi`].
pub struct Bmi323Async<T> {
    pub(crate) transport: T,
    pub(crate) kind: TransportKind,
    pub(crate) accel_range: AccelRange,
    pub(crate) gyro_range: GyroRange,
    pub(crate) _not_sync: PhantomData<fn() -> ()>,
}

impl<I2C> Bmi323<SyncI2cTransport<I2C>>
where
    I2C: I2c,
{
    /// Create a blocking BMI323 driver over an I2C bus.
    ///
    /// `address` is the 7-bit BMI323 I2C address selected by hardware.
    pub fn new_i2c(i2c: I2C, address: u8) -> Self {
        Self {
            transport: SyncI2cTransport { bus: i2c, address },
            kind: TransportKind::I2c,
            accel_range: AccelRange::G8,
            gyro_range: GyroRange::Dps2000,
        }
    }

    /// Consume the driver and return ownership of the underlying I2C bus.
    pub fn destroy(self) -> I2C {
        self.transport.bus
    }
}

impl<SPI> Bmi323<SyncSpiTransport<SPI>>
where
    SPI: SpiDevice<u8>,
{
    /// Create a blocking BMI323 driver over an SPI device.
    pub fn new_spi(spi: SPI) -> Self {
        Self {
            transport: SyncSpiTransport { bus: spi },
            kind: TransportKind::Spi,
            accel_range: AccelRange::G8,
            gyro_range: GyroRange::Dps2000,
        }
    }

    /// Consume the driver and return ownership of the underlying SPI device.
    pub fn destroy(self) -> SPI {
        self.transport.bus
    }
}

impl<I2C> Bmi323Async<AsyncI2cTransport<I2C>>
where
    I2C: AsyncI2c,
{
    /// Create an async BMI323 driver over an I2C bus.
    ///
    /// `address` is the 7-bit BMI323 I2C address selected by hardware.
    pub fn new_i2c(i2c: I2C, address: u8) -> Self {
        Self {
            transport: AsyncI2cTransport { bus: i2c, address },
            kind: TransportKind::I2c,
            accel_range: AccelRange::G8,
            gyro_range: GyroRange::Dps2000,
            _not_sync: PhantomData,
        }
    }

    /// Consume the driver and return ownership of the underlying I2C bus.
    pub fn destroy(self) -> I2C {
        self.transport.bus
    }
}

impl<SPI> Bmi323Async<AsyncSpiTransport<SPI>>
where
    SPI: AsyncSpiDevice<u8>,
{
    /// Create an async BMI323 driver over an SPI device.
    pub fn new_spi(spi: SPI) -> Self {
        Self {
            transport: AsyncSpiTransport { bus: spi },
            kind: TransportKind::Spi,
            accel_range: AccelRange::G8,
            gyro_range: GyroRange::Dps2000,
            _not_sync: PhantomData,
        }
    }

    /// Consume the driver and return ownership of the underlying SPI device.
    pub fn destroy(self) -> SPI {
        self.transport.bus
    }
}
