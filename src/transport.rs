use embedded_hal::i2c::I2c;
use embedded_hal::spi::{Operation, SpiDevice};
use embedded_hal_async::i2c::I2c as AsyncI2c;
use embedded_hal_async::spi::SpiDevice as AsyncSpiDevice;

use crate::{Bmi323, Bmi323Async};

/// Blocking I2C transport wrapper used by [`Bmi323`].
///
/// Most users do not need to construct this directly. Prefer
/// [`Bmi323::new_i2c`](crate::Bmi323::new_i2c).
pub struct SyncI2cTransport<I2C> {
    pub(crate) bus: I2C,
    pub(crate) address: u8,
}

/// Blocking SPI transport wrapper used by [`Bmi323`].
///
/// Most users do not need to construct this directly. Prefer
/// [`Bmi323::new_spi`](crate::Bmi323::new_spi).
pub struct SyncSpiTransport<SPI> {
    pub(crate) bus: SPI,
}

/// Async I2C transport wrapper used by [`Bmi323Async`].
///
/// Most users do not need to construct this directly. Prefer
/// [`Bmi323Async::new_i2c`](crate::Bmi323Async::new_i2c).
pub struct AsyncI2cTransport<I2C> {
    pub(crate) bus: I2C,
    pub(crate) address: u8,
}

/// Async SPI transport wrapper used by [`Bmi323Async`].
///
/// Most users do not need to construct this directly. Prefer
/// [`Bmi323Async::new_spi`](crate::Bmi323Async::new_spi).
pub struct AsyncSpiTransport<SPI> {
    pub(crate) bus: SPI,
}

/// Low-level blocking register access contract used by the blocking driver.
///
/// This trait is public so the driver can remain transport-agnostic, but most
/// users will rely on the built-in I2C and SPI implementations.
pub trait SyncAccess {
    /// Underlying bus error type returned by the transport.
    type BusError;

    /// Read a single 16-bit register payload from the BMI323.
    fn read_word(&mut self, reg: u8) -> Result<u16, Self::BusError>;

    /// Write a single 16-bit register payload to the BMI323.
    fn write_word(&mut self, reg: u8, word: u16) -> Result<(), Self::BusError>;

    /// Read multiple consecutive 16-bit register payloads starting at `reg`.
    fn read_words(&mut self, reg: u8, words: &mut [u16]) -> Result<(), Self::BusError>;
}

#[allow(async_fn_in_trait)]
/// Low-level async register access contract used by the async driver.
///
/// This trait is public so the driver can remain transport-agnostic, but most
/// users will rely on the built-in I2C and SPI implementations.
pub trait AsyncAccess {
    /// Underlying bus error type returned by the transport.
    type BusError;

    /// Read a single 16-bit register payload from the BMI323.
    async fn read_word(&mut self, reg: u8) -> Result<u16, Self::BusError>;

    /// Write a single 16-bit register payload to the BMI323.
    async fn write_word(&mut self, reg: u8, word: u16) -> Result<(), Self::BusError>;

    /// Read multiple consecutive 16-bit register payloads starting at `reg`.
    async fn read_words(&mut self, reg: u8, words: &mut [u16]) -> Result<(), Self::BusError>;
}

impl<I2C> SyncAccess for Bmi323<SyncI2cTransport<I2C>>
where
    I2C: I2c,
{
    type BusError = I2C::Error;

    fn read_word(&mut self, reg: u8) -> Result<u16, Self::BusError> {
        let mut bytes = [0u8; 4];
        self.transport
            .bus
            .write_read(self.transport.address, &[reg], &mut bytes)?;
        Ok(u16::from_le_bytes([bytes[2], bytes[3]]))
    }

    fn write_word(&mut self, reg: u8, word: u16) -> Result<(), Self::BusError> {
        let [lo, hi] = word.to_le_bytes();
        self.transport
            .bus
            .write(self.transport.address, &[reg, lo, hi])
    }

    fn read_words(&mut self, reg: u8, words: &mut [u16]) -> Result<(), Self::BusError> {
        let mut bytes = [0u8; 2 * 64 + 2];
        let byte_len = words.len() * 2 + 2;
        self.transport
            .bus
            .write_read(self.transport.address, &[reg], &mut bytes[..byte_len])?;
        for (index, word) in words.iter_mut().enumerate() {
            let offset = 2 + index * 2;
            *word = u16::from_le_bytes([bytes[offset], bytes[offset + 1]]);
        }
        Ok(())
    }
}

impl<SPI> SyncAccess for Bmi323<SyncSpiTransport<SPI>>
where
    SPI: SpiDevice<u8>,
{
    type BusError = SPI::Error;

    fn read_word(&mut self, reg: u8) -> Result<u16, Self::BusError> {
        let cmd = 0x80 | (reg & 0x7F);
        let mut bytes = [0u8; 3];
        let mut ops = [Operation::Write(&[cmd]), Operation::Read(&mut bytes)];
        self.transport.bus.transaction(&mut ops)?;
        Ok(u16::from_le_bytes([bytes[1], bytes[2]]))
    }

    fn write_word(&mut self, reg: u8, word: u16) -> Result<(), Self::BusError> {
        let [lo, hi] = word.to_le_bytes();
        let payload = [reg & 0x7F, lo, hi];
        self.transport.bus.write(&payload)
    }

    fn read_words(&mut self, reg: u8, words: &mut [u16]) -> Result<(), Self::BusError> {
        let cmd = 0x80 | (reg & 0x7F);
        let mut bytes = [0u8; 2 * 64 + 1];
        let byte_len = words.len() * 2 + 1;
        let mut ops = [
            Operation::Write(&[cmd]),
            Operation::Read(&mut bytes[..byte_len]),
        ];
        self.transport.bus.transaction(&mut ops)?;
        for (index, word) in words.iter_mut().enumerate() {
            let offset = 1 + index * 2;
            *word = u16::from_le_bytes([bytes[offset], bytes[offset + 1]]);
        }
        Ok(())
    }
}

impl<I2C> AsyncAccess for Bmi323Async<AsyncI2cTransport<I2C>>
where
    I2C: AsyncI2c,
{
    type BusError = I2C::Error;

    async fn read_word(&mut self, reg: u8) -> Result<u16, Self::BusError> {
        let mut bytes = [0u8; 4];
        self.transport
            .bus
            .write_read(self.transport.address, &[reg], &mut bytes)
            .await?;
        Ok(u16::from_le_bytes([bytes[2], bytes[3]]))
    }

    async fn write_word(&mut self, reg: u8, word: u16) -> Result<(), Self::BusError> {
        let [lo, hi] = word.to_le_bytes();
        self.transport
            .bus
            .write(self.transport.address, &[reg, lo, hi])
            .await
    }

    async fn read_words(&mut self, reg: u8, words: &mut [u16]) -> Result<(), Self::BusError> {
        let mut bytes = [0u8; 2 * 64 + 2];
        let byte_len = words.len() * 2 + 2;
        self.transport
            .bus
            .write_read(self.transport.address, &[reg], &mut bytes[..byte_len])
            .await?;
        for (index, word) in words.iter_mut().enumerate() {
            let offset = 2 + index * 2;
            *word = u16::from_le_bytes([bytes[offset], bytes[offset + 1]]);
        }
        Ok(())
    }
}

impl<SPI> AsyncAccess for Bmi323Async<AsyncSpiTransport<SPI>>
where
    SPI: AsyncSpiDevice<u8>,
{
    type BusError = SPI::Error;

    async fn read_word(&mut self, reg: u8) -> Result<u16, Self::BusError> {
        let cmd = 0x80 | (reg & 0x7F);
        let mut bytes = [0u8; 3];
        let mut ops = [Operation::Write(&[cmd]), Operation::Read(&mut bytes)];
        self.transport.bus.transaction(&mut ops).await?;
        Ok(u16::from_le_bytes([bytes[1], bytes[2]]))
    }

    async fn write_word(&mut self, reg: u8, word: u16) -> Result<(), Self::BusError> {
        let [lo, hi] = word.to_le_bytes();
        let payload = [reg & 0x7F, lo, hi];
        self.transport.bus.write(&payload).await
    }

    async fn read_words(&mut self, reg: u8, words: &mut [u16]) -> Result<(), Self::BusError> {
        let cmd = 0x80 | (reg & 0x7F);
        let mut bytes = [0u8; 2 * 64 + 1];
        let byte_len = words.len() * 2 + 1;
        let mut ops = [
            Operation::Write(&[cmd]),
            Operation::Read(&mut bytes[..byte_len]),
        ];
        self.transport.bus.transaction(&mut ops).await?;
        for (index, word) in words.iter_mut().enumerate() {
            let offset = 1 + index * 2;
            *word = u16::from_le_bytes([bytes[offset], bytes[offset + 1]]);
        }
        Ok(())
    }
}
