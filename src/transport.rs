use embedded_hal::spi::Operation;

#[cfg(not(feature = "async"))]
use embedded_hal::i2c::I2c as HalI2c;
#[cfg(not(feature = "async"))]
use embedded_hal::spi::SpiDevice as HalSpiDevice;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c as HalI2c;
#[cfg(feature = "async")]
use embedded_hal_async::spi::SpiDevice as HalSpiDevice;

use crate::Bmi323;

/// Maximum number of 16-bit words that a single `read_words` call may request.
///
/// Limited by the internal transfer scratch buffer. Callers that need more
/// data must split the request into chunks of at most this size.
pub const MAX_WORDS_PER_READ: usize = 64;

/// I2C transport wrapper used by [`Bmi323`].
///
/// Most users do not need to construct this directly. Prefer
/// [`Bmi323::new_i2c`](crate::Bmi323::new_i2c).
pub struct I2cTransport<I2C> {
    pub(crate) bus: I2C,
    pub(crate) address: u8,
}

/// SPI transport wrapper used by [`Bmi323`].
///
/// Most users do not need to construct this directly. Prefer
/// [`Bmi323::new_spi`](crate::Bmi323::new_spi).
pub struct SpiTransport<SPI> {
    pub(crate) bus: SPI,
}

/// Low-level register access contract used by the driver.
///
/// This trait is public so the driver can remain transport-agnostic, but most
/// users will rely on the built-in I2C and SPI implementations.
///
/// In async mode (`features = ["async"]`) the methods are `async fn`.
/// In blocking mode (`features = ["blocking"]`, the default) they are regular `fn`.
#[cfg(not(feature = "async"))]
pub trait Access {
    /// Underlying bus error type returned by the transport.
    type BusError;
    /// Read a single 16-bit register payload from the BMI323.
    fn read_word(&mut self, reg: u8) -> Result<u16, Self::BusError>;
    /// Write a single 16-bit register payload to the BMI323.
    fn write_word(&mut self, reg: u8, word: u16) -> Result<(), Self::BusError>;
    /// Read multiple consecutive 16-bit register payloads starting at `reg`.
    fn read_words(&mut self, reg: u8, words: &mut [u16]) -> Result<(), Self::BusError>;
}

#[cfg(feature = "async")]
#[allow(async_fn_in_trait)]
pub trait Access {
    /// Underlying bus error type returned by the transport.
    type BusError;
    /// Read a single 16-bit register payload from the BMI323.
    async fn read_word(&mut self, reg: u8) -> Result<u16, Self::BusError>;
    /// Write a single 16-bit register payload to the BMI323.
    async fn write_word(&mut self, reg: u8, word: u16) -> Result<(), Self::BusError>;
    /// Read multiple consecutive 16-bit register payloads starting at `reg`.
    async fn read_words(&mut self, reg: u8, words: &mut [u16]) -> Result<(), Self::BusError>;
}

// ---- Blocking I2C impl ----

#[cfg(not(feature = "async"))]
impl<I2C: HalI2c> Access for Bmi323<I2cTransport<I2C>> {
    type BusError = I2C::Error;

    fn read_word(&mut self, reg: u8) -> Result<u16, Self::BusError> {
        // 2 dummy bytes precede payload on I2C reads (§7.2.4.2)
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
        // 2 dummy bytes precede payload on I2C reads (§7.2.4.2); 2 bytes per word
        let mut bytes = [0u8; 2 + MAX_WORDS_PER_READ * 2];
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

// ---- Async I2C impl ----

#[cfg(feature = "async")]
impl<I2C: HalI2c> Access for Bmi323<I2cTransport<I2C>> {
    type BusError = I2C::Error;

    async fn read_word(&mut self, reg: u8) -> Result<u16, Self::BusError> {
        // 2 dummy bytes precede payload on I2C reads (§7.2.4.2)
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
        // 2 dummy bytes precede payload on I2C reads (§7.2.4.2); 2 bytes per word
        let mut bytes = [0u8; 2 + MAX_WORDS_PER_READ * 2];
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

// ---- Blocking SPI impl ----

#[cfg(not(feature = "async"))]
impl<SPI: HalSpiDevice<u8>> Access for Bmi323<SpiTransport<SPI>> {
    type BusError = SPI::Error;

    fn read_word(&mut self, reg: u8) -> Result<u16, Self::BusError> {
        // MSB of address byte set to 1 for reads (§7.2.3)
        let cmd = 0x80 | (reg & 0x7F);
        // 1 dummy byte precedes the 2 payload bytes on SPI reads (§7.2.3)
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
        // MSB of address byte set to 1 for reads; 1 dummy byte precedes payload (§7.2.3)
        let cmd = 0x80 | (reg & 0x7F);
        // 1 dummy byte + 2 bytes per word
        let mut bytes = [0u8; 1 + MAX_WORDS_PER_READ * 2];
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

// ---- Async SPI impl ----

#[cfg(feature = "async")]
impl<SPI: HalSpiDevice<u8>> Access for Bmi323<SpiTransport<SPI>> {
    type BusError = SPI::Error;

    async fn read_word(&mut self, reg: u8) -> Result<u16, Self::BusError> {
        // MSB of address byte set to 1 for reads (§7.2.3)
        let cmd = 0x80 | (reg & 0x7F);
        // 1 dummy byte precedes the 2 payload bytes on SPI reads (§7.2.3)
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
        // MSB of address byte set to 1 for reads; 1 dummy byte precedes payload (§7.2.3)
        let cmd = 0x80 | (reg & 0x7F);
        // 1 dummy byte + 2 bytes per word
        let mut bytes = [0u8; 1 + MAX_WORDS_PER_READ * 2];
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
