#![no_std]
#![doc = include_str!("../README.md")]

#[cfg(all(feature = "blocking", feature = "async"))]
compile_error!(
    "features \"blocking\" and \"async\" are mutually exclusive; \
    choose one or neither (async is the default when neither is specified)"
);

#[cfg(test)]
extern crate std;

mod driver;
mod driver_impl;
mod registers;
mod transport;
mod types;

pub use driver::Bmi323;
pub use transport::{Access, I2cTransport, MAX_WORDS_PER_READ, SpiTransport};
pub use types::*;

#[cfg(test)]
mod tests;
