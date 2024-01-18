use embedded_hal::spi;

/// Indicates that the `LSM6DSO` instance is not initialized yet
#[derive(Debug)]
pub struct Uninitialized;

/// Indicates that the `LSM6DSO` instance is ready to be used
#[derive(Debug)]
pub struct Ready;

pub struct LSM6DSO<SPI, State> {
    ll: crate::ll::LSM6DSO<SPI>,
    _state: State,
}

impl<SPI> LSM6DSO<SPI, Uninitialized> {
    /// Create a new instance of `DW3000`
    ///
    /// Requires the SPI peripheral and the chip select pin that are connected
    /// to the DW3000.
    pub fn new(spi: SPI) -> Self {
        LSM6DSO {
            ll: crate::ll::LSM6DSO::new(spi),
            _state: Uninitialized,
        }
    }

    /// Direct low level access to the underlying peripheral
    pub fn ll(&mut self) -> &mut crate::ll::LSM6DSO<SPI> {
        &mut self.ll
    }
}
