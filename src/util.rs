use dw3000::{self};
use hal::{
    gpio::{GpioPin, Output, PushPull},
    peripherals::SPI2,
    prelude::*,
    spi::{master::Spi, FullDuplexMode},
};

/// Calls `s_wait` on the `Sending` state of the DW3000 driver asynchronously.
///
/// When `s_wait` returns `nb::Error::WouldBlock`, this function will wait for
/// the DW3000's IRQ output to go high, and then call `s_wait` again.
#[inline]
pub async fn nonblocking_s_wait(
    dw3000: &mut dw3000::DW3000<
        Spi<'static, SPI2, FullDuplexMode>,
        GpioPin<Output<PushPull>, 8>,
        dw3000::Sending,
    >,
    int_gpio: &mut impl embedded_hal_async::digital::Wait,
) -> Result<
    dw3000::time::Instant,
    dw3000::Error<Spi<'static, SPI2, FullDuplexMode>, GpioPin<Output<PushPull>, 8>>,
> {
    loop {
        match dw3000.s_wait() {
            Ok(instant) => return Ok(instant),
            Err(nb::Error::WouldBlock) => {
                // Wait for the IRQ output to go high
                int_gpio.wait_for_high().await.unwrap();
                continue;
            }
            Err(nb::Error::Other(e)) => return Err(e),
        }
    }
}
