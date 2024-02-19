use core::future::Future;

use dw3000_ng::{self, time::Instant};
use embassy_futures::select::{select, Either};
use hal::macros::ram;

// Protocol Crate
use crate::util::nonblocking_wait;

/// Listen for a packet with a callback function
#[ram]
pub async fn listen_for_packet<SPI, INT, CANCEL>(
    mut dw3000: dw3000_ng::DW3000<SPI, dw3000_ng::Ready>,
    dwm_config: dw3000_ng::Config,
    mut int_gpio: &mut INT,
    cancel: CANCEL,
    callback: impl FnOnce(&[u8], Instant),
) -> (dw3000_ng::DW3000<SPI, dw3000_ng::Ready>, Result<(), ()>)
where
    SPI: embedded_hal::spi::SpiDevice<u8>,
    SPI::Error: core::fmt::Debug + defmt::Format,
    INT: embedded_hal_async::digital::Wait,
    CANCEL: Future,
{
    defmt::trace!("Listening for packet...");
    let mut rxing = dw3000.receive(dwm_config).unwrap_or_else(|e| {
        defmt::error!("Failed to start receiving: {:?}", e);
        panic!("Failed to start receiving");
    });

    defmt::trace!("RX mode set!");

    let mut buf = [0u8; 128];

    let result = select(
        nonblocking_wait(
            || {
                defmt::trace!("Waiting for receive...");
                rxing.r_wait_buf(&mut buf)
            },
            &mut int_gpio,
        ),
        cancel,
    )
    .await;

    // If not timeout, then we have a result
    if let Either::First(result) = result {
        if result.is_err() {
            defmt::error!("Failed to receive: {:?}", result.unwrap_err());

            rxing.force_idle().unwrap();
            dw3000 = rxing.finish_receiving().unwrap();
            return (dw3000, Err(()));
        }

        let (msg_length, rx_time) = result.unwrap();

        callback(&buf[..msg_length], rx_time);
    } else {
        defmt::debug!("Timeout waiting for packet!");

        rxing.force_idle().unwrap();
        dw3000 = rxing.finish_receiving().unwrap();
        return (dw3000, Err(()));
    }

    rxing.force_idle().unwrap();
    dw3000 = rxing.finish_receiving().unwrap();

    (dw3000, Ok(()))
}
