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

        // Check IP_TOAST to ensure the ToA status is good
        // TODO: Move this out of the listen_for_packet function
        let ip_toast = rxing.ll().ip_ts().read().unwrap().ip_toast();
        if ip_toast != 0 {
            defmt::error!("IP_TOAST: {}", ip_toast);

            rxing.force_idle().unwrap();
            dw3000 = rxing.finish_receiving().unwrap();
            return (dw3000, Err(()));
        }

        // Check FP_INDEX to ensure the frame is valid
        let fp_index = rxing.ll().ip_diag_8().read().unwrap().ip_fp();

        // fp_index is a 16-bit fixed point number, 10.6 format
        // The integer part is the first 10 bits, and the fractional part is the last 6 bits
        let fp_index_int = fp_index >> 6;
        if fp_index_int < 730 && fp_index_int > 0 {
            defmt::warn!("FP_INDEX DETECTED AT: {}", fp_index >> 6);

            rxing.force_idle().unwrap();
            dw3000 = rxing.finish_receiving().unwrap();
            return (dw3000, Err(()));
        }

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
