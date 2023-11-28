use core::{future::Future, result};

use dw3000_ng::{self, time::Instant};
use embassy_futures::select::{select, Either};
use hal::{
    gpio::{GpioPin, Input, PullDown},
    macros::ram,
};

use arbitrary_int::{u4, u40, u48};

// Protocol Crate
use magic_loc_protocol::packet::PollPacket;

use crate::{config::MagicLocConfig, util::nonblocking_wait};

use smoltcp::wire::{
    Ieee802154Address, Ieee802154Frame, Ieee802154FrameType, Ieee802154FrameVersion, Ieee802154Pan,
    Ieee802154Repr,
};

/// Listen for a packet with a callback function
#[ram]
pub async fn listen_for_packet<SPI, CS, INT, CANCEL>(
    mut dw3000: dw3000_ng::DW3000<SPI, CS, dw3000_ng::Ready>,
    dwm_config: dw3000_ng::Config,
    mut int_gpio: &mut INT,
    cancel: CANCEL,
    callback: impl FnOnce(&[u8], Instant),
) -> (dw3000_ng::DW3000<SPI, CS, dw3000_ng::Ready>, Result<(), ()>)
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug + defmt::Format,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug + defmt::Format,
    <CS as embedded_hal::digital::v2::OutputPin>::Error: core::fmt::Debug + defmt::Format,
    INT: embedded_hal_async::digital::Wait,
    CANCEL: Future,
{
    let mut rxing = dw3000.receive(dwm_config).expect("Failed to receive.");

    let mut buf = [0u8; 128];

    let result = select(
        nonblocking_wait(
            || {
                defmt::trace!("Waiting for receive...");
                let status = rxing.r_wait_buf(&mut buf);
                let sys_status = rxing.ll().sys_status().read().unwrap();
                defmt::trace!("SYS_STATUS: {:?}", sys_status);
                return status;
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

    return (dw3000, Ok(()));
}
