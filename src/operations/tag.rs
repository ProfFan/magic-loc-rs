use core::future::{pending, Future};

use dw3000_ng::{self, time::Instant};
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

/// Wait for the poll packet from the anchor to arrive
#[ram]
pub async fn wait_for_poll<SPI, CS, INT, CANCEL>(
    dw3000: dw3000_ng::DW3000<SPI, CS, dw3000_ng::Ready>,
    dwm_config: dw3000_ng::Config,
    node_config: &MagicLocConfig,
    mut int_gpio: &mut INT,
    cancel: CANCEL,
) -> (
    dw3000_ng::DW3000<SPI, CS, dw3000_ng::Ready>,
    Option<(u16, u40, Instant)>,
)
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug + defmt::Format,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug + defmt::Format,
    <CS as embedded_hal::digital::v2::OutputPin>::Error: core::fmt::Debug + defmt::Format,
    INT: embedded_hal_async::digital::Wait,
    CANCEL: Future,
{
    let mut poll_received: Option<(u16, u40, Instant)> = None;
    let ready =
        super::common::listen_for_packet(dw3000, dwm_config, &mut int_gpio, cancel, |buf, rx_ts| {
            let frame = Ieee802154Frame::new_checked(&buf[..]);

            if frame.is_err() {
                return;
            }

            let frame = frame.unwrap();

            defmt::debug!("Frame: {:?}", frame);

            if let Some(Ieee802154Address::Short(src_addr)) = frame.src_addr() {
                let src_addr = u16::from_le_bytes(src_addr);

                defmt::debug!("Src Addr: {:X}", src_addr);

                let index_anchor = node_config
                    .network_topology
                    .anchor_addrs
                    .iter()
                    .position(|&x| x == src_addr);
                if let Some(_) = index_anchor {
                    if let Some(payload) = frame.payload() {
                        let poll_packet = magic_loc_protocol::packet::PollPacket::from(
                            u48::from_le_bytes(payload[..6].try_into().unwrap()),
                        );

                        if poll_packet.packet_type() == magic_loc_protocol::packet::PacketType::Poll
                        {
                            defmt::info!("Poll packet received!");
                            poll_received = Some((src_addr, poll_packet.tx_timestamp(), rx_ts));
                        }
                    }
                }
            }
        })
        .await;

    return (ready, poll_received);
}
