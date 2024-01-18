use core::future::Future;

use dw3000_ng::{self, time::Instant};
use hal::macros::ram;

use arbitrary_int::{u4, u40, u48};

// Protocol Crate
use magic_loc_protocol::packet::{FinalPacket, PacketType, ResponsePacket};
use zerocopy::FromBytes;

use crate::{config::MagicLocConfig, util::nonblocking_wait};

use smoltcp::wire::{
    Ieee802154Address, Ieee802154Frame, Ieee802154FrameType, Ieee802154FrameVersion, Ieee802154Pan,
    Ieee802154Repr,
};

/// Wait for the poll packet from the anchor to arrive
#[ram]
pub async fn wait_for_poll<SPI, INT, CANCEL>(
    dw3000: dw3000_ng::DW3000<SPI, dw3000_ng::Ready>,
    dwm_config: dw3000_ng::Config,
    node_config: &MagicLocConfig,
    mut int_gpio: &mut INT,
    cancel: CANCEL,
) -> (
    dw3000_ng::DW3000<SPI, dw3000_ng::Ready>,
    Option<(u16, u40, Instant, u8)>,
)
where
    SPI: embedded_hal::spi::SpiDevice<u8>,
    SPI::Error: core::fmt::Debug + defmt::Format,
    INT: embedded_hal_async::digital::Wait,
    CANCEL: Future,
{
    let mut poll_received: Option<(u16, u40, Instant, u8)> = None;
    let (ready, _) = super::common::listen_for_packet(
        dw3000,
        dwm_config,
        &mut int_gpio,
        cancel,
        |buf, rx_ts| {
            let frame = Ieee802154Frame::new_checked(&buf[..]);

            if frame.is_err() {
                return;
            }

            let frame = frame.unwrap();

            defmt::debug!("Frame: {:?}", frame);

            if let Some(Ieee802154Address::Short(src_addr)) = frame.src_addr() {
                let src_addr = u16::from_le_bytes(src_addr);

                defmt::debug!("Src Addr: {:#X}", src_addr);

                let index_anchor = node_config
                    .network_topology
                    .anchor_addrs
                    .iter()
                    .position(|&x| x == src_addr);
                if index_anchor.is_some() {
                    if let Some(payload) = frame.payload() {
                        let poll_packet = magic_loc_protocol::packet::PollPacket::from(
                            u48::from_le_bytes(payload[..6].try_into().unwrap()),
                        );

                        if poll_packet.packet_type() == magic_loc_protocol::packet::PacketType::Poll
                        {
                            defmt::debug!("Poll packet received!");
                            poll_received = Some((
                                src_addr,
                                poll_packet.tx_timestamp(),
                                rx_ts,
                                frame.sequence_number().unwrap(),
                            ));
                        }
                    }
                }
            }
        },
    )
    .await;

    return (ready, poll_received);
}

#[ram]
pub async fn send_response_packet_at<SPI, INT>(
    dw3000: dw3000_ng::DW3000<SPI, dw3000_ng::Ready>,
    dwm_config: dw3000_ng::Config,
    config: &MagicLocConfig,
    mut int_gpio: &mut INT,
    delay_tx_time: u32,
    sequence_number: u8,
) -> (dw3000_ng::DW3000<SPI, dw3000_ng::Ready>, Result<(), ()>)
where
    SPI: embedded_hal::spi::SpiDevice<u8>,
    SPI::Error: core::fmt::Debug + defmt::Format,
    INT: embedded_hal_async::digital::Wait,
{
    let response_packet = ResponsePacket::new(PacketType::Response, u4::new(0));

    let mut tx_buffer = [0u8; 32];

    let packet = Ieee802154Repr {
        frame_type: Ieee802154FrameType::Data,
        security_enabled: false,
        frame_pending: false,
        ack_request: false,
        pan_id_compression: true,
        frame_version: Ieee802154FrameVersion::Ieee802154,
        sequence_number: Some(sequence_number),
        dst_pan_id: Some(Ieee802154Pan(config.uwb_pan_id)),
        dst_addr: Some(Ieee802154Address::BROADCAST),
        src_pan_id: None,
        src_addr: Some(Ieee802154Address::from_bytes(
            &config.uwb_addr.to_le_bytes(),
        )),
    };

    let mut frame = Ieee802154Frame::new_unchecked(&mut tx_buffer);
    packet.emit(&mut frame);

    // Send the frame
    let mac_packet_size = packet.buffer_len() + 1;

    defmt::debug!("Sending frame of size: {}", mac_packet_size);

    let payload = frame.payload_mut().unwrap();
    payload[..1].copy_from_slice(&u8::from(response_packet).to_le_bytes());

    let mut txing = dw3000
        .send_raw(
            &tx_buffer[..mac_packet_size],
            dw3000_ng::hl::SendTime::Delayed(Instant::new((delay_tx_time as u64) << 8).unwrap()),
            &dwm_config,
        )
        .unwrap();

    let result = nonblocking_wait(
        || {
            defmt::debug!("Waiting for send...");
            txing.s_wait()
        },
        &mut int_gpio,
    )
    .await;

    match result {
        Ok(_) => {
            defmt::debug!("Sent!");
        }
        Err(e) => {
            defmt::error!("Failed to send!");
            defmt::error!("Error: {:?}", e);

            txing.force_idle().unwrap();
            return (txing.finish_sending().unwrap(), Err(()));
        }
    }

    let event_reg = txing.ll().sys_status().read().unwrap();

    defmt::debug!("Should have finished, HPDWARN: {}", event_reg.hpdwarn());

    txing.force_idle().unwrap(); // Seems that if we don't do this, the next send will fail

    let dw3000 = txing.finish_sending().unwrap();

    (dw3000, Ok(()))
}

/// Wait for the final packet from the tag to arrive
///
/// Returns the rx time of the final packet
#[ram]
pub async fn wait_for_final<SPI, INT, CANCEL>(
    dw3000: dw3000_ng::DW3000<SPI, dw3000_ng::Ready>,
    dwm_config: dw3000_ng::Config,
    node_config: &MagicLocConfig,
    mut int_gpio: &mut INT,
    cancel: CANCEL,
) -> (
    dw3000_ng::DW3000<SPI, dw3000_ng::Ready>,
    Option<(u16, FinalPacket, Instant)>,
    bool,
)
where
    SPI: embedded_hal::spi::SpiDevice<u8>,
    SPI::Error: core::fmt::Debug + defmt::Format,
    INT: embedded_hal_async::digital::Wait,
    CANCEL: Future,
{
    let mut final_received: Option<(u16, FinalPacket, Instant)> = None;
    let (ready, recv_ok) = super::common::listen_for_packet(
        dw3000,
        dwm_config,
        &mut int_gpio,
        cancel,
        |buf, rx_ts| {
            let frame = Ieee802154Frame::new_checked(&buf[..]);

            if frame.is_err() {
                return;
            }

            let frame = frame.unwrap();

            defmt::debug!("Frame: {:?}", frame);

            if let Some(Ieee802154Address::Short(src_addr)) = frame.src_addr() {
                let src_addr = u16::from_le_bytes(src_addr);

                defmt::debug!("Src Addr: {:#X}", src_addr);

                let index_tag = node_config
                    .network_topology
                    .anchor_addrs
                    .iter()
                    .position(|&x| x == src_addr);
                if let Some(_) = index_tag {
                    if let Some(payload) = frame.payload() {
                        defmt::debug!("Payload: {:#X}", payload);
                        if let Some(final_packet) = FinalPacket::ref_from(&payload[..21]) {
                            let header = final_packet.header();
                            if header.packet_type() == magic_loc_protocol::packet::PacketType::Final
                            {
                                defmt::debug!("Final packet received!");
                                final_received = Some((src_addr, final_packet.clone(), rx_ts));
                            }
                        }
                    }
                }
            }
        },
    )
    .await;

    (ready, final_received, recv_ok.is_ok())
}
