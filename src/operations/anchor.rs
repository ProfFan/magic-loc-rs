use core::future::pending;

use dw3000_ng::{self, time::Instant};
use hal::gpio::{GpioPin, Input, PullDown};

use arbitrary_int::{u4, u40, u48};

// Protocol Crate
use magic_loc_protocol::packet::PollPacket;
use zerocopy::transmute;

use crate::{config::MagicLocConfig, util::nonblocking_wait};

use smoltcp::wire::{
    Ieee802154Address, Ieee802154Frame, Ieee802154FrameType, Ieee802154FrameVersion, Ieee802154Pan,
    Ieee802154Repr,
};

/// Send a poll packet at a specific device time (in 32-bit ticks)
///
/// NOTE: The 32-bit ticks are the first 32-bits of the 40-bit device time

pub async fn send_poll_packet_at<SPI>(
    mut dw3000: dw3000_ng::DW3000<SPI, dw3000_ng::Ready>,
    dwm_config: &dw3000_ng::Config,
    config: &MagicLocConfig,
    mut int_gpio: &mut GpioPin<Input<PullDown>, 15>,
    at_time: u32,
    sequence_number: u8,
) -> dw3000_ng::DW3000<SPI, dw3000_ng::Ready>
where
    SPI: embedded_hal::spi::SpiDevice<u8>,
    SPI::Error: core::fmt::Debug + defmt::Format,
{
    let mut poll_packet = PollPacket::new(
        magic_loc_protocol::packet::PacketType::Poll,
        u4::new(0),
        u40::new(0x12356789),
    );

    let mut tx_buffer = [0u8; 128];

    let packet = Ieee802154Repr {
        frame_type: Ieee802154FrameType::Data,
        security_enabled: false,
        frame_pending: false,
        ack_request: true,
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
    let mac_packet_size = packet.buffer_len() + 6;

    defmt::debug!("Sending frame of size: {}", mac_packet_size);

    let tx_time = Instant::new((at_time as u64) << 8).unwrap();

    // Set the delayed send time
    poll_packet.set_tx_timestamp(u40::new(tx_time.value()));

    let payload = frame.payload_mut().unwrap();
    payload[..6].copy_from_slice(&u48::from(poll_packet).to_le_bytes());

    let mut txing = dw3000
        .send_raw(
            &tx_buffer[..mac_packet_size],
            dw3000_ng::hl::SendTime::Delayed(tx_time),
            dwm_config,
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

    defmt::debug!("Current TS: {}", tx_time);

    match result {
        Ok(_) => {
            defmt::debug!("Sent!");
        }
        Err(e) => {
            defmt::error!("Failed to send!");
            defmt::error!("Error: {:?}", e);
        }
    }

    let event_reg = txing.ll().sys_status().read().unwrap();

    defmt::debug!("Should have finished, HPDWARN: {}", event_reg.hpdwarn());

    txing.force_idle().unwrap(); // Seems that if we don't do this, the next send will fail

    dw3000 = txing.finish_sending().unwrap();

    dw3000
}

pub async fn send_poll_packet<SPI>(
    mut dw3000: dw3000_ng::DW3000<SPI, dw3000_ng::Ready>,
    dwm_config: &dw3000_ng::Config,
    config: &MagicLocConfig,
    mut int_gpio: &mut GpioPin<Input<PullDown>, 15>,
    delay_ns: u32,
    sequence_number: u8,
) -> (dw3000_ng::DW3000<SPI, dw3000_ng::Ready>, Instant)
where
    SPI: embedded_hal::spi::SpiDevice<u8>,
    SPI::Error: core::fmt::Debug + defmt::Format,
{
    let mut poll_packet = PollPacket::new(
        magic_loc_protocol::packet::PacketType::Poll,
        u4::new(0),
        u40::new(0x12356789),
    );

    let mut tx_buffer = [0u8; 128];

    let packet = Ieee802154Repr {
        frame_type: Ieee802154FrameType::Data,
        security_enabled: false,
        frame_pending: false,
        ack_request: true,
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
    let mac_packet_size = packet.buffer_len() + 6;

    defmt::debug!("Sending frame of size: {}", mac_packet_size);

    let current_ts = Instant::new((dw3000.sys_time().unwrap() as u64) << 8).unwrap();

    // Round to 32-bit
    let delay = dw3000_ng::time::Duration::from_nanos(delay_ns);
    let delayed_tx_time_unrounded = (current_ts + delay).value() % (1 << 40);
    let delayed_tx_time = Instant::new((delayed_tx_time_unrounded >> 9) << 9).unwrap();

    // Set the delayed send time
    poll_packet.set_tx_timestamp(u40::new(delayed_tx_time.value()));

    let payload = frame.payload_mut().unwrap();
    payload[..6].copy_from_slice(&u48::from(poll_packet).to_le_bytes());

    let mut txing = dw3000
        .send_raw(
            &tx_buffer[..mac_packet_size],
            dw3000_ng::hl::SendTime::Delayed(delayed_tx_time),
            dwm_config,
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

    defmt::debug!("Current TS: {}", current_ts);
    defmt::debug!("Delayed TS: {}", delayed_tx_time);

    match result {
        Ok(_) => {
            defmt::debug!("Sent!");
        }
        Err(e) => {
            defmt::error!("Failed to send!");
            defmt::error!("Error: {:?}", e);
        }
    }

    let event_reg = txing.ll().sys_status().read().unwrap();

    defmt::debug!("Should have finished, HPDWARN: {}", event_reg.hpdwarn());

    txing.force_idle().unwrap(); // Seems that if we don't do this, the next send will fail

    dw3000 = txing.finish_sending().unwrap();

    (dw3000, delayed_tx_time)
}

/// Wait for the first poll packet from the first anchor to arrive
pub async fn wait_for_first_poll<SPI>(
    dw3000: dw3000_ng::DW3000<SPI, dw3000_ng::Ready>,
    dwm_config: dw3000_ng::Config,
    node_config: &MagicLocConfig,
    mut int_gpio: &mut GpioPin<Input<PullDown>, 15>,
) -> (
    dw3000_ng::DW3000<SPI, dw3000_ng::Ready>,
    Option<Instant>,
    u8,
)
where
    SPI: embedded_hal::spi::SpiDevice<u8>,
    SPI::Error: core::fmt::Debug + defmt::Format,
{
    let mut poll_received: Option<Instant> = None;
    let mut sequence_number: u8 = 0;
    let (ready, _) = super::common::listen_for_packet(
        dw3000,
        dwm_config,
        &mut int_gpio,
        pending::<()>(),
        |buf, rx_ts| {
            let frame = Ieee802154Frame::new_checked(buf);

            if frame.is_err() {
                return;
            }

            let frame = frame.unwrap();

            defmt::debug!("Frame: {:?}", frame);

            if let Some(src_addr) = frame.src_addr() {
                if src_addr
                    == Ieee802154Address::from_bytes(
                        &node_config.network_topology.anchor_addrs[0].to_le_bytes(),
                    )
                {
                    if let Some(payload) = frame.payload() {
                        let poll_packet = magic_loc_protocol::packet::PollPacket::from(
                            u48::from_le_bytes(payload[..6].try_into().unwrap()),
                        );

                        if poll_packet.packet_type() == magic_loc_protocol::packet::PacketType::Poll
                        {
                            defmt::info!("Poll packet received!");
                            poll_received = Some(rx_ts);
                            sequence_number = frame.sequence_number().unwrap();
                        }
                    }
                }
            }
        },
    )
    .await;

    (ready, poll_received, sequence_number)
}

/// Wait for the response packet from the tag
pub async fn wait_for_response<SPI>(
    dw3000: dw3000_ng::DW3000<SPI, dw3000_ng::Ready>,
    dwm_config: dw3000_ng::Config,
    node_config: &MagicLocConfig,
    mut int_gpio: &mut GpioPin<Input<PullDown>, 15>,
    cancel: impl core::future::Future,
) -> (
    dw3000_ng::DW3000<SPI, dw3000_ng::Ready>,
    Option<(u16, Instant)>,
    bool,
    u8,
)
where
    SPI: embedded_hal::spi::SpiDevice<u8>,
    SPI::Error: core::fmt::Debug + defmt::Format,
{
    let mut response_received: Option<(u16, Instant)> = None;
    let mut received_sequence_number: u8 = 0;
    let (ready, result) = super::common::listen_for_packet(
        dw3000,
        dwm_config,
        &mut int_gpio,
        cancel,
        |buf, rx_ts| {
            let frame = Ieee802154Frame::new_checked(buf);

            if frame.is_err() {
                return;
            }

            let frame = frame.unwrap();

            defmt::debug!("Frame: {:?}", frame);

            if let Some(src_addr) = frame.src_addr() {
                if node_config
                    .network_topology
                    .tag_addrs
                    .contains(&u16::from_le_bytes(src_addr.as_bytes().try_into().unwrap()))
                {
                    if let Some(payload) = frame.payload() {
                        let response_packet = magic_loc_protocol::packet::ResponsePacket::from(
                            u8::from_le_bytes(payload[..1].try_into().unwrap()),
                        );

                        if response_packet.packet_type()
                            == magic_loc_protocol::packet::PacketType::Response
                        {
                            defmt::info!("Response packet received!");
                            response_received = Some((
                                u16::from_le_bytes(src_addr.as_bytes().try_into().unwrap()),
                                rx_ts,
                            ));
                            received_sequence_number = frame.sequence_number().unwrap();
                        }
                    }
                }
            }
        },
    )
    .await;

    (ready, response_received, result.is_err(), received_sequence_number)
}

/// Send the FINAL packet
///
/// NOTE: This is the packet that contains the RX timestamp of the response packet
pub async fn send_final_packet<SPI>(
    mut dw3000: dw3000_ng::DW3000<SPI, dw3000_ng::Ready>,
    dwm_config: &dw3000_ng::Config,
    config: &MagicLocConfig,
    mut int_gpio: &mut GpioPin<Input<PullDown>, 15>,
    response_rx_ts: &[Option<u64>],
    final_tx_slot: u32,
    sequence_number: u8,
) -> dw3000_ng::DW3000<SPI, dw3000_ng::Ready>
where
    SPI: embedded_hal::spi::SpiDevice<u8>,
    SPI::Error: core::fmt::Debug + defmt::Format,
{
    let final_packet = magic_loc_protocol::packet::FinalPacket::new(
        magic_loc_protocol::packet::PacketType::Final,
        u4::new(0),
        core::array::from_fn(|i| u40::new(response_rx_ts[i].unwrap_or(0))),
        u40::new((final_tx_slot as u64) << 8),
    );

    let mut tx_buffer = [0u8; 64];

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
    let mac_packet_size = packet.buffer_len() + 21;

    defmt::debug!("Sending frame of size: {}", mac_packet_size);

    let payload = frame.payload_mut().unwrap();
    let final_bytes: [u8; 21] = transmute!(final_packet);
    payload[..21].copy_from_slice(&final_bytes);

    let mut txing = dw3000
        .send_raw(
            &tx_buffer[..mac_packet_size],
            dw3000_ng::hl::SendTime::Delayed(Instant::new((final_tx_slot as u64) << 8).unwrap()),
            dwm_config,
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

    defmt::debug!("Current TS: {}", response_rx_ts);

    match result {
        Ok(_) => {
            defmt::debug!("Sent!");
        }
        Err(e) => {
            defmt::error!("Failed to send!");
            defmt::error!("Error: {:?}", e);
        }
    }

    let event_reg = txing.ll().sys_status().read().unwrap();

    defmt::debug!("Should have finished, HPDWARN: {}", event_reg.hpdwarn());

    txing.force_idle().unwrap(); // Seems that if we don't do this, the next send will fail

    dw3000 = txing.finish_sending().unwrap();

    dw3000
}
