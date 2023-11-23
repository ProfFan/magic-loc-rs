use core::future::pending;

use dw3000_ng::{self, time::Instant};
use hal::gpio::{GpioPin, Input, PullDown};

use arbitrary_int::{u4, u40, u48};

// Protocol Crate
use magic_loc_protocol::packet::PollPacket;

use crate::{config::MagicLocConfig, util::nonblocking_wait};

use smoltcp::wire::{
    Ieee802154Address, Ieee802154Frame, Ieee802154FrameType, Ieee802154FrameVersion, Ieee802154Pan,
    Ieee802154Repr,
};

/// Send a poll packet at a specific device time (in 32-bit ticks)
///
/// NOTE: The 32-bit ticks are the first 32-bits of the 40-bit device time

pub async fn send_poll_packet_at<SPI, CS>(
    mut dw3000: dw3000_ng::DW3000<SPI, CS, dw3000_ng::Ready>,
    dwm_config: &dw3000_ng::Config,
    config: &MagicLocConfig,
    mut int_gpio: &mut GpioPin<Input<PullDown>, 15>,
    at_time: u32,
) -> dw3000_ng::DW3000<SPI, CS, dw3000_ng::Ready>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug + defmt::Format,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug + defmt::Format,
    <CS as embedded_hal::digital::v2::OutputPin>::Error: core::fmt::Debug + defmt::Format,
{
    let mut poll_packet = PollPacket::new(
        magic_loc_protocol::packet::PacketType::Poll,
        u4::new(0),
        u40::new(0x12356789).into(),
    );

    let mut tx_buffer = [0u8; 128];

    let packet = Ieee802154Repr {
        frame_type: Ieee802154FrameType::Data,
        security_enabled: false,
        frame_pending: false,
        ack_request: true,
        pan_id_compression: true,
        frame_version: Ieee802154FrameVersion::Ieee802154,
        sequence_number: Some(1),
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

    return dw3000;
}

pub async fn send_poll_packet<SPI, CS>(
    mut dw3000: dw3000_ng::DW3000<SPI, CS, dw3000_ng::Ready>,
    dwm_config: &dw3000_ng::Config,
    config: &MagicLocConfig,
    mut int_gpio: &mut GpioPin<Input<PullDown>, 15>,
    delay_ns: u32,
) -> (dw3000_ng::DW3000<SPI, CS, dw3000_ng::Ready>, Instant)
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug + defmt::Format,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug + defmt::Format,
    <CS as embedded_hal::digital::v2::OutputPin>::Error: core::fmt::Debug + defmt::Format,
{
    let mut poll_packet = PollPacket::new(
        magic_loc_protocol::packet::PacketType::Poll,
        u4::new(0),
        u40::new(0x12356789).into(),
    );

    let mut tx_buffer = [0u8; 128];

    let packet = Ieee802154Repr {
        frame_type: Ieee802154FrameType::Data,
        security_enabled: false,
        frame_pending: false,
        ack_request: true,
        pan_id_compression: true,
        frame_version: Ieee802154FrameVersion::Ieee802154,
        sequence_number: Some(1),
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
    let mut delay = dw3000_ng::time::Duration::from_nanos(delay_ns);
    delay = dw3000_ng::time::Duration::new((delay.value() >> 8) << 8).unwrap();
    let delayed_tx_time = current_ts + delay;

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

    return (dw3000, delayed_tx_time);
}

/// Wait for the first poll packet from the first anchor to arrive
pub async fn wait_for_first_poll<SPI, CS>(
    dw3000: dw3000_ng::DW3000<SPI, CS, dw3000_ng::Ready>,
    dwm_config: dw3000_ng::Config,
    node_config: &MagicLocConfig,
    mut int_gpio: &mut GpioPin<Input<PullDown>, 15>,
) -> (
    dw3000_ng::DW3000<SPI, CS, dw3000_ng::Ready>,
    Option<Instant>,
)
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug + defmt::Format,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug + defmt::Format,
    <CS as embedded_hal::digital::v2::OutputPin>::Error: core::fmt::Debug + defmt::Format,
{
    let mut poll_received: Option<Instant> = None;
    let ready =
        super::common::listen_for_packet(dw3000, dwm_config, &mut int_gpio, pending::<()>(), |buf, rx_ts| {
            let frame = Ieee802154Frame::new_checked(&buf[..]);

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
                        }
                    }
                }
            }
        })
        .await;

    return (ready, poll_received);
}
