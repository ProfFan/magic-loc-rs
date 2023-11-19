use dw3000_ng::{self, hl::ConfigGPIOs, time::Instant};
use embassy_time::{Duration, Ticker, Timer};
use hal::{
    gpio::{GpioPin, Input, Output, PullDown, PushPull},
    peripherals::SPI2,
    prelude::*,
    spi::{master::Spi, FullDuplexMode},
};

use arbitrary_int::{u4, u40, u48};
use heapless::Vec;

// Protocol Crate
use magic_loc_protocol::{anchor_state_machine::*, packet::PollPacket};

use crate::{config::MagicLocConfig, util::nonblocking_wait};

use smoltcp::wire::{
    Ieee802154Address, Ieee802154Frame, Ieee802154FrameType, Ieee802154FrameVersion, Ieee802154Pan,
    Ieee802154Repr,
};

#[embassy_executor::task]
pub async fn uwb_anchor_task(
    bus: Spi<'static, SPI2, FullDuplexMode>,
    cs_gpio: GpioPin<Output<PushPull>, 8>,
    mut rst_gpio: GpioPin<Output<PushPull>, 9>,
    mut int_gpio: GpioPin<Input<PullDown>, 15>,
    node_config: MagicLocConfig,
) -> ! {
    defmt::info!("UWB Anchor Task Start!");

    let mut config = dw3000_ng::Config::default();
    config.bitrate = dw3000_ng::configs::BitRate::Kbps850;

    // Reset
    rst_gpio.set_low().unwrap();

    Timer::after(Duration::from_millis(10)).await;

    rst_gpio.set_high().unwrap();

    defmt::info!("DW3000 Reset!");

    Timer::after(Duration::from_millis(200)).await;

    let mut dw3000 = dw3000_ng::DW3000::new(bus, cs_gpio)
        .init()
        .expect("Failed init.")
        .config(config)
        .expect("Failed config.");

    dw3000.gpio_config(ConfigGPIOs::enable_led()).unwrap();
    dw3000
        .ll()
        .led_ctrl()
        .modify(|_, w| w.blink_tim(0x2))
        .unwrap();

    Timer::after(Duration::from_millis(200)).await;

    // Disable SPIRDY interrupt
    dw3000.disable_spirdy_interrupt().unwrap();

    dw3000.enable_tx_interrupts().unwrap();

    // Calculate the time to send
    let time_frame = magic_loc_protocol::util::frame_tx_time(12, &config, true);

    defmt::info!("Time to send: {:?} ns", time_frame);

    let fsm = AnchorSideStateMachine::new(
        node_config.uwb_addr,
        Vec::from_slice(&node_config.network_topology.anchor_addrs).unwrap(),
        Vec::from_slice(&node_config.network_topology.tag_addrs).unwrap(),
    );

    let is_first_anchor = node_config.uwb_addr == node_config.network_topology.anchor_addrs[0];

    let mut ticker = Ticker::every(Duration::from_millis(1000));

    loop {
        if is_first_anchor {
            // First anchor will send the first frame
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
                dst_pan_id: Some(Ieee802154Pan(node_config.uwb_pan_id)),
                dst_addr: Some(Ieee802154Address::BROADCAST),
                src_pan_id: None,
                src_addr: Some(Ieee802154Address::from_bytes(
                    &node_config.uwb_addr.to_le_bytes(),
                )),
            };

            let mut frame = Ieee802154Frame::new_unchecked(&mut tx_buffer);
            packet.emit(&mut frame);

            // Send the frame
            let mac_packet_size = packet.buffer_len() + 6;

            defmt::info!("Sending frame of size: {}", mac_packet_size);

            let current_ts = Instant::new((dw3000.sys_time().unwrap() as u64) << 8).unwrap();

            // Round to 32-bit
            let mut delay = dw3000_ng::time::Duration::from_nanos(300 * 1000);
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
                    config,
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

            defmt::info!("Current TS: {}", current_ts);
            defmt::info!("Delayed TS: {}", delayed_tx_time);

            match result {
                Ok(_) => {
                    defmt::info!("Sent!");
                }
                Err(e) => {
                    defmt::error!("Failed to send!");
                    match e {
                        dw3000_ng::Error::Spi(_) => {
                            defmt::error!("SPI Error!");
                        }
                        dw3000_ng::Error::Fcs => defmt::error!("Fcs"),
                        dw3000_ng::Error::Phy => defmt::error!("Phy"),
                        dw3000_ng::Error::BufferTooSmall { required_len } => {
                            defmt::error!("BufferTooSmall {}", required_len)
                        }
                        dw3000_ng::Error::ReedSolomon => defmt::error!("ReedSolomon"),
                        dw3000_ng::Error::FrameWaitTimeout => defmt::error!("FrameWaitTimeout"),
                        dw3000_ng::Error::Overrun => defmt::error!("Overrun"),
                        dw3000_ng::Error::PreambleDetectionTimeout => {
                            defmt::error!("PreambleDetectionTimeout")
                        }
                        dw3000_ng::Error::SfdTimeout => defmt::error!("SfdTimeout"),
                        dw3000_ng::Error::FrameFilteringRejection => {
                            defmt::error!("FrameFilteringRejection")
                        }
                        dw3000_ng::Error::Frame(_) => defmt::error!("Frame"),
                        dw3000_ng::Error::DelayedSendTooLate => {
                            defmt::error!("DelayedSendTooLate")
                        }
                        dw3000_ng::Error::DelayedSendPowerUpWarning => {
                            defmt::error!("DelayedSendPowerUpWarning")
                        }
                        dw3000_ng::Error::Ssmarshal(_) => defmt::error!("Ssmarshal"),
                        dw3000_ng::Error::InvalidConfiguration => {
                            defmt::error!("InvalidConfiguration")
                        }
                        dw3000_ng::Error::RxNotFinished => defmt::error!("RxNotFinished"),
                        dw3000_ng::Error::StillAsleep => defmt::error!("StillAsleep"),
                        dw3000_ng::Error::BadRssiCalculation => {
                            defmt::error!("BadRssiCalculation")
                        }
                        dw3000_ng::Error::RxConfigFrameFilteringUnsupported => {
                            defmt::error!("RxConfigFrameFilteringUnsupported")
                        }
                    }
                }
            }

            let event_reg = txing.ll().sys_status().read().unwrap();

            defmt::debug!("Should have finished, HPDWARN: {}", event_reg.hpdwarn());

            txing.force_idle().unwrap(); // Seems that if we don't do this, the next send will fail

            dw3000 = txing.finish_sending().unwrap();
        }

        ticker.next().await;
    }
}
