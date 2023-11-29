use dw3000_ng::{self, hl::ConfigGPIOs};
use embassy_time::{Duration, Instant, Timer};
use hal::{
    gpio::{GpioPin, Input, Output, PullDown, PushPull},
    peripherals::SPI2,
    prelude::*,
    spi::{master::Spi, FullDuplexMode},
};

use heapless::Vec;

// Protocol Crate
use magic_loc_protocol::anchor_state_machine::*;

use crate::{
    config::MagicLocConfig,
    operations::anchor::{
        send_final_packet, send_poll_packet_at, wait_for_first_poll, wait_for_response,
    },
};

use crate::operations::anchor::send_poll_packet;

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
    dw3000.disable_interrupts().unwrap();

    dw3000.enable_tx_interrupts().unwrap();
    dw3000.enable_rx_interrupts().unwrap();

    // Calculate the time to send
    let time_frame = magic_loc_protocol::util::frame_tx_time(12, &config, true);

    defmt::info!("Time to send: {:?} ns", time_frame);

    // Read DW3000 Device ID
    let dev_id = dw3000.ll().dev_id().read().unwrap();

    if dev_id.model() != 0x03 {
        defmt::error!("Invalid DW3000 model: {:#x}", dev_id.model());
        panic!();
    }

    let mut fsm = AnchorSideStateMachine::new(
        node_config.uwb_addr,
        Vec::from_slice(&node_config.network_topology.anchor_addrs).unwrap(),
        Vec::from_slice(&node_config.network_topology.tag_addrs).unwrap(),
    );

    let is_first_anchor = node_config.uwb_addr == node_config.network_topology.anchor_addrs[0];
    let my_index = node_config
        .network_topology
        .anchor_addrs
        .iter()
        .position(|&x| x == node_config.uwb_addr)
        .unwrap();

    loop {
        let fsm_waiting;

        // If we are the first anchor, we will send the first frame
        // Otherwise, we will wait for the first frame from the first anchor.
        //
        // All anchors will then wait for response packets from the tags, with a timeout
        // of 1ms after the expected time to receive the last response packet

        // The TX time for the final frame, in 32-bit DW3000 time
        let final_tx_slot;

        const GUARD_INTERVAL_US: u32 = 3000; // 3 us

        // The deadline when we stop waiting for response packets, in system time
        let response_recv_deadline;
        if is_first_anchor {
            // First anchor will send the first frame
            let poll_tx_ts;
            (dw3000, poll_tx_ts) =
                send_poll_packet(dw3000, &config, &node_config, &mut int_gpio, 300 * 1000).await;

            defmt::info!("Poll packet sent!");

            let response_expected_time_us = 1000
                * (node_config.network_topology.anchor_addrs.len()
                    + node_config.network_topology.tag_addrs.len()) as u32;
            response_recv_deadline =
                Instant::now() + Duration::from_micros(response_expected_time_us as u64);
            final_tx_slot = (poll_tx_ts.value()
                + ((response_expected_time_us + GUARD_INTERVAL_US) as u64 * 638976 / 10))
                .wrapping_rem(1 << 40)
                .div_ceil(1 << 8) as u32;
            fsm_waiting = fsm.waiting_for_response(poll_tx_ts.value());
        } else {
            defmt::debug!("Waiting for poll packet from anchor 1...");
            // First wait for the poll packet from the first anchor
            let mut poll_received = None;
            while poll_received.is_none() {
                (dw3000, poll_received) =
                    wait_for_first_poll(dw3000, config, &node_config, &mut int_gpio).await;
            }

            defmt::info!("Poll packet from anchor 1 received!");

            // Wait for the time slot to send the response
            let poll_rx_ts = poll_received.unwrap();
            let delay_us = 1000 * my_index as u64;
            let delay_device = delay_us * 638976 / 10;

            // Need to wrap around full 32-bit
            let delay_tx_time_32 =
                ((poll_rx_ts.value() + delay_device) % (1 << 40)).div_ceil(1 << 8) as u32;

            // Send the poll packet
            dw3000 = send_poll_packet_at(
                dw3000,
                &config,
                &node_config,
                &mut int_gpio,
                delay_tx_time_32,
            )
            .await;

            defmt::info!("Poll packet sent at {}!", (delay_tx_time_32 as u64) << 8);

            let response_expected_time_us = 1000
                * (node_config.network_topology.anchor_addrs.len() - my_index
                    + node_config.network_topology.tag_addrs.len()) as u64;
            response_recv_deadline =
                Instant::now() + Duration::from_micros(response_expected_time_us as u64);

            // We use this because this would be more accurate than using our tx time
            let response_expected_period = 1000
                * (node_config.network_topology.anchor_addrs.len()
                    + node_config.network_topology.tag_addrs.len()) as u32;
            final_tx_slot = (poll_rx_ts.value()
                + ((response_expected_period + (my_index as u32) * 1000 + GUARD_INTERVAL_US)
                    as u64
                    * 638976
                    / 10))
                .wrapping_rem(1 << 40)
                .div_ceil(1 << 8) as u32;
            fsm_waiting = fsm.waiting_for_response((delay_tx_time_32 as u64) << 8);
        }

        // Wait for the response packet
        let mut response_rx_ts: [Option<u64>; 3] = [None; 3];
        let mut should_terminate = false;

        while !should_terminate {
            let timeout_future = Timer::at(response_recv_deadline);

            let rx_addr_time;
            let timed_out;
            (dw3000, rx_addr_time, timed_out) =
                wait_for_response(dw3000, config, &node_config, &mut int_gpio, timeout_future)
                    .await;

            if timed_out {
                defmt::error!("Response packet timeout!");
                break;
            }

            if let Some((rx_addr, rx_time)) = rx_addr_time {
                // Get the index of the anchor
                let tag_index = node_config
                    .network_topology
                    .tag_addrs
                    .iter()
                    .position(|&x| x == rx_addr);
                if let Some(tag_index) = tag_index {
                    // Set the rx time for the anchor
                    response_rx_ts[tag_index] = Some(rx_time.value());

                    defmt::debug!("Response packet from anchor {} received!", tag_index);

                    if tag_index == node_config.network_topology.anchor_addrs.len() - 1 {
                        should_terminate = true;
                    }
                } else {
                    defmt::error!("Invalid anchor address: {:X}", rx_addr);
                }
            }
        }

        // Print the received response packet times
        for (i, &rx_time) in response_rx_ts.iter().enumerate() {
            defmt::info!("Anchor {} response time: {:?}", i, rx_time);
        }

        // Send the final packet

        defmt::debug!("Sending final packet at {}!", (final_tx_slot as u64) << 8);

        dw3000 = send_final_packet(
            dw3000,
            &config,
            &node_config,
            &mut int_gpio,
            &response_rx_ts,
            final_tx_slot,
        )
        .await;

        let fsm_sending_final = fsm_waiting.sending_final();

        fsm = fsm_sending_final.idle();

        defmt::debug!("Idle!");
    }
}
