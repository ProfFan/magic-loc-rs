use core::future::pending;

use dw3000_ng::{self, hl::ConfigGPIOs};
use embassy_time::{Duration, Instant, Timer};
use hal::{
    gpio::{GpioPin, Input, Output, PullDown, PushPull},
    peripherals::SPI2,
    prelude::*,
    spi::{master::Spi, FullDuplexMode},
};

use heapless::Vec;
use magic_loc_protocol::tag_state_machine::TagSideStateMachine;

use crate::{
    config::MagicLocConfig,
    operations::tag::{send_response_packet_at, wait_for_final, wait_for_poll},
};

/// Task for the UWB Tag
///
/// This task runs the Tag side state machine
///
/// Basic operation is as follows:
/// 1. Receive poll frames from all anchors
/// 2. Based on the poll frames, calculate the tx time for the response for this tag
/// 3. Send the response
/// 4. Wait for the final frame from the anchors
/// 5. Calculate the TWR to all anchors
/// 6. Go back to step 1
#[embassy_executor::task(pool_size = 2)]
pub async fn uwb_task(
    bus: Spi<'static, SPI2, FullDuplexMode>,
    cs_gpio: GpioPin<Output<PushPull>, 8>,
    mut rst_gpio: GpioPin<Output<PushPull>, 9>,
    mut int_gpio: GpioPin<Input<PullDown>, 15>,
    config: MagicLocConfig,
) -> ! {
    defmt::info!("UWB Task Start!");

    let mut uwb_config = dw3000_ng::Config::default();
    uwb_config.bitrate = dw3000_ng::configs::BitRate::Kbps850;

    // Reset
    rst_gpio.set_low().unwrap();

    Timer::after(Duration::from_millis(10)).await;

    rst_gpio.set_high().unwrap();

    defmt::info!("DW3000 Reset!");

    Timer::after(Duration::from_millis(200)).await;

    let mut dw3000 = dw3000_ng::DW3000::new(bus, cs_gpio)
        .init()
        .expect("Failed init.")
        .config(uwb_config)
        .expect("Failed config.");

    dw3000.gpio_config(ConfigGPIOs::enable_led()).unwrap();

    // Enable RX LED
    dw3000.ll().gpio_mode().modify(|_, w| w.msgp0(0x1)).unwrap();

    dw3000
        .ll()
        .led_ctrl()
        .modify(|_, w| w.blink_tim(0x2))
        .unwrap();

    Timer::after(Duration::from_millis(200)).await;

    // Disable SPIRDY interrupt
    dw3000.disable_spirdy_interrupt().unwrap();

    dw3000.enable_tx_interrupts().unwrap();
    dw3000.enable_rx_interrupts().unwrap();

    // Tag state machine
    let mut tag_sm = TagSideStateMachine::new(
        config.uwb_addr,
        Vec::from_slice(&config.network_topology.anchor_addrs).unwrap(),
        Vec::from_slice(&config.network_topology.tag_addrs).unwrap(),
    );

    let my_index: usize = config
        .network_topology
        .tag_addrs
        .iter()
        .position(|&x| x == config.uwb_addr)
        .unwrap();

    loop {
        let mut waiting_poll = tag_sm.waiting_for_anchor_poll();

        let mut delay_ns = None;
        let mut response_rx_timeout = None;
        let mut response_tx_slot: Option<u32> = None;

        while response_tx_slot.is_none() {
            // As a tag, we wait for the POLL frame from an anchor
            // To prevent loss of sync if we miss the first anchor's POLL, since the TX time
            // for all anchors is all determined by the first frame, we also use the
            // other frames to calculate the TX time for the response
            let rx_addr_time;
            (dw3000, rx_addr_time) =
                wait_for_poll(dw3000, uwb_config, &config, &mut int_gpio, pending::<()>()).await;

            if let Some((rx_addr, tx_time, rx_time)) = rx_addr_time {
                // Get the index of the anchor
                let anchor_index = config
                    .network_topology
                    .anchor_addrs
                    .iter()
                    .position(|&x| x == rx_addr)
                    .unwrap();
                // Set the rx time for the anchor
                waiting_poll.set_poll_rx_ts_idx(anchor_index, rx_time.value());
                waiting_poll.set_poll_tx_ts_idx(anchor_index, tx_time.value());

                if response_tx_slot.is_none() {
                    // Calculate the response tx time
                    let tag_index = config
                        .network_topology
                        .tag_addrs
                        .iter()
                        .position(|&x| x == config.uwb_addr)
                        .unwrap();
                    response_rx_timeout = Some(
                        Instant::now() + Duration::from_micros(1000 * (8 - anchor_index) as u64),
                    );
                    delay_ns = Some(1000 * 1000 * (8 + tag_index - anchor_index) as u64);
                    let delay_device = delay_ns.unwrap() * 64; // 64 ticks per ns
                    response_tx_slot = Some(
                        ((rx_time.value().div_ceil(1 << 8) + delay_device.div_ceil(1 << 8))
                            % (1 << 32)) as u32,
                    )
                }
            }
        }

        let delay_ns = delay_ns.unwrap();
        let response_rx_timeout = response_rx_timeout.unwrap();
        let response_tx_slot = response_tx_slot.unwrap();

        defmt::info!(
            "Response delay = {}ns, tx slot = {}",
            delay_ns,
            response_tx_slot
        );

        // Continue waiting for the poll frame from the other anchors
        let timeout_time = response_rx_timeout;
        let mut should_terminate = false;

        while !should_terminate {
            let timeout_future = Timer::at(timeout_time);

            let rx_addr_time;
            (dw3000, rx_addr_time) =
                wait_for_poll(dw3000, uwb_config, &config, &mut int_gpio, timeout_future).await;

            if let Some((rx_addr, tx_time, rx_time)) = rx_addr_time {
                // Get the index of the anchor
                let anchor_index = config
                    .network_topology
                    .anchor_addrs
                    .iter()
                    .position(|&x| x == rx_addr)
                    .unwrap();
                // Set the rx time for the anchor
                waiting_poll.set_poll_rx_ts_idx(anchor_index, rx_time.value());
                waiting_poll.set_poll_tx_ts_idx(anchor_index, tx_time.value());

                if anchor_index == config.network_topology.anchor_addrs.len() - 1 {
                    should_terminate = true;
                }
            } else {
                // Timeout
                should_terminate = true;
            }
        }

        // Send the response
        dw3000.force_idle().unwrap();

        let result;
        (dw3000, result) =
            send_response_packet_at(dw3000, uwb_config, &config, &mut int_gpio, response_tx_slot)
                .await;

        if let Err(_) = result {
            defmt::error!("Response send fail!");

            tag_sm = waiting_poll.waiting_for_anchor_final().idle();
            continue;
        }

        let mut waiting_final = waiting_poll.waiting_for_anchor_final();

        const GUARD_INTERVAL_US: u32 = 10 * 1000; // 3ms
        let last_final_expected_timeout = 1000
            * (config.network_topology.tag_addrs.len() - my_index
                + config.network_topology.anchor_addrs.len()) as u32
            + GUARD_INTERVAL_US;

        let timeout_time =
            Instant::now() + Duration::from_micros(last_final_expected_timeout as u64);

        // Wait for the final frame from the anchors
        loop {
            let timeout_future = Timer::at(timeout_time);

            let (result, recv_ok);
            (dw3000, result, recv_ok) =
                wait_for_final(dw3000, uwb_config, &config, &mut int_gpio, timeout_future).await;

            if !recv_ok {
                // Timeout
                defmt::error!("Final frame timeout!");
                break;
            }

            if result.is_none() {
                // Not my frame
                defmt::debug!("Not my frame!");
                continue;
            }

            let (rx_addr, packet, rx_time) = result.unwrap();

            waiting_final.set_final_tx_ts(rx_addr, packet.tx_timestamp.value().value());
            waiting_final.set_final_rx_ts(rx_addr, rx_time.value());
            waiting_final
                .set_response_rx_ts(rx_addr, packet.rx_timestamps[my_index].value().value());
        }

        defmt::info!(
            "All done, response_rx_ts = {:?}",
            waiting_final.response_rx_ts
        );

        tag_sm = waiting_final.idle();
    }
}
