use core::future::pending;

use dw3000_ng::{self, hl::ConfigGPIOs};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{with_timeout, Duration, Timer, Instant};
use hal::{
    gpio::{GpioPin, Input, Output, PullDown, PushPull},
    peripherals::SPI2,
    prelude::*,
    spi::{master::Spi, FullDuplexMode},
};

use heapless::Vec;
use magic_loc_protocol::tag_state_machine::TagSideStateMachine;
use smoltcp::wire::{
    Ieee802154Address, Ieee802154Frame, Ieee802154FrameType, Ieee802154FrameVersion, Ieee802154Pan,
    Ieee802154Repr,
};

use crate::{config::MagicLocConfig, operations::tag::wait_for_poll, util::nonblocking_wait};

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

    loop {
        let mut waiting_poll = tag_sm.waiting_for_anchor_poll();

        let mut delay_ns = None;
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

                if response_tx_slot.is_none() {
                    // Calculate the response tx time
                    let tag_index = config
                        .network_topology
                        .tag_addrs
                        .iter()
                        .position(|&x| x == config.uwb_addr)
                        .unwrap();
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
        let response_tx_slot = response_tx_slot.unwrap();

        defmt::info!(
            "Response delay = {}ns, tx slot = {}",
            delay_ns,
            response_tx_slot
        );

        // Continue waiting for the poll frame from the other anchors
        let timeout_time = Instant::now() + Duration::from_micros(delay_ns.div_ceil(1000));
        let mut last_anchor_received = false;

        while !last_anchor_received {
            let timeout_future = Timer::at(timeout_time);
            
            let rx_addr_time;
            (dw3000, rx_addr_time) =
                wait_for_poll(dw3000, uwb_config, &config, &mut int_gpio, timeout_future)
                .await;

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

                if anchor_index == config.network_topology.anchor_addrs.len() - 1 {
                    last_anchor_received = true;
                }
            }
        }

        let waiting_final = waiting_poll.waiting_for_anchor_final();

        tag_sm = waiting_final.idle();
    }
}
