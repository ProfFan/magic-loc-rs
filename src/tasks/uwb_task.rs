use core::{cell::RefCell, future::pending};

use binrw::io::Cursor;
use dw3000_ng::{self, hl::ConfigGPIOs};
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_time::{Duration, Instant, Timer};
use hal::{
    dma::ChannelCreator1,
    dma_descriptors,
    gpio::{GpioPin, Input, Output, PullDown, PushPull},
    peripherals::SPI2,
    prelude::*,
    spi::{
        master::{dma::WithDmaSpi2, Spi},
        FullDuplexMode,
    },
    FlashSafeDma,
};

use heapless::Vec;
use magic_loc_protocol::tag_state_machine::TagSideStateMachine;

use crate::{
    config::MagicLocConfig,
    operations::{
        host::RangeReport,
        tag::{send_response_packet_at, wait_for_final, wait_for_poll},
    },
    tasks::write_to_usb_serial_buffer,
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
    dma_channel: ChannelCreator1,
) -> ! {
    defmt::info!("UWB Task Start!");

    // let (mut dma_tx, mut dma_rx) = dma_descriptors!(32000);

    // let bus = bus.with_dma(dma_channel.configure(
    //     false,
    //     &mut dma_tx,
    //     &mut dma_rx,
    //     hal::dma::DmaPriority::Priority0,
    // ));

    // Enable DMA interrupts
    hal::interrupt::enable(
        hal::peripherals::Interrupt::DMA_IN_CH1,
        hal::interrupt::Priority::Priority2,
    )
    .unwrap();
    hal::interrupt::enable(
        hal::peripherals::Interrupt::DMA_OUT_CH1,
        hal::interrupt::Priority::Priority2,
    )
    .unwrap();

    // let bus = FlashSafeDma::<_, 32000>::new(bus);

    let bus = NoopMutex::new(RefCell::new(bus));

    let device = SpiDevice::new(&bus, cs_gpio);

    let mut uwb_config = dw3000_ng::Config::default();
    uwb_config.bitrate = dw3000_ng::configs::BitRate::Kbps850;

    // Reset
    rst_gpio.set_low().unwrap();

    Timer::after(Duration::from_millis(10)).await;

    rst_gpio.set_high().unwrap();

    defmt::info!("DW3000 Reset!");

    Timer::after(Duration::from_millis(200)).await;

    let mut dw3000 = dw3000_ng::DW3000::new(device)
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
        let mut poll_rx_timeout = None;
        let mut response_tx_slot: Option<u32> = None;
        let mut sequence_number = 0;

        while response_tx_slot.is_none() {
            // As a tag, we wait for the POLL frame from an anchor
            // To prevent loss of sync if we miss the first anchor's POLL, since the TX time
            // for all anchors is all determined by the first frame, we also use the
            // other frames to calculate the TX time for the response
            defmt::debug!("Waiting for poll frame...");

            let rx_addr_time;
            (dw3000, rx_addr_time) = wait_for_poll(
                dw3000,
                uwb_config,
                &config,
                &mut int_gpio,
                Timer::after_millis(10),
            )
            .await;

            if let Some((rx_addr, tx_time, rx_time, sequence_number_)) = rx_addr_time {
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
                    poll_rx_timeout = Some(
                        Instant::now()
                            + Duration::from_micros(1000 * (8 - anchor_index) as u64 - 800),
                    );
                    delay_ns = Some(1000 * 1000 * (8 + tag_index - anchor_index) as u64);
                    let delay_device = delay_ns.unwrap() * 64; // 64 ticks per ns

                    // NOTE: the last bit of the tx time is always 0 as it is ignored by the DW3000
                    response_tx_slot = Some(
                        (((rx_time.value() + delay_device).div_ceil(1 << 9) << 1) % (1 << 32))
                            as u32,
                    );

                    // Set the sequence number
                    sequence_number = sequence_number_;
                }
            }
        }

        let delay_ns = delay_ns.unwrap();
        let poll_rx_timeout = poll_rx_timeout.unwrap();
        let response_tx_slot = response_tx_slot.unwrap();

        defmt::debug!(
            "Response delay = {}ns, tx slot = {}",
            delay_ns,
            response_tx_slot
        );

        // Continue waiting for the poll frame from the other anchors
        let timeout_time = poll_rx_timeout;
        let mut should_terminate = false;

        while !should_terminate {
            let timeout_future = Timer::at(timeout_time);

            let rx_addr_time;
            (dw3000, rx_addr_time) =
                wait_for_poll(dw3000, uwb_config, &config, &mut int_gpio, timeout_future).await;

            if let Some((rx_addr, tx_time, rx_time, sequence_number_)) = rx_addr_time {
                if sequence_number_ != sequence_number {
                    // Not my frame
                    defmt::error!("Not my frame!");
                    continue;
                }
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
        (dw3000, result) = send_response_packet_at(
            dw3000,
            uwb_config,
            &config,
            &mut int_gpio,
            response_tx_slot,
            sequence_number,
        )
        .await;

        if result.is_err() {
            defmt::error!("Response send fail!");

            tag_sm = waiting_poll.waiting_for_anchor_final().idle();
            continue;
        }

        // Set the response tx time
        waiting_poll.response_tx_ts = (response_tx_slot as u64) << 8;

        let mut waiting_final = waiting_poll.waiting_for_anchor_final();

        const GUARD_INTERVAL_US: u32 = 3 * 1000; // 3ms
        let last_final_expected_timeout = 1000
            * (config.network_topology.tag_addrs.len() - my_index
                + config.network_topology.anchor_addrs.len()) as u32
            + GUARD_INTERVAL_US;

        let timeout_time =
            Instant::now() + Duration::from_micros(last_final_expected_timeout as u64);

        defmt::debug!(
            "Timeout time = {:?}, now = {:?}",
            timeout_time,
            Instant::now()
        );

        // Wait for the final frame from the anchors
        loop {
            defmt::debug!("Waiting for final frame...");

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

            if rx_addr == *config.network_topology.anchor_addrs.last().unwrap() {
                // This is the last frame
                break;
            }
        }

        defmt::debug!(
            "All done, response_rx_ts = {:?}",
            waiting_final.response_rx_ts
        );

        // Calculate the TWR to all anchors using AltDS-TWR Formula
        let mut twr_results: [f64; 8] = [core::f64::NEG_INFINITY; 8];
        for i in 0..8 {
            if waiting_final.response_rx_ts[i] == 0 {
                continue;
            }
            const SEC_PER_TICK: f64 = 1.0 / 499200000.0 / 128.0;
            const C_LIGHT: f64 = 299792458.0; // m/s

            let t1 = waiting_final.poll_tx_ts[i] as i64;
            let t2 = waiting_final.poll_rx_ts[i] as i64;
            let t3 = waiting_final.response_tx_ts as i64;
            let t4 = waiting_final.response_rx_ts[i] as i64;
            let t5 = waiting_final.final_tx_ts[i] as i64;
            let t6 = waiting_final.final_rx_ts[i] as i64;

            defmt::debug!(
                "t1 = {}, t2 = {}, t3 = {}, t4 = {}, t5 = {}, t6 = {}",
                t1,
                t2,
                t3,
                t4,
                t5,
                t6
            );

            #[allow(non_snake_case)]
            {
                let R_a_hat = (t4 - t1).rem_euclid(1 << 40) as i128;
                let D_a_hat = (t3 - t2).rem_euclid(1 << 40) as i128;
                let R_b_hat = (t6 - t3).rem_euclid(1 << 40) as i128;
                let D_b_hat = (t5 - t4).rem_euclid(1 << 40) as i128;

                defmt::debug!(
                    "R_a_hat = {}, D_a_hat = {}, R_b_hat = {}, D_b_hat = {}",
                    R_a_hat,
                    D_a_hat,
                    R_b_hat,
                    D_b_hat
                );

                let tof_ticks = (R_a_hat * R_b_hat - D_a_hat * D_b_hat)
                    / (R_a_hat + R_b_hat + D_a_hat + D_b_hat);
                let tof = tof_ticks as f64 * SEC_PER_TICK;
                let dist = tof * C_LIGHT;

                twr_results[i] = dist;
            }
        }

        defmt::debug!("SEQ = {}, TWR Results = {:?}", sequence_number, twr_results);

        // Send the range report to the host
        let range_report = RangeReport {
            tag_addr: config.uwb_addr,
            seq_num: sequence_number,
            system_ts: Instant::now().as_micros(),
            trigger_txts: waiting_final.poll_tx_ts[0],
            ranges: twr_results,
        };

        let mut range_buffer: [u8; 128] = [0; 128];
        let mut cursor = Cursor::new(&mut range_buffer[..]);
        binrw::BinWrite::write(&range_report, &mut cursor).unwrap();
        let report_len = cursor.position() as usize;

        let mut buffer: [u8; 128] = [0; 128];
        let (header, data) = buffer.split_at_mut(2);
        header.copy_from_slice(&[0xFF, 0x01]);

        let mut encoder = defmt::Encoder::new();
        let mut cursor = 0;
        let mut write_bytes = |bytes: &[u8]| {
            data[cursor..cursor + bytes.len()].copy_from_slice(bytes);
            cursor += bytes.len();
        };
        encoder.start_frame(&mut write_bytes);
        encoder.write(&range_buffer[..report_len], &mut write_bytes);
        encoder.end_frame(&mut write_bytes);

        let _ = write_to_usb_serial_buffer(&buffer[..cursor + 3]);

        // clear waiting_final.response_rx_ts
        waiting_final.response_rx_ts.fill(0);

        tag_sm = waiting_final.idle();
    }
}
