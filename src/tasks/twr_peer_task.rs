use core::cell::{OnceCell, RefCell};

use arbitrary_int::Number;
use dw3000_ng::{self, hl::ConfigGPIOs};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Instant, Timer};
use esp_fast_serial::write_to_usb_serial_buffer;
use hal::{
    gpio::{Input, Output},
    peripherals::SPI2,
    spi::master::Spi,
    Blocking,
};


use crate::{
    config::MagicLocConfig,
    operations::{
        anchor::send_final_packet,
        tag::{send_response_packet_at, wait_for_final, wait_for_poll},
    },
};

/// Task for the UWB Tag
///
/// This task runs the Tag side state machine
///
/// Basic operation is as follows:
///
/// # As Initiator
///
/// 1. Send the Poll packet
/// 2. Wait for the Response packet
/// 3. Send the Final packet
///
/// # As Responder
///
/// 1. Wait for the Poll packet
/// 2. Send the Response packet
/// 3. Wait for the Final packet
#[embassy_executor::task(pool_size = 1)]
pub async fn symmetric_twr_anchor_task(
    bus: Spi<'static, Blocking, SPI2>,
    cs_gpio: Output<'static>,
    mut rst_gpio: Output<'static>,
    mut int_gpio: Input<'static>,
    config: MagicLocConfig,
) -> ! {
    defmt::info!("Starting TWR Anchor Task");

    let spi_bus = OnceCell::<embassy_sync::blocking_mutex::Mutex<NoopRawMutex, _>>::new();
    let _ = spi_bus.set(embassy_sync::blocking_mutex::Mutex::new(RefCell::new(bus)));

    let spidev = embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice::new(
        spi_bus.get().unwrap(),
        cs_gpio,
    );

    let mut dwm_config = dw3000_ng::Config::default();
    dwm_config.bitrate = dw3000_ng::configs::BitRate::Kbps6800;
    dwm_config.sts_len = dw3000_ng::configs::StsLen::StsLen128;
    dwm_config.sts_mode = dw3000_ng::configs::StsMode::StsMode1;

    // Reset
    rst_gpio.set_low();

    Timer::after(Duration::from_millis(10)).await;

    rst_gpio.set_high();

    defmt::info!("DW3000 Reset!");

    Timer::after(Duration::from_millis(200)).await;

    let mut dw3000 = dw3000_ng::DW3000::new(spidev).init().expect("Failed init.");

    let antenna_delay = dw3000.read_otp(0x0B).unwrap().to_le_bytes();

    // (RX, TX): (2-byte LE, 2-byte LE)
    let (rx_delay, tx_delay) = (
        u16::from_le_bytes(antenna_delay[0..2].try_into().unwrap()) * 56 / 100,
        u16::from_le_bytes(antenna_delay[2..4].try_into().unwrap()) * 44 / 100,
    );

    defmt::info!("Antenna Delay: RX: {}, TX: {}", rx_delay, tx_delay);

    let mut dw3000 = dw3000
        .config(dwm_config, embassy_time::Delay)
        .expect("Failed config.");

    dw3000.gpio_config(ConfigGPIOs::enable_led()).unwrap();

    dw3000
        .ll()
        .led_ctrl()
        .modify(|_, w| w.blink_tim(0x2))
        .unwrap();

    // Enable Super Deterministic Code (SDC)
    dw3000.ll().sys_cfg().modify(|_, w| w.cp_sdc(0x1)).unwrap();

    // Set STS_MNTH
    dw3000
        .ll()
        .sts_conf_0()
        .modify(|_, w| w.sts_rtm(17))
        .unwrap();

    // Enable PDoA
    dw3000
        .ll()
        .sys_cfg()
        .modify(|_, w| w.pdoa_mode(0x3))
        .unwrap();

    Timer::after(Duration::from_millis(200)).await;

    // Disable SPIRDY interrupt
    dw3000.disable_interrupts().unwrap();
    dw3000.enable_tx_interrupts().unwrap();
    dw3000.enable_rx_interrupts().unwrap();

    // Read DW3000 Device ID
    let dev_id = dw3000.ll().dev_id().read().unwrap();

    if dev_id.model() != 0x03 {
        defmt::error!("Invalid DW3000 model: {:#x}", dev_id.model());
        panic!();
    }

    // Set MINDIAG to 0
    dw3000
        .ll()
        .cia_conf()
        .modify(|_, w| w.mindiag(0b0))
        .unwrap();

    // Enable ACC_CLK and ACC_MCLK
    dw3000
        .ll()
        .clk_ctrl()
        .modify(|_, w| w.acc_clk_en(0b1).acc_mclk_en(0b1))
        .unwrap();

    defmt::info!("DW3000 Initialized!");

    let sequence_number = 0;

    loop {
        // Wait for the Poll packet

        let poll_wait_result;
        (dw3000, poll_wait_result) = wait_for_poll(
            dw3000,
            dwm_config,
            &config,
            &mut int_gpio,
            Timer::after_secs(1),
        )
        .await;

        if poll_wait_result.is_none() {
            let _ = write_to_usb_serial_buffer(b"Poll Timeout!\n");

            continue;
        }

        let (rx_addr, poll_tx_time, poll_rx_time, sequence_number) = poll_wait_result.unwrap();

        // Subtract the antenna delay from the RX time
        let poll_rx_time = poll_rx_time - dw3000_ng::time::Duration::new(rx_delay as u64).unwrap();

        // Send the Response packet
        let delay_tx_time_approx = poll_rx_time + dw3000_ng::time::Duration::from_nanos(10000_000);

        // Actual TX time register is 32 bits with last 1 bit ignored
        let response_delay_tx_time: u32 = ((delay_tx_time_approx.value() >> 8) & (!0x1)) as u32;

        let result;
        (dw3000, result) = send_response_packet_at(
            dw3000,
            dwm_config,
            &config,
            &mut int_gpio,
            response_delay_tx_time,
            sequence_number,
        )
        .await;

        let response_delay_tx_time =
            dw3000_ng::time::Instant::new((response_delay_tx_time as u64) << 8).unwrap()
                + dw3000_ng::time::Duration::new(tx_delay as u64).unwrap();

        if result.is_err() {
            let _ = write_to_usb_serial_buffer(b"Response send error!\n");

            continue;
        }

        // Wait for the Final packet
        let (final_wait_result, recv_ok);
        (dw3000, final_wait_result, recv_ok) = wait_for_final(
            dw3000,
            dwm_config,
            &config,
            &mut int_gpio,
            Timer::after_millis(10),
        )
        .await;

        if final_wait_result.is_none() {
            let _ = write_to_usb_serial_buffer(b"Final packet receive failed!\n");

            continue;
        }

        let (src_addr, final_packet, final_rxts, seq) = final_wait_result.unwrap();

        let final_rxts = final_rxts - dw3000_ng::time::Duration::new(rx_delay as u64).unwrap();

        const SEC_PER_TICK: f64 = 1.0 / 499200000.0 / 128.0;
        const C_LIGHT: f64 = 299792458.0; // m/s

        // Calculate the distance
        let t1 = poll_tx_time.value() as i64; // Remote Poll TX time
        let t2 = poll_rx_time.value() as i64; // Local Poll RX time
        let t3 = response_delay_tx_time.value() as i64; // Local Response TX time
        let t4 = final_packet.rx_timestamps[0].value().value() as i64; // Remote Response RX time
        let t5 = final_packet.tx_timestamp.value().value() as i64; // Remote Final TX time
        let t6 = final_rxts.value() as i64; // Local Final RX time

        #[allow(non_snake_case)]
        {
            let duration_between = |t1: i64, t2: i64| {
                if t1 > t2 {
                    t1 - t2 // No wrap around
                } else {
                    t1 + (1 << 40) - t2 // 40-bit wrap around
                }
            };

            let R_a_hat = duration_between(t4, t1);
            let D_a_hat = duration_between(t3, t2);
            let R_b_hat = duration_between(t6, t3);
            let D_b_hat = duration_between(t5, t4);

            defmt::debug!(
                "R_a_hat = {}, D_a_hat = {}, R_b_hat = {}, D_b_hat = {}",
                R_a_hat,
                D_a_hat,
                R_b_hat,
                D_b_hat
            );

            let tof_ticks =
                (R_a_hat * R_b_hat - D_a_hat * D_b_hat) / (R_a_hat + R_b_hat + D_a_hat + D_b_hat);
            let tof = tof_ticks as f64 * SEC_PER_TICK;
            let dist = tof * C_LIGHT;

            defmt::debug!(
                "t1 = {}, t2 = {}, t3 = {}, t4 = {}, t5 = {}, t6 = {}",
                t1,
                t2,
                t3,
                t4,
                t5,
                t6
            );

            const DIST_BIAS: f64 = 70.7;

            // Send the distance to the host
            let _ = write_to_usb_serial_buffer(
                alloc::format!(
                    "[{}] DIST: {:.2} m\n",
                    Instant::now().as_micros(),
                    dist - DIST_BIAS
                )
                .as_bytes(),
            );

            let final_tx_time = final_rxts + dw3000_ng::time::Duration::from_nanos(3_000_000);
            let delay_tx_time_final: u32 = ((final_tx_time.value() >> 8) & (!0x1)) as u32;

            // Send back the t2, t3 and t6 to the peer
            dw3000 = send_final_packet(
                dw3000,
                &dwm_config,
                &config,
                &mut int_gpio,
                &[Some(t2 as u64), Some(t3 as u64), Some(t6 as u64)],
                delay_tx_time_final,
                sequence_number,
                Some(tx_delay),
            )
            .await;
        }
    }
}
