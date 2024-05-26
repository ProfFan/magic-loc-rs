use core::cell::RefCell;

/// TDoA Anchor
///
/// The TDoA mode does not require any RX on the anchor side
use dw3000_ng::{self, hl::ConfigGPIOs};
use dw3000_ng::configs::StsLen;
use dw3000_ng::configs::StsMode::{StsMode1, StsMode2, StsModeND};
use dw3000_ng::configs::UwbChannel::Channel9;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_time::{Duration, Instant, Ticker, Timer};

use hal::{
    gpio::{GpioPin, Input, Output, PullDown, PushPull},
    peripherals::SPI2,
    prelude::*,
    spi::{master::Spi, FullDuplexMode},
};

use crate::{config::MagicLocConfig, operations::anchor::send_poll_packet_at};

#[embassy_executor::task]
#[ram]
pub async fn tdoa_anchor_task(
    bus: Spi<'static, SPI2, FullDuplexMode>,
    cs_gpio: GpioPin<Output<PushPull>, 8>,
    mut rst_gpio: GpioPin<Output<PushPull>, 9>,
    mut int_gpio: GpioPin<Input<PullDown>, 15>,
    node_config: MagicLocConfig,
) -> ! {
    defmt::info!("Starting TDoA Anchor task");

    let bus = NoopMutex::new(RefCell::new(bus));
    let spidev = SpiDevice::new(&bus, cs_gpio);

    let mut dwm_config = dw3000_ng::Config::default();
    dwm_config.bitrate = dw3000_ng::configs::BitRate::Kbps6800;
    dwm_config.sts_len = StsLen::StsLen128;
    dwm_config.sts_mode = StsMode1;

    // Reset
    rst_gpio.set_low();

    Timer::after(Duration::from_millis(10)).await;

    rst_gpio.set_high();

    defmt::info!("DW3000 Reset!");

    Timer::after(Duration::from_millis(200)).await;

    let mut dw3000 = dw3000_ng::DW3000::new(spidev)
        .init()
        .expect("Failed init.")
        .config(dwm_config)
        .expect("Failed config.");

    dw3000.gpio_config(ConfigGPIOs::enable_led()).unwrap();
    dw3000
        .ll()
        .led_ctrl()
        .modify(|_, w| w.blink_tim(0x2))
        .unwrap();

    // Enable Super Deterministic Code (SDC)
    dw3000.ll().sys_cfg().modify(|_, w| w.cp_sdc(0x1)).unwrap();

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

    let is_first_anchor = node_config.network_topology.anchor_addrs[0] == node_config.uwb_addr;

    let mut ticker = Ticker::every(Duration::from_millis(4));

    let mut sequence_number = 0;
    loop {
        // If we are the first anchor, we just start transmitting the poll packet
        if is_first_anchor {
            let current_dw_time = dw3000.sys_time().unwrap();

            // NOTE: The delayed TX time (DX_TIME) is in units of ~4ns
            // Since the least significant 1 bit is ignored, the actual resolution is 8ns

            defmt::info!("Current DW Time: {}", current_dw_time);

            // We need to allow enough time for delayed TX to complete, considering the time needed
            // for the SPI transaction.
            let next_tx_time = current_dw_time + (300000) / 4; // 300us
            let next_tx_time = next_tx_time & 0xFF_FF_FF_FE; // Clear the last bit

            let current_time = Instant::now();
            dw3000 = send_poll_packet_at(
                dw3000,
                &dwm_config,
                &node_config,
                &mut int_gpio,
                next_tx_time,
                sequence_number,
            )
            .await;

            let elapsed = current_time.elapsed();
            defmt::info!("Elapsed: {:?}us", elapsed.as_micros());
        }

        sequence_number += 1;
        ticker.next().await;
    }
}
