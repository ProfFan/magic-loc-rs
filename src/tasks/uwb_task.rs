use dw3000_ng::{self, hl::ConfigGPIOs};
use embassy_time::{Duration, Timer};
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

use crate::{config::MagicLocConfig, util::nonblocking_wait};

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
    dw3000
        .ll()
        .led_ctrl()
        .modify(|_, w| w.blink_tim(0x2))
        .unwrap();

    Timer::after(Duration::from_millis(200)).await;

    // Disable SPIRDY interrupt
    dw3000.disable_spirdy_interrupt().unwrap();

    dw3000.enable_tx_interrupts().unwrap();

    // Tag state machine
    let mut tag_sm = TagSideStateMachine::new(
        config.uwb_addr,
        Vec::from_slice(&config.network_topology.anchor_addrs).unwrap(),
        Vec::from_slice(&config.network_topology.tag_addrs).unwrap(),
    );

    loop {
        // As a tag, we wait for the POLL frame from the 1st anchor

        Timer::after(Duration::from_millis(10)).await;
    }
}
