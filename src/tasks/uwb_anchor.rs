use dw3000_ng::{self, hl::ConfigGPIOs};
use embassy_time::{Duration, Ticker, Timer};
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
    operations::anchor::{listen_for_packet, wait_for_first_poll},
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
    dw3000.disable_spirdy_interrupt().unwrap();

    dw3000.enable_tx_interrupts().unwrap();

    // Calculate the time to send
    let time_frame = magic_loc_protocol::util::frame_tx_time(12, &config, true);

    defmt::info!("Time to send: {:?} ns", time_frame);

    let mut fsm = AnchorSideStateMachine::new(
        node_config.uwb_addr,
        Vec::from_slice(&node_config.network_topology.anchor_addrs).unwrap(),
        Vec::from_slice(&node_config.network_topology.tag_addrs).unwrap(),
    );

    let is_first_anchor = node_config.uwb_addr == node_config.network_topology.anchor_addrs[0];

    let mut ticker = Ticker::every(Duration::from_millis(1000));

    loop {
        let fsm_waiting;
        if is_first_anchor {
            // First anchor will send the first frame
            let poll_tx_ts;
            (dw3000, poll_tx_ts) =
                send_poll_packet(dw3000, &config, &node_config, &mut int_gpio).await;

            defmt::info!("Poll packet sent!");

            fsm_waiting = fsm.waiting_for_response(poll_tx_ts.value());
        } else {
            // First wait for the poll packet from the first anchor
            let mut poll_received = false;
            while !poll_received {
                (dw3000, poll_received) =
                    wait_for_first_poll(dw3000, &config, &node_config, &mut int_gpio).await;
            }

            fsm_waiting = fsm.waiting_for_response(1u64);
        }

        let fsm_sending_final = fsm_waiting.sending_final();

        fsm = fsm_sending_final.idle();

        ticker.next().await;
    }
}
