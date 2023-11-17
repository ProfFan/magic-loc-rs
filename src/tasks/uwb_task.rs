use dw3000_ng::{self, hl::ConfigGPIOs};
use embassy_time::{Duration, Timer};
use hal::{
    gpio::{GpioPin, Input, Output, PullDown, PushPull},
    peripherals::SPI2,
    prelude::*,
    spi::{master::Spi, FullDuplexMode},
};

use smoltcp::wire::{
    Ieee802154Address, Ieee802154Frame, Ieee802154FrameType, Ieee802154FrameVersion, Ieee802154Pan,
    Ieee802154Repr,
};

use crate::util::nonblocking_s_wait;

#[embassy_executor::task(pool_size = 2)]
pub async fn uwb_task(
    bus: Spi<'static, SPI2, FullDuplexMode>,
    cs_gpio: GpioPin<Output<PushPull>, 8>,
    mut rst_gpio: GpioPin<Output<PushPull>, 9>,
    mut int_gpio: GpioPin<Input<PullDown>, 15>,
) -> ! {
    defmt::info!("UWB Task Start!");

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

    loop {
        let start_time = embassy_time::Instant::now();

        let addr = smoltcp::wire::Ieee802154Address::Short([0x01, 0x02]);
        let repr = Ieee802154Repr {
            frame_type: Ieee802154FrameType::Data,
            security_enabled: false,
            frame_pending: false,
            ack_request: true,
            pan_id_compression: true,
            frame_version: Ieee802154FrameVersion::Ieee802154,
            sequence_number: Some(1),
            dst_pan_id: Some(Ieee802154Pan(0xabcd)),
            dst_addr: Some(Ieee802154Address::Absent),
            src_pan_id: None,
            src_addr: Some(addr),
        };

        let mut bytes = [0u8; 40];
        let header_size = repr.buffer_len();
        let mut frame = Ieee802154Frame::new_unchecked(&mut bytes[..]);
        repr.emit(&mut frame);

        let payload = &mut bytes[header_size..];

        // Write to payload
        payload[0] = 0x01;

        let mut sending = dw3000
            .send_raw(&bytes, dw3000_ng::hl::SendTime::Now, config)
            .expect("Failed to send.");

        let time_to_sending = start_time.elapsed().as_micros();

        // Wait for the transmission to complete
        let send_result = embassy_time::with_timeout(
            Duration::from_millis(500),
            nonblocking_s_wait(&mut sending, &mut int_gpio),
        )
        .await;

        let send_ts = send_result.unwrap();
        dw3000 = sending.finish_sending().expect("Failed to finish sending.");

        let time_to_sent = start_time.elapsed().as_micros();

        defmt::info!(
            "S/C/TS: {}, TTS: {:?}, TTT: {:?}",
            send_ts.unwrap().value(),
            time_to_sending,
            time_to_sent,
        );

        Timer::after(Duration::from_millis(1000)).await;
    }
}
