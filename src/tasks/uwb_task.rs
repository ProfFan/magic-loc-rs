use dw3000::{self, hl::ConfigGPIOs};
use embassy_time::{Duration, Timer};
use hal::{
    gpio::{GpioPin, Input, Output, PullDown, PushPull},
    peripherals::SPI2,
    prelude::*,
    spi::{master::Spi, FullDuplexMode},
};

use crate::util::nonblocking_s_wait;

#[embassy_executor::task]
pub async fn uwb_task(
    bus: Spi<'static, SPI2, FullDuplexMode>,
    cs_gpio: GpioPin<Output<PushPull>, 8>,
    mut rst_gpio: GpioPin<Output<PushPull>, 9>,
    mut int_gpio: GpioPin<Input<PullDown>, 15>,
) -> ! {
    defmt::info!("UWB Task Start!");

    let mut config = dw3000::Config::default();
    config.bitrate = dw3000::configs::BitRate::Kbps850;

    // Reset
    rst_gpio.set_low().unwrap();

    Timer::after(Duration::from_millis(10)).await;

    rst_gpio.set_high().unwrap();

    defmt::info!("DW3000 Reset!");

    Timer::after(Duration::from_millis(200)).await;

    let mut dw3000 = dw3000::DW3000::new(bus, cs_gpio)
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

        let mut sending = dw3000
            .send_raw(
                &[
                    0xDEu8, 0xAD, 0xBE, 0xEF, 0xDEu8, 0xAD, 0xBE, 0xEF, 0xDEu8, 0xAD, 0xBE, 0xEF,
                    0xDEu8, 0xAD, 0xBE, 0xEF, 0xDEu8, 0xAD, 0xBE, 0xEF,
                ],
                dw3000::hl::SendTime::Now,
                config,
            )
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

        Timer::after(Duration::from_millis(1200)).await;
    }
}
