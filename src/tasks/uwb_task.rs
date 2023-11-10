use dw3000::{self, hl::ConfigGPIOs};
use embassy_time::{Duration, Timer};
use embedded_hal_async::digital::Wait;
use hal::{
    gpio::{GpioPin, Input, Output, PullDown, PushPull},
    peripherals::SPI2,
    prelude::*,
    spi::{master::Spi, FullDuplexMode},
};

use crate::nb;

#[embassy_executor::task]
pub async fn uwb_task(
    bus: Spi<'static, SPI2, FullDuplexMode>,
    cs_gpio: GpioPin<Output<PushPull>, 8>,
    mut rst_gpio: GpioPin<Output<PushPull>, 9>,
    mut int_gpio: GpioPin<Input<PullDown>, 15>,
) -> ! {
    log::info!("UWB Task Start!");

    let mut config = dw3000::Config::default();
    config.bitrate = dw3000::configs::BitRate::Kbps6800;

    // Reset
    rst_gpio.set_low().unwrap();

    Timer::after(Duration::from_millis(10)).await;

    rst_gpio.set_high().unwrap();

    log::info!("DW3000 Reset!");

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

    log::info!("Time to send: {:?} ns", time_frame);

    loop {
        // int_gpio.wait_for_falling_edge().await.unwrap();
        let mut sending = dw3000
            .send(
                &[
                    0xDEu8, 0xAD, 0xBE, 0xEF, 0xDEu8, 0xAD, 0xBE, 0xEF, 0xDEu8, 0xAD, 0xBE, 0xEF,
                    0xDEu8, 0xAD, 0xBE, 0xEF, 0xDEu8, 0xAD, 0xBE, 0xEF,
                ],
                dw3000::hl::SendTime::Now,
                config,
            )
            .expect("Failed to send.");

        // wait for interrupt
        int_gpio.wait_for_high().await.unwrap();

        // Clear interrupt
        loop {
            let state = sending.s_wait();
            match state {
                Ok(inst) => {
                    log::info!("Send complete! at {:?}", inst);

                    break;
                }
                Err(e) => {
                    // Check error type
                    match e {
                        nb::Error::WouldBlock => {
                            // IRQ fired, but send not complete
                            log::info!("IRQ Fired, but send not complete!");
                            continue;
                        }
                        nb::Error::Other(e) => {
                            // Some other error
                            log::info!("Other error: {:?}", e);
                            break;
                        }
                    }
                }
            }
        }
        dw3000 = sending.finish_sending().expect("Failed to finish sending.");

        Timer::after(Duration::from_millis(1200)).await;
    }
}
