use dw3000_ng::{self, hl::ConfigGPIOs};
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
pub async fn uwb_anchor_task(
    bus: Spi<'static, SPI2, FullDuplexMode>,
    cs_gpio: GpioPin<Output<PushPull>, 8>,
    mut rst_gpio: GpioPin<Output<PushPull>, 9>,
    mut int_gpio: GpioPin<Input<PullDown>, 15>,
) -> ! {
    defmt::info!("UWB Anchor Task Start!");

    let mut config = dw3000_ng::Config::default();
    config.bitrate = dw3000_ng::configs::BitRate::Kbps6800;

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

    loop {
        // int_gpio.wait_for_falling_edge().await.unwrap();
        let mut rxing = dw3000.receive(config).expect("Failed to receive.");

        // Clear interrupt
        loop {
            // wait for interrupt
            int_gpio.wait_for_high().await.unwrap();

            let mut buffer = [0u8; 1024];
            let state = rxing.r_wait(&mut buffer);
            match state {
                Ok(_) => {
                    break;
                }
                Err(e) => {
                    // Check error type
                    match e {
                        nb::Error::WouldBlock => {
                            // IRQ fired, but rx not complete
                            defmt::info!("IRQ Fired, but rx not complete!");
                            continue; // Continue waiting for interrupt
                        }
                        nb::Error::Other(_) => {
                            // Some other error
                            defmt::info!("Other error: SPI");
                            break;
                        }
                    }
                }
            }
        }
        dw3000 = rxing.finish_receiving().expect("Failed to finish rxing.");
    }
}
