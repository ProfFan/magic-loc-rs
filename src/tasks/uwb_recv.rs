use dw3000::{self};
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
pub async fn uwb_recv_task(
    bus: Spi<'static, SPI2, FullDuplexMode>,
    cs_gpio: GpioPin<Output<PushPull>, 8>,
    mut rst_gpio: GpioPin<Output<PushPull>, 9>,
    mut int_gpio: GpioPin<Input<PullDown>, 15>,
) -> ! {
    log::info!("UWB Recv Task Start!");

    let config = dw3000::Config::default();

    // Reset
    rst_gpio.set_low().unwrap();

    Timer::after(Duration::from_millis(10)).await;

    rst_gpio.set_high().unwrap();

    Timer::after(Duration::from_millis(200)).await;

    let dw3000 = dw3000::DW3000::new(bus, cs_gpio)
        .init()
        .expect("Failed init.");

    let mut dw3000 = dw3000.config(config).expect("Failed config.");

    Timer::after(Duration::from_millis(200)).await;

    dw3000.disable_interrupts().unwrap();

    dw3000.enable_rx_interrupts().unwrap();

    loop {
        // int_gpio.wait_for_falling_edge().await.unwrap();
        let mut rxing = dw3000.receive(config).expect("Failed to receive.");

        // wait for interrupt
        int_gpio.wait_for_high().await.unwrap();

        // Clear interrupt
        loop {
            let mut buffer = [0u8; 1024];
            let state = rxing.r_wait(&mut buffer);
            match state {
                Ok(inst) => {
                    log::info!("Recv complete! at {:?}", inst);

                    break;
                }
                Err(e) => {
                    // Check error type
                    match e {
                        nb::Error::WouldBlock => {
                            // IRQ fired, but rx not complete
                            log::info!("IRQ Fired, but rx not complete!");
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
        dw3000 = rxing.finish_receiving().expect("Failed to finish rxing.");
    }
}
