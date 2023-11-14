use dw3000::{self, hl::ConfigGPIOs};
use embassy_time::{Duration, Timer};
use embedded_hal_async::digital::Wait;
use hal::{
    gpio::{GpioPin, Input, Output, PullDown, PushPull},
    peripherals::SPI2,
    prelude::*,
    spi::{master::Spi, FullDuplexMode},
};

/// Calls `s_wait` on the `Sending` state of the DW3000 driver asynchronously.
///
/// When `s_wait` returns `nb::Error::WouldBlock`, this function will wait for
/// the DW3000's IRQ output to go high, and then call `s_wait` again.
#[inline]
pub async fn nonblocking_s_wait(
    dw3000: &mut dw3000::DW3000<
        Spi<'static, SPI2, FullDuplexMode>,
        GpioPin<Output<PushPull>, 8>,
        dw3000::Sending,
    >,
    int_gpio: &mut impl embedded_hal_async::digital::Wait,
) -> Result<
    dw3000::time::Instant,
    dw3000::Error<Spi<'static, SPI2, FullDuplexMode>, GpioPin<Output<PushPull>, 8>>,
> {
    loop {
        match dw3000.s_wait() {
            Ok(instant) => return Ok(instant),
            Err(nb::Error::WouldBlock) => {
                // Wait for the IRQ output to go high
                int_gpio.wait_for_high().await.unwrap();
                continue;
            }
            Err(nb::Error::Other(e)) => return Err(e),
        }
    }
}

#[embassy_executor::task]
pub async fn uwb_task(
    bus: Spi<'static, SPI2, FullDuplexMode>,
    cs_gpio: GpioPin<Output<PushPull>, 8>,
    mut rst_gpio: GpioPin<Output<PushPull>, 9>,
    mut int_gpio: GpioPin<Input<PullDown>, 15>,
) -> ! {
    defmt::info!("UWB Task Start!");

    let mut config = dw3000::Config::default();
    // config.bitrate = dw3000::configs::BitRate::Kbps6800;

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
        let send_result = nonblocking_s_wait(&mut sending, &mut int_gpio).await;

        let send_ts = send_result.unwrap();
        dw3000 = sending.finish_sending().expect("Failed to finish sending.");

        let time_to_sent = start_time.elapsed().as_micros();

        defmt::info!(
            "S/C/TS: {}, TTS: {:?}, TTT: {:?}",
            send_ts.value(),
            time_to_sending,
            time_to_sent,
        );

        Timer::after(Duration::from_millis(1200)).await;
    }
}
