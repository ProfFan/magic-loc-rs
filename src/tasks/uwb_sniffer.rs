use arbitrary_int::u48;
use dw3000_ng::{self, hl::ConfigGPIOs};
use embassy_time::{Duration, Timer};
use hal::{
    gpio::{GpioPin, Input, Output, PullDown, PushPull},
    peripherals::SPI2,
    prelude::*,
    spi::{master::Spi, FullDuplexMode},
};

use magic_loc_protocol::packet::FinalPacket;
use smoltcp::wire::Ieee802154Frame;
use zerocopy::transmute;

use crate::util::nonblocking_wait;

#[embassy_executor::task(pool_size = 2)]
pub async fn uwb_sniffer(
    bus: Spi<'static, SPI2, FullDuplexMode>,
    cs_gpio: GpioPin<Output<PushPull>, 8>,
    mut rst_gpio: GpioPin<Output<PushPull>, 9>,
    mut int_gpio: GpioPin<Input<PullDown>, 15>,
) -> ! {
    defmt::info!("UWB Sniffer Task Start!");

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
        .gpio_mode()
        .modify(|_, w| w.msgp0(0b01))
        .unwrap();

    dw3000
        .ll()
        .led_ctrl()
        .modify(|_, w| w.blink_tim(0x2))
        .unwrap();

    Timer::after(Duration::from_millis(200)).await;

    // Disable SPIRDY interrupt
    dw3000.disable_spirdy_interrupt().unwrap();

    // dw3000.enable_rx_interrupts().unwrap();
    dw3000
        .ll()
        .sys_enable()
        .modify(|_, w| {
            w.rxphe_en(0b1)
                .rxfr_en(0b1)
                .rxfcg_en(0b1)
                .rxfce_en(0b1)
                .rxrfsl_en(0b1)
                .rxfto_en(0b1)
                .rxovrr_en(0b1)
                .rxpto_en(0b1)
                .rxsto_en(0b1)
                .rxprej_en(0b1)
        })
        .unwrap();

    loop {
        let mut rxing = dw3000.receive(config).expect("Failed to receive.");

        let mut buf = [0u8; 128];
        let result = nonblocking_wait(|| rxing.r_wait_buf(&mut buf), &mut int_gpio).await;

        if result.is_err() {
            defmt::error!("Failed to receive: {:?}", result.unwrap_err());

            dw3000 = rxing.finish_receiving().unwrap();
            continue;
        }

        let (msg_length, rx_time) = result.unwrap();

        const FCS_LEN: usize = 2;
        let frame = Ieee802154Frame::new_checked(&buf[..msg_length - FCS_LEN]);
        let fcs = &buf[msg_length - FCS_LEN..msg_length];

        match frame {
            Ok(frame) => {
                defmt::info!(
                    "T: {}, Frame: {:?}, {:#x}, FCS={:#x}",
                    rx_time,
                    frame,
                    frame.payload(),
                    fcs
                );
                if frame.payload().unwrap().len() == 6 {
                    let mut poll_bytes: [u8; 6] = [0; 6];
                    poll_bytes.copy_from_slice(frame.payload().unwrap());

                    // Is probably a PollPacket
                    let packet = magic_loc_protocol::packet::PollPacket::try_from(
                        u48::from_le_bytes(poll_bytes),
                    );

                    if packet.is_ok() {
                        defmt::info!("PollPacket: {:?}", packet);
                    }
                }
                if frame.payload().unwrap().len() == 1 {
                    let mut response_bytes: [u8; 1] = [0; 1];
                    response_bytes.copy_from_slice(frame.payload().unwrap());

                    // Is probably a ResponsePacket
                    let packet = magic_loc_protocol::packet::ResponsePacket::try_from(
                        u8::from_le_bytes(response_bytes),
                    );

                    if packet.is_ok() {
                        defmt::info!("ResponsePacket: {:?}", packet);
                    }
                }
                if frame.payload().unwrap().len() == 21 {
                    let mut final_bytes: [u8; 21] = [0; 21];
                    final_bytes.copy_from_slice(frame.payload().unwrap());

                    // Is probably a FinalPacket
                    let packet: FinalPacket = transmute!(final_bytes);
                    let header = packet.header();

                    if header.packet_type() == magic_loc_protocol::packet::PacketType::Final {
                        defmt::info!("FinalPacket: {:?}", packet);
                    }
                }
            }
            Err(e) => {
                defmt::error!("Failed to parse frame: {:?}", e);
                defmt::error!("Buffer: {:x}", &buf);
            }
        }

        dw3000 = rxing.finish_receiving().unwrap();
    }
}
