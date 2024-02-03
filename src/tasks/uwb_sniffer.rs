use core::cell::RefCell;

use arbitrary_int::u48;
use binrw::{io::Cursor, BinWrite};
use dw3000_ng::{self, hl::ConfigGPIOs};
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::spi as async_spi;
use hal::{
    dma::ChannelCreator1,
    dma_descriptors,
    gpio::{GpioPin, Input, Output, PullDown, PushPull},
    peripherals::SPI2,
    prelude::*,
    spi::{
        master::{dma::WithDmaSpi2, Spi},
        FullDuplexMode,
    },
    FlashSafeDma,
};

use magic_loc_protocol::packet::FinalPacket;
use smoltcp::wire::Ieee802154Frame;
use zerocopy::{transmute, transmute_mut};

use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;

use crate::{
    operations::host::{CirReport, RawCirSample},
    util::nonblocking_wait,
};

#[inline]
async fn indirect_reg_read<SPI>(
    dw3000: &mut dw3000_ng::DW3000<SPI, dw3000_ng::SingleBufferReceiving>,
    reg: u8,
    offset: u16,
    buf: &mut [u8],
) where
    SPI: embedded_hal::spi::SpiDevice<u8>,
    SPI::Error: core::fmt::Debug + defmt::Format,
{
    // Write the indirect address
    dw3000
        .ll()
        .ptr_addr_a()
        .write(|w| w.ptra_base(reg))
        .unwrap();

    // Write the indirect offset
    dw3000
        .ll()
        .ptr_offset_a()
        .write(|w| w.ptra_ofs(offset))
        .unwrap();

    defmt::trace!("Reading indirect data...");
    // Read the indirect data
    let spi = dw3000.ll().bus();
    spi.transaction(&mut [
        async_spi::Operation::Write(&[0x1Du8 << 1]),
        async_spi::Operation::Read(buf),
    ])
    .unwrap();

    defmt::trace!("Indirect data read!");
}

#[embassy_executor::task(pool_size = 2)]
#[ram]
pub async fn uwb_sniffer(
    bus: Spi<'static, SPI2, FullDuplexMode>,
    cs_gpio: GpioPin<Output<PushPull>, 8>,
    mut rst_gpio: GpioPin<Output<PushPull>, 9>,
    mut int_gpio: GpioPin<Input<PullDown>, 15>,
    dma_channel: ChannelCreator1,
) -> ! {
    defmt::info!("UWB Sniffer Task Start!");

    let (mut dma_tx, mut dma_rx) = dma_descriptors!(32000);

    let bus = bus.with_dma(dma_channel.configure(
        false,
        &mut dma_tx,
        &mut dma_rx,
        hal::dma::DmaPriority::Priority0,
    ));

    // Enable DMA interrupts
    hal::interrupt::enable(
        hal::peripherals::Interrupt::DMA_IN_CH1,
        hal::interrupt::Priority::Priority2,
    )
    .unwrap();
    hal::interrupt::enable(
        hal::peripherals::Interrupt::DMA_OUT_CH1,
        hal::interrupt::Priority::Priority2,
    )
    .unwrap();

    let bus = FlashSafeDma::<_, 32000>::new(bus);

    let bus = NoopMutex::new(RefCell::new(bus));

    let device = SpiDevice::new(&bus, cs_gpio);

    let mut config = dw3000_ng::Config::default();
    config.bitrate = dw3000_ng::configs::BitRate::Kbps850;

    // Reset
    rst_gpio.set_low().unwrap();

    Timer::after(Duration::from_millis(10)).await;

    rst_gpio.set_high().unwrap();

    defmt::info!("DW3000 Reset!");

    Timer::after(Duration::from_millis(200)).await;

    let mut dw3000 = dw3000_ng::DW3000::new(device)
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

    // Set MINDIAG to 0
    dw3000
        .ll()
        .cia_conf()
        .modify(|_, w| w.mindiag(0b0))
        .unwrap();

    // Enable ACC_CLK and ACC_MCLK
    dw3000
        .ll()
        .clk_ctrl()
        .modify(|_, w| w.acc_clk_en(0b1).acc_mclk_en(0b1))
        .unwrap();

    defmt::info!("DW3000 Configured!");

    loop {
        defmt::trace!("Start receiving...");

        let mut rxing = dw3000.receive(config).expect("Failed to receive.");

        let mut buf = [0u8; 128];
        let result = nonblocking_wait(|| rxing.r_wait_buf(&mut buf), &mut int_gpio).await;

        if result.is_err() {
            defmt::error!("Failed to receive: {:?}", result.unwrap_err());

            dw3000 = rxing.finish_receiving().unwrap();
            continue;
        }

        defmt::trace!("Receive success!");

        let (msg_length, rx_time) = result.unwrap();

        const FCS_LEN: usize = 2;
        let frame = Ieee802154Frame::new_checked(&buf[..msg_length - FCS_LEN]);
        let fcs = &buf[msg_length - FCS_LEN..msg_length];

        match frame {
            Ok(frame) => {
                defmt::debug!(
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
                        defmt::debug!("PollPacket: {:?}", packet);
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
                        defmt::debug!("ResponsePacket: {:?}", packet);
                    }
                }
                if frame.payload().unwrap().len() == 21 {
                    let mut final_bytes: [u8; 21] = [0; 21];
                    final_bytes.copy_from_slice(frame.payload().unwrap());

                    // Is probably a FinalPacket
                    let packet: FinalPacket = transmute!(final_bytes);
                    let header = packet.header();

                    if header.packet_type() == magic_loc_protocol::packet::PacketType::Final {
                        defmt::debug!("FinalPacket: {:?}", packet);
                    }
                }

                defmt::trace!("Start CIR acquisition...");

                // Package and send to host
                let src_addr = frame.src_addr().unwrap();
                let fp_index = rxing.ll().ip_diag_0().read().unwrap().ip_peaki();
                let ip_poa = rxing.ll().ip_ts().read().unwrap().ip_poa();

                defmt::trace!(
                    "src_addr: {:#x}, fp_index: {:#x}, ip_poa: {:#x}",
                    src_addr,
                    fp_index,
                    ip_poa
                );

                let mut cir_report = CirReport {
                    src_addr: u16::from_le_bytes(src_addr.as_bytes().try_into().unwrap()),
                    system_ts: Instant::now().as_micros(),
                    seq_num: frame.sequence_number().unwrap_or(0),
                    fp_index: fp_index,
                    ip_poa: ip_poa,
                    start_index: fp_index - 10,
                    cir_size: 16,
                    cir: [RawCirSample {
                        real: [0; 3],
                        imag: [0; 3],
                    }; 16],
                };

                if fp_index + cir_report.cir_size > 1024 {
                    defmt::error!("CIR buffer overflow!");
                } else {
                    let cir_buffer: &mut [u8; 16 * 6] = transmute_mut!(&mut cir_report.cir);

                    defmt::trace!("Reading CIR buffer...");

                    indirect_reg_read(
                        &mut rxing,
                        0x15,
                        cir_report.start_index,
                        &mut cir_buffer[..],
                    )
                    .await;

                    defmt::debug!("CIR Report: {:?}", cir_report);

                    // Write to USB serial buffer
                    let mut cir_report_buffer: [u8; core::mem::size_of::<CirReport>()] =
                        [0; core::mem::size_of::<CirReport>()];

                    let mut binrw_cursor = Cursor::new(&mut cir_report_buffer[..]);
                    cir_report.write(&mut binrw_cursor).unwrap();

                    let cir_buffer_len = binrw_cursor.position() as usize;

                    let mut buffer = [0u8; core::mem::size_of::<CirReport>() + 4];
                    let (header, data) = buffer.split_at_mut(2);
                    header.copy_from_slice(&[0xFF, 0x01]);

                    let mut encoder = defmt::Encoder::new();
                    let mut cursor = 0;
                    let mut write_bytes = |bytes: &[u8]| {
                        data.as_mut()[cursor..cursor + bytes.len()].copy_from_slice(bytes);
                        cursor += bytes.len();
                    };

                    encoder.start_frame(&mut write_bytes);
                    encoder.write(&cir_report_buffer[..cir_buffer_len], &mut write_bytes);
                    encoder.end_frame(&mut write_bytes);

                    defmt::trace!("Writting to USB serial buffer...");

                    defmt::trace!("Cursor: {}", cursor);

                    let result = crate::tasks::serial_comm::write_to_usb_serial_buffer(
                        &buffer[..cursor + 3],
                    );

                    if result.is_err() {
                        Timer::after_millis(1000).await;
                        defmt::error!("Failed to write to USB serial buffer: {:?}", result);
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
