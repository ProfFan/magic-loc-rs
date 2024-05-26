use core::{cell::RefCell, future::pending};

use arbitrary_int::{u40, u48};
use binrw::io::Cursor;
use dw3000_ng::{self, hl::ConfigGPIOs};
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_time::{Duration, Instant, Timer};
use hal::{
    dma::ChannelCreator1,
    gpio::{GpioPin, Input, Output, PullDown, PushPull},
    peripherals::SPI2,
    prelude::*,
    spi::{master::Spi, FullDuplexMode},
};

use magic_loc_protocol::packet::PollPacket;
use smoltcp::wire::Ieee802154Frame;
use zerocopy::transmute_mut;

use crate::{
    config::MagicLocConfig,
    operations::{
        common::indirect_reg_read,
        host::{PrnReport, RawCirSample},
    },
    tasks::write_to_usb_serial_buffer,
    util::nonblocking_wait,
};

use binrw::BinWrite;
use dw3000_ng::configs::UwbChannel::Channel9;

pub enum WaitForPollError<SPI: embedded_hal::spi::ErrorType> {
    WrongFrameFormat,
    DwError(dw3000_ng::hl::Error<SPI>),
}

impl<SPI> core::fmt::Debug for WaitForPollError<SPI>
where
    SPI: embedded_hal::spi::ErrorType,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            WaitForPollError::WrongFrameFormat => write!(f, "WrongFrameFormat"),
            WaitForPollError::DwError(e) => write!(f, "DwError: {:?}", e),
        }
    }
}

#[ram]
async fn wait_for_poll_with_cir<SPI>(
    dw3000: dw3000_ng::DW3000<SPI, dw3000_ng::Ready>,
    dwm_config: dw3000_ng::Config,
    int_gpio: &mut GpioPin<Input<PullDown>, 15>,
) -> (
    Result<
        (
            u16,
            Instant,
            PollPacket,
            u40,
            u8,
            [RawCirSample; 16],
            u16,
            u16,
            u16,
            u32,
            u32,
        ),
        WaitForPollError<SPI>,
    >,
    dw3000_ng::DW3000<SPI, dw3000_ng::Ready>,
)
where
    SPI: embedded_hal::spi::SpiDevice,
    SPI::Error: defmt::Format,
{
    let mut rxing = dw3000.receive(dwm_config).expect("Failed to receive.");

    let mut buf = [0u8; 128];
    let result = nonblocking_wait(|| rxing.r_wait_buf(&mut buf), int_gpio).await;
    let received_system_ts = Instant::now();

    if result.is_err() {
        defmt::error!("Failed to receive: {:?}", result);

        return (
            Err(WaitForPollError::DwError(result.unwrap_err())),
            rxing.finish_receiving().unwrap(),
        );
    }

    defmt::trace!("Receive success!");

    let (msg_length, rx_time, _) = result.unwrap();

    const FCS_LEN: usize = 2;
    let frame = Ieee802154Frame::new_checked(&buf[..msg_length - FCS_LEN]);
    let fcs = &buf[msg_length - FCS_LEN..msg_length];

    return match frame {
        Ok(frame) => {
            let src_addr = frame
                .src_addr()
                .unwrap_or(smoltcp::wire::Ieee802154Address::Short([0, 0]));
            let src_addr = u16::from_le_bytes(src_addr.as_bytes().try_into().unwrap());

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
                let packet = magic_loc_protocol::packet::PollPacket::try_from(u48::from_le_bytes(
                    poll_bytes,
                ));

                if packet.is_ok() {
                    // Read CIR samples around the peak
                    let ip_poa = rxing.ll().ip_ts().read().unwrap().ip_poa();
                    let pdoa = rxing.ll().pdoa().read().unwrap().pdoa();
                    let fp_index = rxing.ll().ip_diag_8().read().unwrap().ip_fp();
                    let rx_rawst = rxing.ll().rx_rawst().read().unwrap().value();
                    let carrier_recovery_integrator =
                        rxing.ll().drx_car_int().read().unwrap().value();

                    let fp_offset = fp_index >> 6; // 10.6 fixed point

                    defmt::info!("CIR: ip_poa: {:#x}, pdoa: {:#x}, fp_offset: {:#x}, rx_rawst: {:#x}, carrier_recovery_integrator: {:#x}", ip_poa, pdoa, fp_offset, rx_rawst, carrier_recovery_integrator);

                    let mut cir_samples = [RawCirSample::default(); 16];

                    // Indirect read at the peak index
                    let cir_buffer: &mut [u8; 16 * 6] = transmute_mut!(&mut cir_samples);

                    indirect_reg_read(&mut rxing, 0x15, fp_offset - 8, &mut cir_buffer[..]).await;

                    let dw3000 = rxing.finish_receiving().unwrap();

                    return (
                        Ok((
                            src_addr,
                            received_system_ts,
                            packet.unwrap(),
                            u40::new(rx_time.value()),
                            frame.sequence_number().unwrap(),
                            cir_samples,
                            ip_poa,
                            pdoa,
                            fp_index,
                            carrier_recovery_integrator,
                            rx_rawst,
                        )),
                        dw3000,
                    );
                }
            }

            (
                Err(WaitForPollError::WrongFrameFormat),
                rxing.finish_receiving().unwrap(),
            )
        }
        Err(e) => {
            defmt::error!("Failed to parse frame: {:?}", e);
            (
                Err(WaitForPollError::WrongFrameFormat),
                rxing.finish_receiving().unwrap(),
            )
        }
    }
}

#[embassy_executor::task]
#[ram]
pub async fn passive_tag_task(
    bus: Spi<'static, SPI2, FullDuplexMode>,
    cs_gpio: GpioPin<Output<PushPull>, 8>,
    mut rst_gpio: GpioPin<Output<PushPull>, 9>,
    mut int_gpio: GpioPin<Input<PullDown>, 15>,
    config: MagicLocConfig,
) {
    let bus = NoopMutex::new(RefCell::new(bus));
    let spidev = SpiDevice::new(&bus, cs_gpio);

    let mut dwm_config = dw3000_ng::Config::default();
    dwm_config.bitrate = dw3000_ng::configs::BitRate::Kbps6800;
    dwm_config.sts_len = dw3000_ng::configs::StsLen::StsLen128;
    dwm_config.sts_mode = dw3000_ng::configs::StsMode::StsMode1;

    // Reset
    rst_gpio.set_low();

    Timer::after(Duration::from_millis(10)).await;

    rst_gpio.set_high();

    defmt::info!("DW3000 Reset!");

    Timer::after(Duration::from_millis(200)).await;

    let mut dw3000 = dw3000_ng::DW3000::new(spidev)
        .init()
        .expect("Failed init.")
        .config(dwm_config)
        .expect("Failed config.");

    dw3000.gpio_config(ConfigGPIOs::enable_led()).unwrap();

    dw3000
        .ll()
        .led_ctrl()
        .modify(|_, w| w.blink_tim(0x2))
        .unwrap();

    // Enable Super Deterministic Code (SDC)
    dw3000.ll().sys_cfg().modify(|_, w| w.cp_sdc(0x1)).unwrap();

    // Set STS_MNTH
    dw3000.ll().sts_conf_0().modify(|_, w| w.sts_rtm(17)).unwrap();

    // Enable PDoA
    dw3000.ll().sys_cfg().modify(|_, w| w.pdoa_mode(0x3)).unwrap();

    Timer::after(Duration::from_millis(200)).await;

    // Disable SPIRDY interrupt
    dw3000.disable_interrupts().unwrap();
    dw3000.enable_tx_interrupts().unwrap();
    dw3000.enable_rx_interrupts().unwrap();

    // Read DW3000 Device ID
    let dev_id = dw3000.ll().dev_id().read().unwrap();

    if dev_id.model() != 0x03 {
        defmt::error!("Invalid DW3000 model: {:#x}", dev_id.model());
        panic!();
    }

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

    defmt::info!("DW3000 Initialized!");

    loop {
        // let cancel = Timer::after(Duration::from_millis(1000));

        let system_ts = Instant::now();

        let result;
        (result, dw3000) = wait_for_poll_with_cir(dw3000, dwm_config, &mut int_gpio).await;

        if result.is_err() {
            continue;
        }

        let (
            src_addr, //
            system_ts_rx,
            poll_packet,
            rxts,
            seq_num,
            cir,
            ip_poa,
            pdoa,
            fp_index,
            cfo,
            rx_rawst,
        ) = result.unwrap();

        defmt::info!(
            "Packet from {:#x}, txts: {}, rxts: {}, seq_num: {:#x}",
            system_ts_rx,
            poll_packet.tx_timestamp().value(),
            rxts.value(),
            seq_num
        );

        // Send the packet to the USB serial buffer
        let prn_report = PrnReport {
            src_addr,
            system_ts: system_ts.as_micros(),
            packet_txts: poll_packet.tx_timestamp().value(),
            packet_rxts: rxts.value(),
            seq_num,
            ip_poa,
            pdoa,
            fp_index,
            start_index: 8,
            carrier_freq_offset: cfo,
            rx_rawst,
            cir,
        };

        let mut prn_buffer: [u8; 256] = [0; 256];
        let mut cursor = Cursor::new(&mut prn_buffer[..]);

        prn_report.write(&mut cursor).unwrap();

        let report_len = cursor.position() as usize;

        let mut buffer: [u8; 256] = [0; 256];
        let (header, data) = buffer.split_at_mut(2);
        header.copy_from_slice(&[0xFF, 0x01]);

        let mut encoder = defmt::Encoder::new();
        let mut cursor = 0;
        let mut write_bytes = |bytes: &[u8]| {
            data.as_mut()[cursor..cursor + bytes.len()].copy_from_slice(bytes);
            cursor += bytes.len();
        };

        encoder.start_frame(&mut write_bytes);
        encoder.write(&prn_buffer[..report_len], &mut write_bytes);
        encoder.end_frame(&mut write_bytes);

        let _ = write_to_usb_serial_buffer(&buffer[..cursor + 3]);
    }
}
