use dw3000_ng::{self, time::Instant};
use hal::gpio::{GpioPin, Input, PullDown};

use arbitrary_int::{u4, u40, u48};

// Protocol Crate
use magic_loc_protocol::packet::PollPacket;

use crate::{config::MagicLocConfig, util::nonblocking_wait};

use smoltcp::wire::{
    Ieee802154Address, Ieee802154Frame, Ieee802154FrameType, Ieee802154FrameVersion, Ieee802154Pan,
    Ieee802154Repr,
};

pub async fn send_poll_packet<SPI, CS>(
    mut dw3000: dw3000_ng::DW3000<SPI, CS, dw3000_ng::Ready>,
    dwm_config: &dw3000_ng::Config,
    config: &MagicLocConfig,
    mut int_gpio: &mut GpioPin<Input<PullDown>, 15>,
    delay_ns: u32,
) -> (dw3000_ng::DW3000<SPI, CS, dw3000_ng::Ready>, Instant)
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug + defmt::Format,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug + defmt::Format,
    <CS as embedded_hal::digital::v2::OutputPin>::Error: core::fmt::Debug + defmt::Format,
{
    let mut poll_packet = PollPacket::new(
        magic_loc_protocol::packet::PacketType::Poll,
        u4::new(0),
        u40::new(0x12356789).into(),
    );

    let mut tx_buffer = [0u8; 128];

    let packet = Ieee802154Repr {
        frame_type: Ieee802154FrameType::Data,
        security_enabled: false,
        frame_pending: false,
        ack_request: true,
        pan_id_compression: true,
        frame_version: Ieee802154FrameVersion::Ieee802154,
        sequence_number: Some(1),
        dst_pan_id: Some(Ieee802154Pan(config.uwb_pan_id)),
        dst_addr: Some(Ieee802154Address::BROADCAST),
        src_pan_id: None,
        src_addr: Some(Ieee802154Address::from_bytes(
            &config.uwb_addr.to_le_bytes(),
        )),
    };

    let mut frame = Ieee802154Frame::new_unchecked(&mut tx_buffer);
    packet.emit(&mut frame);

    // Send the frame
    let mac_packet_size = packet.buffer_len() + 6;

    defmt::debug!("Sending frame of size: {}", mac_packet_size);

    let current_ts = Instant::new((dw3000.sys_time().unwrap() as u64) << 8).unwrap();

    // Round to 32-bit
    let mut delay = dw3000_ng::time::Duration::from_nanos(delay_ns);
    delay = dw3000_ng::time::Duration::new((delay.value() >> 8) << 8).unwrap();
    let delayed_tx_time = current_ts + delay;

    // Set the delayed send time
    poll_packet.set_tx_timestamp(u40::new(delayed_tx_time.value()));

    let payload = frame.payload_mut().unwrap();
    payload[..6].copy_from_slice(&u48::from(poll_packet).to_le_bytes());

    let mut txing = dw3000
        .send_raw(
            &tx_buffer[..mac_packet_size],
            dw3000_ng::hl::SendTime::Delayed(delayed_tx_time),
            dwm_config,
        )
        .unwrap();

    let result = nonblocking_wait(
        || {
            defmt::debug!("Waiting for send...");
            txing.s_wait()
        },
        &mut int_gpio,
    )
    .await;

    defmt::debug!("Current TS: {}", current_ts);
    defmt::debug!("Delayed TS: {}", delayed_tx_time);

    match result {
        Ok(_) => {
            defmt::debug!("Sent!");
        }
        Err(e) => {
            defmt::error!("Failed to send!");
            defmt::error!("Error: {:?}", e);
        }
    }

    let event_reg = txing.ll().sys_status().read().unwrap();

    defmt::debug!("Should have finished, HPDWARN: {}", event_reg.hpdwarn());

    txing.force_idle().unwrap(); // Seems that if we don't do this, the next send will fail

    dw3000 = txing.finish_sending().unwrap();

    return (dw3000, delayed_tx_time);
}

/*
        cplock,     1,  1, u8; /// Clock PLL Lock
        spicrce,    2,  2, u8; /// External Sync Clock Reset
        aat,        3,  3, u8; /// Automatic Acknowledge Trigger
        txfrb,      4,  4, u8; /// TX Frame Begins
        txprs,      5,  5, u8; /// TX Preamble Sent
        txphs,      6,  6, u8; /// TX PHY Header Sent
        txfrs,      7,  7, u8; /// TX Frame Sent
        rxprd,      8,  8, u8; /// RX Preamble Detected
        rxsfdd,     9,  9, u8; /// RX SFD Detected
        ciadone,   10, 10, u8; /// LDE Processing Done
        rxphd,     11, 11, u8; /// RX PHY Header Detect
        rxphe,     12, 12, u8; /// RX PHY Header Error
        rxfr,      13, 13, u8; /// RX Data Frame Ready
        rxfcg,     14, 14, u8; /// RX FCS Good
        rxfce,     15, 15, u8; /// RX FCS Error
        rxfsl,     16, 16, u8; /// RX Reed-Solomon Frame Sync Loss
        rxfto,     17, 17, u8; /// RX Frame Wait Timeout
        ciaerr,    18, 18, u8; /// Leading Edge Detection Error
        vwarn,     19, 19, u8; /// Low voltage warning
        rxovrr,    20, 20, u8; /// RX Overrun
        rxpto,     21, 21, u8; /// Preamble detection timeout
        spirdy,    23, 23, u8; /// SPI ready for host access
        rcinit,    24, 24, u8; /// RC INIT
        pll_hilo,  25, 25, u8; /// lock PLL Losing Lock
        rxsto,     26, 26, u8; /// Receive SFD timeout
        hpdwarn,   27, 27, u8; /// Half Period Delay Warning
        cperr,     28, 28, u8; /// Scramble Timestamp Sequence (STS) error
        arfe,      29, 29, u8; /// Automatic Frame Filtering rejection
        rxprej,    29, 29, u8; /// Receiver Preamble Rejection
        vt_det,    33, 33, u8; /// Voltage or temperature variation detected
        gpioirq,   36, 36, u8; /// GPIO interrupt
        aes_done,  37, 37, u8; /// AES-DMA operation complete
        aes_err,   38, 38, u8; /// AES-DMA error
        cmd_err,   39, 39, u8; /// Command error
        spi_ovf,   40, 40, u8; /// SPI overflow error
        spi_unf,   41, 41, u8; /// SPI underflow error
        spierr,    42, 42, u8; /// SPI collision error
        cca_fail,  43, 43, u8; /// This event will be set as a result of failure of CMD_CCA_TX to transmit a packet
 */
pub async fn listen_for_packet<SPI, CS>(
    mut dw3000: dw3000_ng::DW3000<SPI, CS, dw3000_ng::Ready>,
    dwm_config: dw3000_ng::Config,
    mut int_gpio: &mut GpioPin<Input<PullDown>, 15>,
    callback: impl FnOnce(&[u8], Instant),
) -> dw3000_ng::DW3000<SPI, CS, dw3000_ng::Ready>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug + defmt::Format,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug + defmt::Format,
    <CS as embedded_hal::digital::v2::OutputPin>::Error: core::fmt::Debug + defmt::Format,
{
    let mut rxing = dw3000.receive(dwm_config).expect("Failed to receive.");

    let mut buf = [0u8; 128];

    let result = nonblocking_wait(
        || {
            defmt::trace!("Waiting for receive...");
            let status = rxing.r_wait_buf(&mut buf);
            defmt::trace!("SYS_STATUS:");
            let sys_status = rxing.ll().sys_status().read().unwrap();
            {
                defmt::trace!("irqs = {}", sys_status.irqs());
                defmt::trace!("cplock = {}", sys_status.cplock());
                defmt::trace!("spicrce = {}", sys_status.spicrce());
                defmt::trace!("aat = {}", sys_status.aat());
                defmt::trace!("txfrb = {}", sys_status.txfrb());
                defmt::trace!("txprs = {}", sys_status.txprs());
                defmt::trace!("txphs = {}", sys_status.txphs());
                defmt::trace!("txfrs = {}", sys_status.txfrs());
                defmt::trace!("rxprd = {}", sys_status.rxprd());
                defmt::trace!("rxsfdd = {}", sys_status.rxsfdd());
                defmt::trace!("ciadone = {}", sys_status.ciadone());
                defmt::trace!("rxphd = {}", sys_status.rxphd());
                defmt::trace!("rxphe = {}", sys_status.rxphe());
                defmt::trace!("rxfr = {}", sys_status.rxfr());
                defmt::trace!("rxfcg = {}", sys_status.rxfcg());
                defmt::trace!("rxfce = {}", sys_status.rxfce());
                defmt::trace!("rxfsl = {}", sys_status.rxfsl());
                defmt::trace!("rxfto = {}", sys_status.rxfto());
                defmt::trace!("ciaerr = {}", sys_status.ciaerr());
                defmt::trace!("vwarn = {}", sys_status.vwarn());
                defmt::trace!("rxovrr = {}", sys_status.rxovrr());
                defmt::trace!("rxpto = {}", sys_status.rxpto());
                defmt::trace!("spirdy = {}", sys_status.spirdy());
                defmt::trace!("rcinit = {}", sys_status.rcinit());
                defmt::trace!("pll_hilo = {}", sys_status.pll_hilo());
                defmt::trace!("rxsto = {}", sys_status.rxsto());
                defmt::trace!("hpdwarn = {}", sys_status.hpdwarn());
                defmt::trace!("cperr = {}", sys_status.cperr());
                defmt::trace!("arfe = {}", sys_status.arfe());
                defmt::trace!("rxprej = {}", sys_status.rxprej());
                defmt::trace!("vt_det = {}", sys_status.vt_det());
                defmt::trace!("gpioirq = {}", sys_status.gpioirq());
                defmt::trace!("aes_done = {}", sys_status.aes_done());
                defmt::trace!("aes_err = {}", sys_status.aes_err());
                defmt::trace!("cmd_err = {}", sys_status.cmd_err());
                defmt::trace!("spi_ovf = {}", sys_status.spi_ovf());
                defmt::trace!("spi_unf = {}", sys_status.spi_unf());
                defmt::trace!("spierr = {}", sys_status.spierr());
                defmt::trace!("cca_fail = {}", sys_status.cca_fail());
            }
            if sys_status.spierr() == 1 {
                // Read the SPI error status
                let spi_err = rxing.ll().spi_collision().read().unwrap();
                defmt::error!("SPI Collision: {}", spi_err.value());
            }
            return status;
        },
        &mut int_gpio,
    )
    .await;

    if result.is_err() {
        defmt::error!("Failed to receive: {:?}", result.unwrap_err());

        rxing.force_idle().unwrap();
        dw3000 = rxing.finish_receiving().unwrap();
        return dw3000;
    }

    let (msg_length, rx_time) = result.unwrap();

    callback(&buf[..msg_length], rx_time);

    rxing.force_idle().unwrap();
    dw3000 = rxing.finish_receiving().unwrap();

    return dw3000;
}

/// Wait for the first poll packet from the first anchor to arrive
pub async fn wait_for_first_poll<SPI, CS>(
    dw3000: dw3000_ng::DW3000<SPI, CS, dw3000_ng::Ready>,
    dwm_config: dw3000_ng::Config,
    node_config: &MagicLocConfig,
    mut int_gpio: &mut GpioPin<Input<PullDown>, 15>,
) -> (dw3000_ng::DW3000<SPI, CS, dw3000_ng::Ready>, bool)
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug + defmt::Format,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug + defmt::Format,
    <CS as embedded_hal::digital::v2::OutputPin>::Error: core::fmt::Debug + defmt::Format,
{
    let mut poll_received = false;
    let ready = listen_for_packet(dw3000, dwm_config, &mut int_gpio, |buf, _| {
        let frame = Ieee802154Frame::new_checked(&buf[..]).unwrap();

        defmt::debug!("Frame: {:?}", frame);

        if let Some(src_addr) = frame.src_addr() {
            if src_addr
                == Ieee802154Address::from_bytes(
                    &node_config.network_topology.anchor_addrs[0].to_le_bytes(),
                )
            {
                if let Some(payload) = frame.payload() {
                    let poll_packet = magic_loc_protocol::packet::PollPacket::from(
                        u48::from_le_bytes(payload[..6].try_into().unwrap()),
                    );

                    if poll_packet.packet_type() == magic_loc_protocol::packet::PacketType::Poll {
                        defmt::info!("Poll packet received!");
                        poll_received = true;
                    }
                }
            }
        }
    })
    .await;

    return (ready, poll_received);
}
