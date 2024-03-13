// Sync trigger task

use core::{cell::RefCell, future::poll_fn, sync::atomic::AtomicBool, task::Poll};

use arbitrary_int::{u40, u48};
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_sync::{
    blocking_mutex::{raw::CriticalSectionRawMutex, NoopMutex},
    mutex::Mutex,
    waitqueue::AtomicWaker,
};
use embassy_time::{Duration, Instant, Timer};
use embedded_hal::digital::OutputPin;
use hal::{
    clock::Clocks,
    dma::ChannelCreator1,
    dma_descriptors,
    gpio::{GpioPin, Input, Output, PullDown, PushPull},
    macros::{interrupt, ram},
    pcnt::PCNT,
    peripherals::{Interrupt, SPI2},
    prelude::_fugit_RateExtU32,
    spi::{
        master::{dma::WithDmaSpi2, Spi},
        FullDuplexMode,
    },
    FlashSafeDma,
};
use smoltcp::wire::{Ieee802154Address, Ieee802154Frame};

use crate::util::nonblocking_wait;

static PCNT_EVENT: AtomicBool = AtomicBool::new(false);
static mut PCNT_TRIGGER_TIME: embassy_time::Instant = embassy_time::Instant::from_ticks(0);
static UNIT0: Mutex<CriticalSectionRawMutex, RefCell<Option<hal::pcnt::unit::Unit>>> =
    Mutex::new(RefCell::new(None));
static WAKER: AtomicWaker = AtomicWaker::new();

static mut MASTER_POLL_SEQ: u8 = 0;
static mut MASTER_POLL_TIME: Option<Instant> = None;
static mut MASTER_POLL_TXTS: Option<u40> = None;

#[embassy_executor::task]
#[ram]
pub async fn sync_trigger_task(
    mut trigger_pin: GpioPin<Input<PullDown>, 13>,
    pcnt: PCNT<'static>,
) -> ! {
    defmt::info!("Trigger Task Start!");

    let mut u0 = pcnt.get_unit(hal::pcnt::unit::Number::Unit0);

    u0.configure(hal::pcnt::unit::Config {
        low_limit: -100,
        high_limit: 100,
        filter: Some(80),
        thresh0: 1,
        ..Default::default()
    })
    .unwrap();

    let mut ch0 = u0.get_channel(hal::pcnt::channel::Number::Channel0);

    ch0.configure(
        hal::pcnt::channel::PcntSource::always_high(),
        hal::pcnt::channel::PcntSource::from_pin(&mut trigger_pin),
        hal::pcnt::channel::Config {
            lctrl_mode: hal::pcnt::channel::CtrlMode::Disable,
            hctrl_mode: hal::pcnt::channel::CtrlMode::Keep,
            pos_edge: hal::pcnt::channel::EdgeMode::Increment,
            neg_edge: hal::pcnt::channel::EdgeMode::Hold,
            invert_ctrl: false,
            invert_sig: false,
        },
    );

    u0.events(hal::pcnt::unit::Events {
        low_limit: false,
        high_limit: false,
        thresh0: true,
        thresh1: false,
        zero: false,
    });

    u0.listen();
    u0.resume();

    UNIT0.lock().await.replace(Some(u0));

    // Enable the PCNT interrupt
    hal::interrupt::enable(Interrupt::PCNT, hal::interrupt::Priority::Priority3).unwrap();

    defmt::info!("Trigger Task Loop Start");

    loop {
        // Wait for the PCNT event
        let _ = poll_fn(|cx| -> Poll<Result<(), ()>> {
            WAKER.register(cx.waker());

            if PCNT_EVENT.load(core::sync::atomic::Ordering::Relaxed) {
                PCNT_EVENT.store(false, core::sync::atomic::Ordering::Relaxed);
                return Poll::Ready(Ok(()));
            }

            Poll::Pending
        })
        .await;
        // Timer::after_secs(1).await;

        let pcnt_time = unsafe { PCNT_TRIGGER_TIME };
        defmt::info!("PCNT Event at {:?}", pcnt_time);

        critical_section::with(|_| {
            let mut u0 = UNIT0.try_lock().unwrap();
            let u0 = u0.get_mut().as_mut().unwrap();

            u0.clear();
        });

        // Calculate the time difference between the PCNT event and the last poll packet
        let (poll_time, poll_txts, poll_seq) = critical_section::with(|_| unsafe {
            (MASTER_POLL_TIME, MASTER_POLL_TXTS, MASTER_POLL_SEQ)
        });

        if poll_time.is_none() || poll_txts.is_none() {
            defmt::error!("No poll packet received!");
            continue;
        }

        let poll_time = poll_time.unwrap();
        let poll_txts = poll_txts.unwrap();

        defmt::info!(
            "Triggered, TRIGGER_TS: {:?}, LOCAL_TS: {:?}, TXTS: {:?}, SEQ: {}",
            pcnt_time.as_micros(),
            poll_time.as_micros(),
            poll_txts.value(),
            poll_seq
        );
    }
}

#[embassy_executor::task]
#[ram]
pub async fn trigger_message_listener(
    mut bus: Spi<'static, SPI2, FullDuplexMode>,
    cs_gpio: GpioPin<Output<PushPull>, 8>,
    mut rst_gpio: GpioPin<Output<PushPull>, 9>,
    mut int_gpio: GpioPin<Input<PullDown>, 15>,
    dma_channel: ChannelCreator1,
    clocks: Clocks<'static>,
    node_config: crate::config::MagicLocConfig,
) {
    defmt::info!("Trigger Message Listener Start!");

    // let (mut dma_tx, mut dma_rx) = dma_descriptors!(32000);

    // bus.change_bus_frequency(32u32.MHz(), &clocks);
    // let bus = bus.with_dma(dma_channel.configure(
    //     false,
    //     &mut dma_tx,
    //     &mut dma_rx,
    //     hal::dma::DmaPriority::Priority0,
    // ));

    // // Enable DMA interrupts
    // hal::interrupt::enable(
    //     hal::peripherals::Interrupt::DMA_IN_CH1,
    //     hal::interrupt::Priority::Priority2,
    // )
    // .unwrap();
    // hal::interrupt::enable(
    //     hal::peripherals::Interrupt::DMA_OUT_CH1,
    //     hal::interrupt::Priority::Priority2,
    // )
    // .unwrap();

    // let bus = FlashSafeDma::<_, 32000>::new(bus);

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

    dw3000
        .gpio_config(dw3000_ng::hl::ConfigGPIOs::enable_led())
        .unwrap();

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

        // Capture the time the packet was received
        let system_time = embassy_time::Instant::now();

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
                    let packet = magic_loc_protocol::packet::PollPacket::from(u48::from_le_bytes(
                        poll_bytes,
                    ));

                    defmt::debug!("PollPacket: {:?}", packet);

                    if let Some(addr) = frame.src_addr() {
                        if let Some(pan) = frame.dst_pan_id() {
                            if let Ieee802154Address::Short(sa) = addr {
                                defmt::debug!("PollPacket from {:?} pan {:x}", sa, pan);
                                if pan.0 == node_config.uwb_pan_id
                                    && u16::from_le_bytes(sa)
                                        == node_config.network_topology.anchor_addrs[0]
                                {
                                    critical_section::with(|_| {
                                        unsafe {
                                            MASTER_POLL_TIME = Some(system_time);
                                        };
                                        unsafe {
                                            MASTER_POLL_TXTS = Some(packet.tx_timestamp());
                                        }
                                        unsafe {
                                            MASTER_POLL_SEQ = frame.sequence_number().unwrap_or(0);
                                        }
                                    });
                                    defmt::debug!(
                                        "PollPacket, LOCAL_TS: {:?}, TXTS: {:?}",
                                        system_time,
                                        rx_time
                                    );
                                }
                            }
                        }
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

#[interrupt]
fn PCNT() {
    let mut u0 = UNIT0.try_lock().unwrap();
    let u0 = u0.get_mut().as_mut().unwrap();

    if u0.interrupt_set() {
        u0.reset_interrupt();
    }

    unsafe {
        PCNT_TRIGGER_TIME = embassy_time::Instant::now();
    }

    PCNT_EVENT.store(true, core::sync::atomic::Ordering::Release);
    WAKER.wake();
}
