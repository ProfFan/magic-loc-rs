#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod config;
mod operations;
mod tasks;
mod util;

extern crate alloc;
use core::mem::MaybeUninit;
use embassy_executor::Spawner;

use esp_storage::FlashStorage;

use esp_partition_table as ept;

use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_println as _;

use esp_wifi;

use hal::{
    clock::{ClockControl, Clocks},
    cpu_control::{CpuControl, Stack},
    dma::Dma,
    embassy::{self, executor::InterruptExecutor},
    gpio::{self, GpioPin, IO},
    i2c::I2C,
    interrupt,
    peripherals::{Interrupt, Peripherals, I2C0, SPI2},
    prelude::*,
    spi::{FullDuplexMode, SpiMode},
    usb_serial_jtag::UsbSerialJtag,
    Blocking,
};
use static_cell::StaticCell;

// Stack for the second core
static mut APP_CORE_STACK: Stack<65536> = Stack::new();

use crate::tasks::battery_manager;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

#[embassy_executor::task]
async fn led_blinker(mut led: GpioPin<hal::gpio::Output<hal::gpio::PushPull>, 7>, id: u32) -> ! {
    loop {
        for _ in 0..id {
            led.set_high();
            Timer::after(Duration::from_millis(100)).await;
            led.set_low();
            Timer::after(Duration::from_millis(100)).await;
        }
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[embassy_executor::task(pool_size = 12)]
async fn startup_task(clocks: Clocks<'static>) -> ! {
    let peripherals = unsafe { Peripherals::steal() };
    let spawner = Spawner::for_current_executor().await;
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let system = peripherals.SYSTEM.split();

    // Read the partition table
    let partition_table = ept::PartitionTable::default();
    let mut storage = FlashStorage::new();
    let mut storage_offset: Option<u32> = None;
    for partition in partition_table.iter_storage(&mut storage, false) {
        let partition = partition.unwrap();
        defmt::info!(
            "Partition: {:?}, offset: {:#x}",
            partition.name(),
            partition.offset
        );
        if partition.name() == "storage" {
            storage_offset = Some(partition.offset);
        }
    }

    let storage_offset = storage_offset.unwrap();
    unsafe {
        config::STORAGE_OFFSET = storage_offset;
    }

    // let config = config::MagicLocConfig {
    //     mode: config::Mode::TdoaAnchor,
    //     uwb_addr: 0x3001,
    //     uwb_pan_id: 0xDEAD,
    //     enable_imu: config::ImuConfig::LSM6DSO(config::LSM6DSOConfig {
    //         odr: 0x06,
    //         fs: 0x02,
    //     }),
    //     network_topology: config::NetworkTopology {
    //         anchor_addrs: [0x3001, 0x3002, 0x3003, 0x3004, 0x3005, 0x3006, 0x3007, 0x3008],
    //         tag_addrs: [0x0001, 0x0002, 0x0003],
    //     },
    //     cir_acq_options: Some(config::CirAcquisitionOptions {
    //         start_tap: 0,
    //         num_samples: 128,
    //     }),
    // };

    // config::write_config(&config).await.unwrap();

    // Before loading the config, delay for 1 second to allow the board to be flashed
    Timer::after(Duration::from_secs(1)).await;

    // Load config from flash
    let config = config::load_config().await.unwrap();

    defmt::info!("Config: {:#x}", config);

    interrupt::enable(Interrupt::GPIO, interrupt::Priority::Priority3).unwrap();

    let led = io.pins.gpio7.into_push_pull_output();

    // blink the last digit of the UWB address
    spawner
        .spawn(led_blinker(led, config.uwb_addr as u32 % 0x1000 % 10))
        .ok();

    // 400kHz I2C clock for the SGM41511
    let i2c: I2C<I2C0, Blocking> = hal::i2c::I2C::new(
        peripherals.I2C0,
        io.pins.gpio1,
        io.pins.gpio2,
        400u32.kHz(),
        &clocks,
        None,
    );

    spawner.spawn(battery_manager(i2c)).ok();

    // DMA
    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    // Enable DMA interrupts
    hal::interrupt::enable(
        hal::peripherals::Interrupt::DMA_IN_CH0,
        hal::interrupt::Priority::Priority2,
    )
    .unwrap();
    hal::interrupt::enable(
        hal::peripherals::Interrupt::DMA_OUT_CH0,
        hal::interrupt::Priority::Priority2,
    )
    .unwrap();

    // Interrupt executors
    static INT_EXECUTOR_CORE_0: StaticCell<InterruptExecutor<1>> = StaticCell::new();
    static INT_EXECUTOR_CORE_1: StaticCell<InterruptExecutor<2>> = StaticCell::new();
    static INT_EXECUTOR_CORE_0_P3: StaticCell<InterruptExecutor<3>> = StaticCell::new();
    let executor_core0 =
        InterruptExecutor::new(system.software_interrupt_control.software_interrupt1);
    let executor_core0 = INT_EXECUTOR_CORE_0.init(executor_core0);
    let executor_core1 =
        InterruptExecutor::new(system.software_interrupt_control.software_interrupt2);
    let executor_core1 = INT_EXECUTOR_CORE_1.init(executor_core1);
    let executor_core0_p3 =
        InterruptExecutor::new(system.software_interrupt_control.software_interrupt3);
    let executor_core0_p3 = INT_EXECUTOR_CORE_0_P3.init(executor_core0_p3);

    let core0_spawner = executor_core0.start(interrupt::Priority::Priority2);
    let core0_spawner_p3 = executor_core0_p3.start(interrupt::Priority::Priority3);

    // Enable the serial task
    spawner.spawn(tasks::serial_comm_task()).ok();

    Timer::after_millis(10).await;

    if config.mode == config::Mode::Tag && config.enable_imu != config::ImuConfig::None {
        defmt::info!("Mode = TAG, starting IMU");

        // IMU Task
        let imu_spi =
            hal::spi::master::Spi::new(peripherals.SPI3, 30u32.MHz(), SpiMode::Mode0, &clocks)
                .with_pins(
                    Some(io.pins.gpio33),
                    Some(io.pins.gpio47),
                    Some(io.pins.gpio40), // GPIO 17 for old boards with bad pinout
                    gpio::NO_PIN,
                );

        // IMU INT
        let int_imu = io.pins.gpio48.into_pull_down_input();

        core0_spawner_p3
            .spawn(tasks::imu_task(
                imu_spi,
                io.pins.gpio34.into_push_pull_output(),
                dma_channel,
                int_imu,
                config,
            ))
            .ok();
    }

    // DW3000 SPI
    let dw3000_spi: hal::spi::master::Spi<SPI2, FullDuplexMode> =
        hal::spi::master::Spi::new(peripherals.SPI2, 36u32.MHz(), SpiMode::Mode0, &clocks)
            .with_mosi(io.pins.gpio35)
            .with_sck(io.pins.gpio36)
            .with_miso(io.pins.gpio37);

    // DW3000 Interrupt
    let int_dw3000 = io.pins.gpio15.into_pull_down_input();
    let cs_dw3000 = io.pins.gpio8.into_push_pull_output();
    let rst_dw3000 = io.pins.gpio9.into_push_pull_output();

    let mut cpu_control = CpuControl::new(system.cpu_control);
    let cpu1_fnctn = move || {
        let spawner = executor_core1.start(interrupt::Priority::Priority1);

        match &config.mode {
            config::Mode::Anchor => {
                defmt::info!("Mode = Anchor, starting anchor task");

                spawner
                    .spawn(tasks::uwb_anchor_task(
                        dw3000_spi,
                        cs_dw3000,
                        rst_dw3000,
                        int_dw3000,
                        config,
                        dma.channel1,
                    ))
                    .ok();
            }
            config::Mode::Tag => {
                defmt::info!("Mode = Tag, starting tag task");

                spawner
                    .spawn(tasks::uwb_task(
                        dw3000_spi,
                        cs_dw3000,
                        rst_dw3000,
                        int_dw3000,
                        config,
                        dma.channel1,
                    ))
                    .ok();
            }
            config::Mode::Sniffer => {
                defmt::info!("Mode = Sniffer, starting sniffer task");

                spawner
                    .spawn(tasks::uwb_sniffer(
                        dw3000_spi,
                        cs_dw3000,
                        rst_dw3000,
                        int_dw3000,
                        config,
                        dma.channel1,
                    ))
                    .ok();
            }
            config::Mode::SyncTrigger => {
                defmt::info!("Mode = Sync Trigger, starting sync trigger task");

                core0_spawner_p3
                    .spawn(tasks::sync_trigger_task(
                        io.pins.gpio13.into_pull_down_input(),
                        peripherals.PCNT,
                    ))
                    .ok();

                spawner
                    .spawn(tasks::trigger_message_listener(
                        dw3000_spi,
                        cs_dw3000,
                        rst_dw3000,
                        int_dw3000,
                        dma.channel1,
                        clocks,
                        config,
                    ))
                    .ok();
            }
            config::Mode::TdoaAnchor => {
                defmt::info!("Mode = TDoA Anchor, starting TDoA anchor task");

                spawner
                    .spawn(tasks::tdoa_anchor_task(
                        dw3000_spi, cs_dw3000, rst_dw3000, int_dw3000, config,
                    ))
                    .ok();
            }
            config::Mode::TdoaTag => {
                defmt::info!("Mode = TDoA Tag, starting TDoA tag task");

                spawner
                    .spawn(tasks::passive_tag_task(
                        dw3000_spi, cs_dw3000, rst_dw3000, int_dw3000, config,
                    ))
                    .ok();
            } //
              // default => {
              //     defmt::error!("Unsupported mode: {:?}", default);
              // }
        }

        // Just loop to show that the main thread does not need to poll the executor.
        loop {}
    };
    let _guard = cpu_control
        .start_app_core(
            unsafe { &mut *core::ptr::addr_of_mut!(APP_CORE_STACK) },
            cpu1_fnctn,
        )
        .unwrap();

    loop {
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[main]
async fn main(spawner: Spawner) -> ! {
    esp_println::println!("Init!");
    esp_println::logger::init_logger_from_env();
    init_heap();

    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks =
        ClockControl::configure(system.clock_control, hal::clock::CpuClock::Clock240MHz).freeze();

    // let timer_group0 = hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks);
    let systick = hal::systimer::SystemTimer::new_async(peripherals.SYSTIMER);
    embassy::init(&clocks, systick);

    spawner.must_spawn(startup_task(clocks));

    loop {
        Timer::after_secs(1).await;
    }
}
