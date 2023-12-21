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

use hal::{
    clock::{ClockControl, Clocks},
    cpu_control::{CpuControl, Stack},
    embassy::{
        self,
        executor::{FromCpu1, FromCpu2, InterruptExecutor},
    },
    gdma::Gdma,
    gpio::GpioPin,
    i2c::I2C,
    interrupt,
    peripherals::{Interrupt, Peripherals, I2C0, SPI2},
    prelude::*,
    spi::{FullDuplexMode, SpiMode},
    UsbSerialJtag, IO,
};

// Stack for the second core
static mut APP_CORE_STACK: Stack<8192> = Stack::new();

static INT_EXECUTOR_CORE_0: InterruptExecutor<FromCpu1> = InterruptExecutor::new();
static INT_EXECUTOR_CORE_1: InterruptExecutor<FromCpu2> = InterruptExecutor::new();

#[interrupt]
fn FROM_CPU_INTR1() {
    unsafe { INT_EXECUTOR_CORE_0.on_interrupt() }
}

#[interrupt]
fn FROM_CPU_INTR2() {
    unsafe { INT_EXECUTOR_CORE_1.on_interrupt() }
}

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
async fn led_blinker(mut led: GpioPin<hal::gpio::Output<hal::gpio::PushPull>, 7>) -> ! {
    loop {
        led.set_high().unwrap();

        loop {
            led.toggle().unwrap();
            Timer::after(Duration::from_millis(1_000)).await;
        }
    }
}

#[embassy_executor::task(pool_size = 8)]
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

    // drop storage
    drop(storage);

    // let config = config::MagicLocConfig {
    //     mode: config::Mode::Tag,
    //     uwb_addr: 0x0003,
    //     uwb_pan_id: 0xBEEF,
    //     network_topology: config::NetworkTopology {
    //         anchor_addrs: [0x1001, 0x1002, 0x1003, 0x1004, 0x1005, 0x1006, 0x1007, 0x1008],
    //         tag_addrs: [0x0001, 0x0002, 0x0003],
    //     },
    // };

    // config::write_config(&config).await.unwrap();

    // Load config from flash
    let config = config::load_config().await.unwrap();

    defmt::info!("Config: {:#x}", config);

    interrupt::enable(Interrupt::GPIO, interrupt::Priority::Priority3).unwrap();

    let led = io.pins.gpio7.into_push_pull_output();

    spawner.spawn(led_blinker(led)).ok();

    // 400kHz I2C clock for the SGM41511
    let i2c: I2C<I2C0> = hal::i2c::I2C::new(
        peripherals.I2C0,
        io.pins.gpio1,
        io.pins.gpio2,
        400u32.kHz(),
        &clocks,
    );

    spawner.spawn(battery_manager(i2c)).ok();

    // DMA
    let dma = Gdma::new(peripherals.DMA);
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

    let core0_spawner = INT_EXECUTOR_CORE_0.start(interrupt::Priority::Priority1);

    // Enable the serial task
    let serial_jtag = UsbSerialJtag::new(peripherals.USB_DEVICE);

    core0_spawner
        .spawn(tasks::serial_comm_task(serial_jtag))
        .ok();

    if config.mode == config::Mode::Tag && false {
        defmt::info!("Mode = TAG, starting IMU");

        // IMU Task
        let imu_spi =
            hal::spi::master::Spi::new(peripherals.SPI3, 30u32.MHz(), SpiMode::Mode0, &clocks)
                .with_pins(
                    Some(io.pins.gpio33),
                    Some(io.pins.gpio47),
                    Some(io.pins.gpio17),
                    Some(io.pins.gpio34),
                );

        // IMU INT
        let int_imu = io.pins.gpio48.into_pull_down_input();
        // int_imu.listen(hal::gpio::Event::HighLevel);

        core0_spawner
            .spawn(tasks::imu_task(imu_spi, dma_channel, int_imu, config))
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
        let spawner = INT_EXECUTOR_CORE_1.start(interrupt::Priority::Priority1);

        // spawner
        //     .spawn(tasks::uwb_task(
        //         dw3000_spi, cs_dw3000, rst_dw3000, int_dw3000,
        //     ))
        //     .ok();

        match &config.mode {
            config::Mode::Anchor => {
                defmt::info!("Mode = Anchor, starting anchor task");

                spawner
                    .spawn(tasks::uwb_anchor_task(
                        dw3000_spi, cs_dw3000, rst_dw3000, int_dw3000, config,
                    ))
                    .ok();
            }
            config::Mode::Tag => {
                defmt::info!("Mode = Tag, starting tag task");

                spawner
                    .spawn(tasks::uwb_task(
                        dw3000_spi, cs_dw3000, rst_dw3000, int_dw3000, config,
                    ))
                    .ok();
            }
            config::Mode::Sniffer => {
                defmt::info!("Mode = Sniffer, starting sniffer task");

                spawner
                    .spawn(tasks::uwb_sniffer(
                        dw3000_spi, cs_dw3000, rst_dw3000, int_dw3000,
                    ))
                    .ok();
            }
        }

        // Just loop to show that the main thread does not need to poll the executor.
        loop {}
    };
    let _guard = cpu_control
        .start_app_core(unsafe { &mut APP_CORE_STACK }, cpu1_fnctn)
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

    embassy::init(
        &clocks,
        hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    spawner.must_spawn(startup_task(clocks));

    loop {
        Timer::after_secs(1).await;
    }
}
