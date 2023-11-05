#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod config;
mod tasks;

extern crate alloc;
use config::MagicLocConfig;
use core::mem::MaybeUninit;
use embassy_executor::Spawner;

use embedded_storage::{ReadStorage, Storage};
use esp_storage::FlashStorage;
use postcard::from_bytes;

const NVS_ADDR: u32 = 0x9000;

use embassy_time::{Duration, Timer};
use esp_backtrace as _;
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
    IO,
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

/// Power-on configuration loader
async fn load_config() -> Option<config::MagicLocConfig> {
    let mut storage = FlashStorage::new();
    log::info!("Flash size = {}", storage.capacity());

    let mut buf = [0u8; core::mem::size_of::<MagicLocConfig>()];

    return storage
        .read(NVS_ADDR, &mut buf)
        .map_err(|_| ())
        .and_then(|_| -> Result<MagicLocConfig, ()> {
            let config = from_bytes::<MagicLocConfig>(&buf);

            if config.is_err() {
                log::error!("Failed, reason {:?}", config.as_ref());
                return Err(());
            }

            return config.map_err(|_| ());
        })
        .or_else(|_| -> Result<_, ()> {
            log::info!("No config found!");
            // Make default
            let config = MagicLocConfig::default();

            // save to flash
            let mut buf = [0u8; core::mem::size_of::<MagicLocConfig>()];
            let buf = postcard::to_slice(&config, &mut buf).unwrap();

            storage.write(NVS_ADDR, buf).unwrap();

            log::info!("Saved default config!");

            return Ok(config);
        })
        .ok();
}

async fn write_config(config: &MagicLocConfig) -> Result<(), ()> {
    let mut storage = FlashStorage::new();

    // save to flash
    let mut buf = [0u8; core::mem::size_of::<MagicLocConfig>()];
    let buf = postcard::to_slice(config, &mut buf).map_err(|_| ())?;

    storage.write(NVS_ADDR, buf).map_err(|_| ())?;

    log::info!("Saved config!");

    return Ok(());
}

#[embassy_executor::task]
async fn startup_task(clocks: Clocks<'static>) -> ! {
    let peripherals = unsafe { Peripherals::steal() };
    let spawner = Spawner::for_current_executor().await;
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let system = peripherals.SYSTEM.split();

    // Load config from flash
    let mut config = load_config().await.unwrap();

    // config.mode = config::Mode::Anchor;
    // write_config(&config).await.unwrap();

    log::info!("Config: {:?}", config);

    interrupt::enable(Interrupt::GPIO, interrupt::Priority::Priority2).unwrap();

    let led = io.pins.gpio7.into_push_pull_output();

    spawner.spawn(led_blinker(led)).ok();

    // 100kHz I2C clock for the SGM41511
    let i2c: I2C<I2C0> = hal::i2c::I2C::new(
        peripherals.I2C0,
        io.pins.gpio1,
        io.pins.gpio2,
        100u32.kHz(),
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

    if config.mode == config::Mode::Tag {
        log::info!("Mode = TAG, starting IMU");


        let imu_spawner = INT_EXECUTOR_CORE_0.start(interrupt::Priority::Priority1);
        // IMU Task
        let imu_spi = hal::spi::master::Spi::new(
            peripherals.SPI3,
            io.pins.gpio33,
            io.pins.gpio47,
            io.pins.gpio17,
            io.pins.gpio34,
            30u32.MHz(),
            SpiMode::Mode0,
            &clocks,
        );

        // IMU INT
        let int_imu = io.pins.gpio48.into_pull_down_input();
        // int_imu.listen(hal::gpio::Event::HighLevel);

        imu_spawner
            .spawn(tasks::imu_task(imu_spi, dma_channel, int_imu))
            .ok();
    }

    // DW3000 SPI
    let dw3000_spi: hal::spi::master::Spi<SPI2, FullDuplexMode> = hal::spi::master::Spi::new_no_cs(
        peripherals.SPI2,
        io.pins.gpio36,
        io.pins.gpio35,
        io.pins.gpio37,
        30u32.MHz(),
        SpiMode::Mode0,
        &clocks,
    );

    // DW3000 Interrupt
    let int_dw3000 = io.pins.gpio15.into_pull_down_input();
    let cs_dw3000 = io.pins.gpio8.into_push_pull_output();
    let rst_dw3000 = io.pins.gpio9.into_push_pull_output();

    let mut cpu_control = CpuControl::new(system.cpu_control);
    let cpu1_fnctn = move || {
        let spawner = INT_EXECUTOR_CORE_1.start(interrupt::Priority::Priority1);

        spawner
            .spawn(tasks::uwb_task(
                dw3000_spi, cs_dw3000, rst_dw3000, int_dw3000,
            ))
            .ok();

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
