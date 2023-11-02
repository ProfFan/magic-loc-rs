#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod tasks;

extern crate alloc;
use core::mem::MaybeUninit;
use embassy_executor::Spawner;

use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use hal::{
    clock::ClockControl,
    cpu_control::{CpuControl, Stack},
    embassy::{
        self,
        executor::{FromCpu1, FromCpu2, InterruptExecutor},
    },
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
async fn some_task() {
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
    let clocks = ClockControl::configure(system.clock_control, hal::clock::CpuClock::Clock240MHz).freeze();
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    interrupt::enable(Interrupt::GPIO, interrupt::Priority::Priority1).unwrap();

    embassy::init(
        &clocks,
        hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    spawner.spawn(some_task()).ok();

    // 100kHz I2C clock for the SGM41511
    let i2c: I2C<I2C0> = hal::i2c::I2C::new(
        peripherals.I2C0,
        io.pins.gpio1,
        io.pins.gpio2,
        100u32.kHz(),
        &clocks,
    );

    spawner.spawn(battery_manager(i2c)).ok();

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
    let mut int_dw3000 = io.pins.gpio15.into_pull_down_input();
    int_dw3000.internal_pull_down(true);
    
    int_dw3000.listen(hal::gpio::Event::HighLevel);

    let cs_dw3000 = io.pins.gpio8.into_push_pull_output();
    let rst_dw3000 = io.pins.gpio9.into_push_pull_output();

    let mut cpu_control = CpuControl::new(system.cpu_control);
    let cpu1_fnctn = move || {
        let spawner = INT_EXECUTOR_CORE_1.start(interrupt::Priority::Priority1);

        spawner
            .spawn(tasks::uwb_task(dw3000_spi, cs_dw3000, rst_dw3000, int_dw3000))
            .ok();

        // Just loop to show that the main thread does not need to poll the executor.
        loop {}
    };
    let _guard = cpu_control
        .start_app_core(unsafe { &mut APP_CORE_STACK }, cpu1_fnctn)
        .unwrap();

    let mut led = io.pins.gpio7.into_push_pull_output();

    led.set_high().unwrap();

    loop {
        led.toggle().unwrap();
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

// #[entry]
// fn main() -> ! {
//     init_heap();
//     let peripherals = Peripherals::take();
//     let mut system = peripherals.SYSTEM.split();
//     let clocks = ClockControl::max(system.clock_control).freeze();

//     // setup logger
//     // To change the log_level change the env section in .cargo/config.toml
//     // or remove it and set ESP_LOGLEVEL manually before running cargo run
//     // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
//     esp_println::logger::init_logger_from_env();
//     log::info!("Logger is setup");
//     println!("Hello world!");
//     let timer = TimerGroup::new(
//         peripherals.TIMG1,
//         &clocks,
//         &mut system.peripheral_clock_control,
//     )
//     .timer0;
//     let _init = initialize(
//         EspWifiInitFor::Wifi,
//         timer,
//         Rng::new(peripherals.RNG),
//         system.radio_clock_control,
//         &clocks,
//     )
//     .unwrap();

//     let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
//     let mut led = io.pins.gpio7.into_push_pull_output();

//     led.set_high().unwrap();

//     // Initialize the Delay peripheral, and use it to toggle the LED state in a
//     // loop.
//     let mut delay = Delay::new(&clocks);

//     loop {
//         led.toggle().unwrap();
//         delay.delay_ms(500u32);
//     }
// }
