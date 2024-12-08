#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod config;
mod operations;
mod tasks;
mod util;

extern crate alloc;
use embassy_executor::Spawner;

use esp_hal_embassy::InterruptExecutor;
use esp_storage::FlashStorage;

use esp_partition_table as ept;

use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_println as _;

use esp_wifi as _;

use hal as esp_hal;
use hal::gpio::Input;
use hal::i2c::master::I2c;
use hal::interrupt::software::SoftwareInterruptControl;
use hal::timer::systimer::SystemTimer;
use hal::timer::timg::TimerGroup;
use hal::timer::AnyTimer;
use hal::{
    cpu_control::{CpuControl, Stack},
    dma::Dma,
    gpio::{self, Output},
    interrupt,
    peripherals::{Interrupt, Peripherals, SPI2},
    prelude::*,
    Blocking,
};
use static_cell::StaticCell;

// Stack for the second core
static mut APP_CORE_STACK: Stack<65536> = Stack::new();

use crate::tasks::battery_manager;

#[embassy_executor::task]
async fn led_blinker(mut led: esp_hal::gpio::Output<'static>, id: u32) -> ! {
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

#[embassy_executor::task(pool_size = 2)]
async fn startup_task() -> ! {
    let peripherals = unsafe { Peripherals::steal() };
    let spawner = Spawner::for_current_executor().await;

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
    //     mode: config::Mode::SymmetricTwr,
    //     uwb_addr: 0x0001,
    //     uwb_pan_id: 0xDEAD,
    //     enable_imu: config::ImuConfig::LSM6DSO(config::LSM6DSOConfig {
    //         odr: 0x06,
    //         fs: 0x02,
    //     }),
    //     network_topology: config::NetworkTopology {
    //         anchor_addrs: [
    //             0x0001, 0x3001, 0x3003, 0x3004, 0x3005, 0x3006, 0x3007, 0x3008,
    //         ],
    //         tag_addrs: [0x3001, 0x0001, 0x0003],
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

    let led = Output::new(peripherals.GPIO7, gpio::Level::Low);

    // blink the last digit of the UWB address
    spawner
        .spawn(led_blinker(led, config.uwb_addr as u32 % 0x1000 % 10))
        .ok();

    let bms_sda = Input::new(peripherals.GPIO1, gpio::Pull::Up);
    let bms_scl = Input::new(peripherals.GPIO2, gpio::Pull::Up);

    // 400kHz I2C clock for the SGM41511
    let i2c: I2c<Blocking> = hal::i2c::master::I2c::new(
        peripherals.I2C0,
        hal::i2c::master::Config {
            frequency: 400u32.kHz(),
            ..Default::default()
        },
    )
    .with_sda(bms_sda)
    .with_scl(bms_scl);

    spawner.spawn(battery_manager(i2c)).ok();

    // DMA
    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    // Enable DMA interrupts
    // hal::interrupt::enable(
    //     hal::peripherals::Interrupt::DMA_IN_CH0,
    //     hal::interrupt::Priority::Priority2,
    // )
    // .unwrap();
    // hal::interrupt::enable(
    //     hal::peripherals::Interrupt::DMA_OUT_CH0,
    //     hal::interrupt::Priority::Priority2,
    // )
    // .unwrap();

    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    // Interrupt executors
    static INT_EXECUTOR_CORE_0: StaticCell<InterruptExecutor<1>> = StaticCell::new();
    static INT_EXECUTOR_CORE_1: StaticCell<InterruptExecutor<2>> = StaticCell::new();
    static INT_EXECUTOR_CORE_0_P3: StaticCell<InterruptExecutor<0>> = StaticCell::new();
    let executor_core0 = InterruptExecutor::new(sw_ints.software_interrupt1);
    let executor_core0 = INT_EXECUTOR_CORE_0.init(executor_core0);
    let executor_core1 = InterruptExecutor::new(sw_ints.software_interrupt2);
    let executor_core1 = INT_EXECUTOR_CORE_1.init(executor_core1);
    let executor_core0_p3 = InterruptExecutor::new(sw_ints.software_interrupt0);
    let executor_core0_p3 = INT_EXECUTOR_CORE_0_P3.init(executor_core0_p3);

    let core0_spawner = executor_core0.start(interrupt::Priority::Priority2);
    let core0_spawner_p3 = executor_core0_p3.start(interrupt::Priority::Priority3);

    // Enable the serial task
    core0_spawner
        .spawn(esp_fast_serial::serial_comm_task(peripherals.USB_DEVICE))
        .unwrap();

    Timer::after_millis(10).await;

    if config.mode == config::Mode::Tag
        || config.mode == config::Mode::SymmetricTwr && config.enable_imu != config::ImuConfig::None
    {
        defmt::info!("Mode = TAG, starting IMU");

        // IMU Task
        let imu_spi = hal::spi::master::Spi::new_typed_with_config(
            peripherals.SPI3,
            hal::spi::master::Config {
                frequency: 30u32.MHz(),
                mode: hal::spi::SpiMode::Mode0,
                ..Default::default()
            },
        )
        .with_sck(peripherals.GPIO33)
        .with_mosi(peripherals.GPIO40)
        .with_miso(peripherals.GPIO47);

        // IMU INT
        let int_imu = Input::new(peripherals.GPIO48, gpio::Pull::Down);

        core0_spawner_p3
            .spawn(tasks::imu_task(
                imu_spi,
                Output::new(peripherals.GPIO34, gpio::Level::High),
                dma_channel,
                int_imu,
                config,
            ))
            .ok();
    }

    // DW3000 SPI
    let dw3000_spi: hal::spi::master::Spi<'_, Blocking, SPI2> =
        hal::spi::master::Spi::new_typed_with_config(
            peripherals.SPI2,
            hal::spi::master::Config {
                frequency: 36u32.MHz(),
                mode: hal::spi::SpiMode::Mode0,
                ..Default::default()
            },
        )
        .with_mosi(peripherals.GPIO35)
        .with_sck(peripherals.GPIO36)
        .with_miso(peripherals.GPIO37);

    // DW3000 Interrupt
    let int_dw3000 = Input::new(peripherals.GPIO15, gpio::Pull::Down);
    let cs_dw3000 = Output::new(peripherals.GPIO8, gpio::Level::High);
    let rst_dw3000 = Output::new(peripherals.GPIO9, gpio::Level::Low);

    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
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
                        Input::new(peripherals.GPIO13, gpio::Pull::Down),
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
            }
            config::Mode::SymmetricTwr => {
                defmt::info!("Mode = Symmetric TWR, starting symmetric TWR task");

                if config.uwb_addr == 0x0001 {
                    spawner
                        .spawn(tasks::symmetric_twr_task(
                            dw3000_spi,
                            cs_dw3000,
                            rst_dw3000,
                            int_dw3000,
                            config,
                            dma.channel1,
                        ))
                        .ok();
                } else {
                    spawner
                        .spawn(tasks::symmetric_twr_anchor_task(
                            dw3000_spi, cs_dw3000, rst_dw3000, int_dw3000, config,
                        ))
                        .ok();
                }
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
    let mut hal_config = hal::Config::default();
    hal_config.cpu_clock = hal::clock::CpuClock::Clock240MHz;

    esp_println::println!("Init!");
    esp_println::logger::init_logger_from_env();

    let peripherals = hal::init(hal_config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timer0: AnyTimer = timg0.timer0.into();
    let timer1: AnyTimer = timg0.timer1.into();
    let systimer = SystemTimer::new(peripherals.SYSTIMER).split::<_>();
    let timer2: AnyTimer = systimer.alarm0.into_target().into();
    let timer3: AnyTimer = systimer.alarm1.into_target().into();
    esp_hal_embassy::init([timer0, timer1, timer2, timer3]);
    esp_alloc::heap_allocator!(128 * 1024);

    spawner.must_spawn(startup_task());

    loop {
        Timer::after_secs(1).await;
    }
}
