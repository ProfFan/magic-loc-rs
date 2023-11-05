use embassy_time::{Duration, Timer};
use embedded_hal_async::digital::Wait;
use hal::{
    gdma::ChannelCreator0,
    gpio::{GpioPin, Input, PullDown},
    peripherals::SPI3,
    spi::{
        master::{prelude::*, Spi},
        FullDuplexMode,
    },
};

use lsm6dso::LSM6DSO;

#[embassy_executor::task]
pub async fn imu_task(
    bus: Spi<'static, SPI3, FullDuplexMode>,
    dma_channel: ChannelCreator0,
    mut int1: GpioPin<Input<PullDown>, 48>,
) -> ! {
    log::info!("IMU Task Start!");

    let mut descriptors = [0u32; 8 * 3];
    let mut rx_descriptors = [0u32; 8 * 3];

    let bus = bus.with_dma(dma_channel.configure(
        false,
        &mut descriptors,
        &mut rx_descriptors,
        hal::dma::DmaPriority::Priority0,
    ));

    let mut imu = LSM6DSO::new(bus);

    // SPI Transaction with async
    let send_buf = [0x0Fu8 | 0x80, 0x00];
    let mut recv_buf = [0u8; 2];
    embedded_hal_async::spi::SpiBus::transfer(&mut imu.ll().spi_bus(), &mut recv_buf, &send_buf)
        .await
        .unwrap();

    log::info!("WHOAMI Async: {:0x}", recv_buf[1]);

    // Use the high level to read the WHOAMI register
    let whoami = imu.ll().who_am_i().read().unwrap();

    log::info!("WHOAMI: {:0x}", whoami.value());

    // Assert that the WHOAMI register is correct
    if whoami.value() != 0x6C {
        log::error!("WHOAMI register is incorrect!");

        loop {
            Timer::after(Duration::from_millis(100)).await;
        }
    }

    // Reset the device
    imu.ll().ctrl3_c().modify(|_, w| w.sw_reset(0x1)).unwrap();

    // Wait for the reset to complete
    loop {
        let reset = imu.ll().ctrl3_c().read().unwrap();

        if reset.sw_reset() == 0 {
            break;
        }

        Timer::after(Duration::from_millis(10)).await;
    }

    // Set the gyro to 2000dps 833Hz
    imu.ll()
        .ctrl2_g()
        .modify(|_, w| w.fs_g(12).odr_g(7))
        .unwrap();

    // Set the accelerometer to 8g 833Hz
    imu.ll()
        .ctrl1_xl()
        .modify(|_, w| w.fs_xl(3).odr_xl(7))
        .unwrap();

    // Disable block data update
    imu.ll().ctrl3_c().modify(|_, w| w.bdu(0)).unwrap();

    // Read the actual ODR
    let freq_fine_b = imu.ll().freq_fine().read().unwrap();
    let odr_actual = (6667.0 + ((0.0015 * (freq_fine_b.value() as f32)) * 6667.0)) / 8.0;

    log::info!("ODR Actual: {}", odr_actual);

    // Set INT1 to DRDY_XL
    imu.ll()
        .int1_ctrl()
        .modify(|_, w| w.int1_drdy_xl(1))
        .unwrap();

    // Read once to clear the interrupt
    let _ = imu.ll().all_readouts().read().unwrap();

    loop {
        // Wait for the interrupt
        int1.wait_for_high().await.unwrap();

        let mut gyro_data = [0u16; 3];
        let mut accel_data = [0u16; 3];

        let all_readouts = imu.ll().all_readouts().read().unwrap();

        gyro_data[0] = all_readouts.outx_g();
        gyro_data[1] = all_readouts.outy_g();
        gyro_data[2] = all_readouts.outz_g();

        accel_data[0] = all_readouts.outx_a();
        accel_data[1] = all_readouts.outy_a();
        accel_data[2] = all_readouts.outz_a();

        log::debug!(
            "Gyro: {:} {:} {:}, Accel: {:} {:} {:}",
            (gyro_data[0] as i16),
            (gyro_data[1] as i16),
            (gyro_data[2] as i16),
            (accel_data[0] as i16),
            (accel_data[1] as i16),
            (accel_data[2] as i16)
        );
    }
}
