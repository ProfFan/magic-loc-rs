use embassy_time::{Duration, Timer};
use embedded_hal_async::digital::Wait;
use hal::{
    gpio::{GpioPin, Input, Output, PullDown, PushPull},
    peripherals::SPI3,
    prelude::*,
    spi::{master::Spi, FullDuplexMode},
};

use lsm6dso::LSM6DSO;

#[embassy_executor::task]
pub async fn imu_task(bus: Spi<'static, SPI3, FullDuplexMode>) -> ! {
    log::info!("IMU Task Start!");

    let mut imu = LSM6DSO::new(bus);

    // SPI Transaction
    let mut buf = [0x0F | 0x80, 0x00];
    let result = imu.ll().spi_bus().transfer(&mut buf).unwrap();

    log::info!("WHOAMI: {:0x}", result[1]);

    // Use the high level to read the WHOAMI register
    let whoami = imu.ll().who_am_i().read().unwrap();

    log::info!("WHOAMI: {:0x}", whoami.value());

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

    loop {
        let mut gyro_data = [0u16; 3];
        let mut accel_data = [0u16; 3];

        let all_readouts = imu.ll().all_readouts().read().unwrap();

        gyro_data[0] = all_readouts.outx_g();
        gyro_data[1] = all_readouts.outy_g();
        gyro_data[2] = all_readouts.outz_g();

        accel_data[0] = all_readouts.outx_a();
        accel_data[1] = all_readouts.outy_a();
        accel_data[2] = all_readouts.outz_a();

        log::info!(
            "Gyro: {:7.2} {:7.2} {:7.2}, Accel: {:7.2} {:7.2} {:7.2}",
            (gyro_data[0] as i16 as f32) * (70.0f32 * 0.001f32),
            (gyro_data[1] as i16 as f32) * (70.0f32 * 0.001f32),
            (gyro_data[2] as i16 as f32) * (70.0f32 * 0.001f32),
            (accel_data[0] as i16 as f32) * (0.244f32 * 9.81f32 * 0.001f32),
            (accel_data[1] as i16 as f32) * (0.244f32 * 9.81f32 * 0.001f32),
            (accel_data[2] as i16 as f32) * (0.244f32 * 9.81f32 * 0.001f32)
        );

        Timer::after(Duration::from_millis(30)).await;
    }
}
