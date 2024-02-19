use binrw::{io::Cursor, prelude::BinWrite};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::digital::Wait;
use hal::{
    dma::{ChannelCreator0, DmaDescriptor},
    gpio::{GpioPin, Input, Output, PullDown, PushPull},
    macros::ram,
    peripherals::SPI3,
    spi::{
        master::{prelude::*, Spi},
        FullDuplexMode,
    },
    FlashSafeDma,
};

use lsm6dso::LSM6DSO;

use crate::{
    config::MagicLocConfig, operations::host::ImuReport, tasks::write_to_usb_serial_buffer,
};

#[embassy_executor::task]
#[ram]
pub async fn imu_task(
    bus: Spi<'static, SPI3, FullDuplexMode>,
    chip_select: GpioPin<Output<PushPull>, 34>,
    dma_channel: ChannelCreator0,
    mut int1: GpioPin<Input<PullDown>, 48>,
    config: MagicLocConfig,
) -> ! {
    defmt::info!("IMU Task Start!");

    let mut descriptors = [DmaDescriptor::EMPTY; 8 * 3];
    let mut rx_descriptors = [DmaDescriptor::EMPTY; 8 * 3];

    let bus = bus.with_dma(dma_channel.configure(
        false,
        &mut descriptors,
        &mut rx_descriptors,
        hal::dma::DmaPriority::Priority0,
    ));

    // let bus = FlashSafeDma::<_, 128>::new(bus);

    let bus = Mutex::<NoopRawMutex, _>::new(bus);
    let device = SpiDevice::new(&bus, chip_select);

    let mut imu = LSM6DSO::new(device);

    // SPI Transaction with async
    let send_buf = [0x0Fu8 | 0x80, 0x00];
    let mut recv_buf = [0u8; 2];
    embedded_hal_async::spi::SpiDevice::transfer(imu.ll().spi_bus(), &mut recv_buf, &send_buf)
        .await
        .unwrap();

    defmt::info!("WHOAMI Async: {:#x}", recv_buf[1]);

    // Use the high level to read the WHOAMI register
    let whoami = imu.ll().who_am_i().async_read().await.unwrap();

    defmt::info!("WHOAMI: {:#x}", whoami.value());

    // Assert that the WHOAMI register is correct
    if whoami.value() != 0x6C {
        defmt::error!("WHOAMI register is incorrect!");

        loop {
            Timer::after(Duration::from_millis(100)).await;
        }
    }

    // Reset the device
    imu.ll()
        .ctrl3_c()
        .async_modify(|_, w| w.sw_reset(0x1))
        .await
        .unwrap();

    // Wait for the reset to complete
    loop {
        let reset = imu.ll().ctrl3_c().async_read().await.unwrap();

        if reset.sw_reset() == 0 {
            break;
        }

        Timer::after(Duration::from_millis(10)).await;
    }

    // Set the gyro to 2000dps 833Hz
    imu.ll()
        .ctrl2_g()
        .async_modify(|_, w| w.fs_g(0b1100).odr_g(7))
        .await
        .unwrap();

    // Set the accelerometer to 8g 833Hz
    imu.ll()
        .ctrl1_xl()
        .async_modify(|_, w| w.fs_xl(3).odr_xl(7))
        .await
        .unwrap();

    // Disable block data update
    imu.ll()
        .ctrl3_c()
        .async_modify(|_, w| w.bdu(0))
        .await
        .unwrap();

    // Read the actual ODR
    let freq_fine_b = imu.ll().freq_fine().async_read().await.unwrap();
    let odr_actual = (6667.0 + ((0.0015 * (freq_fine_b.value() as f32)) * 6667.0)) / 8.0;

    defmt::info!("ODR Actual: {}", odr_actual);

    // Set INT1 to DRDY_XL
    imu.ll()
        .int1_ctrl()
        .async_modify(|_, w| w.int1_drdy_xl(1))
        .await
        .unwrap();

    // Read once to clear the interrupt
    let _ = imu.ll().all_readouts().async_read().await.unwrap();

    // Enable the interrupt
    hal::gpio::Pin::listen(&mut int1, hal::gpio::Event::HighLevel);

    let mut count: i32 = 0;
    let mut ts_now = Instant::now();
    loop {
        // Wait for the interrupt
        int1.wait_for_high().await.unwrap();

        let mut imu_data = ImuReport::default();

        imu_data.tag_addr = config.uwb_addr;
        imu_data.system_ts = Instant::now().as_micros();

        let all_readouts = imu.ll().all_readouts().async_read().await.unwrap();

        imu_data.gyro[0] = all_readouts.outx_g() as u32;
        imu_data.gyro[1] = all_readouts.outy_g() as u32;
        imu_data.gyro[2] = all_readouts.outz_g() as u32;

        imu_data.accel[0] = all_readouts.outx_a() as u32;
        imu_data.accel[1] = all_readouts.outy_a() as u32;
        imu_data.accel[2] = all_readouts.outz_a() as u32;

        defmt::trace!(
            "Gyro: {} {} {}, Accel {} {} {}",
            (imu_data.gyro[0] as i16),
            (imu_data.gyro[1] as i16),
            (imu_data.gyro[2] as i16),
            (imu_data.accel[0] as i16),
            (imu_data.accel[1] as i16),
            (imu_data.accel[2] as i16)
        );

        count += 1;

        // Send the data to the host
        let mut imu_buffer: [u8; 128] = [0; 128];
        let mut imu_buffer_cursor = Cursor::new(&mut imu_buffer[..]);

        imu_data.write(&mut imu_buffer_cursor).unwrap();
        let imu_buffer_len = imu_buffer_cursor.position() as usize;

        let mut buffer = [0u8; 128];
        let (header, data) = buffer.split_at_mut(2);
        header.copy_from_slice(&[0xFF, 0x01]);

        let mut encoder = defmt::Encoder::new();
        let mut cursor = 0;
        let mut write_bytes = |bytes: &[u8]| {
            data[cursor..cursor + bytes.len()].copy_from_slice(bytes);
            cursor += bytes.len();
        };
        encoder.start_frame(&mut write_bytes);
        encoder.write(&imu_buffer[..imu_buffer_len], &mut write_bytes);
        encoder.end_frame(&mut write_bytes);

        let _ = write_to_usb_serial_buffer(&buffer[..cursor + 3]);

        if count == 800 {
            let ts_now_new = Instant::now();
            let ts_diff = ts_now_new - ts_now;
            ts_now = ts_now_new;

            defmt::info!("IMU Freq = {}", 800_000.0 / (ts_diff.as_millis() as f32));

            count = 0;
        }
    }
}
