use hal::{i2c::I2C, peripherals::I2C0, prelude::*};

use embassy_time::{Duration, Timer};

#[embassy_executor::task]
pub async fn battery_manager(mut i2c: I2C<'static, I2C0>) {
    log::info!("BMS Task Start!");

    loop {
        let mut data = [0u8; 1];

        // Register 0x07, 1 byte
        i2c.write_read(0x6B, &[0x07], &mut data).ok();

        // Get current time
        let now = embassy_time::Instant::now().as_millis();

        log::info!("[{:}] SGM41511 0x07: {:x}", now, data[0]);

        if data[0] & 0b00100000 == 0b00100000 {
            log::info!("BATFET is OFF");
        } else {
            log::info!("BATFET is ON, setting to OFF");

            // Register 0x07, 1 byte
            i2c.write(0x6B, &[0x07, data[0] | 0b00100000u8]).ok();
        }

        Timer::after(Duration::from_millis(1_000)).await;
    }
}
