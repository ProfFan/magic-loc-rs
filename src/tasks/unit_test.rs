// Unit tests task used for debugging

use embassy_time::{Duration, Timer};

/// Test task for smoltcp's 802.15.4 wire format
#[embassy_executor::task]
pub async fn unit_test() -> ! {
    defmt::info!("Unit Test Task Start!");

    loop {
        // Sleep forever
        Timer::after(Duration::from_secs(1)).await;
    }
}
