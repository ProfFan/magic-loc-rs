// Unit tests task used for debugging

use embassy_time::{Duration, Timer};
// use smoltcp;
// use smoltcp::wire::{Ieee802154Address, Ieee802154Frame, Ieee802154Repr, Ieee802154FrameType, Ieee802154FrameVersion, Ieee802154Pan};

/// Test task for smoltcp's 802.15.4 wire format
#[embassy_executor::task]
pub async fn unit_test() -> ! {
    defmt::info!("Unit Test Task Start!");

    loop {
        // Sleep forever
        Timer::after(Duration::from_secs(1)).await;
    }
}
