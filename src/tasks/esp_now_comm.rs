use embassy_futures::{select, yield_now};
use embassy_time::{Duration, Timer};
use esp_wifi::{esp_now::PeerInfo, EspWifiInitialization};
use hal::{macros::ram, peripherals::WIFI};

#[embassy_executor::task]
#[ram]
pub async fn esp_now_comm_task(wifi: WIFI, wifi_init: EspWifiInitialization) -> ! {
    defmt::info!("Starting ESP-NOW communication task");

    let esp_now = esp_wifi::esp_now::EspNow::new(&wifi_init, wifi).unwrap();
    defmt::info!("esp-now version {}", esp_now.get_version().unwrap());

    let (manager, mut sender, mut receiver) = esp_now.split();

    manager
        .set_rate(esp_wifi::esp_now::WifiPhyRate::RateMcs7Sgi)
        .unwrap();
    manager.set_pmk(&[0x00; 16]).unwrap();

    manager
        .add_peer(esp_wifi::esp_now::PeerInfo {
            peer_address: [0x24, 0x0a, 0xc4, 0x00, 0x00, 0x01],
            lmk: None,
            channel: Some(1),
            encrypt: false,
        })
        .unwrap();

    defmt::info!("Peer added");

    let mut peer = manager.fetch_peer(true);
    while let Ok(p) = peer {
        defmt::info!("Peer: {}", p);
        peer = manager.fetch_peer(false);
    }

    Timer::after_secs(2).await;

    for _ in 0..10 {
        let mut count = 0;
        let timeout_time = embassy_time::Instant::now() + Duration::from_secs(1);
        loop {
            let timeout_future = Timer::at(timeout_time);

            let result = select::select(
                sender.send_async(
                    &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],
                    &[0xFE; 220],
                ),
                timeout_future
            )
            .await;

            if let select::Either::First(Ok(_)) = result {
                count += 1;
            } else {
                break;
            }

            yield_now().await;
        }

        defmt::info!("Sent {} packets per second", count);
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
