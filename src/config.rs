use binrw::{io::Cursor, *};
use defmt::Format;

use embedded_storage::{ReadStorage, Storage};
use esp_storage::FlashStorage;

pub(crate) static mut STORAGE_OFFSET: u32 = 0x9900;

/// enum for device operation mode
#[derive(Default, Debug, Format, PartialEq, Clone, Copy)]
#[binrw]
#[brw(repr(u8))]
pub enum Mode {
    /// Device is in anchor mode
    #[default]
    Anchor = 0,
    /// Device is in tag mode
    Tag = 1,
    /// Device is in sniffer mode
    Sniffer = 2,
}

#[derive(Default, Debug, Clone, Copy)]
#[binrw]
pub struct NetworkTopology {
    pub anchor_addrs: [u16; 8],
    pub tag_addrs: [u16; 3],
}

/// Implementation of custom formatter for NetworkTopology
impl Format for NetworkTopology {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "anchor_addrs: [");
        let (last, other) = self.anchor_addrs.split_last().unwrap();
        for addr in other {
            defmt::write!(f, "{:#x}, ", addr);
        }
        defmt::write!(f, "{:#x}", last);
        defmt::write!(f, "], tag_addrs: [");
        let (last, other) = self.tag_addrs.split_last().unwrap();
        for addr in other {
            defmt::write!(f, "{:#x}, ", addr);
        }
        defmt::write!(f, "{:#x}", last);
        defmt::write!(f, "]");
    }
}

/// Config struct saved to flash
#[binrw]
#[brw(magic = b"MAGL", little)]
#[derive(Default, Debug, Format, Clone, Copy)]
pub struct MagicLocConfig {
    pub uwb_addr: u16,
    pub uwb_pan_id: u16,
    pub mode: Mode,
    pub network_topology: NetworkTopology,
}

impl MagicLocConfig {
    pub const MAX_SIZE: usize = 128;
}

/// Power-on configuration loader
pub async fn load_config() -> Option<MagicLocConfig> {
    let mut storage = FlashStorage::new();
    defmt::info!("Flash size = {}", storage.capacity());

    let mut buf = [0u8; MagicLocConfig::MAX_SIZE];

    storage
        .read(unsafe { STORAGE_OFFSET }, &mut buf)
        .map_err(|_| ())
        .and_then(|_| -> Result<MagicLocConfig, ()> {
            let config = MagicLocConfig::read(&mut Cursor::new(&buf)).map_err(|_| ());

            if config.is_err() {
                defmt::error!(
                    "Failed, reason {:?}",
                    config.expect_err("Failed to parse config")
                );
                defmt::error!("Buffer = {:x}", &buf);
                return Err(());
            }

            config.map_err(|_| ())
        })
        .ok()
}

#[allow(dead_code)]
pub async fn write_config(config: &MagicLocConfig) -> Result<(), ()> {
    let mut storage = FlashStorage::new();

    // save to flash
    let mut buf = [0u8; MagicLocConfig::MAX_SIZE];
    MagicLocConfig::write(config, &mut Cursor::new(buf.as_mut_slice())).map_err(|_| ())?;

    storage
        .write(unsafe { STORAGE_OFFSET }, &buf)
        .map_err(|_| ())?;

    defmt::info!("Saved config!");

    return Ok(());
}
