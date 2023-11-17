use binrw::*;
use defmt::Format;

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

/// Config struct saved to flash
#[binrw]
#[brw(magic = b"MAGL", little)]
#[derive(Default, Debug, Format, Clone, Copy)]
pub struct MagicLocConfig {
    pub uwb_addr: u16,
    pub uwb_pan_id: u16,
    pub mode: Mode,
}

impl MagicLocConfig {
    pub const MAX_SIZE: usize = 128;
}
