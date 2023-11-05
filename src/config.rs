use serde::{Deserialize, Serialize};

/// enum for device operation mode
#[derive(Default, Debug, Serialize, Deserialize, PartialEq)]
#[repr(u8)]
pub enum Mode {
    /// Device is in anchor mode
    #[default]
    Anchor = 0,
    /// Device is in tag mode
    Tag = 1,
}

/// Config struct saved to flash
#[derive(Default, Debug, Serialize, Deserialize)]
pub struct MagicLocConfig {
    pub uwb_addr: u16,
    pub uwb_pan_id: u16,
    pub mode: Mode,
}
