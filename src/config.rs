use serde::{Deserialize, Serialize};

/// enum for device operation mode
#[derive(Default, Debug, Serialize, Deserialize)]
pub enum Mode {
    /// Device is in anchor mode
    #[default]
    Anchor,
    /// Device is in tag mode
    Tag,
}

/// Config struct saved to flash
#[derive(Default, Debug, Serialize, Deserialize)]
pub struct MagicLocConfig {
    pub uwb_addr: u16,
    pub uwb_pan_id: u16,
    pub mode: Mode,
}
