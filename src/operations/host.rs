// Communication with the host computer

use binrw::{io::Cursor, *};

use defmt::Format;

#[derive(Default, Debug, Format, PartialEq, Clone, Copy)]
#[binrw]
#[brw(magic = b"RNG", little)]
pub struct RangeReport {
    pub tag_addr: u16,
    pub system_ts: u64,
    pub seq_num: u8,
    pub trigger_txts: u64,
    pub ranges: [f64; 8],
}

#[derive(Default, Debug, Format, PartialEq, Clone, Copy)]
#[binrw]
#[brw(magic = b"IMU", little)]
pub struct ImuReport {
    pub tag_addr: u16,
    pub system_ts: u64,
    pub accel: [u32; 3],
    pub gyro: [u32; 3],
}
