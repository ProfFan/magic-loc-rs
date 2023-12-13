// Communication with the host computer

use binrw::{io::Cursor, *};

use defmt::Format;

#[derive(Default, Debug, Format, PartialEq, Clone, Copy)]
#[binrw]
#[brw(magic = b"RNG", little)]
pub struct RangeReport {
    pub seq_num: u8,
    pub tag_addr: u16,
    pub trigger_txts: u64,
    pub ranges: [f64; 8],
}
