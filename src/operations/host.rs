// Communication with the host computer
use binrw::*;

use defmt::Format;
use zerocopy_derive::{AsBytes, FromBytes, FromZeroes};

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

#[derive(Default, Debug, PartialEq, Clone, Copy, AsBytes, FromZeroes, FromBytes)]
#[binrw]
#[repr(C)]
pub struct RawCirSample {
    pub real: [u8; 3],
    pub imag: [u8; 3],
}

/// Implementation of custom formatter for RawCirSample
///
/// real and image are 3-byte (24 bit) signed integers
impl Format for RawCirSample {
    fn format(&self, f: defmt::Formatter) {
        // Sign extend according to the most significant bit
        let real = i32::from_le_bytes([
            self.real[0],
            self.real[1],
            self.real[2],
            if self.real[2] & 0x80 == 0x80 {
                0xFF
            } else {
                0x00
            },
        ]);
        let imag = i32::from_le_bytes([
            self.imag[0],
            self.imag[1],
            self.imag[2],
            if self.imag[2] & 0x80 == 0x80 {
                0xFF
            } else {
                0x00
            },
        ]);

        defmt::write!(f, "{}+{}j", real, imag);
    }
}

// Assert size of RawCirSample
const _: [(); 6] = [(); core::mem::size_of::<RawCirSample>()];

#[derive(Debug, Format, Clone, Copy)]
#[binrw]
#[brw(magic = b"CIR", little)]
pub struct CirReport {
    pub src_addr: u16,
    pub system_ts: u64,
    pub seq_num: u8,
    pub ip_poa: u16,      // Phase of Arrival
    pub fp_index: u16,    // First Path Index
    pub start_index: u16, // Start Index of CIR
    pub cir_size: u16,
    pub cir: [RawCirSample; 16],
}
