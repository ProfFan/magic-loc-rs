use core::{fmt, marker::PhantomData};

// #[cfg(feature = "blocking")]
use embedded_hal::blocking::spi;
#[cfg(feature = "async")]
use embedded_hal_async::spi as async_spi;

pub struct LSM6DSO<SPI> {
    bus: SPI,
}

impl<SPI> LSM6DSO<SPI> {
    /// Create a new instance of `LSM6DSO`
    ///
    /// Requires the SPI peripheral and the chip select pin that are connected
    /// to the LSM6DSO.
    pub fn new(spi: SPI) -> Self {
        LSM6DSO { bus: spi }
    }

    /// Direct access to the SPI bus
    pub fn spi_bus(&mut self) -> &mut SPI {
        &mut self.bus
    }
}

/// Provides access to a register
///
/// You can get an instance for a given register using one of the methods on
/// [`LSM6DSO`].
pub struct RegAccessor<'s, R, SPI>(&'s mut LSM6DSO<SPI>, PhantomData<R>);

impl<'s, R, SPI> RegAccessor<'s, R, SPI>
where
    SPI: spi::Transfer<u8> + spi::Write<u8>,
{
    /// Read from the register
    pub fn read(&mut self) -> Result<R::Read, Error<SPI>>
    where
        R: Register + Readable,
    {
        let mut r = R::read();
        let buffer = R::buffer(&mut r);

        init_header::<R>(false, buffer);
        self.0.bus.transfer(buffer).map_err(Error::Transfer)?;

        Ok(r)
    }

    /// Write to the register
    pub fn write<F>(&mut self, f: F) -> Result<(), Error<SPI>>
    where
        R: Register + Writable,
        F: FnOnce(&mut R::Write) -> &mut R::Write,
    {
        let mut w = R::write();
        f(&mut w);

        let buffer = R::buffer(&mut w);
        init_header::<R>(true, buffer);

        <SPI as spi::Write<u8>>::write(&mut self.0.bus, buffer).map_err(Error::Write)?;

        Ok(())
    }

    /// Modify the register
    pub fn modify<F>(&mut self, f: F) -> Result<(), Error<SPI>>
    where
        R: Register + Readable + Writable,
        F: for<'r> FnOnce(&mut R::Read, &'r mut R::Write) -> &'r mut R::Write,
    {
        let mut r = self.read()?;
        let mut w = R::write();

        <R as Writable>::buffer(&mut w).copy_from_slice(<R as Readable>::buffer(&mut r));

        f(&mut r, &mut w);

        let buffer = <R as Writable>::buffer(&mut w);
        init_header::<R>(true, buffer);

        <SPI as spi::Write<u8>>::write(&mut self.0.bus, buffer).map_err(Error::Write)?;

        Ok(())
    }
}

/// Provide async access to a register
#[cfg(feature = "async")]
impl<'s, R, SPI> RegAccessor<'s, R, SPI>
where
    SPI: spi::Transfer<u8>,
    SPI: spi::Write<u8, Error = <SPI as spi::Transfer<u8>>::Error>,
    SPI: async_spi::SpiBus<Error = <SPI as spi::Transfer<u8>>::Error>,
{
    /// Read from the register
    pub async fn async_read(&mut self) -> Result<R::Read, Error<SPI>>
    where
        R: Register + Readable,
        [(); R::LEN]: Sized,
    {
        let mut r = R::read();
        let mut buffer = R::buffer(&mut r);
        let mut write_buffer: [u8; R::LEN] = [0u8; R::LEN];

        init_header::<R>(false, &mut write_buffer);
        async_spi::SpiBus::transfer(&mut self.0.bus, &mut buffer, &write_buffer)
            .await
            .map_err(|e| Error::Transfer(e))?;

        defmt::debug!("Read: {:?}", buffer);

        Ok(r)
    }

    /// Write to the register
    pub async fn async_write<F>(&mut self, f: F) -> Result<(), Error<SPI>>
    where
        R: Register + Writable,
        F: FnOnce(&mut R::Write) -> &mut R::Write,
    {
        let mut w = R::write();
        f(&mut w);

        let buffer = R::buffer(&mut w);

        let header_size = init_header::<R>(true, buffer);

        async_spi::SpiBus::write(&mut self.0.bus, &buffer[header_size..])
            .await
            .map_err(|e| Error::Write(e))?;

        Ok(())
    }

    /// Modify the register
    pub async fn async_modify<F>(&mut self, f: F) -> Result<(), Error<SPI>>
    where
        R: Register + Readable + Writable,
        F: for<'r> FnOnce(&mut R::Read, &'r mut R::Write) -> &'r mut R::Write,
        [(); R::LEN]: Sized,
    {
        let mut r = self.async_read().await?;
        let mut w = R::write();

        <R as Writable>::buffer(&mut w).copy_from_slice(<R as Readable>::buffer(&mut r));

        f(&mut r, &mut w);

        let buffer = <R as Writable>::buffer(&mut w);
        let header_size = init_header::<R>(true, buffer);

        async_spi::SpiBus::write(&mut self.0.bus, &buffer[header_size..])
            .await
            .map_err(Error::Write)?;

        Ok(())
    }
}

/// An SPI error that can occur when communicating with the LSM6DSO
pub enum Error<SPI>
where
    SPI: spi::Transfer<u8> + spi::Write<u8>,
{
    /// SPI error occured during a transfer transaction
    Transfer(<SPI as spi::Transfer<u8>>::Error),

    /// SPI error occured during a write transaction
    Write(<SPI as spi::Write<u8>>::Error),
}

// We can't derive this implementation, as the compiler will complain that the
// associated error type doesn't implement `Debug`.
impl<SPI> fmt::Debug for Error<SPI>
where
    SPI: spi::Transfer<u8> + spi::Write<u8>,
    <SPI as spi::Transfer<u8>>::Error: fmt::Debug,
    <SPI as spi::Write<u8>>::Error: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Error::Transfer(error) => write!(f, "Transfer({:?})", error),
            Error::Write(error) => write!(f, "Write({:?})", error),
        }
    }
}

/// Initializes the SPI message header
///
/// Initializes the SPI message header for accessing a given register, writing
/// the header directly into the provided buffer. Returns the length of the
/// header that was written.
fn init_header<R: Register>(write: bool, buffer: &mut [u8]) -> usize {
    buffer[0] = (((!write as u8) << 7) & 0x80) | R::ID;

    1
}

/// Implemented for all registers
///
/// This is a mostly internal crate that should not be implemented or used
/// directly by users of this crate. It is exposed through the public API
/// though, so it can't be made private.
///
/// The LSM6DSO user manual, section 7.1, specifies what the values of the
/// constant should be for each register.
pub trait Register {
    /// The register index
    const ID: u8;

    /// The lenght of the register
    const LEN: usize;
}

/// Marker trait for registers that can be read from
///
/// This is a mostly internal crate that should not be implemented or used
/// directly by users of this crate. It is exposed through the public API
/// though, so it can't be made private.
pub trait Readable {
    /// The type that is used to read from the register
    type Read;

    /// Return the read type for this register
    fn read() -> Self::Read;

    /// Return the read type's internal buffer
    fn buffer(r: &mut Self::Read) -> &mut [u8];
}

/// Marker trait for registers that can be written to
///
/// This is a mostly internal crate that should not be implemented or used
/// directly by users of this crate. It is exposed through the public API
/// though, so it can't be made private.
pub trait Writable {
    /// The type that is used to write to the register
    type Write;

    /// Return the write type for this register
    fn write() -> Self::Write;

    /// Return the write type's internal buffer
    fn buffer(w: &mut Self::Write) -> &mut [u8];
}

/// Generates register implementations
macro_rules! impl_register {
    (
        $(
            $id:expr,
            $len:expr,
            $rw:tt,
            $name:ident($name_lower:ident) {
            #[$doc:meta]
            $(
                $field:ident,
                $first_bit:expr,
                $last_bit:expr,
                $ty:ty;
                #[$field_doc:meta]
            )*
            }
        )*
    ) => {
        $(
            #[$doc]
            #[allow(non_camel_case_types)]
            pub struct $name;

            impl Register for $name {
                const ID:     u8    = $id;
                const LEN:    usize = $len;
            }

            impl $name {
                const HEADER_LEN: usize = 1; // Always one byte for the header
            }

            #[$doc]
            pub mod $name_lower {
                use core::fmt;


                const HEADER_LEN: usize = super::$name::HEADER_LEN;


                /// Used to read from the register
                pub struct R(pub(crate) [u8; HEADER_LEN + $len]);

                impl R {
                    $(
                        #[$field_doc]
                        pub fn $field(&self) -> $ty {
                            use core::mem::size_of;
                            use crate::ll::FromBytes;

                            // The index (in the register data) of the first
                            // byte that contains a part of this field.
                            const START: usize = $first_bit / 8;

                            // The index (in the register data) of the byte
                            // after the last byte that contains a part of this
                            // field.
                            const END: usize = $last_bit  / 8 + 1;

                            // The number of bytes in the register data that
                            // contain part of this field.
                            const LEN: usize = END - START;

                            // Get all bytes that contain our field. The field
                            // might fill out these bytes completely, or only
                            // some bits in them.
                            let mut bytes = [0; LEN];
                            bytes[..LEN].copy_from_slice(
                                &self.0[START+HEADER_LEN .. END+HEADER_LEN]
                            );

                            // Before we can convert the field into a number and
                            // return it, we need to shift it, to make sure
                            // there are no other bits to the right of it. Let's
                            // start by determining the offset of the field
                            // within a byte.
                            const OFFSET_IN_BYTE: usize = $first_bit % 8;

                            if OFFSET_IN_BYTE > 0 {
                                // Shift the first byte. We always have at least
                                // one byte here, so this always works.
                                bytes[0] >>= OFFSET_IN_BYTE;

                                // If there are more bytes, let's shift those
                                // too.
                                // We need to allow exceeding bitshifts in this
                                // loop, as we run into that if `OFFSET_IN_BYTE`
                                // equals `0`. Please note that we never
                                // actually encounter that at runtime, due to
                                // the if condition above.
                                let mut i = 1;
                                #[allow(arithmetic_overflow)]
                                while i < LEN {
                                    bytes[i - 1] |=
                                        bytes[i] << 8 - OFFSET_IN_BYTE;
                                    bytes[i] >>= OFFSET_IN_BYTE;
                                    i += 1;
                                }
                            }

                            // If the field didn't completely fill out its last
                            // byte, we might have bits from unrelated fields
                            // there. Let's erase those before doing the final
                            // conversion into the field's data type.
                            const SIZE_IN_BITS: usize =
                                $last_bit - $first_bit + 1;
                            const BITS_ABOVE_FIELD: usize =
                                8 - (SIZE_IN_BITS % 8);
                            const SIZE_IN_BYTES: usize =
                                (SIZE_IN_BITS - 1) / 8 + 1;
                            const LAST_INDEX: usize =
                                SIZE_IN_BYTES - 1;
                            if BITS_ABOVE_FIELD < 8 {
                                // Need to allow exceeding bitshifts to make the
                                // compiler happy. They're never actually
                                // encountered at runtime, due to the if
                                // condition.
                                #[allow(arithmetic_overflow)]
                                {
                                    bytes[LAST_INDEX] <<= BITS_ABOVE_FIELD;
                                    bytes[LAST_INDEX] >>= BITS_ABOVE_FIELD;
                                }
                            }

                            // Now all that's left is to convert the bytes into
                            // the field's type. Please note that methods for
                            // converting numbers to/from bytes are coming to
                            // stable Rust, so we might be able to remove our
                            // custom infrastructure here. Tracking issue:
                            // https://github.com/rust-lang/rust/issues/52963
                            let bytes = if bytes.len() > size_of::<$ty>() {
                                &bytes[..size_of::<$ty>()]
                            }
                            else {
                                &bytes
                            };
                            <$ty as FromBytes>::from_bytes(bytes)
                        }
                    )*
                }

                impl fmt::Debug for R {
                    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
                        write!(f, "0x")?;
                        for i in (0 .. $len).rev() {
                            write!(f, "{:02x}", self.0[HEADER_LEN + i])?;
                        }

                        Ok(())
                    }
                }


                /// Used to write to the register
                pub struct W(pub(crate) [u8; HEADER_LEN + $len]);

                impl W {
                    $(
                        #[$field_doc]
                        pub fn $field(&mut self, value: $ty) -> &mut Self {
                            use crate::ll::ToBytes;

                            // Convert value into bytes
                            let source = <$ty as ToBytes>::to_bytes(value);

                            // Now, let's figure out where the bytes are located
                            // within the register array.
                            const START:          usize = $first_bit / 8;
                            const END:            usize = $last_bit  / 8 + 1;
                            const OFFSET_IN_BYTE: usize = $first_bit % 8;

                            // Also figure out the length of the value in bits.
                            // That's going to come in handy.
                            const LEN: usize = $last_bit - $first_bit + 1;


                            // We need to track how many bits are left in the
                            // value overall, and in the value's current byte.
                            let mut bits_left         = LEN;
                            let mut bits_left_in_byte = 8;

                            // We also need to track how many bits have already
                            // been written to the current target byte.
                            let mut bits_written_to_byte = 0;

                            // Now we can take the bytes from the value, shift
                            // them, mask them, and write them into the target
                            // array.
                            let mut source_i  = 0;
                            let mut target_i  = START;
                            while target_i < END {
                                // Values don't always end at byte boundaries,
                                // so we need to mask the bytes when writing to
                                // the slice.
                                // Let's start out assuming we can write to the
                                // whole byte of the slice. This will be true
                                // for the middle bytes of our value.
                                let mut mask = 0xff;

                                // Let's keep track of the offset we're using to
                                // write to this byte. We're going to need it.
                                let mut offset_in_this_byte = 0;

                                // If this is the first byte we're writing to
                                // the slice, we need to remove the lower bits
                                // of the mask.
                                if target_i == START {
                                    mask <<= OFFSET_IN_BYTE;
                                    offset_in_this_byte = OFFSET_IN_BYTE;
                                }

                                // If this is the last byte we're writing to the
                                // slice, we need to remove the higher bits of
                                // the mask. Please note that we could be
                                // writing to _both_ the first and the last
                                // byte.
                                if target_i == END - 1 {
                                    let shift =
                                        8 - bits_left - offset_in_this_byte;
                                    mask <<= shift;
                                    mask >>= shift;
                                }

                                mask <<= bits_written_to_byte;

                                // Read the value from `source`
                                let value = source[source_i]
                                    >> 8 - bits_left_in_byte
                                    << offset_in_this_byte
                                    << bits_written_to_byte;

                                // Zero the target bits in the slice, then write
                                // the value.
                                self.0[HEADER_LEN + target_i] &= !mask;
                                self.0[HEADER_LEN + target_i] |= value & mask;

                                // The number of bits that were expected to be
                                // written to the target byte.
                                let bits_needed = mask.count_ones() as usize;

                                // The number of bits we actually wrote to the
                                // target byte.
                                let bits_used = bits_needed.min(
                                    bits_left_in_byte - offset_in_this_byte
                                );

                                bits_left -= bits_used;
                                bits_written_to_byte += bits_used;

                                // Did we use up all the bits in the source
                                // byte? If so, we can move on to the next one.
                                if bits_left_in_byte > bits_used {
                                    bits_left_in_byte -= bits_used;
                                }
                                else {
                                    bits_left_in_byte =
                                        8 - (bits_used - bits_left_in_byte);

                                    source_i += 1;
                                }

                                // Did we write all the bits in the target byte?
                                // If so, we can move on to the next one.
                                if bits_used == bits_needed {
                                    target_i += 1;
                                    bits_written_to_byte = 0;
                                }
                            }

                            self
                        }
                    )*
                }
            }

            impl_rw!($rw, $name, $name_lower, $len);
        )*


        impl<SPI> LSM6DSO<SPI> {
            $(
                #[$doc]
                pub fn $name_lower(&mut self) -> RegAccessor<$name, SPI> {
                    RegAccessor(self, PhantomData)
                }
            )*
        }
    }
}

// Helper macro, used internally by `impl_register!`
macro_rules! impl_rw {
    (RO, $name:ident, $name_lower:ident, $len:expr) => {
        impl_rw!(@R, $name, $name_lower, $len);
    };
    (RW, $name:ident, $name_lower:ident, $len:expr) => {
        impl_rw!(@R, $name, $name_lower, $len);
        impl_rw!(@W, $name, $name_lower, $len);
    };

    (@R, $name:ident, $name_lower:ident, $len:expr) => {
        impl Readable for $name {
            type Read = $name_lower::R;

            fn read() -> Self::Read {
                $name_lower::R([0; Self::HEADER_LEN + $len])
            }

            fn buffer(r: &mut Self::Read) -> &mut [u8] {
                &mut r.0
            }
        }
    };
    (@W, $name:ident, $name_lower:ident, $len:expr) => {
        impl Writable for $name {
            type Write = $name_lower::W;

            fn write() -> Self::Write {
                $name_lower::W([0; Self::HEADER_LEN + $len])
            }

            fn buffer(w: &mut Self::Write) -> &mut [u8] {
                &mut w.0
            }
        }
    };
}

impl_register! {

    0x01, 1, RW, FUNC_CFG_ACCESS(func_cfg_access) { /// Embedded functions configuration register
        not_used_01, 0, 5, u8;  /// Unused (0)
        shub_reg_access, 6, 6, u8;  /// Enable access to external sensors via I²C master interface (default 0, disabled)
        func_cfg_en,     7, 7, u8;  /// Enable embedded functions register bank (default 0, disabled)
    }
    0x0D, 1, RW, INT1_CTRL(int1_ctrl) { /// INT1 pin control register
        int1_drdy_xl, 0, 0, u8;  /// Accelerometer data ready on INT1 (default 0, disabled)
        int1_drdy_g,  1, 1, u8;  /// Gyroscope data ready on INT1 (default 0, disabled)
        int1_boot,    2, 2, u8;  /// Boot status available on INT1 (default 0, disabled)
        fifo_th,      3, 3, u8;  /// FIFO threshold interrupt on INT1 (default 0, disabled)
        fifo_ovr,     4, 4, u8;  /// FIFO overrun interrupt on INT1 (default 0, disabled)
        fifo_full,    5, 5, u8;  /// FIFO full interrupt on INT1 (default 0, disabled)
        int1_cnt_bdr, 6, 6, u8;  /// Latched/pulsed interrupt (default 0, pulsed)
        den_drdy_flag,7, 7, u8;  /// DEN data ready interrupt on INT1 (default 0, disabled)
    }
    0x0F, 1, RO, WHO_AM_I(who_am_i) { /// Device identifier
        value,     0,  7, u8;  /// Value
    }
    0x10, 1, RW, CTRL1_XL(ctrl1_xl) { /// Accelerometer control register 1
        not_used_01, 0, 0, u8;  /// Unused (0)
        lpf1_bw_sel, 1, 1, u8;  /// LPF1 bandwidth selection (default 0, ODR/2)
        fs_xl,      2, 3, u8;  /// Accelerometer full-scale selection (default 0, ±2g)
        odr_xl,     4, 7, u8;  /// Accelerometer output data rate and power mode selection (default 0, power down)
    }
    0x11, 1, RW, CTRL2_G(ctrl2_g) { /// Gyroscope control register 2
        fs_g,       0, 3, u8;  /// Gyroscope full-scale selection (default 0, ±250dps)
        odr_g,      4, 7, u8;  /// Gyroscope output data rate selection (default 0, power down)
    }
    0x12, 1, RW, CTRL3_C(ctrl3_c) { /// Extended Unique Identifier
        sw_reset,    0, 0, u8;  /// Software reset
        not_used_01, 1, 1, u8;  /// Unused (0)
        if_inc,      2, 2, u8;  /// Register address auto increment (default 1)
        sim,         3, 3, u8;  /// SPI Serial Interface Mode (default 0, 4-wire)
        pp_od,       4, 4, u8;  /// Push-pull/open-drain selection on INT1 and INT2 pins
        h_lactive,   5, 5, u8;  /// Interrupt activation level (default 0, active high)
        bdu,         6, 6, u8;  /// Block data update (default 0, continuous update)
        boot,        7, 7, u8;  /// Reboot memory content (default 0, normal mode)
    }
    0x13, 1, RW, CTRL4_C(ctrl4_c) { /// Control register 4
        not_used_01, 0, 0, u8;  /// Unused (0)
        lpf1_sel_g,  1, 1, u8;  /// LPF1 bandwidth selection (default 0, ODR/2)
        i2c_disable, 2, 2, u8;  /// Disable I2C interface (default 0, I2C enabled)
        drdy_mask,   3, 3, u8;  /// Data-ready pulsed/level sensitive (default 0, pulsed)
        not_used_02, 4, 4, u8;  /// Unused (0)
        int2_on_int1,5, 5, u8;  /// INT2 on INT1 (default 0, INT2 disabled)
        sleep_g,     6, 6, u8;  /// Gyroscope sleep mode enable (default 0, disabled)
        not_used_03, 7, 7, u8;  /// Unused (0)
    }
    0x20, 14, RO, ALL_READOUTS(all_readouts) { /// All readouts
        out_temp,  0, 15, u16;  /// Temperature output register
        outx_g,   16, 31, u16;  /// Gyroscope X-axis angular rate output register
        outy_g,   32, 47, u16;  /// Gyroscope Y-axis angular rate output register
        outz_g,   48, 63, u16;  /// Gyroscope Z-axis angular rate output register
        outx_a,   64, 79, u16;  /// Accelerometer X-axis output register
        outy_a,   80, 95, u16;  /// Accelerometer Y-axis output register
        outz_a,  96, 111, u16;  /// Accelerometer Z-axis output register
    }
    0x20, 2, RO, OUT_TEMP_L(out_temp) { /// Temperature output register
        value, 0, 15, u16;  /// Value
    }
    0x22, 2, RO, OUTX_G(outx_g) { /// Gyroscope X-axis angular rate output register
        value, 0, 15, u16;  /// Value
    }
    0x24, 2, RO, OUTY_G(outy_g) { /// Gyroscope Y-axis angular rate output register
        value, 0, 15, u16;  /// Value
    }
    0x26, 2, RO, OUTZ_G(outz_g) { /// Gyroscope Z-axis angular rate output register
        value, 0, 15, u16;  /// Value
    }
    0x28, 2, RO, OUTX_A(outx_a) { /// Accelerometer X-axis output register
        value, 0, 15, u16;  /// Value
    }
    0x2A, 2, RO, OUTY_A(outy_a) { /// Accelerometer Y-axis output register
        value, 0, 15, u16;  /// Value
    }
    0x2C, 2, RO, OUTZ_A(outz_a) { /// Accelerometer Z-axis output register
        value, 0, 15, u16;  /// Value
    }
    0x63, 1, RW, FREQ_FINE(freq_fine) { /// Fine frequency control register
        value, 0, 7, u8;  /// Fine frequency control
    }
}

/// Internal trait used by `impl_registers!`
trait FromBytes {
    fn from_bytes(bytes: &[u8]) -> Self;
}

/// Internal trait used by `impl_registers!`
trait ToBytes {
    type Bytes;

    fn to_bytes(self) -> Self::Bytes;
}

/// Internal macro used to implement `FromBytes`/`ToBytes`
macro_rules! impl_bytes {
    ($($ty:ty,)*) => {
        $(
            impl FromBytes for $ty {
                fn from_bytes(bytes: &[u8]) -> Self {
                    let mut val = 0;

                    for (i, &b) in bytes.iter().enumerate() {
                        val |= (b as $ty) << (i * 8);
                    }

                    val
                }
            }

            impl ToBytes for $ty {
                type Bytes = [u8; ::core::mem::size_of::<$ty>()];

                fn to_bytes(self) -> Self::Bytes {
                    let mut bytes = [0; ::core::mem::size_of::<$ty>()];

                    for (i, b) in bytes.iter_mut().enumerate() {
                        let shift = 8 * i;
                        let mask  = 0xff << shift;

                        *b = ((self & mask) >> shift) as u8;
                    }

                    bytes
                }
            }
        )*
    }
}

impl_bytes! {
    u8,
    u16,
    u32,
    u64,
    u128,
}
