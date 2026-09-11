// SPDX-License-Identifier: MPL-2.0

//! Primitives for reading an AML byte stream.
//!
//! This module implements the container-level encodings of AML — package
//! lengths and name strings — together with a bounds-checked cursor over the
//! stream.
//!
//! Reference: <https://download.intel.com/download/idptools/61122v004.pdf>,
//! section 19.2 ("ACPI Software Programming Model"), in particular the
//! package length encoding (section 19.2.4) and the name string encoding
//! (section 19.2.2).

use alloc::{string::String, vec::Vec};

/// The parse errors of a static AML walk.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AmlError {
    /// The stream ended before the requested bytes could be read.
    UnexpectedEnd,
    /// A package length encoded more length bytes than the format allows.
    InvalidPkgLength,
    /// A name string used a character that is not part of the namespace
    /// alphabet, or a segment count that exceeds the allowed maximum.
    InvalidNameString,
}

/// The opcodes and prefixes that the static walk needs to recognize.
///
/// Reference: <https://download.intel.com/download/idptools/61122v004.pdf>,
/// section 20 ("ACPI Machine Language (AML) Specification").
pub(crate) mod opcode {
    /// `NullName` and `One`/`Ones` style single-byte data opcodes share the
    /// range below the `ByteConstPrefix`; each of the constants here occupies
    /// exactly one byte with no operands.
    pub const ZERO: u8 = 0x00;
    /// Prefix of an integer constant whose width is given by the opcode.
    pub const BYTE_CONST_PREFIX: u8 = 0x0a;
    /// `StringOp`: a null-terminated ASCII string follows.
    pub const STRING_PREFIX: u8 = 0x0d;
    /// `BufferOp`: a buffer with a package-wrapped size term follows.
    pub const BUFFER: u8 = 0x11;
    /// `ScopeOp`: a package-wrapped term list with a name path follows.
    pub const SCOPE: u8 = 0x10;
    /// `NameOp`: declares a namespace object with its initial value.
    pub const NAME: u8 = 0x08;
    /// `ReturnOp`: returns the evaluation of its argument.
    pub const RETURN: u8 = 0xa4;
    /// The extended opcode prefix shared by all two-byte opcodes.
    pub const EXT_PREFIX: u8 = 0x5b;
    /// `DeviceOp`: a package-wrapped term list describing a device.
    pub const EXT_DEVICE: u8 = 0x82;
    /// `MethodOp`: a package-wrapped named method with a flags byte.
    pub const EXT_METHOD: u8 = 0x14;
    /// `FieldOp`: a package-wrapped field list within an operation region.
    pub const EXT_FIELD: u8 = 0x81;
    /// `IndexFieldOp`.
    pub const EXT_INDEX_FIELD: u8 = 0x86;
    /// `BankFieldOp`.
    pub const EXT_BANK_FIELD: u8 = 0x87;
    /// `DataRegionOp`.
    pub const EXT_DATA_REGION: u8 = 0x88;
    /// `OperationRegionOp`: fixed-width operands follow the name.
    pub const EXT_OPERATION_REGION: u8 = 0x80;
    /// `PowerResOp`.
    pub const EXT_POWER_RES: u8 = 0x84;
    /// `ThermalZoneOp`.
    pub const EXT_THERMAL_ZONE: u8 = 0x85;
    /// `ProcessorOp`.
    pub const EXT_PROCESSOR: u8 = 0x83;
    /// `MutexOp`.
    pub const EXT_MUTEX: u8 = 0x01;
    /// `EventOp`.
    pub const EXT_EVENT: u8 = 0x02;
}

/// A bounds-checked cursor over an AML byte stream.
pub struct AmlStream<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> AmlStream<'a> {
    /// Creates a cursor over the given AML bytes.
    pub fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }

    /// Returns the current position in the stream.
    pub fn pos(&self) -> usize {
        self.pos
    }

    /// Returns the absolute end position of the stream.
    pub(crate) fn end(&self) -> usize {
        self.data.len()
    }

    /// Returns whether the whole stream has been consumed.
    pub fn is_empty(&self) -> bool {
        self.pos >= self.data.len()
    }

    /// Returns the next byte without consuming it, if any.
    pub fn peek(&self) -> Option<u8> {
        self.data.get(self.pos).copied()
    }

    /// Reads one byte.
    pub fn read_u8(&mut self) -> Result<u8, AmlError> {
        let byte = self
            .data
            .get(self.pos)
            .copied()
            .ok_or(AmlError::UnexpectedEnd)?;
        self.pos += 1;
        Ok(byte)
    }

    /// Reads a little-endian multi-byte integer of the given width.
    pub fn read_int(&mut self, width: usize) -> Result<u64, AmlError> {
        let mut value: u64 = 0;
        for i in 0..width {
            let byte = self.read_u8()?;
            value |= (byte as u64) << (i * 8);
        }
        Ok(value)
    }

    /// Reads the given number of bytes.
    pub(crate) fn read_bytes(&mut self, len: usize) -> Result<Vec<u8>, AmlError> {
        if self.remaining() < len {
            return Err(AmlError::UnexpectedEnd);
        }
        let bytes = self.data[self.pos..self.pos + len].to_vec();
        self.pos += len;
        Ok(bytes)
    }

    /// Overwrites the position, e.g. to jump to the end of a package whose
    /// length was parsed up front.
    pub(crate) fn set_pos(&mut self, pos: usize) {
        self.pos = pos;
    }

    /// Returns the number of unconsumed bytes.
    pub(crate) fn remaining(&self) -> usize {
        self.data.len().saturating_sub(self.pos)
    }

    /// Reads an AML package length, which encodes its own byte width.
    ///
    /// The two high bits of the lead byte select how many bytes (0 to 3,
    /// little-endian) carry the length. With no extra bytes the length is
    /// the low six bits of the lead byte; otherwise the low nibble of the
    /// lead byte joins the extra bytes at bit 4, 12, and 20. The length
    /// covers the package length bytes and everything that follows, but not
    /// the opcode byte.
    ///
    /// Reference:
    /// <https://elixir.bootlin.com/linux/v6.16/source/drivers/acpi/acpica/psargs.c>
    /// (`acpi_ps_get_next_package_length`).
    pub fn read_pkg_length(&mut self) -> Result<u32, AmlError> {
        let lead = self.read_u8()?;
        let extra = (lead >> 6) as usize;
        let mut length = if extra == 0 {
            (lead & 0x3f) as u32
        } else {
            (lead & 0x0f) as u32
        };
        for i in 0..extra {
            let byte = self.read_u8()? as u32;
            length |= byte << (4 + i * 8);
        }
        Ok(length)
    }

    /// Reads a name string: an optional root/prefix part followed by one or
    /// more four-character name segments.
    pub fn read_name_string(&mut self) -> Result<String, AmlError> {
        let mut path = String::new();

        loop {
            match self.peek() {
                Some(b'\\') => {
                    path.push('\\');
                    self.read_u8()?;
                }
                Some(b'^') => {
                    path.push('^');
                    self.read_u8()?;
                }
                _ => break,
            }
        }

        match self.peek() {
            // `DualNameOp`: exactly two segments follow.
            Some(0x2e) => {
                self.read_u8()?;
                path.push_str(&self.read_name_seg()?);
                path.push('.');
                path.push_str(&self.read_name_seg()?);
            }
            // `MultiNameOp`: a segment count byte, then that many segments.
            Some(0x2f) => {
                self.read_u8()?;
                let count = self.read_u8()?;
                if count == 0 {
                    return Err(AmlError::InvalidNameString);
                }
                for i in 0..count {
                    if i > 0 {
                        path.push('.');
                    }
                    path.push_str(&self.read_name_seg()?);
                }
            }
            _ => {
                path.push_str(&self.read_name_seg()?);
            }
        }

        Ok(path)
    }

    /// Reads a single four-character name segment. Trailing NUL padding is
    /// replaced with underscores, as the namespace alphabet prescribes.
    fn read_name_seg(&mut self) -> Result<String, AmlError> {
        let mut seg = String::with_capacity(4);
        for _ in 0..4 {
            let byte = self.read_u8()?;
            if !(byte.is_ascii_alphanumeric() || byte == b'_' || byte == 0) {
                return Err(AmlError::InvalidNameString);
            }
            seg.push(if byte == 0 { '_' } else { byte as char });
        }
        Ok(seg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pkg_length_short_form() {
        let mut stream = AmlStream::new(&[0x2b]);
        // With no extra length bytes, all six low bits are the length.
        assert_eq!(stream.read_pkg_length().unwrap(), 0x2b);
        assert_eq!(stream.pos(), 1);
    }

    #[test]
    fn pkg_length_one_extra_byte() {
        let mut stream = AmlStream::new(&[0x45, 0x04]);
        assert_eq!(stream.read_pkg_length().unwrap(), 0x45);
        assert_eq!(stream.pos(), 2);
    }

    #[test]
    fn pkg_length_three_extra_bytes() {
        let mut stream = AmlStream::new(&[0xff, 0x44, 0x33, 0x22]);
        assert_eq!(stream.read_pkg_length().unwrap(), 0x223344f);
    }

    #[test]
    fn name_string_simple_seg() {
        let mut stream = AmlStream::new(b"I2CA");
        assert_eq!(stream.read_name_string().unwrap(), "I2CA");
    }

    #[test]
    fn name_string_nul_padding_becomes_underscores() {
        let mut stream = AmlStream::new(b"TPD\0");
        assert_eq!(stream.read_name_string().unwrap(), "TPD_");
    }

    #[test]
    fn name_string_rooted_multi_name() {
        // `\\_SB.I2CA` as a multi-name string: root char, multi-name op,
        // two segments.
        let mut stream = AmlStream::new(&[
            0x5c, 0x2f, 0x02, 0x5f, b'S', b'B', 0x5f, b'I', b'2', b'C', b'A',
        ]);
        assert_eq!(stream.read_name_string().unwrap(), "\\_SB_.I2CA");
    }

    #[test]
    fn name_string_rejects_bad_characters() {
        let mut stream = AmlStream::new(b"I2\x01A");
        assert_eq!(
            stream.read_name_string().unwrap_err(),
            AmlError::InvalidNameString
        );
    }

    #[test]
    fn read_int_little_endian() {
        let mut stream = AmlStream::new(&[0x00, 0x20, 0xdc, 0xfe]);
        assert_eq!(stream.read_int(4).unwrap(), 0xfedc2000);
    }
}
