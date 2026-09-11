// SPDX-License-Identifier: MPL-2.0

//! Decoding of ACPI resource templates.
//!
//! The `_CRS` object of a device evaluates to a buffer of chained resource
//! descriptors. This module decodes the descriptors needed to attach an I2C
//! device: the serial bus connections that carry the bus address and the
//! controller path, the fixed and qword memory ranges that carry the
//! controller's own MMIO window, and the interrupt assignments.
//!
//! Reference: <https://download.intel.com/download/idptools/61122v004.pdf>,
//! section 6.4 ("Resource Data Types for ACPI") and the wire layouts in the
//! Linux `include/acpi/acrestyp.h` / ACPICA `amlresrc.h`.

use alloc::{string::String, vec::Vec};

/// A decoded resource descriptor.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Resource {
    /// An `I2cSerialBusV2` connection: the slave address on the bus and the
    /// path of the bus controller the device hangs off.
    I2cSerialBus {
        slave_address: u16,
        controller_path: String,
    },
    /// A 32-bit fixed memory range.
    Memory32Fixed { base: u32, length: u32 },
    /// A 64-bit memory range.
    QWordMemory { base: u64, length: u64 },
    /// An extended interrupt assignment.
    ExtendedIrq { interrupt: u32 },
    /// A legacy IRQ assignment; the interrupt number is the lowest set bit
    /// of the mask.
    Irq { interrupt: u32 },
    /// A GPIO interrupt connection. The interrupt line is not consumed by
    /// polled drivers, so no details are decoded.
    GpioInt,
    /// A serial bus connection other than I2C (UART, SPI, CSI2), decoded
    /// only up to its bus type.
    SerialBus { bus_type: u8 },
    /// A descriptor this parser does not decode.
    Unknown { descriptor_type: u8 },
}

/// The large descriptor type bytes (bit 7 set) that this module decodes.
///
/// Reference:
/// <https://elixir.bootlin.com/linux/v6.16/source/drivers/acpi/acpica/aclocal.h>
/// (`ACPI_RESOURCE_NAME_*`).
mod large_type {
    /// `EndTag` (small descriptor, wire byte 0x79: type 15, length 1).
    pub const END_TAG: u8 = 0x79;
    /// `Memory32Fixed`.
    pub const FIXED_MEMORY32: u8 = 0x86;
    /// `ExtendedIRQ`.
    pub const EXTENDED_IRQ: u8 = 0x89;
    /// `Address64`: the qword-family address space descriptor.
    pub const ADDRESS64: u8 = 0x8a;
    /// `GpioInt`/`GpioIo` connection descriptor.
    pub const GPIO_CONNECTION: u8 = 0x8c;
    /// `I2cSerialBusV2`/`UartSerialBusV2`/... connection descriptor.
    pub const SERIAL_BUS_CONNECTION: u8 = 0x8e;
}

/// Decodes a chain of resource descriptors, as stored in a `_CRS` buffer.
///
/// The chain ends with an `EndTag` descriptor; decoding also stops at the
/// end of the buffer. Undecodable descriptors are returned as
/// [`Resource::Unknown`] so the caller can see the full resource inventory.
pub fn parse_resource_buffer(buf: &[u8]) -> Vec<Resource> {
    let mut resources = Vec::new();
    let mut pos = 0;
    while pos < buf.len() {
        let byte = buf[pos];
        // Large descriptors: bits 6:0 name the type and a two-byte little-
        // endian length follows the type byte. Small descriptors: bits 6:3
        // name the type and bits 2:0 give the body length. Neither length
        // includes its own header.
        let (name, header_len, body_len) = if byte & 0x80 != 0 {
            let length = match buf.get(pos + 1..pos + 3) {
                Some(bytes) => u16::from_le_bytes([bytes[0], bytes[1]]) as usize,
                None => break,
            };
            if length == 0 {
                break;
            }
            (byte, 3, length)
        } else {
            let length = (byte & 0x07) as usize;
            if length == 0 {
                break;
            }
            (byte, 1, length)
        };

        // A zero-length body (e.g. a lone EndTag) is handled per descriptor
        // below; everything else must fit in the buffer.
        let body = match buf.get(pos + header_len..pos + header_len + body_len) {
            Some(bytes) => bytes,
            None => break,
        };
        pos += header_len + body_len;

        match name {
            large_type::END_TAG => break, // EndTag terminates the chain
            0x20..=0x23 => {
                // Small IRQ descriptor: a two-byte mask followed by a flags
                // byte. The lowest set mask bit is the assigned interrupt; a
                // zero mask names no interrupt at all.
                if body.len() < 2 {
                    break;
                }
                let mask = u16::from_le_bytes([body[0], body[1]]);
                if mask != 0 {
                    resources.push(Resource::Irq {
                        interrupt: mask.trailing_zeros(),
                    });
                }
            }
            _ => {
                let resource = match name {
                    large_type::FIXED_MEMORY32 => decode_fixed_memory32(body),
                    large_type::EXTENDED_IRQ => decode_extended_irq(body),
                    large_type::ADDRESS64 => decode_address64(body),
                    large_type::GPIO_CONNECTION => decode_gpio(body),
                    large_type::SERIAL_BUS_CONNECTION => decode_serial_bus(body),
                    _ => Resource::Unknown {
                        descriptor_type: name,
                    },
                };
                resources.push(resource);
            }
        }
    }
    resources
}

/// Decodes a `Memory32Fixed` body: write status (1), base address (4),
/// length (4).
fn decode_fixed_memory32(body: &[u8]) -> Resource {
    if body.len() < 9 {
        return Resource::Unknown {
            descriptor_type: large_type::FIXED_MEMORY32,
        };
    }
    let read_u32 = |off: usize| -> u32 {
        u32::from_le_bytes([body[off], body[off + 1], body[off + 2], body[off + 3]])
    };
    Resource::Memory32Fixed {
        base: read_u32(1),
        length: read_u32(5),
    }
}

/// Decodes an `ExtendedIRQ` body: flags, interrupt count, and the interrupt
/// numbers, followed by an optional null-terminated source string.
fn decode_extended_irq(body: &[u8]) -> Resource {
    // Layout: flags (1), interrupt count (1), interrupt numbers (4 * count),
    // then the resource source string.
    let count = body.get(1).copied().unwrap_or(0) as usize;
    if count == 0 {
        return Resource::ExtendedIrq { interrupt: 0 };
    }
    let interrupt = match body.get(2..6) {
        Some(bytes) => u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
        None => 0,
    };
    Resource::ExtendedIrq { interrupt }
}

/// Decodes an `Address64` body: the address range sits at a fixed offset
/// after the type-specific attributes, granularity, and translation fields.
fn decode_address64(body: &[u8]) -> Resource {
    // Layout: resource type (1: 0 = memory range), general flags (1),
    // type-specific flags (1), type-specific attributes (1), granularity
    // (8), min address (8), max address (8), translation offset (8),
    // address length (8).
    let read_u64 = |off: usize| -> u64 {
        let mut bytes = [0u8; 8];
        for (i, byte) in bytes.iter_mut().enumerate() {
            *byte = body.get(off + i).copied().unwrap_or(0);
        }
        u64::from_le_bytes(bytes)
    };
    // Only memory ranges back an MMIO window; I/O and bus ranges are not
    // decoded.
    if body.first().copied() != Some(0) {
        return Resource::Unknown {
            descriptor_type: large_type::ADDRESS64,
        };
    }
    Resource::QWordMemory {
        base: read_u64(11),
        length: read_u64(35),
    }
}

/// Decodes a GPIO connection body. The pin and vendor details are of no
/// use to a polled driver, so only the decoded presence matters.
fn decode_gpio(_body: &[u8]) -> Resource {
    Resource::GpioInt
}

/// Decodes a serial bus connection body. Only I2C connections are decoded;
/// the layout of the common part is:
///
/// ```text
/// revision (1), source index (1), type (1), flags (1),
/// type-specific flags (2), type revision (1), type data length (2),
/// type data (type data length bytes), source string
/// ```
fn decode_serial_bus(body: &[u8]) -> Resource {
    // The common part must at least cover the type data length field.
    if body.len() < 9 {
        return Resource::Unknown {
            descriptor_type: large_type::SERIAL_BUS_CONNECTION,
        };
    }
    let type_data_length = u16::from_le_bytes([body[7], body[8]]) as usize;
    // The type data and the trailing source string must fit in the body.
    if body.len() < 9 + type_data_length {
        return Resource::Unknown {
            descriptor_type: large_type::SERIAL_BUS_CONNECTION,
        };
    }
    let type_data = &body[9..9 + type_data_length];

    // AML_RESOURCE_SERIAL_COMMON `type` byte: 1 = I2C.
    if body.get(2).copied() != Some(1) {
        return Resource::SerialBus { bus_type: body[2] };
    }

    // The I2C type data carries the connection speed (4 bytes) followed by
    // the slave address (2 bytes); the trailing string names the controller.
    // Ten-bit addressing shares the flag word and is not supported here.
    if type_data.len() < 6 || type_data[0] & 0x01 != 0 {
        return Resource::Unknown {
            descriptor_type: large_type::SERIAL_BUS_CONNECTION,
        };
    }
    let slave_address = match type_data.get(4..6) {
        Some(bytes) => u16::from_le_bytes([bytes[0], bytes[1]]),
        None => {
            return Resource::Unknown {
                descriptor_type: large_type::SERIAL_BUS_CONNECTION,
            };
        }
    };
    let controller_path = trailing_source_string(&body[9 + type_data_length..]);
    Resource::I2cSerialBus {
        slave_address,
        controller_path,
    }
}

/// Extracts the null-terminated source string that closes a descriptor
/// body.
fn trailing_source_string(body: &[u8]) -> String {
    let mut string = String::new();
    for &byte in body {
        if byte == 0 {
            break;
        }
        string.push(byte as char);
    }
    string
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The `_CRS` buffer of the I2C controller of a Hygon laptop, byte for
    /// byte as it appears in the DSDT: an IRQ and a 4 KiB fixed memory range.
    #[test]
    fn decodes_controller_crs() {
        let buf: [u8; 18] = [
            0x23, 0x00, 0x04, 0x01, // IRQ: mask 0x0400, edge/active-high
            0x86, 0x09, 0x00, // Memory32Fixed, length 9
            0x01, // write status: writeable
            0x00, 0x20, 0xdc, 0xfe, // base address 0xfedc2000
            0x00, 0x10, 0x00, 0x00, // length 0x1000
            0x79, 0x00, // EndTag
        ];

        let resources = parse_resource_buffer(&buf);
        assert_eq!(resources.len(), 2);
        assert_eq!(resources[0], Resource::Irq { interrupt: 10 });
        assert_eq!(
            resources[1],
            Resource::Memory32Fixed {
                base: 0xfedc_2000,
                length: 0x1000,
            }
        );
    }

    /// The `_CRS` buffer of the I2C touchpad of the same laptop: an I2C
    /// serial bus connection on the controller above, a GPIO interrupt, and
    /// the end tag.
    #[test]
    fn decodes_touchpad_crs() {
        let buf: [u8; 65] = [
            0x8e, 0x19, 0x00, // Serial bus connection, length 25
            0x00, 0x01, 0x01, 0x02, 0x00, 0x00, 0x01, 0x06, 0x00, 0x80, 0x1a, 0x06,
            0x00, // connection speed 400 kHz
            0x2c, 0x00, // slave address 0x2c
            0x5c, 0x5f, 0x53, 0x42, 0x2e, 0x49, 0x32, 0x43, 0x41, 0x00, // \_SB.I2CA
            0x8c, 0x20, 0x00, // GPIO connection, length 32
            0x01, 0x00, 0x01, 0x00, 0x12, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x17, 0x00, 0x00,
            0x19, 0x00, 0x23, 0x00, 0x00, 0x00, 0x09, 0x00, 0x5c, 0x5f, 0x53, 0x42, 0x2e, 0x47,
            0x50, 0x49, 0x41, 0x00, // \_SB.GPIA
            0x79, 0x00, // EndTag
        ];

        let resources = parse_resource_buffer(&buf);
        assert_eq!(resources.len(), 2);
        assert_eq!(
            resources[0],
            Resource::I2cSerialBus {
                slave_address: 0x2c,
                controller_path: String::from("\\_SB.I2CA"),
            }
        );
        assert!(matches!(resources[1], Resource::GpioInt { .. }));
    }

    #[test]
    fn stops_at_end_tag() {
        // Trailing garbage after the EndTag is not decoded.
        let buf = [0x79, 0x00, 0xff, 0xff];
        assert!(parse_resource_buffer(&buf).is_empty());
    }
}
