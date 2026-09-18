// SPDX-License-Identifier: MPL-2.0

//! A pattern-scan parser for device declarations in an AML stream.
//!
//! Instead of walking the AML term list sequentially (which requires
//! understanding every AML opcode), this module scans the raw bytes for the
//! `5b 82` DeviceOp pattern. At each match it reads the PkgLength to find
//! the device body extent, validates the four-character name, and only then
//! scans the body for the named objects that carry hardware descriptions:
//! `_HID`, `_CID`, and `_CRS`. A match whose name is not a valid NameSeg is
//! treated as coincidental data (a `5b 82` pair inside a method body or a
//! buffer) and skipped byte-wise, never trusted to locate a device boundary.
//!
//! Devices nest in AML; the scanner keeps a stack of open device bodies so
//! that a nested declaration (a touchpad inside its I2C controller, as
//! firmware commonly emits) becomes its own device with a full path, and
//! its named objects are not attributed to the parent.
//!
//! Method bodies are skipped whole. The one pattern decoded is the trivial
//! `_CRS` accessor firmware emits for a static resource template:
//! `Method(_CRS) { Return(SCCG) }`, which resolves to the named buffer the
//! method returns.
//!
//! The parser must never panic on firmware bytes: every read is
//! bounds-checked, and a value that does not fit its declared body is
//! rejected rather than sliced. A crash during boot-time enumeration is
//! far more costly than a missed device.
//!
//! Reference: <https://download.intel.com/download/idptools/61122v004.pdf>,
//! section 20 ("ACPI Machine Language (AML) Specification").

use alloc::{
    collections::BTreeMap,
    format,
    string::{String, ToString},
    vec::Vec,
};

/// A device node found in the AML stream.
#[derive(Clone, Debug)]
pub struct AmlDevice {
    /// The four-character device name, e.g. `I2CA`.
    pub name: String,
    /// The absolute path, e.g. `\_SB.I2CA`.
    pub path: String,
    /// The absolute path of the enclosing device, or `None` when the device
    /// sits directly in a scope. A touchpad declared inside its I2C
    /// controller carries the controller here — the same bus attachment the
    /// resource source string denotes in display form.
    pub parent: Option<String>,
    /// The values of the named objects declared by the device, keyed by
    /// object name (`_HID`, `_CID`, `_CRS`, `_STA`, ...).
    pub objects: BTreeMap<String, ObjectValue>,
}

impl AmlDevice {
    /// Returns the hardware ID: the string value of `_HID`, decoding the
    /// compressed EISA form when the firmware stored an integer.
    pub fn hid(&self) -> Option<String> {
        match self.objects.get("_HID")? {
            ObjectValue::Str(s) => Some(s.clone()),
            ObjectValue::Int(id) => Some(eisa_id_to_string(*id)),
            _ => None,
        }
    }

    /// Returns the compatible IDs: the string values of `_CID`, decoding
    /// compressed EISA form integers like [`Self::hid`] does.
    pub fn cids(&self) -> Vec<String> {
        let mut out = Vec::new();
        let Some(value) = self.objects.get("_CID") else {
            return out;
        };
        match value {
            ObjectValue::Str(s) => out.push(s.clone()),
            ObjectValue::Int(id) => out.push(eisa_id_to_string(*id)),
            ObjectValue::Buf(bytes) => {
                for chunk in bytes.as_chunks::<4>().0 {
                    let id = u32::from_le_bytes(*chunk);
                    out.push(eisa_id_to_string(id.into()));
                }
            }
        }
        out
    }

    /// Returns the bytes of the resource template behind `_CRS`, if the
    /// object is a buffer.
    pub fn crs_buffer(&self) -> Option<&[u8]> {
        match self.objects.get("_CRS")? {
            ObjectValue::Buf(bytes) => Some(bytes),
            _ => None,
        }
    }
}

/// The value of a named object declared in the namespace.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ObjectValue {
    /// An ASCII string, e.g. `Name(_HID, "BLTP7853")`.
    Str(String),
    /// An integer, e.g. `Name(_STA, 0x0f)`.
    Int(u64),
    /// A byte buffer, e.g. the resource template of `_CRS`.
    Buf(Vec<u8>),
}

/// Decodes a compressed EISA ID integer into its string form.
///
/// The firmware stores the ID byte-swapped (the first letter lands in the
/// lowest byte), so the value is swapped back before the packed letters and
/// hex digits are extracted.
///
/// Reference:
/// <https://elixir.bootlin.com/linux/v6.16/source/drivers/acpi/acpica/exutils.c>
/// (`AcpiUtExEisaIdToString`).
fn eisa_id_to_string(id: u64) -> String {
    let id = u32::from_le_bytes((id as u32).to_le_bytes()).swap_bytes();
    let mut out = String::with_capacity(8);
    let letter = |shift: u32| (b'A' - 1) + ((id >> shift) & 0x1f) as u8;
    let digit = |shift: u32| char::from_digit((id >> shift) & 0xf, 16).unwrap_or('0');
    out.push(letter(26) as char);
    out.push(letter(21) as char);
    out.push(letter(16) as char);
    out.push(digit(12).to_ascii_uppercase());
    out.push(digit(8).to_ascii_uppercase());
    out.push(digit(4).to_ascii_uppercase());
    out.push(digit(0).to_ascii_uppercase());
    out
}

/// Reads an AML PkgLength starting at `pos`. Returns `(length, bytes_used)`.
///
/// A PkgLength counts its own encoding bytes: the package it describes
/// starts at the first length byte and spans exactly `length` bytes. A
/// single-byte length fills bits 5:0 (up to 63); a multi-byte one puts the
/// low nibble in bits 3:0 and the remaining bytes, little-endian, above it.
fn read_pkg_length(data: &[u8], pos: usize) -> Option<(u32, usize)> {
    if pos >= data.len() {
        return None;
    }
    let lead = data[pos];
    let extra = ((lead >> 6) & 3) as usize;
    let length = if extra == 0 {
        (lead & 0x3f) as u32
    } else {
        let mut length = (lead & 0x0f) as u32;
        for i in 0..extra {
            let b = *data.get(pos + 1 + i)?;
            length |= (b as u32) << (4 + i * 8);
        }
        length
    };
    Some((length, 1 + extra))
}

/// Reads a four-character name segment starting at `pos`.
/// NUL and underscore padding is preserved in the output.
fn read_name_seg(data: &[u8], pos: usize) -> Option<(String, usize)> {
    if pos + 4 > data.len() {
        return None;
    }
    let mut seg = String::with_capacity(4);
    for i in 0..4 {
        let b = data[pos + i];
        if !(b == 0 || b == b'_' || b.is_ascii_alphanumeric()) {
            return None;
        }
        seg.push(if b == 0 { '_' } else { b as char });
    }
    Some((seg, pos + 4))
}

/// A DeviceOp whose PkgLength, name, and body extent are all plausible.
type DeviceOp = (usize, usize, usize, String);

/// Finds the next plausible DeviceOp (`5b 82`) in `aml` at or after `from`.
///
/// Returns `(op_pos, body_start, body_end, name)` where the body extent
/// follows the PkgLength and the name is a valid NameSeg. A `5b 82` pair
/// that fails any check is coincidental data; the scan resumes right after
/// the two pattern bytes rather than trusting its PkgLength.
fn next_device_op(aml: &[u8], from: usize) -> Option<DeviceOp> {
    let mut pos = from;
    while pos + 1 < aml.len() {
        if aml[pos] == 0x5b && aml[pos + 1] == 0x82 {
            let op = try_device_op(aml, pos);
            if op.is_some() {
                return op;
            }
        }
        pos += 1;
    }
    None
}

/// Validates the DeviceOp pattern at `pos`, where the `5b` byte sits.
fn try_device_op(aml: &[u8], pos: usize) -> Option<DeviceOp> {
    let (length, len_bytes) = read_pkg_length(aml, pos + 2)?;
    let name_start = pos + 2 + len_bytes;
    let body_start = name_start + 4;
    let body_end = pos + 2 + length as usize;
    if body_end > aml.len() || body_end <= body_start {
        return None;
    }
    let (name, _) = read_name_seg(aml, name_start)?;
    Some((pos, body_start, body_end, name))
}

/// Parses an AML stream and returns the devices it declares, in declaration
/// order with nested devices following their parent.
pub fn parse_devices(aml: &[u8]) -> Vec<AmlDevice> {
    let mut devices = Vec::new();
    // The (body_end, path) of every open enclosing device, innermost last.
    let mut scopes: Vec<(usize, String)> = Vec::new();
    let mut from = 0;
    while let Some((op_pos, body_start, body_end, name)) = next_device_op(aml, from) {
        // Close every enclosing device whose body ended before this one.
        while scopes.last().is_some_and(|&(end, _)| end <= op_pos) {
            scopes.pop();
        }
        let (parent, path) = match scopes.last() {
            Some((_, parent)) => (Some(parent.clone()), format!("{parent}.{name}")),
            None => (None, format!("\\_SB.{name}")),
        };
        // Named objects of this device stop where a nested device begins;
        // the nested device is picked up by the scan in the next iteration.
        let scan_end = next_device_op(aml, body_start)
            .map_or(body_end, |(nested_pos, ..)| nested_pos.min(body_end));
        let mut objects = BTreeMap::new();
        scan_named_objects(aml, body_start, scan_end, &mut objects);
        devices.push(AmlDevice {
            name,
            path: path.clone(),
            parent,
            objects,
        });
        scopes.push((body_end, path));
        from = op_pos + 2;
    }
    devices
}

/// Collects the named objects declared in `[start, end)`, which spans the
/// body of one device and stops before any nested device.
fn scan_named_objects(
    aml: &[u8],
    start: usize,
    end: usize,
    objects: &mut BTreeMap<String, ObjectValue>,
) {
    // Methods whose body is exactly `Return(<name>)`, resolved after the
    // scan so that declaration order does not matter.
    let mut returns: Vec<(String, String)> = Vec::new();
    let mut pos = start;
    while pos < end {
        match aml[pos] {
            0x08 => {
                let Some((name, value, next)) = (|| {
                    if pos + 5 > end {
                        return None;
                    }
                    let obj_name = &aml[pos + 1..pos + 5];
                    if !obj_name
                        .iter()
                        .all(|&b| b.is_ascii_alphanumeric() || b == b'_')
                    {
                        return None;
                    }
                    let (value, next) = parse_object_value(aml, pos + 5, end)?;
                    Some((
                        core::str::from_utf8(obj_name).ok()?.to_string(),
                        value,
                        next,
                    ))
                })() else {
                    pos += 1;
                    continue;
                };
                objects.insert(name, value);
                pos = next;
            }
            0x14 => {
                let Some((name, reference, pkg_end)) = try_trivial_return_method(aml, pos, end)
                else {
                    // A method whose body is anything else is skipped whole:
                    // its bytes are executable code, not namespace data.
                    pos += 1;
                    continue;
                };
                returns.push((name, reference));
                pos = pkg_end;
            }
            _ => pos += 1,
        }
    }
    for (name, reference) in returns {
        if let Some(value) = objects.get(&reference) {
            let value = value.clone();
            objects.insert(name, value);
        }
    }
}

/// Recognizes `Method(NameSeg, Flags) { Return(NameString) }` at `pos`,
/// the accessor firmware emits for a static resource template. Returns the
/// method name, the referenced object name, and the end of the package.
fn try_trivial_return_method(
    aml: &[u8],
    pos: usize,
    limit: usize,
) -> Option<(String, String, usize)> {
    let (pkg_len, len_bytes) = read_pkg_length(aml, pos + 1)?;
    let pkg_end = pos + 1 + pkg_len as usize;
    if pkg_end > limit {
        return None;
    }
    let (name, name_end) = read_name_seg(aml, pos + 1 + len_bytes)?;
    // One flags byte separates the name from the term list.
    let body_start = name_end + 1;
    if body_start >= pkg_end || aml[body_start] != 0xa4 {
        return None;
    }
    // The returned NameString may carry root/caret prefixes.
    let mut ref_pos = body_start + 1;
    while ref_pos < pkg_end && matches!(aml[ref_pos], 0x5c | 0x5e) {
        ref_pos += 1;
    }
    let (reference, ref_end) = read_name_seg(aml, ref_pos)?;
    if ref_end != pkg_end {
        return None;
    }
    Some((name, reference, pkg_end))
}

/// Parses the value of a named object at `pos`, bounded by `limit`.
///
/// Returns the value and the position just past it. Every read is
/// bounds-checked against the stream and the enclosing device body; a
/// value that does not fit yields `None` rather than a panic.
fn parse_object_value(aml: &[u8], pos: usize, limit: usize) -> Option<(ObjectValue, usize)> {
    let int_at = |pos: usize, width: usize| -> Option<u64> {
        if pos + width > limit {
            return None;
        }
        let bytes: [u8; 8] = aml.get(pos..pos + width)?.try_into().ok()?;
        Some(u64::from_le_bytes(bytes))
    };
    match *aml.get(pos)? {
        0x00 => Some((ObjectValue::Int(0), pos + 1)),
        0x01 => Some((ObjectValue::Int(1), pos + 1)),
        0x0a => Some((ObjectValue::Int(int_at(pos + 1, 1)?), pos + 2)),
        0x0b => Some((ObjectValue::Int(int_at(pos + 1, 2)?), pos + 3)),
        0x0c => Some((ObjectValue::Int(int_at(pos + 1, 4)?), pos + 5)),
        0x0e => Some((ObjectValue::Int(int_at(pos + 1, 8)?), pos + 9)),
        0x0d => {
            // String: NUL-terminated within the enclosing body.
            if pos + 1 >= limit {
                return None;
            }
            let end = aml.get(pos + 1..limit)?.iter().position(|&b| b == 0)? + pos + 1;
            let s = core::str::from_utf8(aml.get(pos + 1..end)?).ok()?;
            Some((ObjectValue::Str(s.to_string()), end + 1))
        }
        0x11 => {
            // Buffer: `11 PkgLength BufferSize ByteList`. The PkgLength
            // counts its own bytes, so the data starts after the BufferSize
            // term and ends where the package does.
            let (pkg_len, len_bytes) = read_pkg_length(aml, pos + 1)?;
            let body_start = pos + 1 + len_bytes;
            let body_end = (pos + 1 + pkg_len as usize).min(limit);
            let size_len = match *aml.get(body_start)? {
                0x00 | 0x01 => 1,
                0x0a => 2,
                0x0b => 3,
                0x0c => 5,
                0x0e => 9,
                _ => return None,
            };
            let data_start = body_start + size_len;
            if data_start > body_end {
                return None;
            }
            Some((
                ObjectValue::Buf(aml[data_start..body_end].to_vec()),
                body_end,
            ))
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The byte-exact declaration of the I2C controller `I2CA`, its nested
    /// touchpad `TPD0`, and the start of the sibling controller `I2CB`, as
    /// found in the DSDT of a Hygon laptop.
    const HYGON_I2CA_TPD0: &[u8] = &[
        0x5b, 0x82, 0x4c, 0x14, 0x49, 0x32, 0x43, 0x41, 0x08, 0x5f, 0x48, 0x49, 0x44, 0x0d, 0x48,
        0x59, 0x47, 0x4f, 0x30, 0x30, 0x31, 0x30, 0x00, 0x08, 0x5f, 0x55, 0x49, 0x44, 0x00, 0x08,
        0x5f, 0x43, 0x52, 0x53, 0x11, 0x15, 0x0a, 0x12, 0x23, 0x00, 0x04, 0x01, 0x86, 0x09, 0x00,
        0x01, 0x00, 0x20, 0xdc, 0xfe, 0x00, 0x10, 0x00, 0x00, 0x79, 0x00, 0x14, 0x0b, 0x5f, 0x53,
        0x54, 0x41, 0x00, 0xa4, 0x49, 0x43, 0x41, 0x45, 0x5b, 0x82, 0x48, 0x10, 0x54, 0x50, 0x44,
        0x30, 0x08, 0x5f, 0x48, 0x49, 0x44, 0x0d, 0x42, 0x4c, 0x54, 0x50, 0x37, 0x38, 0x35, 0x33,
        0x00, 0x08, 0x5f, 0x43, 0x49, 0x44, 0x0d, 0x50, 0x4e, 0x50, 0x30, 0x43, 0x35, 0x30, 0x00,
        0x14, 0x09, 0x5f, 0x53, 0x54, 0x41, 0x00, 0xa4, 0x0a, 0x0f, 0x08, 0x53, 0x43, 0x43, 0x47,
        0x11, 0x45, 0x04, 0x0a, 0x41, 0x8e, 0x19, 0x00, 0x01, 0x00, 0x01, 0x02, 0x00, 0x00, 0x01,
        0x06, 0x00, 0x80, 0x1a, 0x06, 0x00, 0x2c, 0x00, 0x5c, 0x5f, 0x53, 0x42, 0x2e, 0x49, 0x32,
        0x43, 0x41, 0x00, 0x8c, 0x20, 0x00, 0x01, 0x00, 0x01, 0x00, 0x12, 0x00, 0x01, 0x00, 0x00,
        0x00, 0x00, 0x17, 0x00, 0x00, 0x19, 0x00, 0x23, 0x00, 0x00, 0x00, 0x09, 0x00, 0x5c, 0x5f,
        0x53, 0x42, 0x2e, 0x47, 0x50, 0x49, 0x41, 0x00, 0x79, 0x00, 0x14, 0x0b, 0x5f, 0x43, 0x52,
        0x53, 0x00, 0xa4, 0x53, 0x43, 0x43, 0x47, 0x14, 0x43, 0x08, 0x5f, 0x44, 0x53, 0x4d, 0x04,
        0xa0, 0x3e, 0x93, 0x68, 0x11, 0x13, 0x0a, 0x10, 0xf7, 0xf6, 0xdf, 0x3c, 0x67, 0x42, 0x55,
        0x45, 0xad, 0x05, 0xb3, 0x0a, 0x3d, 0x89, 0x38, 0xde, 0xa0, 0x15, 0x93, 0x6a, 0x00, 0xa0,
        0x09, 0x93, 0x69, 0x01, 0xa4, 0x11, 0x03, 0x01, 0x03, 0xa1, 0x06, 0xa4, 0x11, 0x03, 0x01,
        0x00, 0xa1, 0x10, 0xa0, 0x07, 0x93, 0x6a, 0x01, 0xa4, 0x0a, 0x20, 0xa1, 0x06, 0xa4, 0x11,
        0x03, 0x01, 0x00, 0xa1, 0x3c, 0xa0, 0x33, 0x93, 0x68, 0x11, 0x13, 0x0a, 0x10, 0x82, 0xeb,
        0x87, 0xef, 0x51, 0xf9, 0xda, 0x46, 0x84, 0xec, 0x14, 0x87, 0x1a, 0xc6, 0xf8, 0x4b, 0xa0,
        0x0e, 0x93, 0x6a, 0x00, 0xa0, 0x09, 0x93, 0x69, 0x01, 0xa4, 0x11, 0x03, 0x01, 0x03, 0xa0,
        0x07, 0x93, 0x6a, 0x01, 0xa4, 0x0a, 0x20, 0xa4, 0x11, 0x03, 0x01, 0x00, 0xa1, 0x06, 0xa4,
        0x11, 0x03, 0x01, 0x00, 0x5b, 0x82, 0x42, 0x04, 0x49, 0x32, 0x43, 0x42, 0x08, 0x5f, 0x48,
        0x49, 0x44, 0x0d, 0x48, 0x59, 0x47, 0x4f, 0x30, 0x30, 0x31, 0x30, 0x00, 0x08, 0x5f, 0x55,
        0x49, 0x44, 0x01, 0x08, 0x5f, 0x43, 0x52, 0x53, 0x11, 0x15, 0x0a, 0x12, 0x23, 0x00, 0x08,
        0x01, 0x86, 0x09, 0x00, 0x01, 0x00, 0x30, 0xdc, 0xfe, 0x00, 0x10, 0x00, 0x00, 0x79, 0x00,
        0x14, 0x0b, 0x5f, 0x53, 0x54, 0x41, 0x00, 0xa4, 0x49, 0x43, 0x42, 0x45, 0x5b, 0x82,
    ];

    /// A DeviceOp whose body ends inside a NameOp value: the buffer's
    /// decoded data would start past the end of the enclosing body. The
    /// parser must reject the value instead of slicing a reversed range.
    const GARBAGE_DEVICE_OP: &[u8] = &[
        0x5b, 0x82, 0x0a, 0x43, 0x30, 0x30, 0x30, // Device C000, 10-byte package
        0x08, 0x5f, 0x48, 0x49, 0x44, // NameOp `_HID` ...
        0x11, 0x09, 0x0a, 0xff, // ... whose value is a Buffer sticking out
    ];

    #[test]
    fn parses_nested_touchpad_with_method_crs() {
        let devices = parse_devices(HYGON_I2CA_TPD0);
        let paths: Vec<_> = devices.iter().map(|d| d.path.as_str()).collect();
        assert_eq!(paths, ["\\_SB.I2CA", "\\_SB.I2CA.TPD0", "\\_SB.I2CB"]);

        let controller = &devices[0];
        assert_eq!(controller.parent, None);
        assert_eq!(controller.hid().as_deref(), Some("HYGO0010"));
        assert!(controller.cids().is_empty());
        // The controller template: an IRQ and a 4 KiB fixed memory range.
        let crs = controller.crs_buffer().unwrap();
        assert_eq!(
            crs,
            &[
                0x23, 0x00, 0x04, 0x01, 0x86, 0x09, 0x00, 0x01, 0x00, 0x20, 0xdc, 0xfe, 0x00, 0x10,
                0x00, 0x00, 0x79, 0x00
            ]
        );

        let touchpad = &devices[1];
        // The touchpad is a child of its controller: this is the bus
        // attachment, matching the display-form source string of the
        // serial bus resource.
        assert_eq!(touchpad.parent.as_deref(), Some("\\_SB.I2CA"));
        assert_eq!(touchpad.hid().as_deref(), Some("BLTP7853"));
        assert_eq!(touchpad.cids(), ["PNP0C50"]);
        // The template comes from `Method(_CRS) { Return(SCCG) }`.
        let crs = touchpad.crs_buffer().unwrap();
        assert_eq!(&crs[..3], &[0x8e, 0x19, 0x00]);
        assert_eq!(&crs[crs.len() - 2..], &[0x79, 0x00]);
        assert_eq!(crs.len(), 65);
        // The slave address and the resource source string of the bus.
        assert_eq!(&crs[16..18], &[0x2c, 0x00]);
        assert_eq!(&crs[18..28], b"\\_SB.I2CA\x00");
    }

    #[test]
    fn rejects_garbage_device_op_without_panicking() {
        let devices = parse_devices(GARBAGE_DEVICE_OP);
        // The name is a plausible NameSeg, so the device is kept, but the
        // `_HID` value that sticks out of the body is dropped.
        assert_eq!(devices.len(), 1);
        assert_eq!(devices[0].name, "C000");
        assert!(devices[0].hid().is_none());
    }

    #[test]
    fn resolves_trivial_return_regardless_of_declaration_order() {
        // `_CRS` references the buffer before the buffer is declared.
        let aml: &[u8] = &[
            0x5b, 0x82, 0x20, 0x54, 0x45, 0x53, 0x54, // Device TEST
            0x14, 0x0b, 0x5f, 0x43, 0x52, 0x53, 0x00, 0xa4, 0x42, 0x55, 0x46, 0x41, 0x08, 0x42,
            0x55, 0x46, 0x41, 0x11, 0x09, 0x0a, 0x04, 0x11, 0x22, 0x33, 0x44, 0x79, 0x00,
        ];
        let devices = parse_devices(aml);
        assert_eq!(devices.len(), 1);
        assert_eq!(
            devices[0].crs_buffer(),
            Some(&[0x11, 0x22, 0x33, 0x44, 0x79, 0x00][..])
        );
    }
}
