// SPDX-License-Identifier: MPL-2.0

//! Static parsing of the ACPI namespace.
//!
//! This module walks the `Scope` and `Device` structure of an AML stream and
//! collects, for every device, the values of the named objects it declares:
//! `_HID`, `_CID`, `_STA`, `_CRS`, and any other object the firmware happens
//! to declare. Only the declarative subset of AML is understood; a term list
//! that contains anything else aborts the walk rather than risk a
//! misparse — the caller can still use the devices found so far.
//!
//! Reference: <https://download.intel.com/download/idptools/61122v004.pdf>,
//! section 20.2.2 ("Term Lists Encoding") and section 19.6 for the object
//! types; the Linux counterpart of the walk is
//! <https://elixir.bootlin.com/linux/v6.16/source/drivers/acpi/acpi_bus.c>
//! (`acpi_walk_namespace` on the device type nodes).

use alloc::{
    collections::BTreeMap,
    string::{String, ToString},
    vec::Vec,
};

use super::pkg::{AmlError, AmlStream, opcode};

/// The value of a named object, as declared in the namespace.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ObjectValue {
    /// An ASCII string, e.g. `Name(_HID, "BLTP7853")`.
    Str(String),
    /// An integer, e.g. `Method(_STA) { Return(0x0f) }`.
    Int(u64),
    /// A byte buffer, e.g. the resource template of `_CRS`.
    Buf(Vec<u8>),
    /// A value the static parser does not model (an unresolved reference, a
    /// package, ...). Present so that the declaration still counts as seen.
    Opaque,
}

/// A device node declared in the namespace.
#[derive(Clone, Debug)]
pub struct AmlDevice {
    /// The four-character device name, e.g. `I2CA`.
    pub name: String,
    /// The absolute path, e.g. `\\_SB.I2CA`.
    pub path: String,
    /// The values of the named objects declared by the device, keyed by
    /// object name (`_HID`, `_CID`, `_CRS`, `_STA`, ...).
    pub objects: BTreeMap<String, ObjectValue>,
}

impl AmlDevice {
    /// Returns the hardware ID: the string value of `_HID`, decoding the
    /// compressed EISA form when the firmware stored an integer.
    ///
    /// Reference:
    /// <https://elixir.bootlin.com/linux/v6.16/source/drivers/acpi/scan.c>
    /// (`acpi_device_hid` over the packed form decoded by
    /// `acpi_eisa_id_to_string`).
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
                // A buffer of packed EISA IDs, one every four bytes.
                for id in bytes.as_chunks::<4>().0 {
                    out.push(eisa_id_to_string(u64::from(u32::from_le_bytes(*id))));
                }
            }
            ObjectValue::Opaque => {}
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

/// Decodes a compressed EISA ID integer into its string form.
///
/// The firmware stores the ID byte-swapped (the first letter lands in the
/// lowest byte), so the value is swapped back before the packed letters and
/// hex digits are extracted.
///
/// Reference:
/// <https://elixir.bootlin.com/linux/v6.16/source/drivers/acpi/acpica/exutils.c>
/// (`AcpiUtExEisaIdToString` over `acpi_ut_dword_byte_swap`) and
/// <https://elixir.bootlin.com/linux/v6.16/source/drivers/acpi/scan.c>
/// (`acpi_eisa_id_to_string`).
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

/// Parses an AML term list and returns the devices it declares, in
/// declaration order.
pub(super) fn parse_devices(aml: &[u8]) -> Vec<AmlDevice> {
    let mut stream = AmlStream::new(aml);
    let end = stream.end();
    let mut devices = Vec::new();
    let _ = walk_term_list(&mut stream, end, "", &mut devices);
    devices
}

/// Walks a term list from the current position to `end`, collecting the
/// devices it declares into `devices`.
///
/// `scope_path` is the absolute path of the enclosing scope (empty for the
/// root). The device whose body is currently being walked is held in
/// `current` and flushed into `devices` when its term list ends or when a
/// nested device begins.
///
/// Returns `Err` when an unknown term forces the walk to stop; the devices
/// collected up to that point remain valid.
fn walk_term_list(
    stream: &mut AmlStream,
    end: usize,
    scope_path: &str,
    devices: &mut Vec<AmlDevice>,
) -> Result<(), AmlError> {
    // The device whose body is currently being walked, together with the end
    // of that body. Held locally so that the borrow of `devices` stays
    // exclusive.
    let mut current: Option<(AmlDevice, usize)> = None;
    while stream.pos() < end {
        // Close a device whose body has ended before interpreting the next
        // term, so that later objects are not attributed to it.
        if let Some((_, dev_end)) = &current
            && stream.pos() >= *dev_end
        {
            flush(&mut current, devices);
        }
        let op = stream.peek().ok_or(AmlError::UnexpectedEnd)?;
        match op {
            opcode::SCOPE => {
                flush(&mut current, devices);
                let body_end = open_pkg_body(stream, end)?;
                let name = stream.read_name_string()?;
                let child_path = join_path(scope_path, &name);
                walk_term_list(stream, body_end, child_path.as_str(), devices)?;
                stream.set_pos(body_end);
            }
            opcode::EXT_PREFIX => {
                // Consume the prefix byte; `peek` above did not advance.
                stream.read_u8()?;
                let ext = stream.read_u8()?;
                match ext {
                    opcode::EXT_DEVICE => {
                        flush(&mut current, devices);
                        let body_end = open_pkg_body_after_ext(stream, end)?;
                        let name = stream.read_name_string()?;
                        let path = join_path(scope_path, &name);
                        current = Some((
                            AmlDevice {
                                name,
                                path,
                                objects: BTreeMap::new(),
                            },
                            body_end,
                        ));
                    }
                    opcode::EXT_METHOD => {
                        let body_end = open_pkg_body_after_ext(stream, end)?;
                        let name = stream.read_name_string()?;
                        let _flags = stream.read_u8()?;
                        // A method body that simply returns a named object is
                        // resolved to that object's declared value; anything
                        // else is skipped whole.
                        if body_end == stream.pos() + 1 + 4
                            && stream.peek() == Some(opcode::RETURN)
                            && current.is_some()
                        {
                            stream.read_u8()?;
                            let reference = stream.read_name_string()?;
                            let value = current
                                .as_ref()
                                .and_then(|(device, _)| device.objects.get(&reference).cloned());
                            if let (Some(value), Some((device, _))) = (value, current.as_mut()) {
                                device.objects.insert(name, value);
                            }
                        }
                        stream.set_pos(body_end);
                    }
                    // Field-like and container terms have fixed structure but
                    // no objects we care about; they are package-wrapped, so
                    // skip them whole.
                    opcode::EXT_FIELD
                    | opcode::EXT_INDEX_FIELD
                    | opcode::EXT_BANK_FIELD
                    | opcode::EXT_DATA_REGION
                    | opcode::EXT_OPERATION_REGION
                    | opcode::EXT_POWER_RES
                    | opcode::EXT_THERMAL_ZONE
                    | opcode::EXT_PROCESSOR
                    | opcode::EXT_MUTEX
                    | opcode::EXT_EVENT => {
                        skip_pkg_body(stream, end)?;
                    }
                    _ => return Err(AmlError::UnexpectedEnd),
                }
            }
            opcode::NAME => {
                stream.read_u8()?;
                let name = stream.read_name_string()?;
                let value = parse_data_object(stream)?;
                if let Some((device, _)) = current.as_mut() {
                    device.objects.insert(name, value);
                }
            }
            opcode::RETURN => {
                stream.read_u8()?;
                parse_data_object(stream)?;
            }
            // Control flow that may wrap devices and scopes; `While` bodies
            // are skipped whole because a static walk must not loop.
            0xa0..=0xa2 => {
                skip_pkg_body(stream, end)?;
            }
            // `Noop`, `Break`, `BreakPoint`: single-byte statements.
            0xa3 | 0x9f | 0xcc => {
                stream.read_u8()?;
            }
            _ => return Err(AmlError::UnexpectedEnd),
        }
    }
    flush(&mut current, devices);
    Ok(())
}

/// Moves a finished device into the device list.
fn flush(current: &mut Option<(AmlDevice, usize)>, devices: &mut Vec<AmlDevice>) {
    if let Some((device, _)) = current.take() {
        devices.push(device);
    }
}

/// Reads a data object: the value forms a `NameOp` may legally carry.
fn parse_data_object(stream: &mut AmlStream) -> Result<ObjectValue, AmlError> {
    match stream.peek().ok_or(AmlError::UnexpectedEnd)? {
        opcode::ZERO => {
            stream.read_u8()?;
            Ok(ObjectValue::Int(0))
        }
        0x01 => {
            stream.read_u8()?;
            Ok(ObjectValue::Int(1))
        }
        0xff => {
            stream.read_u8()?;
            Ok(ObjectValue::Int(u64::MAX))
        }
        opcode::BYTE_CONST_PREFIX => {
            stream.read_u8()?;
            Ok(ObjectValue::Int(stream.read_int(1)?))
        }
        0x0b => {
            stream.read_u8()?;
            Ok(ObjectValue::Int(stream.read_int(2)?))
        }
        0x0c => {
            stream.read_u8()?;
            Ok(ObjectValue::Int(stream.read_int(4)?))
        }
        0x0e => {
            stream.read_u8()?;
            Ok(ObjectValue::Int(stream.read_int(8)?))
        }
        opcode::STRING_PREFIX => {
            stream.read_u8()?;
            let mut string = String::new();
            loop {
                let byte = stream.read_u8()?;
                if byte == 0 {
                    break;
                }
                string.push(byte as char);
            }
            Ok(ObjectValue::Str(string))
        }
        opcode::BUFFER => {
            let body_end = open_pkg_body(stream, stream.end())?;
            // Skip the BufferSize term; the remaining bytes of the package
            // body are the buffer contents.
            let _ = parse_data_object(stream)?;
            let len = body_end.saturating_sub(stream.pos());
            let bytes = stream.read_bytes(len)?;
            Ok(ObjectValue::Buf(bytes))
        }
        // Packages and expressions are not modeled; consume nothing and
        // report the value as opaque.
        _ => Ok(ObjectValue::Opaque),
    }
}

/// Opens a package-wrapped term: consumes the opcode (one or two bytes),
/// reads the package length, and returns the absolute end position of the
/// package body. The body immediately follows the length and ends at the
/// returned position.
///
/// `limit` is the end of the enclosing term list; a package that would
/// extend past it is malformed.
fn open_pkg_body(stream: &mut AmlStream, limit: usize) -> Result<usize, AmlError> {
    let is_extended = stream.peek() == Some(opcode::EXT_PREFIX);
    let header = if is_extended { 2 } else { 1 };
    stream.read_u8()?;
    if is_extended {
        stream.read_u8()?;
    }
    let length = stream.read_pkg_length()? as usize;
    if length < header {
        return Err(AmlError::InvalidPkgLength);
    }
    let start = stream.pos();
    let end = start - header + length;
    if end > limit {
        return Err(AmlError::UnexpectedEnd);
    }
    Ok(end)
}

/// Opens an extended (two-byte opcode) package whose opcode has already
/// been consumed: reads the package length and returns the absolute end
/// position of the package body.
fn open_pkg_body_after_ext(stream: &mut AmlStream, limit: usize) -> Result<usize, AmlError> {
    let length_pos = stream.pos();
    let length = stream.read_pkg_length()? as usize;
    // The length covers the package length bytes themselves and the body.
    let end = length_pos + length;
    if end > limit {
        return Err(AmlError::UnexpectedEnd);
    }
    Ok(end)
}

/// Skips a package-wrapped term entirely.
fn skip_pkg_body(stream: &mut AmlStream, limit: usize) -> Result<(), AmlError> {
    open_pkg_body(stream, limit)?;
    Ok(())
}

/// Joins a namespace path onto a scope path.
fn join_path(scope: &str, name: &str) -> String {
    if name.starts_with('\\') {
        return name.to_string();
    }
    let mut path = String::with_capacity(scope.len() + 1 + name.len());
    path.push_str(scope);
    if !path.is_empty() && !path.ends_with('.') {
        path.push('.');
    }
    path.push_str(name);
    path
}

#[cfg(test)]
mod tests {
    use alloc::vec;

    use super::*;

    /// Wraps a device body into a `DeviceOp`: opcode, package length, name,
    /// body. The package length covers the name, the body, and its own two
    /// bytes, but not the two opcode bytes.
    fn device_op(name: &[u8; 4], body: &[u8]) -> Vec<u8> {
        let mut out = vec![opcode::EXT_PREFIX, opcode::EXT_DEVICE];
        let len = body.len() + name.len() + 2;
        out.push(0x40 | (len & 0x0f) as u8);
        out.push((len >> 4) as u8);
        out.extend_from_slice(name);
        out.extend_from_slice(body);
        out
    }

    /// Wraps a value into a `NameOp` with the given four-character name.
    fn name_op(name: &[u8; 4], value: &[u8]) -> Vec<u8> {
        let mut out = vec![opcode::NAME];
        out.extend_from_slice(name);
        out.extend_from_slice(value);
        out
    }

    #[test]
    fn parses_device_with_static_crs() {
        // Device(I2CA) {
        //   Name(_HID, "HYGO0010")
        //   Name(_CRS, Buffer { irq, fixed memory 32, end tag })
        // }
        let mut body = name_op(b"_HID", &[opcode::STRING_PREFIX]);
        body.extend_from_slice(b"HYGO0010\0");
        body.extend_from_slice(&name_op(b"_CRS", &[opcode::BUFFER, 0x16, 0x0a, 0x12]));
        body.extend_from_slice(&[
            0x23, 0x00, 0x04, 0x01, // IRQ: mask 0x0400, edge/active-high
            0x86, 0x09, 0x00, // Memory32Fixed, length 9
            0x00, 0x01, // write status, write type
            0x00, 0x20, 0xdc, 0xfe, // base address 0xfedc2000
            0x00, 0x10, 0x00, 0x00, // length 0x1000
            0x79, 0x00, // EndTag
        ]);
        let aml = device_op(b"I2CA", &body);

        let devices = parse_devices(&aml);
        assert_eq!(devices.len(), 1);
        assert_eq!(devices[0].name, "I2CA");
        assert_eq!(devices[0].hid().as_deref(), Some("HYGO0010"));
        assert_eq!(devices[0].crs_buffer().unwrap().len(), 19);
    }

    #[test]
    fn resolves_method_returning_named_object() {
        // Device(TPD0) {
        //   Name(_HID, "BLTP7853"), Name(_CID, "PNP0C50"),
        //   Name(SCCG, Buffer { ... }),
        //   Method(_CRS) { Return(SCCG) },
        // }
        let mut body = name_op(b"_HID", &[opcode::STRING_PREFIX]);
        body.extend_from_slice(b"BLTP7853\0");
        body.extend_from_slice(&name_op(b"_CID", &[opcode::STRING_PREFIX]));
        body.extend_from_slice(b"PNP0C50\0");
        body.extend_from_slice(&name_op(b"SCCG", &[opcode::BUFFER, 0x2b, 0x0a, 0x28]));
        body.extend_from_slice(&[0x28; 40]);
        // Method(_CRS, 0) { Return(SCCG) }.
        body.extend_from_slice(&[
            opcode::EXT_PREFIX,
            opcode::EXT_METHOD,
            0x0b,
            b'_',
            b'C',
            b'R',
            b'S',
            0x00,
            opcode::RETURN,
            b'S',
            b'C',
            b'C',
            b'G',
        ]);
        let aml = device_op(b"TPD0", &body);

        let devices = parse_devices(&aml);
        assert_eq!(devices.len(), 1);
        assert_eq!(devices[0].cids(), vec!["PNP0C50".to_string()]);
        // The `_CRS` method resolved to the `SCCG` buffer.
        assert_eq!(&devices[0].crs_buffer().unwrap()[..5], &[0x28; 5]);
    }

    #[test]
    fn unknown_term_stops_the_walk_but_keeps_devices() {
        // A device followed by a byte the walker does not understand.
        let mut body = name_op(b"_HID", &[opcode::STRING_PREFIX]);
        body.extend_from_slice(b"HYGO0010\0");
        let mut aml = device_op(b"I2CA", &body);
        aml.push(0x77);

        let devices = parse_devices(&aml);
        assert_eq!(devices.len(), 1);
        assert_eq!(devices[0].hid().as_deref(), Some("HYGO0010"));
    }
}
