// SPDX-License-Identifier: MPL-2.0

//! A minimal HID report descriptor parser.
//!
//! This understands just enough of the HID item stream to locate the relative
//! "mouse" collection of a multi-interface touchpad and to describe the bit
//! layout of its report, following the USB HID 1.11 specification.

use alloc::vec::Vec;

/// The kind of a report field that we forward to the input subsystem.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FieldKind {
    /// Relative horizontal movement (`REL_X`).
    RelX,
    /// Relative vertical movement (`REL_Y`).
    RelY,
    /// Relative wheel movement (`REL_WHEEL`).
    RelWheel,
    /// A button; the payload is the button number starting at 1.
    Button(u8),
}

/// A single bit field inside a report.
#[derive(Clone, Copy, Debug)]
pub struct Field {
    pub kind: FieldKind,
    /// Bit offset of the field within the report payload.
    pub offset: usize,
    /// Field size in bits.
    pub size: usize,
    /// Whether the field is a signed two's-complement value.
    pub signed: bool,
}

/// The layout of one report.
#[derive(Clone, Debug, Default)]
pub struct ReportLayout {
    pub report_id: u8,
    pub fields: Vec<Field>,
    /// Size of the report payload in bytes, used to size input reads.
    pub payload_bytes: usize,
}

/// Global item state, saved and restored by `Push`/`Pop` items.
#[derive(Clone, Default)]
struct GlobalState {
    usage_page: u16,
    logical_min: i32,
    report_size: usize,
    report_count: usize,
    report_id: u8,
}

/// Local item state, reset by every main item.
#[derive(Default)]
struct LocalState {
    usages: Vec<u16>,
    usage_min: Option<u16>,
    usage_max: Option<u16>,
}

/// Fields accumulated for a single report ID.
#[derive(Default)]
struct ReportAcc {
    report_id: u8,
    fields: Vec<Field>,
    bit_offset: usize,
    has_rel_x: bool,
    has_rel_y: bool,
}

impl ReportAcc {
    fn has_pointer(&self) -> bool {
        self.has_rel_x && self.has_rel_y
    }
}

/// Input item flags (the single data byte of an `Input` main item).
mod input_flags {
    pub const CONSTANT: u8 = 1 << 0;
    pub const VARIABLE: u8 = 1 << 1;
    pub const RELATIVE: u8 = 1 << 2;
}

/// Parses the report descriptor and returns the relative mouse report, if any.
pub fn parse_mouse_report(desc: &[u8]) -> Option<ReportLayout> {
    let mut global = GlobalState {
        report_size: 8,
        report_count: 1,
        ..GlobalState::default()
    };
    let mut global_stack: Vec<GlobalState> = Vec::new();
    let mut local = LocalState::default();
    let mut reports: Vec<ReportAcc> = Vec::new();

    let mut idx = 0;
    while idx < desc.len() {
        let prefix = desc[idx];
        idx += 1;
        let size_code = (prefix & 0x03) as usize;
        let data_len = match size_code {
            0 => 0,
            1 => 1,
            2 => 2,
            3 => 4,
            _ => 0,
        };
        if idx + data_len > desc.len() {
            ostd::warn!("report descriptor: truncated item at byte {}", idx);
            break;
        }
        let data = &desc[idx..idx + data_len];
        idx += data_len;

        let tag = prefix >> 4;
        let item_type = (prefix >> 2) & 0x03;
        match (item_type, tag) {
            // Global items.
            (1, 0x0) => global.usage_page = data_u32(data) as u16,
            (1, 0x1) => global.logical_min = data_i32(data),
            (1, 0x7) => global.report_size = data_u32(data) as usize,
            (1, 0x8) => global.report_id = data_u32(data) as u8,
            (1, 0x9) => global.report_count = data_u32(data) as usize,
            (1, 0xa) => global_stack.push(global.clone()),
            (1, 0xb) => {
                if let Some(saved) = global_stack.pop() {
                    global = saved;
                }
            }
            // Local items.
            (2, 0x0) => local.usages.push(data_u32(data) as u16),
            (2, 0x1) => local.usage_min = Some(data_u32(data) as u16),
            (2, 0x2) => local.usage_max = Some(data_u32(data) as u16),
            // Main items: only Input reports carry data.
            (0, 0x8) => {
                let flags = data.first().copied().unwrap_or(0);
                let usages = expand_usages(&local.usages, local.usage_min, local.usage_max);
                local = LocalState::default();

                if global.report_count == 0 || usages.is_empty() {
                    continue;
                }
                // Constant fields are padding and never carry a value.
                if flags & input_flags::CONSTANT != 0 {
                    continue;
                }
                let report = report_mut(&mut reports, global.report_id);
                for (k, usage) in usages.iter().enumerate() {
                    let offset = report.bit_offset + k * global.report_size;
                    if let Some(kind) = field_kind(global.usage_page, *usage, flags) {
                        report.fields.push(Field {
                            kind,
                            offset,
                            size: global.report_size,
                            signed: global.logical_min < 0,
                        });
                        match kind {
                            FieldKind::RelX => report.has_rel_x = true,
                            FieldKind::RelY => report.has_rel_y = true,
                            _ => {}
                        }
                    }
                }
                report.bit_offset += global.report_size * global.report_count;
            }
            _ => {}
        }
    }

    let layout = reports.iter().find(|r| r.has_pointer()).map(|r| {
        let payload_bits = r.bit_offset;
        ReportLayout {
            report_id: r.report_id,
            fields: r.fields.clone(),
            payload_bytes: payload_bits.div_ceil(8),
        }
    });
    if let Some(layout) = &layout {
        ostd::info!(
            "report descriptor: mouse report id={} payload={} bytes, {} fields",
            layout.report_id,
            layout.payload_bytes,
            layout.fields.len()
        );
    } else {
        ostd::warn!(
            "report descriptor: no relative mouse report found ({} reports parsed)",
            reports.len()
        );
    }
    layout
}

/// Expands the local usages (single usages or a minimum/maximum range) into a
/// concrete list, capped to avoid absurd descriptor-driven allocations.
fn expand_usages(usages: &[u16], usage_min: Option<u16>, usage_max: Option<u16>) -> Vec<u16> {
    if let (Some(min), Some(max)) = (usage_min, usage_max)
        && max >= min
        && (max - min) <= 64
    {
        return (min..=max).collect();
    }
    usages.to_vec()
}

/// Maps a usage and the Input item flags to the field kind we forward, if any.
///
/// Only `Data`, `Variable` fields carry values. Pointer axes (X, Y, wheel)
/// are forwarded only when `Relative` is set — absolute axes belong to the
/// digitizer report of a multi-interface touchpad, not to the pointer — while
/// buttons are forwarded regardless, since mouse descriptors declare them as
/// absolute one-bit fields.
fn field_kind(usage_page: u16, usage: u16, flags: u8) -> Option<FieldKind> {
    if flags & input_flags::VARIABLE == 0 {
        return None;
    }
    match usage_page {
        0x01 => match usage {
            0x30 if flags & input_flags::RELATIVE != 0 => Some(FieldKind::RelX),
            0x31 if flags & input_flags::RELATIVE != 0 => Some(FieldKind::RelY),
            0x38 if flags & input_flags::RELATIVE != 0 => Some(FieldKind::RelWheel),
            _ => None,
        },
        0x09 if (1..=8).contains(&usage) => Some(FieldKind::Button(usage as u8)),
        _ => None,
    }
}

fn report_mut(reports: &mut Vec<ReportAcc>, report_id: u8) -> &mut ReportAcc {
    let idx = match reports.iter().position(|r| r.report_id == report_id) {
        Some(idx) => idx,
        None => {
            reports.push(ReportAcc {
                report_id,
                ..ReportAcc::default()
            });
            reports.len() - 1
        }
    };
    &mut reports[idx]
}

/// Reads a 1/2/4-byte little-endian value as an unsigned number.
fn data_u32(data: &[u8]) -> u32 {
    match data.len() {
        1 => data[0] as u32,
        2 => u16::from_le_bytes([data[0], data[1]]) as u32,
        4 => u32::from_le_bytes([data[0], data[1], data[2], data[3]]),
        _ => 0,
    }
}

/// Reads a 1/2/4-byte little-endian value as a signed number.
fn data_i32(data: &[u8]) -> i32 {
    match data.len() {
        1 => data[0] as i8 as i32,
        2 => i16::from_le_bytes([data[0], data[1]]) as i32,
        4 => i32::from_le_bytes([data[0], data[1], data[2], data[3]]),
        _ => 0,
    }
}

/// Extracts and sign-extends one bit field from a report payload.
pub fn decode_field(payload: &[u8], field: &Field) -> i32 {
    let mut raw: u32 = 0;
    for bit in 0..field.size {
        let idx = field.offset + bit;
        if idx / 8 >= payload.len() {
            return 0;
        }
        let byte = payload[idx / 8];
        raw |= u32::from((byte >> (idx % 8)) & 1) << bit;
    }
    if field.signed && field.size < 32 && raw & (1u32 << (field.size - 1)) != 0 {
        let shift = 32 - field.size;
        ((raw as i32) << shift) >> shift
    } else {
        raw as i32
    }
}
