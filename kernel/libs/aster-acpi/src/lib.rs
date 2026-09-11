// SPDX-License-Identifier: MPL-2.0

//! A static parser for a subset of AML, the ACPI Machine Language.
//!
//! The parser walks the device and scope structure of an AML stream (as found
//! in the DSDT) and collects the named objects that describe hardware: the
//! hardware ID, the compatible ID, and the resource template returned by
//! `_CRS`. It never interprets control flow; methods are only understood when
//! their body simply returns a statically declared object, which is the
//! pattern firmware uses for `_CRS` on the platforms this crate targets.
//!
//! Reference: <https://download.intel.com/download/idptools/61122v004.pdf>
//! (ACPI specification, sections 19 and 20) and the Linux `drivers/acpi/`
//! implementation.

#![no_std]
#![deny(unsafe_code)]

extern crate alloc;

pub mod pkg;

use alloc::vec::Vec;

mod namespace;

pub use namespace::{AmlDevice, ObjectValue};

/// Parses an AML stream and returns the devices it declares, in declaration
/// order.
///
/// The stream is the AML payload of a DSDT or SSDT, i.e. the table bytes
/// without the SDT header. Devices whose `_STA` object reports the status
/// `0x0` are still returned; filtering by presence is the caller's decision.
pub fn devices(aml: &[u8]) -> Vec<AmlDevice> {
    namespace::parse_devices(aml)
}
