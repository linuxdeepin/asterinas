// SPDX-License-Identifier: MPL-2.0

//! ACPI enumeration of I2C controllers.
//!
//! Firmware describes a controller in the DSDT as an ACPI device whose
//! hardware ID names a DesignWare IP (`INT33C2`, `AMDI0010`, ...) and whose
//! `_CRS` carries the MMIO window.
//!
//! The functions here only read the static namespace; the wiring of what they
//! find into the bus lives in the component entry.
//!
//! Reference: the `dw_i2c_acpi_match` table in
//! <https://elixir.bootlin.com/linux/v6.16/source/drivers/i2c/busses/i2c-designware-platdrv.c>.

use alloc::{string::String, vec::Vec};

use aster_acpi::{AmlDevice, Resource, parse_resource_buffer};
use ostd::mm::Paddr;

/// The hardware IDs that select a DesignWare I2C controller, from Linux's
/// `dw_i2c_acpi_match`.
const DESIGNWARE_HIDS: &[&str] = &[
    "80860F41", "808622C1", "AMD0010", "AMDI0010", "AMDI0019", "AMDI0510", "APMC0D0F", "FUJI200B",
    "GOOG5000", "HISI02A1", "HISI02A2", "HISI02A3", "HJMC3001", "HYGO0010", "INT33C2", "INT33C3",
    "INT3432", "INT3433", "INTC10EF", "LECA0003",
];

/// An I2C controller the DSDT describes.
pub(crate) struct Controller {
    /// The normalized namespace path, e.g. `SB.I2CA`.
    pub path: String,
    /// The MMIO window from `_CRS`.
    pub mmio: Option<(Paddr, usize)>,
}

/// Returns the devices whose hardware ID names a DesignWare controller.
///
/// Only controllers the namespace actually declares are returned, so the
/// component never touches the MMIO of a machine that does not have the
/// controller: reading the registers of a machine without the controller
/// hangs the bus.
pub(crate) fn designware_controllers(devices: &[AmlDevice]) -> Vec<Controller> {
    devices
        .iter()
        .filter(|device| {
            device
                .hid()
                .is_some_and(|hid| DESIGNWARE_HIDS.contains(&hid.as_str()))
        })
        .map(|device| Controller {
            path: normalize_path(&device.path),
            mmio: controller_mmio(device),
        })
        .collect()
}

/// Returns the MMIO window of a controller's `_CRS`.
fn controller_mmio(device: &AmlDevice) -> Option<(Paddr, usize)> {
    let crs = device.crs_buffer()?;
    parse_resource_buffer(crs)
        .into_iter()
        .find_map(|resource| match resource {
            Resource::Memory32Fixed { base, length } => Some((base as Paddr, length as usize)),
            _ => None,
        })
}

/// Normalizes a namespace path for comparison: drops the root and prefix
/// markers and strips the underscore padding of every name segment.
fn normalize_path(path: &str) -> String {
    path.trim_start_matches(['\\', '^'])
        .split('.')
        .map(|segment| segment.trim_end_matches('_'))
        .collect::<Vec<_>>()
        .join(".")
}
