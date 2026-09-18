// SPDX-License-Identifier: MPL-2.0

//! ACPI enumeration of I2C controllers and their HID-over-I2C clients.
//!
//! Firmware describes both in the DSDT: a controller is an ACPI device whose
//! hardware ID names a DesignWare IP (`INT33C2`, `AMDI0010`, ...) and whose
//! `_CRS` carries the MMIO window; a client declares its slave address with
//! an `I2cSerialBusV2` resource in its own `_CRS`, and the bus controller is
//! the client's enclosing device — exactly how Linux attaches the client:
//! firmware nests the touchpad under its I2C controller, and enumeration
//! walks the controller's children.
//!
//! The functions here only read the static namespace; the wiring of what they
//! find into the bus lives in the component entry.
//!
//! Reference:
//! <https://elixir.bootlin.com/linux/v6.16/source/drivers/i2c/i2c-core-acpi.c>
//! (`i2c_acpi_get_i2c_resource` for the serial bus resource) and the
//! `dw_i2c_acpi_match` table in
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

/// The compatible ID that HID-over-I2C devices report, as defined by the
/// Microsoft HID-over-I2C specification.
pub(crate) const HID_OVER_I2C_CID: &str = "PNP0C50";

/// An I2C controller the DSDT describes.
pub(crate) struct Controller {
    /// The namespace path, e.g. `\_SB.I2CA`.
    pub path: String,
    /// The MMIO window from `_CRS`.
    pub mmio: Option<(Paddr, usize)>,
}

/// An HID-over-I2C client the DSDT describes.
pub(crate) struct Client {
    pub hid: Option<String>,
    pub cids: Vec<String>,
    /// The 7-bit slave address from the `I2cSerialBusV2` resource.
    pub address: u8,
    /// The namespace path of the enclosing device — the bus controller the
    /// client hangs off.
    pub parent: Option<String>,
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
            path: device.path.clone(),
            mmio: controller_mmio(device),
        })
        .collect()
}

/// Returns the HID-over-I2C devices the DSDT describes, each with the slave
/// address of its serial bus resource and its enclosing device.
pub(crate) fn hid_over_i2c_clients(devices: &[AmlDevice]) -> Vec<Client> {
    let mut clients = Vec::new();
    for device in devices {
        if device.hid().as_deref() != Some(HID_OVER_I2C_CID)
            && !device.cids().iter().any(|id| id == HID_OVER_I2C_CID)
        {
            continue;
        }

        let Some(crs) = device.crs_buffer() else {
            ostd::warn!("touchpad {} has no _CRS buffer, skipping", device.path);
            continue;
        };

        // The client declares its slave address with an `I2cSerialBusV2`
        // resource; Linux decodes the same resource in
        // `i2c_acpi_get_i2c_resource`. The bus controller is not taken from
        // the resource source string: the enclosing device carries the same
        // bus attachment, in the namespace's own terms.
        let Some((slave_address, ..)) =
            parse_resource_buffer(crs)
                .into_iter()
                .find_map(|resource| match resource {
                    Resource::I2cSerialBus { slave_address, .. } => Some((slave_address, ())),
                    _ => None,
                })
        else {
            ostd::warn!(
                "touchpad {}: no I2cSerialBusV2 resource found in {} bytes of _CRS, skipping",
                device.path,
                crs.len()
            );
            continue;
        };

        let Ok(address) = u8::try_from(slave_address) else {
            ostd::warn!(
                "touchpad {} has an out-of-range address, skipping",
                device.path
            );
            continue;
        };

        clients.push(Client {
            hid: device.hid(),
            cids: device.cids().to_vec(),
            address,
            parent: device.parent.clone(),
        });
    }
    clients
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
