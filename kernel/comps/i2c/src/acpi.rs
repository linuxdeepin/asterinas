// SPDX-License-Identifier: MPL-2.0

//! ACPI enumeration of I2C controllers and their HID-over-I2C clients.
//!
//! Firmware describes both in the DSDT: a controller is an ACPI device whose
//! hardware ID names a DesignWare IP (`INT33C2`, `AMDI0010`, ...) and whose
//! `_CRS` carries the MMIO window; a client declares its bus attachment with
//! an `I2cSerialBusV2` resource in its own `_CRS`, whose slave address and
//! resource source string name the device address and the controller.
//!
//! The functions here only read the static namespace; the wiring of what they
//! find into the bus lives in the component entry.
//!
//! Reference:
//! <https://elixir.bootlin.com/linux/v6.16/source/drivers/i2c/i2c-core-acpi.c>
//! (`i2c_acpi_get_i2c_resource` for the serial bus resource and the
//! controller lookup) and the `dw_i2c_acpi_match` table in
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
    /// The normalized namespace path, e.g. `SB.I2CA`.
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
    /// The normalized namespace path of the bus controller.
    pub controller_path: String,
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

/// Returns the HID-over-I2C devices the DSDT describes, each with the slave
/// address and the controller it hangs off.
pub(crate) fn hid_over_i2c_clients(devices: &[AmlDevice]) -> Vec<Client> {
    let mut clients = Vec::new();
    for device in devices {
        if device.hid().as_deref() != Some(HID_OVER_I2C_CID)
            && !device.cids().iter().any(|id| id == HID_OVER_I2C_CID)
        {
            continue;
        }

        let Some(crs) = device.crs_buffer() else {
            ostd::warn!("ACPI device {} has no _CRS, skipping", device.path);
            continue;
        };

        // The client declares its bus attachment with an `I2cSerialBusV2`
        // resource, whose slave address and resource source string name the
        // device address and the bus controller; Linux decodes the same
        // resource in `i2c_acpi_get_i2c_resource`.
        let serial_bus =
            parse_resource_buffer(crs)
                .into_iter()
                .find_map(|resource| match resource {
                    Resource::I2cSerialBus {
                        slave_address,
                        controller_path,
                    } => Some((slave_address, controller_path)),
                    _ => None,
                });
        let Some((slave_address, controller_path)) = serial_bus else {
            ostd::warn!(
                "ACPI device {} has no I2C serial bus resource, skipping",
                device.path
            );
            continue;
        };

        let Ok(address) = u8::try_from(slave_address) else {
            ostd::warn!(
                "ACPI device {} has an out-of-range address, skipping",
                device.path
            );
            continue;
        };

        clients.push(Client {
            hid: device.hid(),
            cids: device.cids().to_vec(),
            address,
            controller_path: normalize_path(&controller_path),
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

/// Normalizes a namespace path for comparison: drops the root and prefix
/// markers and strips the underscore padding of every name segment.
///
/// The resource source string uses the display form (`\_SB.I2CA`) while
/// namespace paths keep the segment padding (`\_SB__.I2CA`), so both sides
/// are normalized before comparing — Linux resolves either form through
/// `acpi_get_handle`.
fn normalize_path(path: &str) -> String {
    path.trim_start_matches(['\\', '^'])
        .split('.')
        .map(|segment| segment.trim_end_matches('_'))
        .collect::<Vec<_>>()
        .join(".")
}
