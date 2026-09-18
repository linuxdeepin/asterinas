// SPDX-License-Identifier: MPL-2.0

//! Component initialization: enumerates the HID-over-I2C touchpad from the
//! DSDT and starts the polling thread for the first one found.

use alloc::{string::String, vec::Vec};
use core::sync::atomic::{AtomicBool, Ordering};

use aster_acpi::{Resource, devices, parse_resource_buffer};
use ostd::{arch::kernel::dsdt_aml_bytes, mm::Paddr};

use crate::{hid_report::parse_mouse_report, i2c_hid::I2cHidDevice};

/// The compatible ID that HID-over-I2C devices report in `_CID`, as defined
/// by the Microsoft HID-over-I2C specification.
///
/// Reference:
/// <https://elixir.bootlin.com/linux/v6.16/source/drivers/i2c/i2c-core-acpi.c>
/// (the `PNP0C50` HID is matched by `i2c_hid_acpi_match`, and the
/// `I2cSerialBusV2` resource is decoded by `i2c_acpi_get_i2c_resource`).
const HID_OVER_I2C_CID: &str = "PNP0C50";

/// Whether the probe has already been started.
static STARTED: AtomicBool = AtomicBool::new(false);

/// An I2C controller together with the HID-over-I2C device hanging off it,
/// as described by the DSDT.
struct TouchpadSlot {
    controller_name: String,
    mmio_base: Paddr,
    mmio_size: usize,
    device_address: u8,
}

/// Starts the touchpad probe.
///
/// The probe runs in a fresh kernel thread. A probe that ends without an
/// active touchpad releases [`STARTED`] so that the probe can be started
/// again, e.g. after touching the touchpad, which may wake a deep-sleeping
/// device.
pub(super) fn start() {
    if STARTED.swap(true, Ordering::AcqRel) {
        ostd::debug!("the touchpad probe is already running");
        return;
    }
    aster_core::spawn_kernel_thread(|| {
        if init() {
            ostd::info!("touchpad active");
        } else {
            STARTED.store(false, Ordering::Release);
            ostd::debug!("no touchpad activated; the probe can be started again");
        }
    });
}

fn aml_len() -> usize {
    dsdt_aml_bytes().map_or(0, <[u8]>::len)
}

fn init() -> bool {
    let Some(aml) = dsdt_aml_bytes() else {
        ostd::debug!(
            "no DSDT available ({} bytes total), skipping touchpad probe",
            aml_len()
        );
        return false;
    };

    let mut activated = false;
    for slot in enumerate(aml) {
        ostd::info!(
            "probing {} at {:#04x} (mmio {:?}, size {:#x})",
            slot.controller_name,
            slot.device_address,
            slot.mmio_base,
            slot.mmio_size
        );
        let probe_name = slot.controller_name.as_str();
        let device = match I2cHidDevice::probe(
            probe_name,
            slot.mmio_base,
            slot.mmio_size,
            slot.device_address,
        ) {
            Ok(device) => device,
            Err(err) => {
                ostd::warn!("probe {} failed: {:?}", probe_name, err);
                continue;
            }
        };

        let Some(layout) = parse_mouse_report(device.report_descriptor()) else {
            ostd::debug!(
                "probe {}: no relative mouse report found, skipping",
                probe_name
            );
            continue;
        };

        ostd::debug!(
            "probe {}: vendor={:#06x} product={:#06x}, mouse report id={}, {} fields",
            probe_name,
            device.descriptor().vendor_id,
            device.descriptor().product_id,
            layout.report_id,
            layout.fields.len()
        );

        if crate::input_dev::init(device, layout) {
            ostd::info!("{} is active", probe_name);
            activated = true;
            break;
        }
    }
    activated
}

/// Enumerates the HID-over-I2C touchpads described by the DSDT.
///
/// Every touchpad declares its bus attachment with an `I2cSerialBusV2`
/// resource, whose slave address names the device address; the bus
/// controller is the touchpad's enclosing device, and its own `_CRS`
/// carries the MMIO window of the DesignWare IP.
///
/// Reference:
/// <https://elixir.bootlin.com/linux/v6.16/source/drivers/i2c/i2c-core-acpi.c>
/// (`i2c_acpi_get_i2c_resource` for the serial bus resource and the
/// controller lookup).
fn enumerate(aml: &[u8]) -> Vec<TouchpadSlot> {
    let all_devices = devices(aml);

    let mut slots = Vec::new();
    for device in &all_devices {
        if device.hid().as_deref() != Some(HID_OVER_I2C_CID)
            && !device.cids().iter().any(|id| id == HID_OVER_I2C_CID)
        {
            continue;
        }

        let Some(crs) = device.crs_buffer() else {
            ostd::warn!("touchpad {} has no _CRS buffer, skipping", device.path);
            continue;
        };

        let Some((slave_address, controller_path)) = parse_resource_buffer(crs)
            .into_iter()
            .find_map(|resource| match resource {
                Resource::I2cSerialBus {
                    slave_address,
                    controller_path,
                } => Some((slave_address, controller_path)),
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

        let Ok(slave_address) = u8::try_from(slave_address) else {
            ostd::warn!(
                "touchpad {} has an out-of-range address, skipping",
                device.path
            );
            continue;
        };

        // The controller that owns the touchpad is its enclosing device,
        // exactly how Linux attaches the client: firmware nests the
        // touchpad under its I2C controller, and enumeration walks the
        // controller's children.
        let controller = device.parent.as_deref().and_then(|parent_path| {
            all_devices
                .iter()
                .find(|candidate| candidate.path == parent_path)
        });
        let Some(controller) = controller.filter(|candidate| candidate.crs_buffer().is_some())
        else {
            ostd::warn!(
                "touchpad {}: controller {} not found among the DSDT devices, skipping",
                device.path,
                controller_path
            );
            continue;
        };

        let Some(crs) = controller.crs_buffer() else {
            continue;
        };
        let Some((base, length)) =
            parse_resource_buffer(crs)
                .into_iter()
                .find_map(|resource| match resource {
                    Resource::Memory32Fixed { base, length } => Some((base, length)),
                    _ => None,
                })
        else {
            ostd::warn!(
                "controller {} has no Memory32Fixed in _CRS, skipping",
                controller.path
            );
            continue;
        };

        slots.push(TouchpadSlot {
            controller_name: controller.hid().unwrap_or_else(|| controller.path.clone()),
            mmio_base: base as Paddr,
            mmio_size: length as usize,
            device_address: slave_address,
        });
    }
    slots
}
