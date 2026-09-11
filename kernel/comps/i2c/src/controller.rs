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
        ostd::info!("probe already started");
        return;
    }
    aster_core::spawn_kernel_thread(|| {
        if init() {
            ostd::info!("touchpad active");
        } else {
            STARTED.store(false, Ordering::Release);
            ostd::info!("no touchpad activated; the probe can be started again");
        }
    });
}

fn init() -> bool {
    let Some(aml) = dsdt_aml_bytes() else {
        ostd::info!("no DSDT available, skipping touchpad probe");
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
            ostd::warn!(
                "probe {}: no relative mouse report found, skipping",
                probe_name
            );
            continue;
        };

        ostd::info!(
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

/// Normalizes a namespace path for comparison: drops the root and prefix
/// markers and strips the underscore padding of every name segment.
fn normalize_path(path: &str) -> String {
    path.trim_start_matches(['\\', '^'])
        .split('.')
        .map(|segment| segment.trim_end_matches('_'))
        .collect::<Vec<_>>()
        .join(".")
}

/// Enumerates the HID-over-I2C touchpads described by the DSDT.
///
/// Every touchpad declares its bus attachment with an `I2cSerialBusV2`
/// resource, whose slave address and resource source string name the device
/// address and the bus controller; the controller's own `_CRS` carries the
/// MMIO window of its DesignWare IP.
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
            ostd::warn!("device {} has no _CRS, skipping", device.path);
            continue;
        };

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
                "device {} has no I2C serial bus resource, skipping",
                device.path
            );
            continue;
        };

        let Ok(slave_address) = u8::try_from(slave_address) else {
            ostd::warn!(
                "device {} has an out-of-range address, skipping",
                device.path
            );
            continue;
        };

        // The touchpad hangs off the controller named by the resource source
        // string; the controller's own `_CRS` describes its MMIO window. The
        // source string uses the display form (`\_SB.I2CA`) while namespace
        // paths keep the segment padding (`\_SB__.I2CA`), so both sides are
        // normalized before comparing — Linux resolves either form through
        // `acpi_get_handle`.
        let wanted = normalize_path(&controller_path);
        let Some(controller) = all_devices.iter().find(|candidate| {
            normalize_path(&candidate.path) == wanted && candidate.crs_buffer().is_some()
        }) else {
            ostd::warn!(
                "controller {} of device {} was not found, skipping",
                controller_path,
                device.path
            );
            continue;
        };

        let Some(crs) = controller.crs_buffer() else {
            continue;
        };
        let memory = parse_resource_buffer(crs)
            .into_iter()
            .find_map(|resource| match resource {
                Resource::Memory32Fixed { base, length } => Some((base, length)),
                _ => None,
            });
        let Some((base, length)) = memory else {
            ostd::warn!(
                "controller {} has no fixed memory range, skipping",
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
