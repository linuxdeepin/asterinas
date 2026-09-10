// SPDX-License-Identifier: MPL-2.0

//! Component initialization: probes the controller that matches the running
//! CPU and activates the touchpad if it responds.

use core::{
    arch::x86_64::CpuidResult,
    sync::atomic::{AtomicBool, Ordering},
};

use ostd::arch::cpu::cpuid;

use crate::{
    device::{AMDI0010_CONTROLLER, HYGO0010_CONTROLLER, I2cControllerInfo},
    hid_report::parse_mouse_report,
    i2c_hid::I2cHidDevice,
};

/// The CPU vendor, which selects which platform's controller table to probe.
///
/// Asterinas does not yet enumerate ACPI platform devices, so the platform
/// is identified by the CPUID vendor string instead: Hygon Dhyana reports
/// `HygonGenuine` and AMD reports `AuthenticAMD`. Only the controller that
/// exists on the current platform is probed, because reading the MMIO window
/// of a controller present on a different platform can hang the bus on
/// AMD/Hygon chipsets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CpuVendor {
    Amd,
    Hygon,
}

impl CpuVendor {
    fn parse(s: &str) -> Option<Self> {
        match s {
            "AuthenticAMD" => Some(Self::Amd),
            "HygonGenuine" => Some(Self::Hygon),
            _ => None,
        }
    }
}

/// Reads the CPUID vendor string of leaf 0.
fn current_cpu_vendor() -> Option<CpuVendor> {
    let CpuidResult { ebx, edx, ecx, .. } = cpuid::cpuid(0, 0)?;
    let mut vendor = [0u8; 12];
    vendor[..4].copy_from_slice(&ebx.to_le_bytes());
    vendor[4..8].copy_from_slice(&edx.to_le_bytes());
    vendor[8..12].copy_from_slice(&ecx.to_le_bytes());
    CpuVendor::parse(core::str::from_utf8(&vendor).ok()?)
}

/// Whether the probe has already been started.
static STARTED: AtomicBool = AtomicBool::new(false);

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
    let Some(vendor) = current_cpu_vendor() else {
        ostd::info!("no known I2C platform (CPU vendor unsupported), skipping touchpad probe");
        return false;
    };
    let controllers: &[I2cControllerInfo] = match vendor {
        CpuVendor::Amd => &[AMDI0010_CONTROLLER],
        CpuVendor::Hygon => &[HYGO0010_CONTROLLER],
    };
    ostd::info!("CPU vendor: {:?}", vendor);

    let mut activated = false;
    for info in controllers {
        ostd::info!(
            "probing {}: mmio={:?} size={:#x} address={:#04x}",
            info.name,
            info.mmio_base,
            info.mmio_size,
            info.address
        );
        let device =
            match I2cHidDevice::probe(info.name, info.mmio_base, info.mmio_size, info.address) {
                Ok(device) => device,
                Err(err) => {
                    ostd::warn!("probe {} failed: {:?}", info.name, err);
                    continue;
                }
            };

        let Some(layout) = parse_mouse_report(device.report_descriptor()) else {
            ostd::warn!(
                "probe {}: no relative mouse report found, skipping",
                info.name
            );
            continue;
        };

        ostd::info!(
            "probe {}: vendor={:#06x} product={:#06x}, mouse report id={}, {} fields",
            info.name,
            device.descriptor().vendor_id,
            device.descriptor().product_id,
            layout.report_id,
            layout.fields.len()
        );

        if crate::input_dev::init(device, layout) {
            ostd::info!("{} is active", info.name);
            activated = true;
            break;
        }
    }
    activated
}
