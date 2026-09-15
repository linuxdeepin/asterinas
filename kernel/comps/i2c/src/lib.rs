// SPDX-License-Identifier: MPL-2.0

//! I2C bus and DesignWare controller support on the device model.
//!
//! This component registers the I2C bus — the device model's first real bus —
//! and populates it from the ACPI namespace: it creates one adapter per
//! DesignWare controller the DSDT describes.
//!
//! Reference: the "HID over I2C Protocol Specification" and the Linux
//! `i2c-designware` driver.

#![no_std]
#![deny(unsafe_code)]
#![cfg(target_arch = "x86_64")]

extern crate alloc;

// Set this crate's log prefix for `ostd::log`.
macro_rules! __log_prefix {
    () => {
        "i2c: "
    };
}

mod acpi;
mod bus;
mod dw;

use alloc::{string::String, sync::Arc, vec::Vec};

use aster_acpi::AmlDevice;
use component::{ComponentInitError, init_component};

pub use self::bus::{
    I2cAdapter, I2cBus, I2cClient, I2cMaster, I2cMatchData, I2cMessage, add_client, bus, register,
};

#[init_component(kthread)]
fn init() -> Result<(), ComponentInitError> {
    if let Err(err) = register() {
        ostd::warn!("failed to register the I2C bus: {:?}", err);
        return Err(ComponentInitError::Unknown);
    }
    // The enumeration runs in its own thread: mapping a controller and
    // reading its registers must not delay the other components, and the
    // controllers are only present on machines whose DSDT describes them.
    aster_core::spawn_kernel_thread(enumerate);
    Ok(())
}

/// Brings up the controllers the DSDT describes.
fn enumerate() {
    let Some(aml) = ostd::arch::kernel::dsdt_aml_bytes() else {
        ostd::info!("no DSDT available, skipping I2C enumeration");
        return;
    };
    let devices = aster_acpi::devices(aml);

    bring_up_controllers(&devices);
}

/// Creates one adapter per DesignWare controller the DSDT describes.
fn bring_up_controllers(devices: &[AmlDevice]) -> Vec<(String, Arc<I2cAdapter>)> {
    let mut adapters = Vec::new();
    for controller in acpi::designware_controllers(devices) {
        let Some((base, size)) = controller.mmio else {
            ostd::warn!(
                "controller {} has no fixed memory range, skipping",
                controller.path
            );
            continue;
        };
        match dw::DesignWareI2c::new(&controller.path, base, size) {
            Ok(master) => match I2cAdapter::new(adapters.len(), Arc::new(master)) {
                Ok(adapter) => adapters.push((controller.path, adapter)),
                Err(err) => ostd::warn!(
                    "controller {} failed to register: {:?}",
                    controller.path,
                    err
                ),
            },
            Err(err) => ostd::warn!(
                "controller {} failed to come up: {:?}",
                controller.path,
                err
            ),
        }
    }
    adapters
}
