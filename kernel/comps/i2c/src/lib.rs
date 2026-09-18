// SPDX-License-Identifier: MPL-2.0

//! I2C bus and DesignWare controller support on the device model.
//!
//! This component registers the I2C bus — the device model's first real bus —
//! and populates it from the ACPI namespace: it creates one adapter per
//! DesignWare controller the DSDT describes and one client per HID-over-I2C
//! device, which the bus then matches with the registered drivers.
//!
//! Reference: the "HID over I2C Protocol Specification" and the Linux
//! `i2c-designware` / `i2c-hid` drivers.

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
use crate::dw::DesignWareI2c;

#[init_component(kthread)]
fn init() -> Result<(), ComponentInitError> {
    if let Err(err) = register() {
        ostd::warn!("failed to register the I2C bus: {:?}", err);
        return Err(ComponentInitError::Unknown);
    }
    // The enumeration runs in its own thread: probing a device sleeps while
    // waiting for it to come out of deep sleep, and the devices are only
    // present on machines whose DSDT describes them.
    aster_core::spawn_kernel_thread(enumerate);
    Ok(())
}

/// Brings up the controllers and clients the DSDT describes.
fn enumerate() {
    let Some(aml) = ostd::arch::kernel::dsdt_aml_bytes() else {
        ostd::info!("no DSDT available, skipping I2C enumeration");
        return;
    };
    let devices = aster_acpi::devices(aml);

    let adapters = bring_up_controllers(&devices);
    register_clients(&devices, &adapters);
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
        match DesignWareI2c::new(&controller.path, base, size) {
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

/// Adds every HID-over-I2C client to the bus hanging off its controller.
///
/// A client whose controller did not come up is skipped with a warning, as
/// Linux skips devices whose adapter never registered. Adding a client offers
/// it to the registered drivers; the first driver whose match data names one
/// of the client's ACPI identifiers probes it.
fn register_clients(devices: &[AmlDevice], adapters: &[(String, Arc<I2cAdapter>)]) {
    for client in acpi::hid_over_i2c_clients(devices) {
        let Some((_, adapter)) = adapters
            .iter()
            .find(|(path, _)| Some(path.as_str()) == client.parent.as_deref())
        else {
            ostd::warn!(
                "touchpad at {:#04x} hangs off controller {:?}, which did not come up, skipping",
                client.address,
                client.parent
            );
            continue;
        };
        let payload = I2cClient::new(
            adapter.clone(),
            client.address,
            client.hid.clone(),
            client.cids.clone(),
        );
        match add_client(adapter, payload) {
            Ok(device) => ostd::info!(
                "added {} at {:#04x} on i2c-{} ({})",
                device.primary_id(),
                device.address(),
                adapter.number(),
                device.modalias()
            ),
            Err(err) => ostd::warn!(
                "client at {:#04x} on i2c-{} failed to register: {:?}",
                client.address,
                adapter.number(),
                err
            ),
        }
    }
}
