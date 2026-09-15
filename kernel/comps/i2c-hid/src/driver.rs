// SPDX-License-Identifier: MPL-2.0

//! The HID-over-I2C driver: takes over the devices the bus matches with it.
//!
//! The driver accepts the devices whose ACPI identifiers name the HID-over-I2C
//! compatible ID. A device it takes over is only driven as a relative pointing
//! device, which is what the built-in touchpads of x86 laptops report; a
//! device without such a report is declined, and stays unbound on the bus.

use alloc::{
    string::{String, ToString},
    sync::Arc,
    vec::Vec,
};
use core::sync::atomic::{AtomicBool, Ordering};

use aster_device::{AnyDevice, BusDevice, Driver, Error, Result};
use aster_i2c::{I2cBus, I2cClient, I2cMatchData};
use ostd::sync::{LocalIrqDisabled, SpinLock};

use crate::{hid::I2cHidDevice, hid_report::parse_mouse_report, input::Bridge};

/// The compatible ID this driver accepts, as defined by the Microsoft
/// HID-over-I2C specification.
///
/// Reference: the `PNP0C50` HID is matched by `i2c_hid_acpi_match` in
/// <https://elixir.bootlin.com/linux/v6.16/source/drivers/hid/i2c-hid/i2c-hid-acpi.c>.
const HID_OVER_I2C_CID: &str = "PNP0C50";

/// The state a bound device carries while the driver holds it.
struct BoundDevice {
    /// The name of the device on the bus.
    name: String,
    /// Tells the device's polling thread to stop.
    stop: Arc<AtomicBool>,
}

/// The HID-over-I2C driver.
pub struct I2cHidDriver {
    match_data: I2cMatchData,
    // Lock order: a leaf; no lock is taken while holding it.
    bound: SpinLock<Vec<BoundDevice>, LocalIrqDisabled>,
}

impl I2cHidDriver {
    pub fn new() -> Self {
        Self {
            match_data: I2cMatchData {
                acpi_ids: &[HID_OVER_I2C_CID],
            },
            bound: SpinLock::new(Vec::new()),
        }
    }
}

impl Default for I2cHidDriver {
    fn default() -> Self {
        Self::new()
    }
}

impl Driver<I2cBus> for I2cHidDriver {
    fn name(&self) -> &str {
        "i2c_hid_acpi"
    }

    fn match_data(&self) -> &I2cMatchData {
        &self.match_data
    }

    fn probe(&self, dev: &Arc<BusDevice<I2cBus>>) -> Result<()> {
        match probe(dev.payload()) {
            Ok(bridge) => {
                self.bound.lock().push(BoundDevice {
                    name: dev.base().name().to_string(),
                    stop: bridge.stop,
                });
                Ok(())
            }
            Err(err) => {
                ostd::warn!("probe {} failed: {:?}", dev.primary_id(), err);
                Err(Error::ProbeFailed)
            }
        }
    }

    fn remove(&self, dev: &Arc<BusDevice<I2cBus>>) {
        let name = dev.base().name().to_string();
        let mut bound = self.bound.lock();
        if let Some(index) = bound.iter().position(|d| d.name == name) {
            let device = bound.remove(index);
            device.stop.store(true, Ordering::Relaxed);
        }
    }
}

/// Brings a matched device up and bridges it to the input subsystem.
///
/// The steps follow Linux's `i2c_hid_probe`: wake a device that fell asleep
/// on the bus, fetch the HID descriptor and the report descriptor, and power
/// the device on before issuing a reset.
fn probe(client: &I2cClient) -> core::result::Result<Bridge, ostd::Error> {
    let device = I2cHidDevice::probe(client.clone())?;

    let Some(layout) = parse_mouse_report(device.report_descriptor()) else {
        ostd::warn!(
            "probe {}: no relative mouse report found, declining",
            client.primary_id()
        );
        return Err(ostd::Error::IoError);
    };

    ostd::info!(
        "probe {}: vendor={:#06x} product={:#06x}, mouse report id={}, {} fields",
        client.primary_id(),
        device.descriptor().vendor_id,
        device.descriptor().product_id,
        layout.report_id,
        layout.fields.len()
    );

    Ok(Bridge::start(device, layout))
}
