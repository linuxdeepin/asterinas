// SPDX-License-Identifier: MPL-2.0

//! The HID-over-I2C input device support.
//!
//! This component registers the HID-over-I2C driver on the I2C bus. The
//! driver takes over the devices the bus matches with it — the built-in
//! touchpads of x86 laptops, which report relative mouse movement — and
//! bridges their input reports to the input subsystem.
//!
//! Reference: the "HID over I2C Protocol Specification" and the Linux
//! `i2c-hid` driver.

#![no_std]
#![deny(unsafe_code)]
#![cfg(target_arch = "x86_64")]

#[macro_use]
extern crate alloc;

// Set this crate's log prefix for `ostd::log`.
macro_rules! __log_prefix {
    () => {
        "i2c-hid: "
    };
}

mod driver;
mod hid;
mod hid_report;
mod input;

use alloc::sync::Arc;

use component::{ComponentInitError, init_component};

#[init_component(kthread)]
fn init() -> Result<(), ComponentInitError> {
    let driver = Arc::new(driver::I2cHidDriver::new());
    if let Err(err) = aster_i2c::bus().register_driver(driver) {
        ostd::warn!("failed to register the HID-over-I2C driver: {:?}", err);
        return Err(ComponentInitError::Unknown);
    }
    Ok(())
}
