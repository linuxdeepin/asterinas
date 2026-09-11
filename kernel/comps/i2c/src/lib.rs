// SPDX-License-Identifier: MPL-2.0

//! I2C bus and HID-over-I2C input device support.
//!
//! This component provides a polled DesignWare I2C master driver together
//! with a minimal HID-over-I2C client. It is currently used to drive the
//! built-in touchpads of x86 laptops that report relative mouse movement.
//!
//! Reference: the "HID over I2C Protocol Specification" and the Linux
//! `i2c-designware` / `i2c-hid` drivers.

#![no_std]
#![deny(unsafe_code)]
#![cfg(target_arch = "x86_64")]

#[macro_use]
extern crate alloc;

// Set this crate's log prefix for `ostd::log`.
macro_rules! __log_prefix {
    () => {
        "i2c: "
    };
}

mod controller;
mod dw;
mod hid_report;
mod i2c_hid;
mod input_dev;

use component::{ComponentInitError, init_component};

#[init_component(kthread)]
fn init() -> Result<(), ComponentInitError> {
    controller::start();
    Ok(())
}
