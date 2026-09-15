// SPDX-License-Identifier: MPL-2.0

//! I2C bus support on the device model.
//!
//! This component registers the I2C bus — the device model's first real bus.
//! The devices on it are created by the enumeration that later commits hang
//! off this entry.

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

mod bus;

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
    Ok(())
}
