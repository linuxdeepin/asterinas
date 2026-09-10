// SPDX-License-Identifier: MPL-2.0

//! Known laptop I2C touchpad configurations.
//!
//! Asterinas does not yet enumerate ACPI platform devices, so the DesignWare
//! I2C controllers that host the built-in touchpads are described by a static
//! table here. The values are taken from the `_CRS` resource templates of the
//! corresponding ACPI nodes:
//!
//! - `\_SB.I2CA` (HYGO0010) on Hygon 3450M laptops, hosting a BLTP7853
//!   (36B6:C001) touchpad at address `0x2C`.
//! - `\_SB.I2CD` (AMDI0010) on AMD Ryzen 3500U laptops, hosting a GXTP7863
//!   (27C6:01E0) touchpad at address `0x5D`.
//!
//! The two controllers belong to different platforms, so only the one that
//! matches the running CPU is probed (see `controller::init`). Reading the
//! MMIO window of a controller that does not exist on the current platform
//! can hang the bus on AMD/Hygon chipsets, which is why the tables must never
//! be probed unconditionally.
//!
//! Once ACPI device enumeration lands, this table should be replaced by
//! resources parsed from the DSDT.

use ostd::mm::Paddr;

/// A DesignWare I2C controller together with the HID device hanging off it.
pub struct I2cControllerInfo {
    /// Human-readable name used in log messages.
    pub name: &'static str,
    /// MMIO base of the DesignWare controller.
    pub mmio_base: Paddr,
    /// Size of the controller MMIO window.
    pub mmio_size: usize,
    /// 7-bit I2C address of the HID device.
    pub address: u8,
}

/// The `HYGO0010` controller on Hygon 3450M laptops.
pub const HYGO0010_CONTROLLER: I2cControllerInfo = I2cControllerInfo {
    name: "HYGO0010 I2CA",
    mmio_base: 0xfedc_2000,
    mmio_size: 0x1000,
    address: 0x2C,
};

/// The `AMDI0010` controller on AMD Ryzen laptops.
pub const AMDI0010_CONTROLLER: I2cControllerInfo = I2cControllerInfo {
    name: "AMDI0010 I2CD",
    mmio_base: 0xfedc_5000,
    mmio_size: 0x1000,
    address: 0x5D,
};
