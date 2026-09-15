// SPDX-License-Identifier: MPL-2.0

//! The I2C bus on the device model.
//!
//! An I2C bus is the set of devices reachable through the masters that drive
//! its controllers. A device on the bus is an [`I2cClient`]: a slave address
//! together with the ACPI identifiers firmware gave it. A driver declares the
//! identifiers it accepts in an [`I2cMatchData`], and the bus binds the first
//! driver whose set names one of the client's identifiers, mirroring how
//! Linux's `i2c_bus_type` matches `i2c_client` devices against the ACPI ids
//! of `i2c_driver`s.
//!
//! The controller behind a bus segment is an [`I2cAdapter`]: a device model
//! [`BareDevice`] under `/sys/devices` that parents the clients hanging off
//! it and serializes their transfers, as Linux's `i2c_adapter` bus lock does.
//!
//! Reference: <https://elixir.bootlin.com/linux/v6.16/source/drivers/i2c/i2c-core-base.c>
//! (`i2c_device_match`, `i2c_device_probe`) and
//! <https://elixir.bootlin.com/linux/v6.16/source/drivers/i2c/i2c-core-acpi.c>
//! (how ACPI identifiers reach the match).

use alloc::{string::String, sync::Arc, vec::Vec};

use aster_device::{
    Attr, BareDevice, Bus, BusDevice, BusHandle, Result, UeventVars, add, register_bus,
};
use ostd::sync::Mutex;

/// A single I2C message (the analogue of an `i2c_msg`).
pub enum I2cMessage<'a> {
    /// Write the given bytes to the bus.
    Write(&'a [u8]),
    /// Read `buf.len()` bytes from the bus.
    Read(&'a mut [u8]),
}

/// A master able to drive transfers to the slaves of one controller.
///
/// Implementations drive one physical controller; the bus knows them only
/// through this interface, so a controller with a different IP needs a new
/// implementation, not a new bus.
pub trait I2cMaster: Send + Sync + 'static {
    /// Performs the given write and read transactions against the slave at
    /// `address`, in order.
    fn transfer(&self, address: u8, msgs: &mut [I2cMessage]) -> ostd::Result<()>;
}

/// The controller that a set of [`I2cClient`]s hang off.
///
/// Transfers through one adapter are serialized: the slaves of a controller
/// share the wires, so two clients must not be driven at once.
pub struct I2cAdapter {
    /// The adapter number, naming the device as `i2c-<number>`.
    number: usize,
    /// The device under `/sys/devices` that parents this adapter's clients.
    device: Arc<BareDevice>,
    master: Arc<dyn I2cMaster>,
    /// Serializes transfers through `master`.
    //
    // Lock order: this lock is a leaf; no lock is taken while holding it.
    transfer_lock: Mutex<()>,
}

impl I2cAdapter {
    /// Creates an adapter driving `master` and adds its device, named
    /// `i2c-<number>`, under `/sys/devices`.
    pub fn new(number: usize, master: Arc<dyn I2cMaster>) -> Result<Arc<Self>> {
        let device = BareDevice::new_root(alloc::format!("i2c-{}", number));
        add(&device)?;
        Ok(Arc::new(Self {
            number,
            device,
            master,
            transfer_lock: Mutex::new(()),
        }))
    }

    /// The adapter number.
    pub fn number(&self) -> usize {
        self.number
    }

    /// The device under `/sys/devices` that parents this adapter's clients.
    pub fn device(&self) -> &Arc<BareDevice> {
        &self.device
    }

    /// Performs the given transactions against the slave at `address`.
    pub fn transfer(&self, address: u8, msgs: &mut [I2cMessage]) -> ostd::Result<()> {
        let _guard = self.transfer_lock.lock();
        self.master.transfer(address, msgs)
    }
}

/// What every device on the I2C bus carries.
#[derive(Clone)]
pub struct I2cClient {
    adapter: Arc<I2cAdapter>,
    /// The 7-bit slave address.
    address: u8,
    /// The hardware ID from `_HID`, if the device declares one.
    hid: Option<String>,
    /// The compatible IDs from `_CID`.
    cids: Vec<String>,
}

impl I2cClient {
    /// Builds a client payload from the identifiers of an ACPI device.
    pub fn new(
        adapter: Arc<I2cAdapter>,
        address: u8,
        hid: Option<String>,
        cids: Vec<String>,
    ) -> Self {
        Self {
            adapter,
            address,
            hid,
            cids,
        }
    }

    /// The adapter this client hangs off.
    pub fn adapter(&self) -> &Arc<I2cAdapter> {
        &self.adapter
    }

    /// Performs the given transactions against this client.
    pub fn transfer(&self, msgs: &mut [I2cMessage]) -> ostd::Result<()> {
        self.adapter.transfer(self.address, msgs)
    }

    /// The 7-bit slave address.
    pub fn address(&self) -> u8 {
        self.address
    }

    /// Every ACPI identifier the device reports: the hardware ID followed by
    /// the compatible IDs.
    pub fn acpi_ids(&self) -> impl Iterator<Item = &str> {
        self.hid
            .iter()
            .map(String::as_str)
            .chain(self.cids.iter().map(String::as_str))
    }

    /// The identifier that stands for the device: the hardware ID, or the
    /// first compatible ID if there is no hardware ID.
    ///
    /// Reference: `acpi_device_hid` in
    /// <https://elixir.bootlin.com/linux/v6.16/source/drivers/acpi/scan.c>.
    pub fn primary_id(&self) -> &str {
        self.hid
            .as_deref()
            .or_else(|| self.cids.first().map(String::as_str))
            .unwrap_or("unknown")
    }

    /// The modalias user space matches drivers with.
    pub fn modalias(&self) -> String {
        alloc::format!("acpi:{}", self.primary_id())
    }
}

/// What an I2C driver declares: the ACPI identifiers it accepts.
#[derive(Clone, Copy)]
pub struct I2cMatchData {
    /// The identifiers that select a device, as `_HID` or `_CID` values.
    pub acpi_ids: &'static [&'static str],
}

/// The attributes every I2C client shows.
const I2C_DEV_ATTRS: &[Attr<BusDevice<I2cBus>>] = &[
    Attr::ro("name", |dev, w| {
        writeln!(w, "{}", dev.primary_id())?;
        Ok(())
    }),
    Attr::ro("modalias", |dev, w| {
        writeln!(w, "{}", dev.modalias())?;
        Ok(())
    }),
];

/// The I2C bus.
pub struct I2cBus;

impl Bus for I2cBus {
    const NAME: &'static str = "i2c";
    type Device = I2cClient;
    type MatchData = I2cMatchData;

    fn matches(&self, dev: &I2cClient, data: &I2cMatchData) -> bool {
        data.acpi_ids
            .iter()
            .any(|id| dev.acpi_ids().any(|dev_id| dev_id == *id))
    }

    fn dev_attrs(&self) -> &'static [Attr<BusDevice<Self>>] {
        I2C_DEV_ATTRS
    }

    fn uevent(&self, dev: &BusDevice<Self>, vars: &mut UeventVars) {
        vars.add("MODALIAS", core::format_args!("{}", dev.modalias()));
    }
}

/// The registered I2C bus.
static BUS: spin::Once<Arc<BusHandle<I2cBus>>> = spin::Once::new();

/// Registers the I2C bus, creating `/sys/bus/i2c`.
///
/// The model keeps one bus of a name, so the first call registers the bus and
/// any later call returns the registered one.
pub fn register() -> Result<Arc<BusHandle<I2cBus>>> {
    if let Some(bus) = BUS.get() {
        return Ok(bus.clone());
    }
    let bus = register_bus(I2cBus)?;
    Ok(BUS.call_once(|| bus).clone())
}

/// Returns the registered I2C bus.
///
/// The bus is registered at component initialization, which precedes the
/// initialization of every component depending on this one.
pub fn bus() -> &'static Arc<BusHandle<I2cBus>> {
    BUS.get().expect("the I2C bus is not registered")
}

/// Adds a client device hanging off `adapter` to the bus.
///
/// The device is named after the adapter number and the slave address, which
/// makes its name in `/sys/bus/i2c/devices` unique across adapters, as Linux's
/// client bus ids (`0-0050`) are. Linux derives the same format from
/// `dev_set_name(&client->dev, "%d-%04x", ...)` in
/// <https://elixir.bootlin.com/linux/v6.16/source/drivers/i2c/i2c-core-base.c>.
pub fn add_client(adapter: &Arc<I2cAdapter>, payload: I2cClient) -> Result<Arc<BusDevice<I2cBus>>> {
    let name = alloc::format!("{}-{:04x}", adapter.number(), payload.address());
    let device = BusDevice::builder(bus(), name, payload)
        .parent(adapter.device().clone())
        .build();
    add(&device)?;
    Ok(device)
}
