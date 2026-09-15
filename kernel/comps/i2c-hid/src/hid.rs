// SPDX-License-Identifier: MPL-2.0

//! The HID-over-I2C protocol: probing a device and reading its reports.
//!
//! Reference: the "HID over I2C Protocol Specification" and the Linux
//! `i2c-hid` driver.

use alloc::vec::Vec;
use core::time::Duration;

use aster_i2c::{I2cClient, I2cMessage};
use ostd::{Error, Result};

/// Fixed register holding the HID descriptor (per the HID-over-I2C spec).
const HID_DESCRIPTOR_REGISTER: u16 = 0x20;

/// Command opcodes.
const OPCODE_RESET: u8 = 0x01;
const OPCODE_SET_POWER: u8 = 0x08;

/// Power states.
const PWR_ON: u8 = 0x00;

/// Upper bound on the report-descriptor and input-report buffers we are
/// willing to allocate.
const MAX_DESCRIPTOR_LENGTH: usize = 4096;

/// The pause between the attempts of the wake-up sequence, in milliseconds.
/// Each retry doubles it.
const STEP_PAUSE_MILLIS: u64 = 1;

/// The settle period after a PWR_ON or RESET request, in milliseconds.
const SETTLE_PAUSE_MILLIS: u64 = 10;

/// Fallback read size when `wMaxInputLength` is zero or implausible.
///
/// Linux does not rely on `wMaxInputLength` alone (some devices report a
/// wrong value), so we take the larger of it and this floor.
const MIN_INPUT_READ_LENGTH: usize = 64;

/// The 30-byte HID descriptor returned by the fixed descriptor register.
///
/// Field offsets follow the "HID over I2C Protocol Specification".
#[derive(Clone, Copy, Debug)]
pub struct HidDescriptor {
    /// `bcdVersion`; must be 1.00.
    pub bcd_version: u16,
    /// Length of the report descriptor (`wReportDescLength`).
    pub report_desc_length: u16,
    /// Register holding the report descriptor (`wReportDescRegister`).
    pub report_desc_register: u16,
    /// Register from which input reports are read (`wInputRegister`).
    pub input_register: u16,
    /// Maximum size of an input report (`wMaxInputLength`).
    pub max_input_length: u16,
    /// Register for HID commands (`wCommandRegister`).
    pub command_register: u16,
    pub vendor_id: u16,
    pub product_id: u16,
}

impl HidDescriptor {
    /// Parses the 30-byte little-endian HID descriptor.
    pub fn parse(buf: &[u8]) -> Self {
        let u16_at = |off: usize| u16::from_le_bytes([buf[off], buf[off + 1]]);
        Self {
            bcd_version: u16_at(2),
            report_desc_length: u16_at(4),
            report_desc_register: u16_at(6),
            input_register: u16_at(8),
            max_input_length: u16_at(10),
            command_register: u16_at(16),
            vendor_id: u16_at(20),
            product_id: u16_at(22),
        }
    }
}

/// A probed HID-over-I2C device hanging off an I2C bus adapter.
pub struct I2cHidDevice {
    client: I2cClient,
    desc: HidDescriptor,
    report_desc: Vec<u8>,
    /// Number of bytes to read per input poll.
    read_len: usize,
}

impl I2cHidDevice {
    /// Probes the HID descriptor and report descriptor of `client`, then
    /// powers the device on and issues a reset.
    pub fn probe(client: I2cClient) -> Result<Self> {
        let address = client.address();
        ostd::info!("probe: target address {:#04x}", address);

        // Linux first issues a plain read to wake devices that fall asleep on
        // the bus after power-on, retrying after a short pause
        // (`i2c_hid_probe_address`). Do the same so that the very first
        // descriptor read below is not the one that fails spuriously.
        //
        // A device that has been idle since boot may be in deep sleep: it ACKs
        // its address but returns no data. Knock with a write of the descriptor
        // register pointer (a plain write transaction) between retries and
        // back off progressively, giving the device time to wake up.
        let mut wake = [0u8; 2];
        let mut wake_ok = plain_read(&client, &mut wake).is_ok();
        for attempt in 0..4u32 {
            if wake_ok {
                break;
            }
            let knock = HID_DESCRIPTOR_REGISTER.to_le_bytes();
            let mut knock_msgs = [I2cMessage::Write(&knock)];
            let _ = client.transfer(&mut knock_msgs);
            aster_core::sleep(Duration::from_millis(STEP_PAUSE_MILLIS << attempt));
            ostd::info!(
                "probe: wake retry {} after a register-pointer knock",
                attempt + 1
            );
            wake_ok = plain_read(&client, &mut wake).is_ok();
        }
        if !wake_ok {
            ostd::warn!(
                "probe: no response from {:#04x} after the wake-up read",
                address
            );
            return Err(Error::IoError);
        }

        let mut desc_buf = [0u8; 30];
        read_register(&client, HID_DESCRIPTOR_REGISTER, &mut desc_buf)?;
        let desc = HidDescriptor::parse(&desc_buf);
        ostd::info!(
            "probe: HID descriptor: descLen={} bcdVersion={:#06x} rptDescLen={} rptDescReg={:#06x} inReg={:#06x} maxInLen={} cmdReg={:#06x} vendor={:#06x} product={:#06x}",
            u16_at(&desc_buf, 0),
            desc.bcd_version,
            desc.report_desc_length,
            desc.report_desc_register,
            desc.input_register,
            desc.max_input_length,
            desc.command_register,
            desc.vendor_id,
            desc.product_id,
        );

        // Validate the descriptor before trusting any of its fields, matching
        // Linux's checks in `i2c_hid_fetch_hid_descriptor`.
        if desc.bcd_version != 0x0100 {
            ostd::warn!(
                "probe: unexpected HID descriptor bcdVersion {:#06x}",
                desc.bcd_version
            );
            return Err(Error::IoError);
        }
        if desc.report_desc_length == 0 || desc.report_desc_length as usize > MAX_DESCRIPTOR_LENGTH
        {
            ostd::warn!(
                "probe: implausible report descriptor length {}",
                desc.report_desc_length
            );
            return Err(Error::IoError);
        }

        let mut report_desc = vec![0u8; desc.report_desc_length as usize];
        read_register(&client, desc.report_desc_register, &mut report_desc)?;
        ostd::info!("probe: report descriptor: {} bytes", report_desc.len());

        set_power(&client, desc.command_register, PWR_ON)?;
        // The HID-over-I2C spec allows devices to need time after a PWR_ON or
        // RESET request before they respond again (Windows drives add a 1ms
        // delay, some vendors more). Give the device a short settle period.
        aster_core::sleep(Duration::from_millis(SETTLE_PAUSE_MILLIS));
        reset(&client, desc.command_register)?;
        aster_core::sleep(Duration::from_millis(SETTLE_PAUSE_MILLIS));

        let read_len =
            (desc.max_input_length as usize).clamp(MIN_INPUT_READ_LENGTH, MAX_DESCRIPTOR_LENGTH);
        ostd::info!(
            "probe: input read size = {} bytes (wMaxInputLength={})",
            read_len,
            desc.max_input_length
        );

        Ok(Self {
            client,
            desc,
            report_desc,
            read_len,
        })
    }

    /// The raw HID report descriptor.
    pub fn report_descriptor(&self) -> &[u8] {
        &self.report_desc
    }

    /// The parsed HID descriptor.
    pub fn descriptor(&self) -> HidDescriptor {
        self.desc
    }

    /// Reads one pending input report.
    ///
    /// Returns `Ok(None)` when the device has no pending report, which makes
    /// this method safe to call in a polling loop.
    pub fn read_input(&mut self) -> Result<Option<Vec<u8>>> {
        let mut buf = vec![0u8; self.read_len];
        let mut messages = [I2cMessage::Read(buf.as_mut_slice())];
        self.client.transfer(&mut messages)?;

        let report_len = u16::from_le_bytes([buf[0], buf[1]]) as usize;
        // A zero length means "no data"; anything below 2 or above the buffer
        // is malformed.
        if report_len <= 2 || report_len > self.read_len {
            return Ok(None);
        }
        Ok(Some(buf[2..report_len].to_vec()))
    }
}

/// Reads a 16-bit device register using a write-then-read transaction.
fn read_register(client: &I2cClient, reg: u16, out: &mut [u8]) -> Result<()> {
    let reg_bytes = reg.to_le_bytes();
    let mut messages = [I2cMessage::Write(&reg_bytes), I2cMessage::Read(out)];
    client.transfer(&mut messages)
}

/// Reads from the device without a preceding register write, used to wake
/// devices up before the first register access.
fn plain_read(client: &I2cClient, out: &mut [u8]) -> Result<()> {
    let mut messages = [I2cMessage::Read(out)];
    client.transfer(&mut messages)
}

/// Sends a command to the command register.
///
/// Commands are encoded as `[report_type << 4 | report_id][opcode]`, see
/// `i2c_hid_encode_command()` in Linux.
fn send_command(
    client: &I2cClient,
    command_register: u16,
    opcode: u8,
    report_id: u8,
) -> Result<()> {
    let reg_bytes = command_register.to_le_bytes();
    let cmd = [reg_bytes[0], reg_bytes[1], report_id, opcode];
    let mut messages = [I2cMessage::Write(&cmd)];
    client.transfer(&mut messages)
}

fn set_power(client: &I2cClient, command_register: u16, power: u8) -> Result<()> {
    send_command(client, command_register, OPCODE_SET_POWER, power)
}

fn reset(client: &I2cClient, command_register: u16) -> Result<()> {
    send_command(client, command_register, OPCODE_RESET, 0)
}

fn u16_at(buf: &[u8], off: usize) -> u16 {
    u16::from_le_bytes([buf[off], buf[off + 1]])
}
