// SPDX-License-Identifier: MPL-2.0

//! A polled DesignWare I2C master controller.
//!
//! This implements just enough of the DW_apb_i2c databook to run 7-bit
//! master transfers without interrupts, following the register semantics of
//! the Linux `i2c-designware` driver.

use alloc::string::{String, ToString};

use ostd::{
    Error, Result,
    io::IoMem,
    mm::{Paddr, VmIoOnce},
};

/// Register offsets of the DW_apb_i2c controller.
mod regs {
    pub const CON: usize = 0x00;
    pub const TAR: usize = 0x04;
    pub const DATA_CMD: usize = 0x10;
    pub const FS_SCL_HCNT: usize = 0x1c;
    pub const FS_SCL_LCNT: usize = 0x20;
    pub const TX_ABRT_SOURCE: usize = 0x80;
    pub const RAW_INTR_STAT: usize = 0x34;
    pub const CLR_TX_ABRT: usize = 0x54;
    pub const CLR_STOP_DET: usize = 0x60;
    pub const ENABLE: usize = 0x6c;
    pub const STATUS: usize = 0x70;
    pub const TXFLR: usize = 0x74;
    pub const ENABLE_STATUS: usize = 0x9c;
    pub const COMP_PARAM_1: usize = 0xf4;
}

const CON_MASTER: u32 = 1 << 0;
const CON_SPEED_FAST: u32 = 2 << 1;
const CON_RESTART_EN: u32 = 1 << 5;
const CON_SLAVE_DISABLE: u32 = 1 << 6;

const ENABLE_ENABLE: u32 = 1 << 0;

const STATUS_RFNE: u32 = 1 << 3;

const RAW_INTR_TX_ABRT: u32 = 1 << 6;
const RAW_INTR_STOP_DET: u32 = 1 << 9;

const DATA_CMD_CMD: u32 = 1 << 8;
const DATA_CMD_STOP: u32 = 1 << 9;
const DATA_CMD_RESTART: u32 = 1 << 10;

const COMP_PARAM_1_TX_MASK: u32 = 0xff << 16;

/// How many register polls before we give up on a single step.
const POLL_LIMIT: u32 = 100_000;

/// How many bytes a single transfer may carry in total.
///
/// The current callers only move a few hundred bytes per transfer (HID
/// descriptors and input reports), so a single in-stack limit keeps the
/// implementation simple. Raise it if the driver ever grows larger users.
const MAX_TRANSFER_BYTES: usize = 4096;

/// A single I2C message (the analogue of an `i2c_msg`).
pub enum I2cMessage<'a> {
    /// Write the given bytes to the bus.
    Write(&'a [u8]),
    /// Read `buf.len()` bytes from the bus.
    Read(&'a mut [u8]),
}

/// A polled DesignWare I2C master controller.
pub struct DesignWareI2c {
    /// Human-readable controller name, used in log lines.
    name: String,
    io_mem: IoMem,
    tx_fifo_depth: u32,
}

impl DesignWareI2c {
    /// Maps the controller's MMIO window and derives the TX FIFO depth.
    pub fn new(name: &str, phys: Paddr, size: usize) -> Result<Self> {
        let io_mem = IoMem::acquire(phys..phys + size)?;
        let tx_fifo_depth = Self::tx_fifo_depth(&io_mem);
        ostd::info!(
            "{}: mapped {:?}, TX FIFO depth {}",
            name, phys, tx_fifo_depth
        );
        Ok(Self {
            name: name.to_string(),
            io_mem,
            tx_fifo_depth,
        })
    }

    fn tx_fifo_depth(io_mem: &IoMem) -> u32 {
        let param: u32 = io_mem.read_once(regs::COMP_PARAM_1).unwrap_or(0);
        let depth = ((param & COMP_PARAM_1_TX_MASK) >> 16) + 1;
        if depth <= 1 { 32 } else { depth }
    }

    fn read_reg(&self, offset: usize) -> u32 {
        self.io_mem.read_once(offset).unwrap_or(0)
    }

    fn write_reg(&self, offset: usize, value: u32) {
        self.io_mem.write_once(offset, &value).unwrap_or(());
    }

    fn enable(&mut self) {
        self.write_reg(regs::ENABLE, ENABLE_ENABLE);
        for _ in 0..POLL_LIMIT {
            if self.read_reg(regs::ENABLE_STATUS) & ENABLE_ENABLE != 0 {
                return;
            }
        }
        ostd::warn!("{}: ENABLE_STATUS never asserted", self.name);
    }

    fn disable(&mut self) {
        self.write_reg(regs::ENABLE, 0);
    }

    fn clear_abort(&mut self) {
        if self.read_reg(regs::RAW_INTR_STAT) & RAW_INTR_TX_ABRT != 0 {
            let _ = self.read_reg(regs::CLR_TX_ABRT);
        }
    }

    fn check_tx_abort(&mut self) -> Result<()> {
        if self.read_reg(regs::RAW_INTR_STAT) & RAW_INTR_TX_ABRT != 0 {
            let _ = self.read_reg(regs::CLR_TX_ABRT);
            return Err(Error::IoError);
        }
        Ok(())
    }

    fn wait_tx_fifo_not_full(&mut self) -> bool {
        for _ in 0..POLL_LIMIT {
            if self.read_reg(regs::TXFLR) < self.tx_fifo_depth {
                return true;
            }
        }
        false
    }

    fn wait_rx_fifo_not_empty(&mut self) -> Result<u32> {
        for _ in 0..POLL_LIMIT {
            let raw = self.read_reg(regs::RAW_INTR_STAT);
            if raw & RAW_INTR_TX_ABRT != 0 {
                let _ = self.read_reg(regs::CLR_TX_ABRT);
                return Err(Error::IoError);
            }
            if self.read_reg(regs::STATUS) & STATUS_RFNE != 0 {
                return Ok(self.read_reg(regs::DATA_CMD) & 0xff);
            }
        }
        Err(Error::IoError)
    }

    /// Ensures the fast-mode SCL timing registers hold sane values.
    ///
    /// Firmware normally pre-programs them for the actual input clock. When it
    /// left them zero, fall back to values derived for a 100 MHz input clock
    /// (the usual AMD/Hygon DesignWare clock) targeting 400 kHz, following the
    /// formulas in the Linux `i2c-designware` driver.
    fn ensure_fast_mode_timing(&mut self) {
        let hcnt = self.read_reg(regs::FS_SCL_HCNT);
        let lcnt = self.read_reg(regs::FS_SCL_LCNT);
        if hcnt == 0 || lcnt == 0 {
            ostd::info!(
                "{}: FS_SCL timing unset (hcnt={:#x}, lcnt={:#x}), using 100 MHz fallback",
                self.name, hcnt, lcnt
            );
            self.write_reg(regs::FS_SCL_HCNT, 0x57);
            self.write_reg(regs::FS_SCL_LCNT, 0x9f);
        }
    }

    /// Executes a transaction of one or more messages to `target_addr`.
    ///
    /// A RESTART is emitted between consecutive messages and a STOP after the
    /// last message, mirroring how `i2c-hid` reads device registers. The
    /// controller is guaranteed to be disabled when the call returns, whether
    /// the transfer succeeded or not.
    pub fn transfer(&mut self, target_addr: u8, messages: &mut [I2cMessage<'_>]) -> Result<()> {
        let total = messages
            .iter()
            .map(|msg| match msg {
                I2cMessage::Write(data) => data.len(),
                I2cMessage::Read(data) => data.len(),
            })
            .sum::<usize>();
        if total > MAX_TRANSFER_BYTES {
            ostd::warn!(
                "{}: transfer of {} bytes exceeds the {} byte limit",
                self.name, total, MAX_TRANSFER_BYTES
            );
            return Err(Error::IoError);
        }

        self.disable();
        self.ensure_fast_mode_timing();
        self.write_reg(
            regs::CON,
            CON_MASTER | CON_SLAVE_DISABLE | CON_SPEED_FAST | CON_RESTART_EN,
        );
        self.write_reg(regs::TAR, target_addr as u32);
        self.clear_abort();
        self.enable();

        let result = self.transfer_inner(messages);

        // Always leave the controller disabled so that a later transfer (or a
        // different driver) starts from a known state.
        self.disable();

        if result.is_err() {
            self.log_transfer_error(target_addr);
        }
        result
    }

    /// Runs the message sequence with the controller enabled.
    fn transfer_inner(&mut self, messages: &mut [I2cMessage<'_>]) -> Result<()> {
        let num_msgs = messages.len();
        let mut need_restart = false;
        for (msg_idx, msg) in messages.iter_mut().enumerate() {
            let is_last_msg = msg_idx + 1 == num_msgs;
            match msg {
                I2cMessage::Write(data) => {
                    let data_len = data.len();
                    for (idx, byte) in data.iter().enumerate() {
                        let is_last_byte = is_last_msg && idx + 1 == data_len;
                        let mut cmd = *byte as u32;
                        if need_restart {
                            cmd |= DATA_CMD_RESTART;
                            need_restart = false;
                        }
                        if is_last_byte {
                            cmd |= DATA_CMD_STOP;
                        }
                        if !self.wait_tx_fifo_not_full() {
                            return Err(Error::IoError);
                        }
                        self.write_reg(regs::DATA_CMD, cmd);
                        self.check_tx_abort()?;
                    }
                }
                I2cMessage::Read(data) => {
                    let data_len = data.len();
                    for (idx, byte) in data.iter_mut().enumerate() {
                        let is_last_byte = is_last_msg && idx + 1 == data_len;
                        let mut cmd = DATA_CMD_CMD;
                        if need_restart {
                            cmd |= DATA_CMD_RESTART;
                            need_restart = false;
                        }
                        if is_last_byte {
                            cmd |= DATA_CMD_STOP;
                        }
                        if !self.wait_tx_fifo_not_full() {
                            return Err(Error::IoError);
                        }
                        self.write_reg(regs::DATA_CMD, cmd);
                        let val = self.wait_rx_fifo_not_empty()?;
                        *byte = val as u8;
                        self.check_tx_abort()?;
                    }
                }
            }
            need_restart = true;
        }

        // Wait for the stop condition before returning control of the bus.
        for _ in 0..POLL_LIMIT {
            let raw = self.read_reg(regs::RAW_INTR_STAT);
            if raw & RAW_INTR_STOP_DET != 0 {
                let _ = self.read_reg(regs::CLR_STOP_DET);
                return Ok(());
            }
            if raw & RAW_INTR_TX_ABRT != 0 {
                let _ = self.read_reg(regs::CLR_TX_ABRT);
                return Err(Error::IoError);
            }
        }
        Err(Error::IoError)
    }

    /// Logs the raw interrupt state and the abort source after a failed
    /// transfer, decoding the reasons that point at specific causes.
    fn log_transfer_error(&self, target_addr: u8) {
        let raw = self.read_reg(regs::RAW_INTR_STAT);
        let abort = self.read_reg(regs::TX_ABRT_SOURCE);
        // DW_apb_i2c TX_ABRT_SOURCE bits, from the databook:
        //   bit 0: 7-bit address NACK (no device at the address)
        //   bit 1: data NACK (device refused a data byte)
        //   bit 3: arbitration lost
        //   bit 7: start condition NACK (bus held low)
        let reason = match abort & 0xff {
            0x1 => "7-bit address NACK (no device at this address)",
            0x2 => "data NACK (device refused a data byte)",
            0x8 => "arbitration lost",
            0x80 => "start condition NACK (bus may be stuck low)",
            _ => "unknown",
        };
        ostd::warn!(
            "{}: transfer to {:#04x} failed: RAW_INTR_STAT={:#x}, TX_ABRT_SOURCE={:#x} ({})",
            self.name, target_addr, raw, abort, reason
        );
    }
}
