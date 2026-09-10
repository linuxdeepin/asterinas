// SPDX-License-Identifier: MPL-2.0

//! Bridges HID input reports to the Asterinas input subsystem.

use alloc::{
    string::{String, ToString},
    sync::Arc,
    vec::Vec,
};
use core::time::Duration;

use aster_input::{
    event_type_codes::{EventTypes, KeyCode, KeyStatus, RelCode, SynEvent},
    input_dev::{
        InputCapability, InputDevice as InputDeviceTrait, InputEvent, InputId,
        RegisteredInputDevice,
    },
};
use ostd::sync::{LocalIrqDisabled, SpinLock};
use spin::Once;

use crate::{
    hid_report::{FieldKind, ReportLayout, decode_field},
    i2c_hid::I2cHidDevice,
};

/// How often the touchpad is polled, in milliseconds.
const POLL_INTERVAL_MS: u64 = 8;

/// The probed HID device, shared with the polling thread.
static DEVICE: Once<SpinLock<I2cHidDevice, LocalIrqDisabled>> = Once::new();
/// The parsed mouse report layout.
static LAYOUT: Once<ReportLayout> = Once::new();
/// The registered input device.
static REGISTERED_DEVICE: Once<RegisteredInputDevice> = Once::new();
/// The currently pressed button mask (bit 0 = left, 1 = right, 2 = middle).
static BUTTON_STATE: SpinLock<u8, LocalIrqDisabled> = SpinLock::new(0);

/// The input device we expose to `aster-input`.
#[derive(Debug)]
struct I2cHidInputDevice {
    name: String,
    phys: String,
    uniq: String,
    id: InputId,
    capability: InputCapability,
}

impl InputDeviceTrait for I2cHidInputDevice {
    fn name(&self) -> &str {
        &self.name
    }

    fn phys(&self) -> &str {
        &self.phys
    }

    fn uniq(&self) -> &str {
        &self.uniq
    }

    fn id(&self) -> InputId {
        self.id
    }

    fn capability(&self) -> &InputCapability {
        &self.capability
    }
}

/// Registers the input device and spawns the polling thread.
///
/// Returns whether this instance became the active touchpad.
pub(super) fn init(device: I2cHidDevice, layout: ReportLayout) -> bool {
    if DEVICE.get().is_some() {
        ostd::warn!("an I2C touchpad is already active, skipping this one");
        return false;
    }

    let desc = device.descriptor();
    let mut capability = InputCapability::new();
    capability.set_supported_event_type(EventTypes::KEY);
    capability.set_supported_event_type(EventTypes::REL);
    capability.set_supported_event_type(EventTypes::SYN);
    capability.set_supported_key(KeyCode::BtnLeft);
    capability.set_supported_key(KeyCode::BtnRight);
    capability.set_supported_key(KeyCode::BtnMiddle);
    capability.set_supported_relative_axis(RelCode::X);
    capability.set_supported_relative_axis(RelCode::Y);
    capability.set_supported_relative_axis(RelCode::Wheel);

    let input_device = Arc::new(I2cHidInputDevice {
        name: "i2c_touchpad".to_string(),
        phys: "i2c-<addr>/input0".to_string(),
        uniq: String::new(),
        id: InputId::new(InputId::BUS_I2C, desc.vendor_id, desc.product_id, 0x0100),
        capability,
    });

    let registered = aster_input::register_device(input_device);
    let _ = REGISTERED_DEVICE.call_once(|| registered);
    let _ = DEVICE.call_once(|| SpinLock::new(device));
    let _ = LAYOUT.call_once(|| layout);

    // A kernel thread is required here: a raw OSTD task carries no kernel
    // thread data, so the kernel scheduler would never schedule it.
    aster_core::spawn_kernel_thread(poll_loop);
    true
}

/// The polling loop: reads a report when the device signals one and paces the
/// bus polls to `POLL_INTERVAL_MS`.
fn poll_loop() {
    loop {
        let (Some(device), Some(layout)) = (DEVICE.get(), LAYOUT.get()) else {
            ostd::warn!("polling thread: device is not initialized, exiting");
            return;
        };

        let events = {
            let mut guard = device.lock();
            poll_once(&mut guard, layout)
        };
        if let Some(events) = events
            && let Some(registered) = REGISTERED_DEVICE.get()
        {
            registered.submit_events(&events);
        }

        aster_core::sleep(Duration::from_millis(POLL_INTERVAL_MS));
    }
}

/// Reads and decodes a single input report.
fn poll_once(device: &mut I2cHidDevice, layout: &ReportLayout) -> Option<Vec<InputEvent>> {
    let report = device.read_input().ok()??;
    decode_report(&report, layout)
}

/// Decodes a report into input events.
fn decode_report(report: &[u8], layout: &ReportLayout) -> Option<Vec<InputEvent>> {
    let payload = if layout.report_id == 0 {
        report
    } else {
        if report.first() != Some(&layout.report_id) {
            return None;
        }
        &report[1..]
    };

    let mut dx = 0i32;
    let mut dy = 0i32;
    let mut wheel = 0i32;
    let mut buttons = 0u8;
    for field in &layout.fields {
        let value = decode_field(payload, field);
        match field.kind {
            FieldKind::RelX => dx += value,
            FieldKind::RelY => dy += value,
            FieldKind::RelWheel => wheel += value,
            FieldKind::Button(n) if n <= 8 => {
                if value != 0 {
                    buttons |= 1 << (n - 1);
                }
            }
            FieldKind::Button(_) => {}
        }
    }

    let mut events = Vec::new();
    if dx != 0 {
        events.push(InputEvent::Relative(RelCode::X, dx));
    }
    if dy != 0 {
        events.push(InputEvent::Relative(RelCode::Y, dy));
    }
    if wheel != 0 {
        events.push(InputEvent::Relative(RelCode::Wheel, wheel));
    }

    let mut old_buttons = BUTTON_STATE.lock();
    let changed = *old_buttons ^ buttons;
    for n in 0..3 {
        if changed & (1 << n) != 0 {
            let pressed = buttons & (1 << n) != 0;
            let key = match n {
                0 => KeyCode::BtnLeft,
                1 => KeyCode::BtnRight,
                _ => KeyCode::BtnMiddle,
            };
            events.push(InputEvent::Key(
                key,
                if pressed {
                    KeyStatus::Pressed
                } else {
                    KeyStatus::Released
                },
            ));
        }
    }
    *old_buttons = buttons;

    events.push(InputEvent::Sync(SynEvent::Report));
    ostd::debug!(
        "decoded: dx={} dy={} wheel={} buttons={:#04b}, {} events",
        dx,
        dy,
        wheel,
        buttons,
        events.len()
    );
    Some(events)
}
