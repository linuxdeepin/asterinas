// SPDX-License-Identifier: MPL-2.0

//! Bridges HID input reports to the Asterinas input subsystem.

use alloc::{
    string::{String, ToString},
    sync::Arc,
    vec::Vec,
};
use core::{
    sync::atomic::{AtomicBool, Ordering},
    time::Duration,
};

use aster_input::{
    event_type_codes::{EventTypes, KeyCode, KeyStatus, RelCode, SynEvent},
    input_dev::{
        InputCapability, InputDevice as InputDeviceTrait, InputEvent, InputId,
        RegisteredInputDevice,
    },
};

use crate::{
    hid::I2cHidDevice,
    hid_report::{FieldKind, ReportLayout, decode_field},
};

/// How often the touchpad is polled, in milliseconds.
const POLL_INTERVAL_MS: u64 = 8;

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

/// The bridge of one probed device: the flag that tells its polling thread
/// to stop.
pub(super) struct Bridge {
    pub(super) stop: Arc<AtomicBool>,
}

impl Bridge {
    /// Registers the input device of `device` and starts its polling thread.
    ///
    /// The thread owns the device, so polling it needs no locking.
    pub(super) fn start(device: I2cHidDevice, layout: ReportLayout) -> Self {
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

        let registered = Arc::new(aster_input::register_device(input_device));
        let stop = Arc::new(AtomicBool::new(false));

        // A kernel thread is required here: a raw OSTD task carries no kernel
        // thread data, so the kernel scheduler would never schedule it.
        aster_core::spawn_kernel_thread({
            let stop = stop.clone();
            let registered = registered.clone();
            move || poll_loop(device, layout, registered, stop)
        });

        Self { stop }
    }
}

/// The polling loop: reads a report when the device signals one and paces the
/// bus polls to `POLL_INTERVAL_MS`.
fn poll_loop(
    mut device: I2cHidDevice,
    layout: ReportLayout,
    registered: Arc<RegisteredInputDevice>,
    stop: Arc<AtomicBool>,
) {
    // The currently pressed button mask (bit 0 = left, 1 = right, 2 = middle).
    let mut buttons: u8 = 0;
    while !stop.load(Ordering::Relaxed) {
        if let Some(report) = device.read_input().ok().flatten()
            && let Some(events) = decode_report(&report, &layout, &mut buttons)
        {
            registered.submit_events(&events);
        }

        // Polling must be paced with the monotonic timer: spinning here once
        // burned a CPU at full speed and starved the system.
        aster_core::sleep(Duration::from_millis(POLL_INTERVAL_MS));
    }
}

/// Decodes a report into input events.
fn decode_report(
    report: &[u8],
    layout: &ReportLayout,
    buttons: &mut u8,
) -> Option<Vec<InputEvent>> {
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
    let mut new_buttons = 0u8;
    for field in &layout.fields {
        let value = decode_field(payload, field);
        match field.kind {
            FieldKind::RelX => dx += value,
            FieldKind::RelY => dy += value,
            FieldKind::RelWheel => wheel += value,
            FieldKind::Button(n) if n <= 8 => {
                if value != 0 {
                    new_buttons |= 1 << (n - 1);
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

    let changed = *buttons ^ new_buttons;
    for n in 0..3 {
        if changed & (1 << n) != 0 {
            let pressed = new_buttons & (1 << n) != 0;
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
    *buttons = new_buttons;

    events.push(InputEvent::Sync(SynEvent::Report));
    ostd::debug!(
        "decoded: dx={} dy={} wheel={} buttons={:#04b}, {} events",
        dx,
        dy,
        wheel,
        new_buttons,
        events.len()
    );
    Some(events)
}
