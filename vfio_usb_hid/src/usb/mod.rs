// Copyright © 2026 Cyberus Technology GmbH
//
// SPDX-License-Identifier: Apache-2.0

mod control;
pub mod hid;

use control::ControlEndpoint;
pub use control::{SetupPacket, UsbPacketResult};
use hid::keyboard::HidKeyboard;
use hid::mouse::HidMouse;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UsbPid {
    Setup,
    In,
    Out,
}

pub trait UsbDevice: Send {
    fn reset(&mut self);
    fn address(&self) -> u8;
    fn packet(
        &mut self,
        pid: UsbPid,
        endpoint: u8,
        data: &[u8],
        max_length: usize,
    ) -> UsbPacketResult;
}

pub struct UsbBus {
    keyboard: HidKeyboard,
    mouse: HidMouse,
}

impl UsbBus {
    pub fn new() -> Self {
        Self {
            keyboard: HidKeyboard::new(),
            mouse: HidMouse::new(),
        }
    }

    pub fn reset_port(&mut self, port: usize) {
        if let Some(device) = self.device_mut(port) {
            device.reset();
        }
    }

    pub fn address(&self, port: usize) -> Option<u8> {
        match port {
            0 => Some(self.keyboard.address()),
            1 => Some(self.mouse.address()),
            _ => None,
        }
    }

    pub fn packet(
        &mut self,
        port: usize,
        pid: UsbPid,
        endpoint: u8,
        data: &[u8],
        max_length: usize,
    ) -> UsbPacketResult {
        self.device_mut(port)
            .map_or(UsbPacketResult::NoDevice, |device| {
                device.packet(pid, endpoint, data, max_length)
            })
    }

    pub fn key(&mut self, usage: u8, pressed: bool) {
        self.keyboard.key(usage, pressed);
    }

    pub fn mouse(&mut self, dx: i32, dy: i32, wheel: i16, buttons: u8) {
        self.mouse.update(dx, dy, wheel, buttons);
    }

    pub fn release_all(&mut self) {
        self.keyboard.release_all();
        self.mouse.release_all();
    }

    pub fn keyboard_leds(&self) -> u8 {
        self.keyboard.leds()
    }

    fn device_mut(&mut self, port: usize) -> Option<&mut dyn UsbDevice> {
        match port {
            0 => Some(&mut self.keyboard),
            1 => Some(&mut self.mouse),
            _ => None,
        }
    }

    #[cfg(test)]
    pub(crate) fn take_keyboard_report(&mut self) -> Option<[u8; 8]> {
        self.keyboard.next_report()
    }

    #[cfg(test)]
    pub(crate) fn take_mouse_report(&mut self) -> Option<Vec<u8>> {
        self.mouse.next_report()
    }
}

impl Default for UsbBus {
    fn default() -> Self {
        Self::new()
    }
}

pub(crate) fn control_packet<D: control::ControlDevice>(
    control: &mut ControlEndpoint,
    device: &mut D,
    pid: UsbPid,
    data: &[u8],
    max_length: usize,
) -> UsbPacketResult {
    control.packet(device, pid, data, max_length)
}
