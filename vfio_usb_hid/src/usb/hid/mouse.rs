// Copyright © 2026 Cyberus Technology GmbH
//
// SPDX-License-Identifier: Apache-2.0

use std::collections::VecDeque;

use log::{debug, info};

use crate::usb::control::{
    ControlDevice, ControlEndpoint, ControlResponse, HID_GET_IDLE, HID_GET_PROTOCOL,
    HID_GET_REPORT, HID_SET_IDLE, HID_SET_PROTOCOL, REQUEST_CLEAR_FEATURE,
    REQUEST_GET_CONFIGURATION, REQUEST_GET_DESCRIPTOR, REQUEST_GET_INTERFACE, REQUEST_GET_STATUS,
    REQUEST_SET_ADDRESS, REQUEST_SET_CONFIGURATION, REQUEST_SET_FEATURE, REQUEST_SET_INTERFACE,
};
use crate::usb::hid::{
    DESCRIPTOR_CONFIGURATION, DESCRIPTOR_DEVICE, DESCRIPTOR_HID, DESCRIPTOR_REPORT,
    REQUEST_TYPE_CLASS, REQUEST_TYPE_STANDARD, configuration_descriptor, device_descriptor,
    hid_descriptor,
};
use crate::usb::{UsbDevice, UsbPacketResult, UsbPid, control_packet};

pub const REPORT_DESCRIPTOR: &[u8] = &[
    0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, 0x09, 0x01, 0xa1, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x03,
    0x15, 0x00, 0x25, 0x01, 0x95, 0x03, 0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x05, 0x81, 0x01,
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38, 0x15, 0x81, 0x25, 0x7f, 0x75, 0x08, 0x95, 0x03,
    0x81, 0x06, 0xc0, 0xc0,
];

const PRODUCT_ID: u16 = 0x0101;
const PROTOCOL_REPORT: u8 = 1;
const MAX_BUTTON_REPORTS: usize = 64;

pub struct HidMouse {
    address: u8,
    configuration: u8,
    protocol: u8,
    idle: u8,
    buttons: u8,
    reported_buttons: u8,
    button_reports: VecDeque<u8>,
    pending_dx: i64,
    pending_dy: i64,
    pending_wheel: i64,
    control: ControlEndpoint,
}

impl HidMouse {
    pub fn new() -> Self {
        Self {
            address: 0,
            configuration: 0,
            protocol: PROTOCOL_REPORT,
            idle: 0,
            buttons: 0,
            reported_buttons: 0,
            button_reports: VecDeque::new(),
            pending_dx: 0,
            pending_dy: 0,
            pending_wheel: 0,
            control: ControlEndpoint::new(),
        }
    }

    pub fn update(&mut self, dx: i32, dy: i32, wheel: i16, buttons: u8) {
        self.pending_dx = self.pending_dx.saturating_add(i64::from(dx));
        self.pending_dy = self.pending_dy.saturating_add(i64::from(dy));
        self.pending_wheel = self.pending_wheel.saturating_add(i64::from(wheel));
        let buttons = buttons & 0x07;
        if self.buttons != buttons {
            self.buttons = buttons;
            let queued_buttons = self
                .button_reports
                .back()
                .copied()
                .unwrap_or(self.reported_buttons);
            if queued_buttons != buttons {
                if self.button_reports.len() == MAX_BUTTON_REPORTS {
                    self.button_reports.pop_front();
                }
                self.button_reports.push_back(buttons);
            }
        }
    }

    pub fn release_all(&mut self) {
        self.pending_dx = 0;
        self.pending_dy = 0;
        self.pending_wheel = 0;
        self.buttons = 0;
        self.button_reports.clear();
        if self.reported_buttons != 0 {
            self.button_reports.push_back(0);
        }
    }

    pub fn next_report(&mut self) -> Option<Vec<u8>> {
        if self.button_reports.is_empty()
            && self.pending_dx == 0
            && self.pending_dy == 0
            && self.pending_wheel == 0
        {
            return None;
        }
        let buttons = self.button_reports.pop_front().unwrap_or(self.buttons);
        let dx = take_delta(&mut self.pending_dx);
        let dy = take_delta(&mut self.pending_dy);
        let wheel = take_delta(&mut self.pending_wheel);
        self.reported_buttons = buttons;
        let mut report = vec![buttons, dx as u8, dy as u8];
        if self.protocol == PROTOCOL_REPORT {
            report.push(wheel as u8);
        }
        Some(report)
    }

    fn current_report(&self) -> Vec<u8> {
        let mut report = vec![self.buttons, 0, 0];
        if self.protocol == PROTOCOL_REPORT {
            report.push(0);
        }
        report
    }

    fn control_packet(&mut self, pid: UsbPid, data: &[u8], max_length: usize) -> UsbPacketResult {
        let mut control = std::mem::take(&mut self.control);
        let result = control_packet(&mut control, self, pid, data, max_length);
        self.control = control;
        result
    }
}

fn take_delta(value: &mut i64) -> i8 {
    let part = (*value).clamp(-127, 127) as i8;
    *value -= i64::from(part);
    part
}

impl Default for HidMouse {
    fn default() -> Self {
        Self::new()
    }
}

impl UsbDevice for HidMouse {
    fn reset(&mut self) {
        self.address = 0;
        self.configuration = 0;
        self.protocol = PROTOCOL_REPORT;
        self.idle = 0;
        self.control.reset();
        self.reported_buttons = 0;
        self.button_reports.clear();
        if self.buttons != 0 {
            self.button_reports.push_back(self.buttons);
        }
        self.pending_dx = 0;
        self.pending_dy = 0;
        self.pending_wheel = 0;
    }

    fn address(&self) -> u8 {
        self.address
    }

    fn packet(
        &mut self,
        pid: UsbPid,
        endpoint: u8,
        data: &[u8],
        max_length: usize,
    ) -> UsbPacketResult {
        match (endpoint, pid) {
            (0, _) => self.control_packet(pid, data, max_length),
            (1, UsbPid::In) if self.configuration != 0 => {
                self.next_report().map_or(UsbPacketResult::Nak, |report| {
                    debug!("USB HID mouse report: {report:02x?}");
                    UsbPacketResult::Success(report[..max_length.min(report.len())].to_vec())
                })
            }
            _ => UsbPacketResult::Stall,
        }
    }
}

impl ControlDevice for HidMouse {
    fn prepare_control(&mut self, setup: crate::usb::SetupPacket) -> ControlResponse {
        match (setup.request_kind(), setup.request) {
            (REQUEST_TYPE_STANDARD, REQUEST_GET_DESCRIPTOR) if setup.direction_in() => {
                let descriptor_type = (setup.value >> 8) as u8;
                let data = match descriptor_type {
                    DESCRIPTOR_DEVICE => device_descriptor(PRODUCT_ID),
                    DESCRIPTOR_CONFIGURATION => {
                        configuration_descriptor(2, REPORT_DESCRIPTOR.len(), 4)
                    }
                    DESCRIPTOR_HID => hid_descriptor(REPORT_DESCRIPTOR.len()),
                    DESCRIPTOR_REPORT => REPORT_DESCRIPTOR.to_vec(),
                    _ => return ControlResponse::Stall,
                };
                ControlResponse::Data(data)
            }
            (REQUEST_TYPE_STANDARD, REQUEST_GET_STATUS) if setup.direction_in() => {
                ControlResponse::Data(vec![0, 0])
            }
            (REQUEST_TYPE_STANDARD, REQUEST_GET_CONFIGURATION) if setup.direction_in() => {
                ControlResponse::Data(vec![self.configuration])
            }
            (REQUEST_TYPE_STANDARD, REQUEST_GET_INTERFACE) if setup.direction_in() => {
                ControlResponse::Data(vec![0])
            }
            (REQUEST_TYPE_STANDARD, REQUEST_SET_ADDRESS | REQUEST_SET_CONFIGURATION)
            | (REQUEST_TYPE_STANDARD, REQUEST_CLEAR_FEATURE | REQUEST_SET_FEATURE)
            | (REQUEST_TYPE_STANDARD, REQUEST_SET_INTERFACE) => ControlResponse::Ack,
            (REQUEST_TYPE_CLASS, HID_GET_REPORT) if setup.direction_in() => {
                ControlResponse::Data(self.current_report())
            }
            (REQUEST_TYPE_CLASS, HID_GET_IDLE) if setup.direction_in() => {
                ControlResponse::Data(vec![self.idle])
            }
            (REQUEST_TYPE_CLASS, HID_GET_PROTOCOL) if setup.direction_in() => {
                ControlResponse::Data(vec![self.protocol])
            }
            (REQUEST_TYPE_CLASS, HID_SET_IDLE | HID_SET_PROTOCOL) => ControlResponse::Ack,
            _ => ControlResponse::Stall,
        }
    }

    fn complete_control(&mut self, setup: crate::usb::SetupPacket, _data: &[u8]) -> bool {
        match (setup.request_kind(), setup.request) {
            (REQUEST_TYPE_STANDARD, REQUEST_SET_ADDRESS) if setup.value <= 127 => {
                self.address = setup.value as u8;
                true
            }
            (REQUEST_TYPE_STANDARD, REQUEST_SET_CONFIGURATION) if setup.value <= 1 => {
                if self.configuration == 0 && setup.value == 1 {
                    info!("USB HID mouse configured");
                }
                self.configuration = setup.value as u8;
                true
            }
            (REQUEST_TYPE_STANDARD, REQUEST_CLEAR_FEATURE | REQUEST_SET_FEATURE)
            | (REQUEST_TYPE_STANDARD, REQUEST_SET_INTERFACE) => true,
            (REQUEST_TYPE_CLASS, HID_SET_IDLE) => {
                self.idle = (setup.value >> 8) as u8;
                true
            }
            (REQUEST_TYPE_CLASS, HID_SET_PROTOCOL) if setup.value <= 1 => {
                self.protocol = setup.value as u8;
                true
            }
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn descriptor_lengths_and_fields_are_exact() {
        let device = device_descriptor(PRODUCT_ID);
        let config = configuration_descriptor(2, REPORT_DESCRIPTOR.len(), 4);
        let hid = hid_descriptor(REPORT_DESCRIPTOR.len());
        assert_eq!(device.len(), 18);
        assert_eq!(config.len(), 34);
        assert_eq!(config[14..17], [0x03, 0x01, 0x02]);
        assert_eq!(hid.len(), 9);
        assert_eq!(
            u16::from_le_bytes([hid[7], hid[8]]) as usize,
            REPORT_DESCRIPTOR.len()
        );
        assert_eq!(REPORT_DESCRIPTOR.len(), 52);
    }

    #[test]
    fn movement_is_split_without_losing_remainders() {
        let mut mouse = HidMouse::new();
        mouse.update(400, -300, 200, 5);
        assert_eq!(mouse.next_report().unwrap(), [5, 127, 129, 127]);
        assert_eq!(mouse.next_report().unwrap(), [5, 127, 129, 73]);
        assert_eq!(mouse.next_report().unwrap(), [5, 127, 210, 0]);
        assert_eq!(mouse.next_report().unwrap(), [5, 19, 0, 0]);
        assert!(mouse.next_report().is_none());
    }

    #[test]
    fn boot_protocol_omits_wheel_and_release_clears_buttons() {
        let mut mouse = HidMouse::new();
        mouse.protocol = 0;
        mouse.update(1, 2, 3, 1);
        assert_eq!(mouse.next_report().unwrap(), [1, 1, 2]);
        mouse.release_all();
        assert_eq!(mouse.next_report().unwrap(), [0, 0, 0]);
    }

    #[test]
    fn rapid_button_press_and_release_are_both_reported() {
        let mut mouse = HidMouse::new();
        mouse.update(0, 0, 0, 1);
        mouse.update(0, 0, 0, 0);

        assert_eq!(mouse.next_report(), Some(vec![1, 0, 0, 0]));
        assert_eq!(mouse.next_report(), Some(vec![0, 0, 0, 0]));
        assert_eq!(mouse.next_report(), None);
    }
}
