// Copyright © 2026 Cyberus Technology GmbH
//
// SPDX-License-Identifier: Apache-2.0

use std::collections::{BTreeSet, VecDeque};

use log::{debug, info};

use crate::usb::control::{
    ControlDevice, ControlEndpoint, ControlResponse, HID_GET_IDLE, HID_GET_PROTOCOL,
    HID_GET_REPORT, HID_SET_IDLE, HID_SET_PROTOCOL, HID_SET_REPORT, REQUEST_CLEAR_FEATURE,
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
    0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x05, 0x07, 0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01, 0x75, 0x08, 0x81, 0x01, 0x95, 0x05, 0x75, 0x01,
    0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02, 0x95, 0x01, 0x75, 0x03, 0x91, 0x01, 0x95, 0x06,
    0x75, 0x08, 0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00, 0xc0,
];

const PRODUCT_ID: u16 = 0x0100;
const MAX_REPORTS: usize = 64;
const PROTOCOL_REPORT: u8 = 1;

pub struct HidKeyboard {
    address: u8,
    configuration: u8,
    protocol: u8,
    idle: u8,
    leds: u8,
    modifiers: u8,
    keys: BTreeSet<u8>,
    reports: VecDeque<[u8; 8]>,
    current_report: [u8; 8],
    control: ControlEndpoint,
}

impl HidKeyboard {
    pub fn new() -> Self {
        Self {
            address: 0,
            configuration: 0,
            protocol: PROTOCOL_REPORT,
            idle: 0,
            leds: 0,
            modifiers: 0,
            keys: BTreeSet::new(),
            reports: VecDeque::new(),
            current_report: [0; 8],
            control: ControlEndpoint::new(),
        }
    }

    pub fn key(&mut self, usage: u8, pressed: bool) {
        let changed = if (0xe0..=0xe7).contains(&usage) {
            let bit = 1 << (usage - 0xe0);
            let old = self.modifiers;
            if pressed {
                self.modifiers |= bit;
            } else {
                self.modifiers &= !bit;
            }
            old != self.modifiers
        } else if usage == 0 {
            false
        } else if pressed {
            self.keys.insert(usage)
        } else {
            self.keys.remove(&usage)
        };
        if changed {
            self.queue_current_report();
        }
    }

    pub fn release_all(&mut self) {
        if self.modifiers != 0 || !self.keys.is_empty() {
            self.modifiers = 0;
            self.keys.clear();
            self.queue_current_report();
        }
    }

    pub fn leds(&self) -> u8 {
        self.leds
    }

    pub fn next_report(&mut self) -> Option<[u8; 8]> {
        self.reports.pop_front()
    }

    fn build_report(&self) -> [u8; 8] {
        let mut report = [0; 8];
        report[0] = self.modifiers;
        if self.keys.len() > 6 {
            report[2..].fill(0x01); // HID ErrorRollOver usage.
        } else {
            for (destination, usage) in report[2..].iter_mut().zip(&self.keys) {
                *destination = *usage;
            }
        }
        report
    }

    fn queue_current_report(&mut self) {
        let report = self.build_report();
        if self.current_report == report && self.reports.back().is_none_or(|last| *last == report) {
            return;
        }
        self.current_report = report;
        if self.reports.len() == MAX_REPORTS {
            self.reports.pop_front();
        }
        if self.reports.back().is_none_or(|last| *last != report) {
            self.reports.push_back(report);
        }
    }

    fn control_packet(&mut self, pid: UsbPid, data: &[u8], max_length: usize) -> UsbPacketResult {
        let mut control = std::mem::take(&mut self.control);
        let result = control_packet(&mut control, self, pid, data, max_length);
        self.control = control;
        result
    }
}

impl Default for HidKeyboard {
    fn default() -> Self {
        Self::new()
    }
}

impl UsbDevice for HidKeyboard {
    fn reset(&mut self) {
        self.address = 0;
        self.configuration = 0;
        self.protocol = PROTOCOL_REPORT;
        self.idle = 0;
        self.control.reset();
        self.reports.clear();
        if self.current_report != [0; 8] {
            self.reports.push_back(self.current_report);
        }
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
                    debug!("USB HID keyboard report: {report:02x?}");
                    UsbPacketResult::Success(report[..max_length.min(report.len())].to_vec())
                })
            }
            _ => UsbPacketResult::Stall,
        }
    }
}

impl ControlDevice for HidKeyboard {
    fn prepare_control(&mut self, setup: crate::usb::SetupPacket) -> ControlResponse {
        match (setup.request_kind(), setup.request) {
            (REQUEST_TYPE_STANDARD, REQUEST_GET_DESCRIPTOR) if setup.direction_in() => {
                let descriptor_type = (setup.value >> 8) as u8;
                let data = match descriptor_type {
                    DESCRIPTOR_DEVICE => device_descriptor(PRODUCT_ID),
                    DESCRIPTOR_CONFIGURATION => {
                        configuration_descriptor(1, REPORT_DESCRIPTOR.len(), 8)
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
                ControlResponse::Data(self.current_report.to_vec())
            }
            (REQUEST_TYPE_CLASS, HID_GET_IDLE) if setup.direction_in() => {
                ControlResponse::Data(vec![self.idle])
            }
            (REQUEST_TYPE_CLASS, HID_GET_PROTOCOL) if setup.direction_in() => {
                ControlResponse::Data(vec![self.protocol])
            }
            (REQUEST_TYPE_CLASS, HID_SET_REPORT) if !setup.direction_in() => ControlResponse::Out,
            (REQUEST_TYPE_CLASS, HID_SET_IDLE | HID_SET_PROTOCOL) => ControlResponse::Ack,
            _ => ControlResponse::Stall,
        }
    }

    fn complete_control(&mut self, setup: crate::usb::SetupPacket, data: &[u8]) -> bool {
        match (setup.request_kind(), setup.request) {
            (REQUEST_TYPE_STANDARD, REQUEST_SET_ADDRESS) if setup.value <= 127 => {
                self.address = setup.value as u8;
                true
            }
            (REQUEST_TYPE_STANDARD, REQUEST_SET_CONFIGURATION) if setup.value <= 1 => {
                if self.configuration == 0 && setup.value == 1 {
                    info!("USB HID keyboard configured");
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
            (REQUEST_TYPE_CLASS, HID_SET_REPORT) if !data.is_empty() => {
                self.leds = data[0] & 0x07;
                true
            }
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn setup(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> [u8; 8] {
        let mut packet = [0; 8];
        packet[0] = request_type;
        packet[1] = request;
        packet[2..4].copy_from_slice(&value.to_le_bytes());
        packet[4..6].copy_from_slice(&index.to_le_bytes());
        packet[6..8].copy_from_slice(&length.to_le_bytes());
        packet
    }

    fn setup_stage(keyboard: &mut HidKeyboard, packet: &[u8; 8]) {
        assert_eq!(
            keyboard.packet(UsbPid::Setup, 0, packet, 8),
            UsbPacketResult::Success(Vec::new())
        );
    }

    fn control_in(keyboard: &mut HidKeyboard, packet: &[u8; 8], length: usize) -> Vec<u8> {
        setup_stage(keyboard, packet);
        let UsbPacketResult::Success(data) = keyboard.packet(UsbPid::In, 0, &[], length) else {
            panic!("control IN data stage failed");
        };
        assert_eq!(
            keyboard.packet(UsbPid::Out, 0, &[], 0),
            UsbPacketResult::Success(Vec::new())
        );
        data
    }

    #[test]
    fn descriptor_lengths_and_fields_are_exact() {
        let device = device_descriptor(PRODUCT_ID);
        let config = configuration_descriptor(1, REPORT_DESCRIPTOR.len(), 8);
        let hid = hid_descriptor(REPORT_DESCRIPTOR.len());
        assert_eq!(device.len(), 18);
        assert_eq!(device[1], DESCRIPTOR_DEVICE);
        assert_eq!(config.len(), 34);
        assert_eq!(u16::from_le_bytes([config[2], config[3]]), 34);
        assert_eq!(config[14..17], [0x03, 0x01, 0x01]);
        assert_eq!(hid.len(), 9);
        assert_eq!(
            u16::from_le_bytes([hid[7], hid[8]]) as usize,
            REPORT_DESCRIPTOR.len()
        );
        assert_eq!(REPORT_DESCRIPTOR.len(), 63);
    }

    #[test]
    fn reports_keys_modifiers_releases_and_rollover() {
        let mut keyboard = HidKeyboard::new();
        keyboard.key(0x04, true);
        assert_eq!(keyboard.next_report().unwrap(), [0, 0, 4, 0, 0, 0, 0, 0]);
        keyboard.key(0xe1, true);
        assert_eq!(keyboard.next_report().unwrap(), [2, 0, 4, 0, 0, 0, 0, 0]);
        for usage in 5..=10 {
            keyboard.key(usage, true);
        }
        assert_eq!(keyboard.next_report().unwrap()[2..], [4, 5, 0, 0, 0, 0]);
        while keyboard.reports.len() > 1 {
            keyboard.next_report();
        }
        assert_eq!(keyboard.next_report().unwrap()[2..], [1; 6]);
        keyboard.release_all();
        assert_eq!(keyboard.next_report().unwrap(), [0; 8]);
    }

    #[test]
    fn standard_enumeration_requests_preserve_control_stages() {
        let mut keyboard = HidKeyboard::new();
        let device = control_in(
            &mut keyboard,
            &setup(0x80, REQUEST_GET_DESCRIPTOR, 0x0100, 0, 18),
            18,
        );
        assert_eq!(device, device_descriptor(PRODUCT_ID));

        setup_stage(&mut keyboard, &setup(0x00, REQUEST_SET_ADDRESS, 5, 0, 0));
        assert_eq!(keyboard.address(), 0);
        assert_eq!(
            keyboard.packet(UsbPid::In, 0, &[], 0),
            UsbPacketResult::Success(Vec::new())
        );
        assert_eq!(keyboard.address(), 5);

        setup_stage(
            &mut keyboard,
            &setup(0x00, REQUEST_SET_CONFIGURATION, 1, 0, 0),
        );
        assert_eq!(
            keyboard.packet(UsbPid::In, 0, &[], 0),
            UsbPacketResult::Success(Vec::new())
        );
        assert_eq!(
            control_in(
                &mut keyboard,
                &setup(0x80, REQUEST_GET_CONFIGURATION, 0, 0, 1),
                1,
            ),
            [1]
        );
        assert_eq!(
            control_in(&mut keyboard, &setup(0x80, REQUEST_GET_STATUS, 0, 0, 2), 2,),
            [0, 0]
        );
    }

    #[test]
    fn hid_protocol_idle_report_and_led_requests_work() {
        let mut keyboard = HidKeyboard::new();
        for (request, value) in [(HID_SET_PROTOCOL, 0), (HID_SET_IDLE, 7 << 8)] {
            setup_stage(&mut keyboard, &setup(0x21, request, value, 0, 0));
            assert_eq!(
                keyboard.packet(UsbPid::In, 0, &[], 0),
                UsbPacketResult::Success(Vec::new())
            );
        }
        assert_eq!(
            control_in(&mut keyboard, &setup(0xa1, HID_GET_PROTOCOL, 0, 0, 1), 1,),
            [0]
        );
        assert_eq!(
            control_in(&mut keyboard, &setup(0xa1, HID_GET_IDLE, 0, 0, 1), 1,),
            [7]
        );

        setup_stage(&mut keyboard, &setup(0x21, HID_SET_REPORT, 0x0200, 0, 1));
        assert_eq!(
            keyboard.packet(UsbPid::Out, 0, &[0x07], 1),
            UsbPacketResult::Success(Vec::new())
        );
        assert_eq!(
            keyboard.packet(UsbPid::In, 0, &[], 0),
            UsbPacketResult::Success(Vec::new())
        );
        assert_eq!(keyboard.leds(), 0x07);
        assert_eq!(
            control_in(&mut keyboard, &setup(0xa1, HID_GET_REPORT, 0x0100, 0, 8), 8,),
            [0; 8]
        );
    }
}
