// Copyright © 2026 Cyberus Technology GmbH
//
// SPDX-License-Identifier: Apache-2.0

use super::UsbPid;

pub const REQUEST_GET_STATUS: u8 = 0x00;
pub const REQUEST_CLEAR_FEATURE: u8 = 0x01;
pub const REQUEST_SET_FEATURE: u8 = 0x03;
pub const REQUEST_SET_ADDRESS: u8 = 0x05;
pub const REQUEST_GET_DESCRIPTOR: u8 = 0x06;
pub const REQUEST_GET_CONFIGURATION: u8 = 0x08;
pub const REQUEST_SET_CONFIGURATION: u8 = 0x09;
pub const REQUEST_GET_INTERFACE: u8 = 0x0a;
pub const REQUEST_SET_INTERFACE: u8 = 0x0b;

pub const HID_GET_REPORT: u8 = 0x01;
pub const HID_GET_IDLE: u8 = 0x02;
pub const HID_GET_PROTOCOL: u8 = 0x03;
pub const HID_SET_REPORT: u8 = 0x09;
pub const HID_SET_IDLE: u8 = 0x0a;
pub const HID_SET_PROTOCOL: u8 = 0x0b;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SetupPacket {
    pub request_type: u8,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub length: u16,
}

impl SetupPacket {
    fn parse(data: &[u8]) -> Option<Self> {
        (data.len() == 8).then(|| Self {
            request_type: data[0],
            request: data[1],
            value: u16::from_le_bytes([data[2], data[3]]),
            index: u16::from_le_bytes([data[4], data[5]]),
            length: u16::from_le_bytes([data[6], data[7]]),
        })
    }

    pub fn direction_in(self) -> bool {
        self.request_type & 0x80 != 0
    }

    pub fn request_kind(self) -> u8 {
        self.request_type & 0x60
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum UsbPacketResult {
    Success(Vec<u8>),
    Nak,
    Stall,
    NoDevice,
}

pub enum ControlResponse {
    Data(Vec<u8>),
    Out,
    Ack,
    Stall,
}

pub trait ControlDevice {
    fn prepare_control(&mut self, setup: SetupPacket) -> ControlResponse;
    fn complete_control(&mut self, setup: SetupPacket, data: &[u8]) -> bool;
}

enum ControlState {
    Idle,
    DataIn {
        setup: SetupPacket,
        data: Vec<u8>,
        offset: usize,
    },
    DataOut {
        setup: SetupPacket,
        data: Vec<u8>,
    },
    StatusIn {
        setup: SetupPacket,
        data: Vec<u8>,
    },
    StatusOut,
    Stalled,
}

pub struct ControlEndpoint {
    state: ControlState,
}

impl ControlEndpoint {
    pub fn new() -> Self {
        Self {
            state: ControlState::Idle,
        }
    }

    pub fn reset(&mut self) {
        self.state = ControlState::Idle;
    }

    pub fn packet<D: ControlDevice>(
        &mut self,
        device: &mut D,
        pid: UsbPid,
        packet_data: &[u8],
        max_length: usize,
    ) -> UsbPacketResult {
        if pid == UsbPid::Setup {
            let Some(setup) = SetupPacket::parse(packet_data) else {
                self.state = ControlState::Stalled;
                return UsbPacketResult::Stall;
            };
            self.state = match device.prepare_control(setup) {
                ControlResponse::Data(mut data) if setup.direction_in() => {
                    data.truncate(usize::from(setup.length));
                    if data.is_empty() {
                        ControlState::StatusOut
                    } else {
                        ControlState::DataIn {
                            setup,
                            data,
                            offset: 0,
                        }
                    }
                }
                ControlResponse::Out if !setup.direction_in() && setup.length != 0 => {
                    ControlState::DataOut {
                        setup,
                        data: Vec::with_capacity(usize::from(setup.length)),
                    }
                }
                ControlResponse::Ack if setup.length == 0 => ControlState::StatusIn {
                    setup,
                    data: Vec::new(),
                },
                _ => ControlState::Stalled,
            };
            return if matches!(self.state, ControlState::Stalled) {
                UsbPacketResult::Stall
            } else {
                UsbPacketResult::Success(Vec::new())
            };
        }

        match (&mut self.state, pid) {
            (
                ControlState::DataIn {
                    setup,
                    data,
                    offset,
                },
                UsbPid::In,
            ) => {
                let end = offset.saturating_add(max_length).min(data.len());
                let result = data[*offset..end].to_vec();
                *offset = end;
                if *offset == data.len() || result.len() < max_length {
                    let _ = setup;
                    self.state = ControlState::StatusOut;
                }
                UsbPacketResult::Success(result)
            }
            (ControlState::DataOut { setup, data }, UsbPid::Out) => {
                let remaining = usize::from(setup.length).saturating_sub(data.len());
                if packet_data.len() > remaining {
                    self.state = ControlState::Stalled;
                    return UsbPacketResult::Stall;
                }
                data.extend_from_slice(packet_data);
                if data.len() == usize::from(setup.length) {
                    self.state = ControlState::StatusIn {
                        setup: *setup,
                        data: std::mem::take(data),
                    };
                }
                UsbPacketResult::Success(Vec::new())
            }
            (ControlState::StatusIn { setup, data }, UsbPid::In) if max_length == 0 => {
                let success = device.complete_control(*setup, data);
                self.state = if success {
                    ControlState::Idle
                } else {
                    ControlState::Stalled
                };
                if success {
                    UsbPacketResult::Success(Vec::new())
                } else {
                    UsbPacketResult::Stall
                }
            }
            (ControlState::StatusOut, UsbPid::Out) if packet_data.is_empty() => {
                self.state = ControlState::Idle;
                UsbPacketResult::Success(Vec::new())
            }
            (ControlState::Stalled, _) | (ControlState::Idle, _) => UsbPacketResult::Stall,
            _ => {
                self.state = ControlState::Stalled;
                UsbPacketResult::Stall
            }
        }
    }
}

impl Default for ControlEndpoint {
    fn default() -> Self {
        Self::new()
    }
}
