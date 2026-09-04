// Copyright © 2026 Cyberus Technology GmbH
//
// SPDX-License-Identifier: Apache-2.0

use std::collections::HashSet;
use std::fs::File;
use std::io::{self, Write};
use std::sync::{Arc, Mutex};

use log::warn;
use vfio_user_common::guest_memory::{GuestAddress, GuestMemoryMap};

use crate::usb::{UsbBus, UsbPacketResult, UsbPid};

pub const UHCI_IO_BAR_SIZE: u64 = 0x20;

const USBCMD: u64 = 0x00;
const USBSTS: u64 = 0x02;
const USBINTR: u64 = 0x04;
const FRNUM: u64 = 0x06;
const FLBASEADD: u64 = 0x08;
const SOFMOD: u64 = 0x0c;
const PORTSC1: u64 = 0x10;
const PORTSC2: u64 = 0x12;

const CMD_RUN_STOP: u16 = 1 << 0;
const CMD_HOST_RESET: u16 = 1 << 1;
const CMD_GLOBAL_RESET: u16 = 1 << 2;
const CMD_VALID_BITS: u16 = 0x00ff;

const STS_USBINT: u16 = 1 << 0;
const STS_USBERR: u16 = 1 << 1;
const STS_RESUME: u16 = 1 << 2;
const STS_HOST_SYSTEM_ERROR: u16 = 1 << 3;
const STS_HOST_CONTROLLER_ERROR: u16 = 1 << 4;
const STS_HALTED: u16 = 1 << 5;
const STS_WRITE_CLEAR: u16 =
    STS_USBINT | STS_USBERR | STS_RESUME | STS_HOST_SYSTEM_ERROR | STS_HOST_CONTROLLER_ERROR;

const PORT_CONNECTED: u16 = 1 << 0;
const PORT_CONNECT_CHANGE: u16 = 1 << 1;
const PORT_ENABLED: u16 = 1 << 2;
const PORT_ENABLE_CHANGE: u16 = 1 << 3;
const PORT_ALWAYS_ONE: u16 = 1 << 7;
const PORT_LOW_SPEED: u16 = 1 << 8;
const PORT_RESET: u16 = 1 << 9;
const PORT_SUSPEND: u16 = 1 << 12;
const PORT_READ_ONLY: u16 = 0x01bb;
const PORT_WRITE_CLEAR: u16 = PORT_CONNECT_CHANGE | PORT_ENABLE_CHANGE;

const LINK_TERMINATE: u32 = 1 << 0;
const LINK_QUEUE_HEAD: u32 = 1 << 1;
const LINK_DEPTH_FIRST: u32 = 1 << 2;
const LINK_ADDRESS_MASK: u32 = 0xffff_fff0;

const TD_CTRL_ACTUAL_LENGTH: u32 = 0x07ff;
const TD_CTRL_STATUS_MASK: u32 = 0x00fe_0000;
const TD_CTRL_TIMEOUT: u32 = 1 << 18;
const TD_CTRL_NAK: u32 = 1 << 19;
const TD_CTRL_BABBLE: u32 = 1 << 20;
const TD_CTRL_STALL: u32 = 1 << 22;
const TD_CTRL_ACTIVE: u32 = 1 << 23;
const TD_CTRL_IOC: u32 = 1 << 24;
const TD_CTRL_SPD: u32 = 1 << 29;

const PID_SETUP: u8 = 0x2d;
const PID_IN: u8 = 0x69;
const PID_OUT: u8 = 0xe1;

const INTERRUPT_CAUSE_IOC: u8 = 1 << 0;
const INTERRUPT_CAUSE_SHORT_PACKET: u8 = 1 << 1;

const MAX_SCHEDULE_ENTRIES: usize = 256;
const MAX_TRANSFER_SIZE: usize = 0x500;

#[derive(Default)]
struct InterruptState {
    event: Option<File>,
    level: bool,
    masked: bool,
}

/// One level-triggered PCI INTx line supplied by the vfio-user client.
#[derive(Clone, Default)]
pub(crate) struct InterruptLine {
    state: Arc<Mutex<InterruptState>>,
}

impl InterruptLine {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    pub(crate) fn install(&self, event: File) -> io::Result<()> {
        let mut state = self.state.lock().unwrap();
        state.event = Some(event);
        state.masked = false;
        Self::signal_if_needed(&mut state)
    }

    pub(crate) fn disable(&self) {
        let mut state = self.state.lock().unwrap();
        state.event = None;
        state.masked = false;
    }

    pub(crate) fn mask(&self) {
        self.state.lock().unwrap().masked = true;
    }

    pub(crate) fn unmask(&self) -> io::Result<()> {
        let mut state = self.state.lock().unwrap();
        state.masked = false;
        Self::signal_if_needed(&mut state)
    }

    fn set_level(&self, level: bool) -> io::Result<()> {
        let mut state = self.state.lock().unwrap();
        state.level = level;
        if !level {
            return Ok(());
        }
        Self::signal_if_needed(&mut state)
    }

    fn signal_if_needed(state: &mut InterruptState) -> io::Result<()> {
        if !state.level || state.masked {
            return Ok(());
        }
        if let Some(event) = &mut state.event {
            event.write_all(&1u64.to_ne_bytes())?;
            state.masked = true;
        }
        Ok(())
    }
}

pub struct UhciController {
    command: u16,
    status: u16,
    interrupt_enable: u16,
    interrupt_causes: u8,
    frame_number: u16,
    frame_list_base: u32,
    sof_modify: u8,
    ports: [u16; 2],
    bus: UsbBus,
    schedule_faulted: bool,
    interrupt: InterruptLine,
}

impl UhciController {
    pub(crate) fn new(interrupt: InterruptLine) -> Self {
        let mut this = Self {
            command: 0,
            status: STS_HALTED,
            interrupt_enable: 0,
            interrupt_causes: 0,
            frame_number: 0,
            frame_list_base: 0,
            sof_modify: 0x40,
            ports: [0; 2],
            bus: UsbBus::new(),
            schedule_faulted: false,
            interrupt,
        };
        this.reset();
        this
    }

    pub fn reset(&mut self) {
        self.command = 0;
        self.status = STS_HALTED;
        self.interrupt_enable = 0;
        self.interrupt_causes = 0;
        self.frame_number = 0;
        self.frame_list_base = 0;
        self.sof_modify = 0x40;
        self.reset_ports();
        self.schedule_faulted = false;
        let _ = self.interrupt.set_level(false);
    }

    fn reset_ports(&mut self) {
        self.bus.reset_port(0);
        self.bus.reset_port(1);
        self.ports
            .fill(PORT_ALWAYS_ONE | PORT_CONNECTED | PORT_CONNECT_CHANGE | PORT_LOW_SPEED);
    }

    pub fn is_running(&self) -> bool {
        self.command & CMD_RUN_STOP != 0 && !self.schedule_faulted
    }

    pub fn input_key(&mut self, usage: u8, pressed: bool) {
        self.bus.key(usage, pressed);
    }

    pub fn input_mouse(&mut self, dx: i32, dy: i32, wheel: i16, buttons: u8) {
        self.bus.mouse(dx, dy, wheel, buttons);
    }

    pub fn release_all_input(&mut self) {
        self.bus.release_all();
    }

    pub fn keyboard_leds(&self) -> u8 {
        self.bus.keyboard_leds()
    }

    #[cfg(test)]
    pub(crate) fn take_keyboard_report(&mut self) -> Option<[u8; 8]> {
        self.bus.take_keyboard_report()
    }

    #[cfg(test)]
    pub(crate) fn take_mouse_report(&mut self) -> Option<Vec<u8>> {
        self.bus.take_mouse_report()
    }

    pub fn tick(&mut self, memory: &GuestMemoryMap) {
        if !self.is_running() {
            return;
        }
        if let Err(error) = self.process_frame(memory) {
            warn!("UHCI schedule stopped after invalid guest DMA: {error}");
            self.schedule_faulted = true;
            self.command &= !CMD_RUN_STOP;
            self.status |= STS_HOST_CONTROLLER_ERROR | STS_HALTED;
            let _ = self.update_interrupt();
        }
    }

    pub fn read(&self, offset: u64, data: &mut [u8]) -> io::Result<()> {
        let mut registers = [0xff; UHCI_IO_BAR_SIZE as usize];
        registers[USBCMD as usize..USBCMD as usize + 2]
            .copy_from_slice(&self.command.to_le_bytes());
        registers[USBSTS as usize..USBSTS as usize + 2].copy_from_slice(&self.status.to_le_bytes());
        registers[USBINTR as usize..USBINTR as usize + 2]
            .copy_from_slice(&self.interrupt_enable.to_le_bytes());
        registers[FRNUM as usize..FRNUM as usize + 2]
            .copy_from_slice(&self.frame_number.to_le_bytes());
        registers[FLBASEADD as usize..FLBASEADD as usize + 4]
            .copy_from_slice(&self.frame_list_base.to_le_bytes());
        registers[SOFMOD as usize] = self.sof_modify;
        registers[PORTSC1 as usize..PORTSC1 as usize + 2]
            .copy_from_slice(&self.ports[0].to_le_bytes());
        registers[PORTSC2 as usize..PORTSC2 as usize + 2]
            .copy_from_slice(&self.ports[1].to_le_bytes());

        let range = checked_io_range(offset, data.len())?;
        data.copy_from_slice(&registers[range]);
        Ok(())
    }

    pub fn write(&mut self, offset: u64, data: &[u8]) -> io::Result<()> {
        checked_io_range(offset, data.len())?;
        match (offset, data.len()) {
            (USBCMD, 2) => self.write_command(u16::from_le_bytes(data.try_into().unwrap())),
            (USBSTS, 2) => {
                let clear = u16::from_le_bytes(data.try_into().unwrap()) & STS_WRITE_CLEAR;
                self.status &= !clear;
                if clear & STS_USBINT != 0 {
                    self.interrupt_causes = 0;
                }
                self.update_interrupt()?;
            }
            (USBINTR, 2) => {
                self.interrupt_enable = u16::from_le_bytes(data.try_into().unwrap()) & 0x000f;
                self.update_interrupt()?;
            }
            (FRNUM, 2) if self.status & STS_HALTED != 0 => {
                self.frame_number = u16::from_le_bytes(data.try_into().unwrap()) & 0x07ff;
            }
            (FLBASEADD, 4) => {
                self.frame_list_base = u32::from_le_bytes(data.try_into().unwrap()) & 0xffff_f000;
            }
            (FLBASEADD, 2) => {
                self.frame_list_base = (self.frame_list_base & 0xffff_0000)
                    | u32::from(u16::from_le_bytes(data.try_into().unwrap()) & 0xf000);
            }
            (offset, 2) if offset == FLBASEADD + 2 => {
                self.frame_list_base = (self.frame_list_base & 0x0000_ffff)
                    | (u32::from(u16::from_le_bytes(data.try_into().unwrap())) << 16);
            }
            (SOFMOD, 1) => self.sof_modify = data[0] & 0x7f,
            (PORTSC1, 2) => self.write_port(0, u16::from_le_bytes(data.try_into().unwrap())),
            (PORTSC2, 2) => self.write_port(1, u16::from_le_bytes(data.try_into().unwrap())),
            _ => {}
        }
        Ok(())
    }

    fn write_command(&mut self, value: u16) {
        if value & (CMD_HOST_RESET | CMD_GLOBAL_RESET) != 0 {
            self.reset();
            return;
        }
        self.command = value & CMD_VALID_BITS;
        if self.command & CMD_RUN_STOP != 0 {
            self.status &= !STS_HALTED;
        } else {
            self.status |= STS_HALTED;
        }
        let _ = self.update_interrupt();
    }

    fn write_port(&mut self, index: usize, value: u16) {
        let previous = self.ports[index];
        if value & PORT_RESET != 0 && previous & PORT_RESET == 0 {
            self.bus.reset_port(index);
        }
        let read_only = previous & PORT_READ_ONLY;
        let mut next = read_only | (value & !PORT_READ_ONLY);
        next &= !(value & PORT_WRITE_CLEAR);
        if next & PORT_CONNECTED == 0 {
            next &= !PORT_ENABLED;
        }
        if previous & PORT_RESET != 0 && value & PORT_RESET == 0 {
            next |= PORT_ENABLED | PORT_ENABLE_CHANGE;
        }
        if next & PORT_RESET != 0 {
            next &= !PORT_SUSPEND;
        }
        next |= PORT_ALWAYS_ONE | PORT_CONNECTED | PORT_LOW_SPEED;
        if previous & PORT_ENABLED != next & PORT_ENABLED {
            next |= PORT_ENABLE_CHANGE;
        }
        self.ports[index] = next;
    }

    fn update_interrupt(&self) -> io::Result<()> {
        let enabled = (self.status & STS_USBINT != 0
            && ((self.interrupt_causes & INTERRUPT_CAUSE_IOC != 0
                && self.interrupt_enable & (1 << 2) != 0)
                || (self.interrupt_causes & INTERRUPT_CAUSE_SHORT_PACKET != 0
                    && self.interrupt_enable & (1 << 3) != 0)))
            || (self.status & STS_USBERR != 0 && self.interrupt_enable & (1 << 0) != 0)
            || (self.status & STS_RESUME != 0 && self.interrupt_enable & (1 << 1) != 0)
            || self.status & (STS_HOST_SYSTEM_ERROR | STS_HOST_CONTROLLER_ERROR) != 0;
        self.interrupt.set_level(enabled)
    }

    fn process_frame(&mut self, memory: &GuestMemoryMap) -> io::Result<()> {
        let entry_offset = u64::from(self.frame_number & 0x03ff) * 4;
        let entry_address = u64::from(self.frame_list_base)
            .checked_add(entry_offset)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "frame list overflows"))?;
        let mut link = memory.read_u32(GuestAddress(entry_address))?;
        let mut current_qh: Option<QueueHead> = None;
        let mut seen_qh = HashSet::new();
        let mut seen_td = HashSet::new();

        for _ in 0..MAX_SCHEDULE_ENTRIES {
            if link & LINK_TERMINATE != 0 {
                break;
            }
            let address = link & LINK_ADDRESS_MASK;
            if link & LINK_QUEUE_HEAD != 0 {
                if !seen_qh.insert(address) {
                    break;
                }
                let qh = QueueHead::read(memory, address)?;
                if qh.element & LINK_TERMINATE != 0 {
                    link = qh.horizontal;
                    current_qh = None;
                } else {
                    link = qh.element;
                    current_qh = Some(qh);
                }
                continue;
            }

            if !seen_td.insert(address) {
                break;
            }
            let mut td = TransferDescriptor::read(memory, address)?;
            let result = self.service_td(memory, &mut td)?;
            td.write_control(memory, address)?;

            match result {
                TdResult::Complete => {
                    let next = td.link;
                    if let Some(qh) = current_qh.as_mut() {
                        qh.element = next;
                        qh.write_element(memory)?;
                        if next & LINK_DEPTH_FIRST != 0 {
                            link = next;
                        } else {
                            link = qh.horizontal;
                            current_qh = None;
                        }
                    } else {
                        link = next;
                    }
                }
                TdResult::ShortPacket | TdResult::StopQueue => {
                    link = current_qh.as_ref().map_or(td.link, |qh| qh.horizontal);
                    current_qh = None;
                }
            }
        }

        self.frame_number = (self.frame_number + 1) & 0x07ff;
        self.update_interrupt()
    }

    fn service_td(
        &mut self,
        memory: &GuestMemoryMap,
        td: &mut TransferDescriptor,
    ) -> io::Result<TdResult> {
        if td.control & TD_CTRL_ACTIVE == 0 {
            if td.control & TD_CTRL_IOC != 0 {
                self.raise_usb_interrupt(INTERRUPT_CAUSE_IOC);
            }
            return Ok(TdResult::StopQueue);
        }
        let pid = match td.token as u8 {
            PID_SETUP => UsbPid::Setup,
            PID_IN => UsbPid::In,
            PID_OUT => UsbPid::Out,
            _ => {
                self.command &= !CMD_RUN_STOP;
                self.status |= STS_HOST_CONTROLLER_ERROR | STS_HALTED;
                return Ok(TdResult::StopQueue);
            }
        };
        let endpoint = ((td.token >> 15) & 0x0f) as u8;
        if pid == UsbPid::Setup && endpoint != 0 {
            self.command &= !CMD_RUN_STOP;
            self.status |= STS_HOST_CONTROLLER_ERROR | STS_HALTED;
            return Ok(TdResult::StopQueue);
        }
        let maximum_length = ((td.token >> 21) + 1) as usize & 0x07ff;
        if maximum_length > MAX_TRANSFER_SIZE {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "UHCI TD transfer length exceeds the frame budget",
            ));
        }
        let output = if pid == UsbPid::In || maximum_length == 0 {
            Vec::new()
        } else {
            memory.read_vec(GuestAddress(u64::from(td.buffer)), maximum_length)?
        };
        let function_address = ((td.token >> 8) & 0x7f) as u8;
        let Some(port) = (0..self.ports.len()).find(|port| {
            self.ports[*port] & PORT_ENABLED != 0
                && self.bus.address(*port) == Some(function_address)
        }) else {
            return Ok(self.complete_error(td, TD_CTRL_TIMEOUT));
        };

        match self
            .bus
            .packet(port, pid, endpoint, &output, maximum_length)
        {
            UsbPacketResult::Nak => {
                td.control |= TD_CTRL_NAK;
                Ok(TdResult::StopQueue)
            }
            UsbPacketResult::NoDevice => Ok(self.complete_error(td, TD_CTRL_TIMEOUT)),
            UsbPacketResult::Stall => Ok(self.complete_error(td, TD_CTRL_STALL)),
            UsbPacketResult::Success(input) => {
                if input.len() > maximum_length {
                    return Ok(self.complete_error(td, TD_CTRL_BABBLE | TD_CTRL_STALL));
                }
                if pid == UsbPid::In && !input.is_empty() {
                    memory.write(GuestAddress(u64::from(td.buffer)), &input)?;
                }
                let actual_length = if pid == UsbPid::In {
                    input.len()
                } else {
                    maximum_length
                };
                td.control &= !(TD_CTRL_STATUS_MASK | TD_CTRL_ACTUAL_LENGTH);
                td.control |= actual_length.wrapping_sub(1) as u32 & TD_CTRL_ACTUAL_LENGTH;
                if td.control & TD_CTRL_IOC != 0 {
                    self.raise_usb_interrupt(INTERRUPT_CAUSE_IOC);
                }
                if pid == UsbPid::In
                    && td.control & TD_CTRL_SPD != 0
                    && actual_length < maximum_length
                {
                    self.raise_usb_interrupt(INTERRUPT_CAUSE_SHORT_PACKET);
                    Ok(TdResult::ShortPacket)
                } else {
                    Ok(TdResult::Complete)
                }
            }
        }
    }

    fn complete_error(&mut self, td: &mut TransferDescriptor, status: u32) -> TdResult {
        td.control &= !TD_CTRL_ACTIVE;
        td.control |= status;
        self.status |= STS_USBERR;
        if td.control & TD_CTRL_IOC != 0 {
            self.raise_usb_interrupt(INTERRUPT_CAUSE_IOC);
        }
        TdResult::StopQueue
    }

    fn raise_usb_interrupt(&mut self, cause: u8) {
        self.interrupt_causes |= cause;
        self.status |= STS_USBINT;
    }
}

#[derive(Clone, Copy)]
struct QueueHead {
    address: u32,
    horizontal: u32,
    element: u32,
}

impl QueueHead {
    fn read(memory: &GuestMemoryMap, address: u32) -> io::Result<Self> {
        Ok(Self {
            address,
            horizontal: memory.read_u32(GuestAddress(u64::from(address)))?,
            element: memory.read_u32(GuestAddress(u64::from(address) + 4))?,
        })
    }

    fn write_element(&self, memory: &GuestMemoryMap) -> io::Result<()> {
        memory.write_u32(GuestAddress(u64::from(self.address) + 4), self.element)
    }
}

struct TransferDescriptor {
    link: u32,
    control: u32,
    token: u32,
    buffer: u32,
}

impl TransferDescriptor {
    fn read(memory: &GuestMemoryMap, address: u32) -> io::Result<Self> {
        let bytes = memory.read_vec(GuestAddress(u64::from(address)), 16)?;
        Ok(Self {
            link: u32::from_le_bytes(bytes[0..4].try_into().unwrap()),
            control: u32::from_le_bytes(bytes[4..8].try_into().unwrap()),
            token: u32::from_le_bytes(bytes[8..12].try_into().unwrap()),
            buffer: u32::from_le_bytes(bytes[12..16].try_into().unwrap()),
        })
    }

    fn write_control(&self, memory: &GuestMemoryMap, address: u32) -> io::Result<()> {
        let address = address.checked_add(4).ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidInput, "UHCI TD address overflows")
        })?;
        memory.write_u32(GuestAddress(u64::from(address)), self.control)
    }
}

enum TdResult {
    Complete,
    ShortPacket,
    StopQueue,
}

fn checked_io_range(offset: u64, length: usize) -> io::Result<std::ops::Range<usize>> {
    let start = usize::try_from(offset)
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "UHCI I/O offset overflow"))?;
    let end = start
        .checked_add(length)
        .filter(|end| *end <= UHCI_IO_BAR_SIZE as usize)
        .ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidInput, "UHCI I/O access out of range")
        })?;
    Ok(start..end)
}

#[cfg(test)]
mod tests {
    use std::fs::OpenOptions;

    use vfio_user::{DmaMapFlags, DmaUnmapFlags};

    use super::*;

    fn read_u16(controller: &UhciController, offset: u64) -> u16 {
        let mut data = [0; 2];
        controller.read(offset, &mut data).unwrap();
        u16::from_le_bytes(data)
    }

    fn mapped_memory(size: u64) -> GuestMemoryMap {
        let path = std::env::temp_dir().join(format!(
            "vfio-usb-hid-uhci-test-{}-{:?}",
            std::process::id(),
            std::thread::current().id()
        ));
        let file = OpenOptions::new()
            .create_new(true)
            .read(true)
            .write(true)
            .open(&path)
            .unwrap();
        file.set_len(size).unwrap();
        std::fs::remove_file(path).unwrap();
        let memory = GuestMemoryMap::new();
        memory
            .map(
                DmaMapFlags::READ_WRITE,
                0,
                GuestAddress(0),
                size,
                Some(file),
            )
            .unwrap();
        memory
    }

    fn token(pid: u8, address: u8, endpoint: u8, length: usize) -> u32 {
        let encoded_length = length.wrapping_sub(1) as u32 & 0x07ff;
        u32::from(pid)
            | (u32::from(address) << 8)
            | (u32::from(endpoint) << 15)
            | (encoded_length << 21)
    }

    fn write_td(
        memory: &GuestMemoryMap,
        address: u32,
        link: u32,
        control: u32,
        token: u32,
        buffer: u32,
    ) {
        let mut bytes = [0; 16];
        bytes[0..4].copy_from_slice(&link.to_le_bytes());
        bytes[4..8].copy_from_slice(&control.to_le_bytes());
        bytes[8..12].copy_from_slice(&token.to_le_bytes());
        bytes[12..16].copy_from_slice(&buffer.to_le_bytes());
        memory
            .write(GuestAddress(u64::from(address)), &bytes)
            .unwrap();
    }

    #[test]
    fn reset_values_and_run_stop_are_uhci_compatible() {
        let mut controller = UhciController::new(InterruptLine::new());
        assert_eq!(read_u16(&controller, USBCMD), 0);
        assert_eq!(read_u16(&controller, USBSTS), STS_HALTED);
        assert_eq!(
            read_u16(&controller, PORTSC1),
            PORT_ALWAYS_ONE | PORT_CONNECTED | PORT_CONNECT_CHANGE | PORT_LOW_SPEED
        );

        controller.write(USBCMD, &1u16.to_le_bytes()).unwrap();
        assert_eq!(read_u16(&controller, USBSTS) & STS_HALTED, 0);
        controller.write(USBCMD, &0u16.to_le_bytes()).unwrap();
        assert_ne!(read_u16(&controller, USBSTS) & STS_HALTED, 0);
    }

    #[test]
    fn frame_registers_apply_masks_and_running_rule() {
        let mut controller = UhciController::new(InterruptLine::new());
        controller.write(FRNUM, &0xffffu16.to_le_bytes()).unwrap();
        assert_eq!(read_u16(&controller, FRNUM), 0x07ff);
        controller
            .write(FLBASEADD, &0x1234_5678u32.to_le_bytes())
            .unwrap();
        let mut base = [0; 4];
        controller.read(FLBASEADD, &mut base).unwrap();
        assert_eq!(u32::from_le_bytes(base), 0x1234_5000);

        controller.write(USBCMD, &1u16.to_le_bytes()).unwrap();
        controller.write(FRNUM, &7u16.to_le_bytes()).unwrap();
        assert_eq!(read_u16(&controller, FRNUM), 0x07ff);
    }

    #[test]
    fn port_reset_enables_attached_low_speed_device() {
        let mut controller = UhciController::new(InterruptLine::new());
        controller
            .write(PORTSC1, &PORT_RESET.to_le_bytes())
            .unwrap();
        controller.write(PORTSC1, &0u16.to_le_bytes()).unwrap();
        let port = read_u16(&controller, PORTSC1);
        assert_ne!(port & PORT_CONNECTED, 0);
        assert_ne!(port & PORT_ENABLED, 0);
        assert_ne!(port & PORT_LOW_SPEED, 0);
    }

    #[test]
    fn synthetic_control_schedule_returns_device_descriptor() {
        let memory = mapped_memory(0x10000);
        let mut controller = UhciController::new(InterruptLine::new());
        controller
            .write(PORTSC1, &PORT_RESET.to_le_bytes())
            .unwrap();
        controller.write(PORTSC1, &0u16.to_le_bytes()).unwrap();

        memory
            .write_u32(GuestAddress(0x1000), 0x2000 | LINK_QUEUE_HEAD)
            .unwrap();
        memory
            .write_u32(GuestAddress(0x2000), LINK_TERMINATE)
            .unwrap();
        memory.write_u32(GuestAddress(0x2004), 0x2100).unwrap();
        let setup = [0x80, 0x06, 0, 1, 0, 0, 18, 0];
        memory.write(GuestAddress(0x3000), &setup).unwrap();
        write_td(
            &memory,
            0x2100,
            0x2140 | LINK_DEPTH_FIRST,
            TD_CTRL_ACTIVE,
            token(PID_SETUP, 0, 0, 8),
            0x3000,
        );
        write_td(
            &memory,
            0x2140,
            0x2180 | LINK_DEPTH_FIRST,
            TD_CTRL_ACTIVE | TD_CTRL_SPD,
            token(PID_IN, 0, 0, 18),
            0x3020,
        );
        write_td(
            &memory,
            0x2180,
            LINK_TERMINATE,
            TD_CTRL_ACTIVE | TD_CTRL_IOC,
            token(PID_OUT, 0, 0, 0),
            0,
        );

        controller
            .write(FLBASEADD, &0x1000u32.to_le_bytes())
            .unwrap();
        controller
            .write(USBINTR, &(1u16 << 2).to_le_bytes())
            .unwrap();
        controller
            .write(USBCMD, &CMD_RUN_STOP.to_le_bytes())
            .unwrap();
        controller.tick(&memory);

        assert_eq!(
            memory.read_vec(GuestAddress(0x3020), 18).unwrap(),
            crate::usb::hid::device_descriptor(0x0100)
        );
        assert_eq!(
            memory.read_u32(GuestAddress(0x2004)).unwrap(),
            LINK_TERMINATE
        );
        assert_ne!(controller.status & STS_USBINT, 0);
        assert_eq!(controller.frame_number, 1);
    }

    #[test]
    fn invalid_or_unmapped_schedule_halts_without_host_access() {
        let mut controller = UhciController::new(InterruptLine::new());
        controller
            .write(FLBASEADD, &0x1000u32.to_le_bytes())
            .unwrap();
        controller
            .write(USBCMD, &CMD_RUN_STOP.to_le_bytes())
            .unwrap();
        controller.tick(&GuestMemoryMap::new());
        assert_ne!(controller.status & STS_HALTED, 0);
        assert_ne!(controller.status & STS_HOST_CONTROLLER_ERROR, 0);
        assert!(!controller.is_running());
    }

    #[test]
    fn self_referencing_queue_head_is_bounded() {
        let memory = mapped_memory(0x4000);
        memory
            .write_u32(GuestAddress(0x1000), 0x2000 | LINK_QUEUE_HEAD)
            .unwrap();
        memory
            .write_u32(GuestAddress(0x2000), 0x2000 | LINK_QUEUE_HEAD)
            .unwrap();
        memory
            .write_u32(GuestAddress(0x2004), LINK_TERMINATE)
            .unwrap();
        let mut controller = UhciController::new(InterruptLine::new());
        controller
            .write(FLBASEADD, &0x1000u32.to_le_bytes())
            .unwrap();
        controller
            .write(USBCMD, &CMD_RUN_STOP.to_le_bytes())
            .unwrap();
        controller.tick(&memory);
        assert_eq!(controller.frame_number, 1);
        assert!(controller.is_running());
    }

    #[test]
    fn interrupt_in_naks_until_a_report_is_available() {
        let memory = mapped_memory(0x10000);
        let mut controller = UhciController::new(InterruptLine::new());
        controller
            .write(PORTSC1, &PORT_RESET.to_le_bytes())
            .unwrap();
        controller.write(PORTSC1, &0u16.to_le_bytes()).unwrap();

        memory
            .write_u32(GuestAddress(0x1000), 0x2000 | LINK_QUEUE_HEAD)
            .unwrap();
        memory.write_u32(GuestAddress(0x1004), 0x2200).unwrap();
        memory.write_u32(GuestAddress(0x1008), 0x2200).unwrap();
        memory
            .write_u32(GuestAddress(0x2000), LINK_TERMINATE)
            .unwrap();
        memory.write_u32(GuestAddress(0x2004), 0x2100).unwrap();
        memory
            .write(GuestAddress(0x3000), &[0, 9, 1, 0, 0, 0, 0, 0])
            .unwrap();
        write_td(
            &memory,
            0x2100,
            0x2140 | LINK_DEPTH_FIRST,
            TD_CTRL_ACTIVE,
            token(PID_SETUP, 0, 0, 8),
            0x3000,
        );
        write_td(
            &memory,
            0x2140,
            LINK_TERMINATE,
            TD_CTRL_ACTIVE,
            token(PID_IN, 0, 0, 0),
            0,
        );
        write_td(
            &memory,
            0x2200,
            LINK_TERMINATE,
            TD_CTRL_ACTIVE | TD_CTRL_IOC | TD_CTRL_SPD,
            token(PID_IN, 0, 1, 8),
            0x3040,
        );

        controller
            .write(FLBASEADD, &0x1000u32.to_le_bytes())
            .unwrap();
        controller
            .write(USBINTR, &(1u16 << 2).to_le_bytes())
            .unwrap();
        controller
            .write(USBCMD, &CMD_RUN_STOP.to_le_bytes())
            .unwrap();
        controller.tick(&memory);
        controller.tick(&memory);
        let nak_control = memory.read_u32(GuestAddress(0x2204)).unwrap();
        assert_ne!(nak_control & TD_CTRL_ACTIVE, 0);
        assert_ne!(nak_control & TD_CTRL_NAK, 0);

        controller.input_key(4, true);
        controller.tick(&memory);
        let completed_control = memory.read_u32(GuestAddress(0x2204)).unwrap();
        assert_eq!(completed_control & (TD_CTRL_ACTIVE | TD_CTRL_NAK), 0);
        assert_eq!(completed_control & TD_CTRL_ACTUAL_LENGTH, 7);
        assert_eq!(
            memory.read_vec(GuestAddress(0x3040), 8).unwrap(),
            [0, 0, 4, 0, 0, 0, 0, 0]
        );
        assert_ne!(controller.status & STS_USBINT, 0);
    }

    #[test]
    fn invalid_and_truncated_descriptors_halt_safely() {
        for (size, link) in [(0x2000, 0x3000 | LINK_QUEUE_HEAD), (0x3008, 0x3000)] {
            let memory = mapped_memory(size);
            memory.write_u32(GuestAddress(0x1000), link).unwrap();
            let mut controller = UhciController::new(InterruptLine::new());
            controller
                .write(FLBASEADD, &0x1000u32.to_le_bytes())
                .unwrap();
            controller
                .write(USBCMD, &CMD_RUN_STOP.to_le_bytes())
                .unwrap();
            controller.tick(&memory);
            assert!(!controller.is_running());
            assert_ne!(controller.status & STS_HOST_CONTROLLER_ERROR, 0);
        }
    }

    #[test]
    fn td_cycles_and_excessive_chains_are_bounded() {
        let memory = mapped_memory(0x10000);
        memory.write_u32(GuestAddress(0x1000), 0x2000).unwrap();
        write_td(&memory, 0x2000, 0x2010, 0, token(PID_IN, 0, 0, 0), 0);
        write_td(&memory, 0x2010, 0x2000, 0, token(PID_IN, 0, 0, 0), 0);
        let mut controller = UhciController::new(InterruptLine::new());
        controller
            .write(FLBASEADD, &0x1000u32.to_le_bytes())
            .unwrap();
        controller
            .write(USBCMD, &CMD_RUN_STOP.to_le_bytes())
            .unwrap();
        controller.tick(&memory);
        assert_eq!(controller.frame_number, 1);

        memory.write_u32(GuestAddress(0x1004), 0x4000).unwrap();
        for index in 0..=MAX_SCHEDULE_ENTRIES {
            let address = 0x4000 + (index as u32 * 0x10);
            write_td(
                &memory,
                address,
                address + 0x10,
                0,
                token(PID_IN, 0, 0, 0),
                0,
            );
        }
        controller.tick(&memory);
        assert_eq!(controller.frame_number, 2);
        assert!(controller.is_running());
    }

    #[test]
    fn dma_unmap_and_overflowing_buffers_stop_schedule_access() {
        let memory = mapped_memory(0x4000);
        memory
            .write_u32(GuestAddress(0x1000), LINK_TERMINATE)
            .unwrap();
        let mut controller = UhciController::new(InterruptLine::new());
        controller
            .write(FLBASEADD, &0x1000u32.to_le_bytes())
            .unwrap();
        controller
            .write(USBCMD, &CMD_RUN_STOP.to_le_bytes())
            .unwrap();
        memory
            .unmap(DmaUnmapFlags::empty(), GuestAddress(0), 0x4000)
            .unwrap();
        controller.tick(&memory);
        assert!(!controller.is_running());

        let memory = mapped_memory(0x4000);
        memory.write_u32(GuestAddress(0x1000), 0x2000).unwrap();
        write_td(
            &memory,
            0x2000,
            LINK_TERMINATE,
            TD_CTRL_ACTIVE,
            token(PID_OUT, 0, 0, 32),
            0xffff_fff0,
        );
        let mut controller = UhciController::new(InterruptLine::new());
        controller
            .write(FLBASEADD, &0x1000u32.to_le_bytes())
            .unwrap();
        controller
            .write(USBCMD, &CMD_RUN_STOP.to_le_bytes())
            .unwrap();
        controller.tick(&memory);
        assert!(!controller.is_running());
    }
}
