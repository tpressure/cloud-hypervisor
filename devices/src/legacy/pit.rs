// Copyright 2026 Cyberus Technology GmbH
//
// SPDX-License-Identifier: Apache-2.0

use std::time::Instant;

use vm_device::BusDevice;

const PIT_FREQUENCY_HZ: u128 = 1_193_182;
const NANOS_PER_SECOND: u128 = 1_000_000_000;
const PIT_CHANNELS: usize = 3;
const PIT_COUNTER_RELOAD: u32 = 0x1_0000;

#[derive(Clone, Copy)]
struct PitChannel {
    reload: u32,
    started: Instant,
    access_mode: u8,
    write_lsb: Option<u8>,
    read_lsb_next: bool,
    latched_count: Option<u16>,
    latched_lsb_next: bool,
}

impl PitChannel {
    fn new() -> Self {
        Self {
            reload: PIT_COUNTER_RELOAD,
            started: Instant::now(),
            access_mode: 3,
            write_lsb: None,
            read_lsb_next: true,
            latched_count: None,
            latched_lsb_next: true,
        }
    }

    fn current_count(&self) -> u16 {
        let ticks =
            (self.started.elapsed().as_nanos() * PIT_FREQUENCY_HZ / NANOS_PER_SECOND) as u32;
        let elapsed = ticks % self.reload;
        let count = self.reload - elapsed;

        if count == PIT_COUNTER_RELOAD {
            0
        } else {
            count as u16
        }
    }

    fn latch_count(&mut self) {
        self.latched_count = Some(self.current_count());
        self.latched_lsb_next = true;
    }

    fn set_reload(&mut self, value: u16) {
        self.reload = if value == 0 {
            PIT_COUNTER_RELOAD
        } else {
            u32::from(value)
        };
        self.started = Instant::now();
        self.read_lsb_next = true;
        self.latched_count = None;
    }

    fn set_access_mode(&mut self, access_mode: u8) {
        self.access_mode = access_mode;
        self.write_lsb = None;
        self.read_lsb_next = true;
    }

    fn read_counter(&mut self) -> u8 {
        if let Some(count) = self.latched_count {
            if self.latched_lsb_next {
                self.latched_lsb_next = false;
                count as u8
            } else {
                self.latched_count = None;
                self.latched_lsb_next = true;
                (count >> 8) as u8
            }
        } else {
            let count = self.current_count();

            match self.access_mode {
                1 => count as u8,
                2 => (count >> 8) as u8,
                _ if self.read_lsb_next => {
                    self.read_lsb_next = false;
                    count as u8
                }
                _ => {
                    self.read_lsb_next = true;
                    (count >> 8) as u8
                }
            }
        }
    }

    fn write_counter(&mut self, value: u8) {
        match self.access_mode {
            1 => self.set_reload(u16::from(value)),
            2 => self.set_reload(u16::from(value) << 8),
            _ => {
                if let Some(lsb) = self.write_lsb.take() {
                    self.set_reload(u16::from(lsb) | (u16::from(value) << 8));
                } else {
                    self.write_lsb = Some(value);
                }
            }
        }
    }
}

/// Minimal i8254 PIT emulation for guests that use the legacy timer during
/// early boot calibration.
pub struct Pit {
    channels: [PitChannel; PIT_CHANNELS],
}

impl Pit {
    pub fn new() -> Self {
        Self {
            channels: std::array::from_fn(|_| PitChannel::new()),
        }
    }

    fn write_command(&mut self, command: u8) {
        let channel = usize::from(command >> 6);
        if channel >= PIT_CHANNELS {
            return;
        }

        let access_mode = (command >> 4) & 0x3;
        if access_mode == 0 {
            self.channels[channel].latch_count();
        } else {
            self.channels[channel].set_access_mode(access_mode);
        }
    }
}

impl BusDevice for Pit {
    fn read(&mut self, _base: u64, offset: u64, data: &mut [u8]) {
        if data.len() != 1 {
            return;
        }

        if let Some(channel) = self.channels.get_mut(offset as usize) {
            data[0] = channel.read_counter();
        }
    }

    fn write(
        &mut self,
        _base: u64,
        offset: u64,
        data: &[u8],
    ) -> Option<std::sync::Arc<std::sync::Barrier>> {
        if data.len() != 1 {
            return None;
        }

        if offset == 3 {
            self.write_command(data[0]);
        } else if let Some(channel) = self.channels.get_mut(offset as usize) {
            channel.write_counter(data[0]);
        }

        None
    }
}
