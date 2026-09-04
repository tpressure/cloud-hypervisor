// Copyright © 2026 Cyberus Technology GmbH
//
// SPDX-License-Identifier: Apache-2.0

use std::io;
use std::mem::size_of;
use std::ops::Range;

use vfio_bindings::bindings::vfio::{
    VFIO_IRQ_INFO_AUTOMASKED, VFIO_IRQ_INFO_EVENTFD, VFIO_IRQ_INFO_MASKABLE,
    VFIO_PCI_BAR4_REGION_INDEX, VFIO_PCI_CONFIG_REGION_INDEX, VFIO_PCI_INTX_IRQ_INDEX,
    VFIO_PCI_REQ_IRQ_INDEX, VFIO_REGION_INFO_FLAG_READ, VFIO_REGION_INFO_FLAG_WRITE,
    vfio_region_info,
};
use vfio_user::{IrqInfo, ServerRegion};

use crate::uhci::UHCI_IO_BAR_SIZE;

pub const PCI_VENDOR_ID: u16 = 0x1b36;
// Prototype ID in the Red Hat/QEMU virtual-device namespace. It is not a
// registered QEMU model ID and must be replaced before production use.
pub const PCI_DEVICE_ID: u16 = 0x00fe;
const PCI_CONFIG_SPACE_SIZE: usize = 4096;
const PCI_REGION_COUNT: usize = VFIO_PCI_CONFIG_REGION_INDEX as usize + 1;
const UHCI_BAR_OFFSET: usize = 0x20;
const UHCI_BAR_RANGE: Range<usize> = UHCI_BAR_OFFSET..UHCI_BAR_OFFSET + 4;
const UHCI_BAR_RESET_VALUE: u32 = 0x0000_0001;
const UHCI_BAR_SIZE_MASK: u32 = 0xffff_ffe1;

pub struct PciConfig {
    bytes: [u8; PCI_CONFIG_SPACE_SIZE],
}

impl PciConfig {
    pub fn new() -> Self {
        let mut this = Self {
            bytes: [0; PCI_CONFIG_SPACE_SIZE],
        };
        this.reset();
        this
    }

    pub fn reset(&mut self) {
        self.bytes.fill(0);
        self.bytes[UHCI_BAR_RANGE].copy_from_slice(&UHCI_BAR_RESET_VALUE.to_le_bytes());
        self.restore_read_only_fields();
    }

    fn restore_read_only_fields(&mut self) {
        self.bytes[0..2].copy_from_slice(&PCI_VENDOR_ID.to_le_bytes());
        self.bytes[2..4].copy_from_slice(&PCI_DEVICE_ID.to_le_bytes());
        self.bytes[6..8].fill(0); // No PCI capabilities.
        self.bytes[0x08] = 0x01; // Revision.
        self.bytes[0x09] = 0x00; // UHCI programming interface.
        self.bytes[0x0a] = 0x03; // USB subclass.
        self.bytes[0x0b] = 0x0c; // Serial bus controller class.
        self.bytes[0x0e] = 0x00; // Single-function endpoint.
        self.bytes[0x10..0x20].fill(0); // BAR0-BAR3 are absent.
        self.bytes[0x24..0x28].fill(0); // BAR5 is absent.
        self.bytes[0x2c..0x2e].copy_from_slice(&PCI_VENDOR_ID.to_le_bytes());
        self.bytes[0x2e..0x30].copy_from_slice(&PCI_DEVICE_ID.to_le_bytes());
        self.bytes[0x30..0x34].fill(0); // Expansion ROM is absent.
        self.bytes[0x34] = 0;
        self.bytes[0x3d] = 1; // INTA#.
        self.bytes[0x60] = 0x10; // USB 1.0 Serial Bus Release Number.
    }

    fn checked_range(offset: u64, length: usize) -> io::Result<Range<usize>> {
        let start = usize::try_from(offset)
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "PCI offset overflow"))?;
        let end = start
            .checked_add(length)
            .filter(|end| *end <= PCI_CONFIG_SPACE_SIZE)
            .ok_or_else(|| {
                io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "PCI config access out of bounds",
                )
            })?;
        Ok(start..end)
    }

    pub fn read(&self, offset: u64, data: &mut [u8]) -> io::Result<()> {
        let range = Self::checked_range(offset, data.len())?;
        data.copy_from_slice(&self.bytes[range]);
        Ok(())
    }

    pub fn write(&mut self, offset: u64, data: &[u8]) -> io::Result<()> {
        let range = Self::checked_range(offset, data.len())?;
        let bar_probe = range == UHCI_BAR_RANGE && data == u32::MAX.to_le_bytes();
        self.bytes[range].copy_from_slice(data);
        self.restore_read_only_fields();
        if bar_probe {
            self.bytes[UHCI_BAR_RANGE].copy_from_slice(&UHCI_BAR_SIZE_MASK.to_le_bytes());
        } else {
            let bar = u32::from_le_bytes(self.bytes[UHCI_BAR_RANGE].try_into().unwrap());
            self.bytes[UHCI_BAR_RANGE].copy_from_slice(&(bar | 1).to_le_bytes());
        }
        Ok(())
    }

    pub fn regions() -> Vec<ServerRegion> {
        (0..PCI_REGION_COUNT)
            .map(|index| {
                let implemented = index == VFIO_PCI_BAR4_REGION_INDEX as usize
                    || index == VFIO_PCI_CONFIG_REGION_INDEX as usize;
                ServerRegion {
                    region_info: vfio_region_info {
                        argsz: size_of::<vfio_region_info>() as u32,
                        flags: if implemented {
                            VFIO_REGION_INFO_FLAG_READ | VFIO_REGION_INFO_FLAG_WRITE
                        } else {
                            0
                        },
                        index: index as u32,
                        cap_offset: 0,
                        size: match index as u32 {
                            VFIO_PCI_BAR4_REGION_INDEX => UHCI_IO_BAR_SIZE,
                            VFIO_PCI_CONFIG_REGION_INDEX => PCI_CONFIG_SPACE_SIZE as u64,
                            _ => 0,
                        },
                        offset: 0,
                    },
                    sparse_areas: Vec::new(),
                    mmap_fd: None,
                }
            })
            .collect()
    }

    pub fn irqs() -> Vec<IrqInfo> {
        (0..=VFIO_PCI_REQ_IRQ_INDEX)
            .map(|index| {
                if index == VFIO_PCI_INTX_IRQ_INDEX {
                    IrqInfo {
                        index,
                        flags: VFIO_IRQ_INFO_EVENTFD
                            | VFIO_IRQ_INFO_MASKABLE
                            | VFIO_IRQ_INFO_AUTOMASKED,
                        count: 1,
                    }
                } else {
                    IrqInfo {
                        index,
                        flags: 0,
                        count: 0,
                    }
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_class_and_bar_probe_are_stable() {
        let mut config = PciConfig::new();
        let mut data = [0; 4];
        config.read(0, &mut data).unwrap();
        assert_eq!(data, [0x36, 0x1b, 0xfe, 0x00]);
        config.read(8, &mut data).unwrap();
        assert_eq!(data, [1, 0, 3, 0x0c]);

        config.write(0x20, &u32::MAX.to_le_bytes()).unwrap();
        config.read(0x20, &mut data).unwrap();
        assert_eq!(u32::from_le_bytes(data), UHCI_BAR_SIZE_MASK);

        config.write(0x10, &u32::MAX.to_le_bytes()).unwrap();
        config.read(0x10, &mut data).unwrap();
        assert_eq!(data, [0; 4]);
    }

    #[test]
    fn command_and_interrupt_line_remain_writable() {
        let mut config = PciConfig::new();
        config.write(4, &[5, 0]).unwrap();
        config.write(0x3c, &[11]).unwrap();
        let mut command = [0; 2];
        let mut line = [0];
        config.read(4, &mut command).unwrap();
        config.read(0x3c, &mut line).unwrap();
        assert_eq!(command, [5, 0]);
        assert_eq!(line, [11]);
    }
}
