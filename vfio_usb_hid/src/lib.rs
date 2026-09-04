// Copyright © 2026 Cyberus Technology GmbH
//
// SPDX-License-Identifier: Apache-2.0

pub mod input;
mod pci;
mod uhci;
pub mod usb;

use std::fs::File;
use std::io;
use std::sync::{Arc, Mutex};

pub use pci::{PCI_DEVICE_ID, PCI_VENDOR_ID};
pub use uhci::{UHCI_IO_BAR_SIZE, UhciController};
use vfio_bindings::bindings::vfio::{
    VFIO_IRQ_SET_ACTION_MASK, VFIO_IRQ_SET_ACTION_TRIGGER, VFIO_IRQ_SET_ACTION_UNMASK,
    VFIO_IRQ_SET_DATA_EVENTFD, VFIO_IRQ_SET_DATA_NONE, VFIO_PCI_BAR4_REGION_INDEX,
    VFIO_PCI_CONFIG_REGION_INDEX, VFIO_PCI_INTX_IRQ_INDEX,
};
use vfio_user::{DmaMapFlags, DmaUnmapFlags, IrqInfo, ServerBackend, ServerRegion};
use vfio_user_common::guest_memory::{GuestAddress, GuestMemoryMap};

use crate::pci::PciConfig;
use crate::uhci::InterruptLine;

pub struct VfioUsbHidBackend {
    config: PciConfig,
    controller: Arc<Mutex<UhciController>>,
    memory: GuestMemoryMap,
    interrupt: InterruptLine,
}

impl VfioUsbHidBackend {
    pub fn new() -> Self {
        let interrupt = InterruptLine::new();
        Self {
            config: PciConfig::new(),
            controller: Arc::new(Mutex::new(UhciController::new(interrupt.clone()))),
            memory: GuestMemoryMap::new(),
            interrupt,
        }
    }

    pub fn regions() -> Vec<ServerRegion> {
        PciConfig::regions()
    }

    pub fn irqs() -> Vec<IrqInfo> {
        PciConfig::irqs()
    }

    pub fn controller(&self) -> Arc<Mutex<UhciController>> {
        Arc::clone(&self.controller)
    }

    pub fn memory(&self) -> GuestMemoryMap {
        self.memory.clone()
    }
}

impl Default for VfioUsbHidBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl ServerBackend for VfioUsbHidBackend {
    fn region_read(&mut self, region: u32, offset: u64, data: &mut [u8]) -> io::Result<()> {
        match region {
            VFIO_PCI_CONFIG_REGION_INDEX => self.config.read(offset, data),
            VFIO_PCI_BAR4_REGION_INDEX => self.controller.lock().unwrap().read(offset, data),
            _ => Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("unimplemented PCI region {region}"),
            )),
        }
    }

    fn region_write(&mut self, region: u32, offset: u64, data: &[u8]) -> io::Result<()> {
        match region {
            VFIO_PCI_CONFIG_REGION_INDEX => self.config.write(offset, data),
            VFIO_PCI_BAR4_REGION_INDEX => self.controller.lock().unwrap().write(offset, data),
            _ => Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("unimplemented PCI region {region}"),
            )),
        }
    }

    fn dma_map(
        &mut self,
        flags: DmaMapFlags,
        offset: u64,
        address: u64,
        size: u64,
        fd: Option<File>,
    ) -> io::Result<()> {
        self.memory
            .map(flags, offset, GuestAddress(address), size, fd)
    }

    fn dma_unmap(&mut self, flags: DmaUnmapFlags, address: u64, size: u64) -> io::Result<()> {
        self.memory
            .unmap(flags, GuestAddress(address), size)
            .map(|_| ())
    }

    fn reset(&mut self) -> io::Result<()> {
        self.config.reset();
        self.controller.lock().unwrap().reset();
        self.memory.clear();
        Ok(())
    }

    fn set_irqs(
        &mut self,
        index: u32,
        flags: u32,
        start: u32,
        count: u32,
        mut fds: Vec<File>,
    ) -> io::Result<()> {
        if index != VFIO_PCI_INTX_IRQ_INDEX || start != 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "only INTx vector zero is supported",
            ));
        }

        let action = flags
            & (VFIO_IRQ_SET_ACTION_MASK | VFIO_IRQ_SET_ACTION_UNMASK | VFIO_IRQ_SET_ACTION_TRIGGER);
        let data = flags & (VFIO_IRQ_SET_DATA_NONE | VFIO_IRQ_SET_DATA_EVENTFD);
        match (action, data, count) {
            (VFIO_IRQ_SET_ACTION_TRIGGER, VFIO_IRQ_SET_DATA_EVENTFD, 1) if fds.len() == 1 => {
                self.interrupt.install(fds.remove(0))
            }
            (VFIO_IRQ_SET_ACTION_TRIGGER, VFIO_IRQ_SET_DATA_NONE, 0) => {
                self.interrupt.disable();
                Ok(())
            }
            (VFIO_IRQ_SET_ACTION_MASK, VFIO_IRQ_SET_DATA_NONE, 1) => {
                self.interrupt.mask();
                Ok(())
            }
            (VFIO_IRQ_SET_ACTION_UNMASK, VFIO_IRQ_SET_DATA_NONE, 1) => self.interrupt.unmask(),
            _ => Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!(
                    "unsupported INTx setup flags=0x{flags:x} count={count} fds={}",
                    fds.len()
                ),
            )),
        }
    }
}

#[cfg(test)]
mod tests {
    use vfio_bindings::bindings::vfio::VFIO_PCI_BAR0_REGION_INDEX;

    use super::*;

    #[test]
    fn exposes_only_the_standard_uhci_io_bar() {
        let regions = VfioUsbHidBackend::regions();
        assert_eq!(
            regions[VFIO_PCI_BAR0_REGION_INDEX as usize]
                .region_info
                .size,
            0
        );
        assert_eq!(
            regions[VFIO_PCI_BAR4_REGION_INDEX as usize]
                .region_info
                .size,
            UHCI_IO_BAR_SIZE
        );
    }
}
