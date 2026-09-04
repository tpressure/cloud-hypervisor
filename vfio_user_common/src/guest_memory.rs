// Copyright © 2026 Cyberus Technology GmbH
//
// SPDX-License-Identifier: Apache-2.0

use std::fs::File;
use std::io;
use std::sync::{Arc, RwLock};

use log::{info, warn};
use vfio_user::{DmaMapFlags, DmaUnmapFlags};
use vm_memory::{FileOffset, MmapRegion};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct GuestAddress(pub u64);

struct GuestMemoryRegion {
    start: GuestAddress,
    size: u64,
    readable: bool,
    writable: bool,
    memory: MmapRegion,
}

impl GuestMemoryRegion {
    fn end(&self) -> Option<u64> {
        self.start.0.checked_add(self.size)
    }

    fn contains(&self, address: GuestAddress, length: usize) -> bool {
        let Ok(length) = u64::try_from(length) else {
            return false;
        };
        let Some(end) = address.0.checked_add(length) else {
            return false;
        };
        self.start.0 <= address.0 && self.end().is_some_and(|region_end| end <= region_end)
    }

    fn offset(&self, address: GuestAddress) -> Option<usize> {
        usize::try_from(address.0.checked_sub(self.start.0)?).ok()
    }
}

#[derive(Default)]
struct MappingState {
    regions: Vec<Arc<GuestMemoryRegion>>,
}

/// Owns the active vfio-user DMA mappings and bounds every guest-memory access.
///
/// A read lock remains held for the complete access, so `DMA_UNMAP` cannot drop
/// an mmap while it is being used. Partial unmaps conservatively invalidate the
/// complete overlapping region.
#[derive(Clone, Default)]
pub struct GuestMemoryMap {
    state: Arc<RwLock<MappingState>>,
}

impl GuestMemoryMap {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn mapping_count(&self) -> usize {
        self.state.read().unwrap().regions.len()
    }

    pub fn contains(&self, address: GuestAddress, length: usize) -> bool {
        let state = self.state.read().unwrap();
        state
            .regions
            .iter()
            .any(|region| region.contains(address, length))
    }

    pub fn map(
        &self,
        flags: DmaMapFlags,
        file_offset: u64,
        address: GuestAddress,
        size: u64,
        file: Option<File>,
    ) -> io::Result<()> {
        if size == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "DMA mapping size is zero",
            ));
        }
        let end = address.0.checked_add(size).ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidInput, "DMA mapping range overflows")
        })?;
        let file = file.ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "DMA mapping has no shared-memory file descriptor; use --memory shared=on",
            )
        })?;
        let size_usize = usize::try_from(size).map_err(|_| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "DMA mapping does not fit the host address space",
            )
        })?;

        let readable = flags.contains(DmaMapFlags::READ);
        let writable = flags.contains(DmaMapFlags::WRITE);
        if !readable && !writable {
            return Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "DMA mapping is neither readable nor writable",
            ));
        }
        let mut prot = 0;
        if readable {
            prot |= libc::PROT_READ;
        }
        if writable {
            prot |= libc::PROT_WRITE;
        }

        let memory = MmapRegion::build(
            Some(FileOffset::new(file, file_offset)),
            size_usize,
            prot,
            libc::MAP_SHARED,
        )
        .map_err(|error| io::Error::other(format!("cannot mmap shared guest RAM: {error}")))?;

        let mut state = self.state.write().unwrap();
        if state.regions.iter().any(|region| {
            region
                .end()
                .is_some_and(|region_end| address.0 < region_end && region.start.0 < end)
        }) {
            return Err(io::Error::new(
                io::ErrorKind::AlreadyExists,
                "DMA mapping overlaps an existing mapping",
            ));
        }
        state.regions.push(Arc::new(GuestMemoryRegion {
            start: address,
            size,
            readable,
            writable,
            memory,
        }));
        state.regions.sort_by_key(|region| region.start);
        info!(
            "DMA_MAP iova=0x{:016x} size=0x{size:x} offset=0x{file_offset:x} permissions={}{}",
            address.0,
            if readable { "read" } else { "" },
            if writable { "/write" } else { "" },
        );
        Ok(())
    }

    pub fn unmap(
        &self,
        flags: DmaUnmapFlags,
        address: GuestAddress,
        size: u64,
    ) -> io::Result<usize> {
        let mut state = self.state.write().unwrap();
        if flags.contains(DmaUnmapFlags::UNMAP_ALL) {
            let removed = state.regions.len();
            state.regions.clear();
            info!("DMA_UNMAP all mappings removed={removed}");
            return Ok(removed);
        }

        let end = address.0.checked_add(size).ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidInput, "DMA unmap range overflows")
        })?;
        let before = state.regions.len();
        state.regions.retain(|region| {
            let overlaps = region
                .end()
                .is_some_and(|region_end| address.0 < region_end && region.start.0 < end);
            if overlaps && (address != region.start || size != region.size) {
                warn!(
                    "partial DMA_UNMAP invalidates complete mapping iova=0x{:x} size=0x{:x}",
                    region.start.0, region.size
                );
            }
            !overlaps
        });
        let removed = before - state.regions.len();
        info!(
            "DMA_UNMAP iova=0x{:016x} size=0x{size:x} removed={removed}",
            address.0
        );
        Ok(removed)
    }

    pub fn clear(&self) {
        self.state.write().unwrap().regions.clear();
    }

    pub fn read(&self, address: GuestAddress, data: &mut [u8]) -> io::Result<()> {
        let state = self.state.read().unwrap();
        let region = state
            .regions
            .iter()
            .find(|region| region.readable && region.contains(address, data.len()))
            .ok_or_else(|| {
                io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!(
                        "guest read outside a readable DMA mapping: address=0x{:x} length=0x{:x}",
                        address.0,
                        data.len()
                    ),
                )
            })?;
        let offset = region.offset(address).ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidInput, "guest read offset overflows")
        })?;

        // SAFETY: `contains` proves every byte is inside the live mmap, and the
        // read lock prevents unmapping. Volatile byte accesses are used because
        // guest CPUs can concurrently modify this shared device memory.
        for (index, byte) in data.iter_mut().enumerate() {
            // SAFETY: The complete access is bounded and the mapping is held
            // live as described above.
            *byte = unsafe { region.memory.as_ptr().add(offset + index).read_volatile() };
        }
        Ok(())
    }

    pub fn write(&self, address: GuestAddress, data: &[u8]) -> io::Result<()> {
        let state = self.state.read().unwrap();
        let region = state
            .regions
            .iter()
            .find(|region| region.writable && region.contains(address, data.len()))
            .ok_or_else(|| {
                io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!(
                        "guest write outside a writable DMA mapping: address=0x{:x} length=0x{:x}",
                        address.0,
                        data.len()
                    ),
                )
            })?;
        let offset = region.offset(address).ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidInput, "guest write offset overflows")
        })?;

        // SAFETY: `contains` proves every byte is inside the live mmap, and the
        // read lock prevents unmapping. See the corresponding read rationale.
        for (index, byte) in data.iter().enumerate() {
            // SAFETY: The complete access is bounded and the mapping is held
            // live as described above.
            unsafe {
                region
                    .memory
                    .as_ptr()
                    .add(offset + index)
                    .write_volatile(*byte);
            }
        }
        Ok(())
    }

    pub fn read_vec(&self, address: GuestAddress, length: usize) -> io::Result<Vec<u8>> {
        let mut data = vec![0; length];
        self.read(address, &mut data)?;
        Ok(data)
    }

    pub fn read_u32(&self, address: GuestAddress) -> io::Result<u32> {
        let mut data = [0; 4];
        self.read(address, &mut data)?;
        Ok(u32::from_le_bytes(data))
    }

    pub fn read_u16(&self, address: GuestAddress) -> io::Result<u16> {
        let mut data = [0; 2];
        self.read(address, &mut data)?;
        Ok(u16::from_le_bytes(data))
    }

    pub fn write_u16(&self, address: GuestAddress, value: u16) -> io::Result<()> {
        self.write(address, &value.to_le_bytes())
    }

    pub fn write_u32(&self, address: GuestAddress, value: u32) -> io::Result<()> {
        self.write(address, &value.to_le_bytes())
    }
}

#[cfg(test)]
mod tests {
    use std::fs::OpenOptions;
    use std::io::{Seek, SeekFrom, Write};
    use std::sync::Barrier;
    use std::thread;
    use std::time::Duration;

    use super::*;

    fn test_file(size: u64) -> File {
        let path = std::env::temp_dir().join(format!(
            "vfio-user-common-test-{}-{:?}",
            std::process::id(),
            thread::current().id()
        ));
        let mut file = OpenOptions::new()
            .create_new(true)
            .read(true)
            .write(true)
            .open(&path)
            .unwrap();
        file.set_len(size).unwrap();
        file.seek(SeekFrom::Start(0x100)).unwrap();
        file.write_all(&[1, 2, 3, 4]).unwrap();
        std::fs::remove_file(path).unwrap();
        file
    }

    #[test]
    fn validates_and_accesses_mapping_bounds() {
        let memory = GuestMemoryMap::new();
        memory
            .map(
                DmaMapFlags::READ_WRITE,
                0,
                GuestAddress(0x1000),
                0x1000,
                Some(test_file(0x1000)),
            )
            .unwrap();

        assert_eq!(memory.read_u32(GuestAddress(0x1100)).unwrap(), 0x0403_0201);
        memory.write_u32(GuestAddress(0x11fc), 0x8877_6655).unwrap();
        assert_eq!(memory.read_u32(GuestAddress(0x11fc)).unwrap(), 0x8877_6655);
        memory.read_u32(GuestAddress(0x1ffe)).unwrap_err();
        memory.read_u32(GuestAddress(u64::MAX - 1)).unwrap_err();
    }

    #[test]
    fn rejects_overlapping_and_missing_file_mappings() {
        let memory = GuestMemoryMap::new();
        assert!(
            memory
                .map(DmaMapFlags::READ_WRITE, 0, GuestAddress(0), 0x1000, None,)
                .is_err()
        );
        memory
            .map(
                DmaMapFlags::READ_WRITE,
                0,
                GuestAddress(0x1000),
                0x1000,
                Some(test_file(0x1000)),
            )
            .unwrap();
        assert!(
            memory
                .map(
                    DmaMapFlags::READ_WRITE,
                    0,
                    GuestAddress(0x1800),
                    0x1000,
                    Some(test_file(0x1000)),
                )
                .is_err()
        );
    }

    #[test]
    fn unmap_waits_for_active_access_and_invalidates_overlap() {
        let memory = GuestMemoryMap::new();
        memory
            .map(
                DmaMapFlags::READ_WRITE,
                0,
                GuestAddress(0),
                0x1000,
                Some(test_file(0x1000)),
            )
            .unwrap();

        let barrier = Arc::new(Barrier::new(2));
        let held_memory = memory.clone();
        let held_barrier = Arc::clone(&barrier);
        let reader = thread::spawn(move || {
            let _guard = held_memory.state.read().unwrap();
            held_barrier.wait();
            thread::sleep(Duration::from_millis(50));
        });
        barrier.wait();
        assert_eq!(
            memory
                .unmap(DmaUnmapFlags::empty(), GuestAddress(0x800), 0x10)
                .unwrap(),
            1
        );
        reader.join().unwrap();
        assert!(!memory.contains(GuestAddress(0), 1));
    }
}
