// Copyright © 2026 Cyberus Technology GmbH
//
// SPDX-License-Identifier: Apache-2.0

pub mod input;

use std::fs::File;
use std::io;

use display::framebuffer::FramebufferSource;
use display::ramfb::{DRM_FORMAT_XRGB8888, RamfbConfig};
use log::{info, warn};
use thiserror::Error;
use vfio_bindings::bindings::vfio::{VFIO_PCI_CONFIG_REGION_INDEX, vfio_region_info};
use vfio_user::{DmaMapFlags, DmaUnmapFlags, IrqInfo, ServerBackend, ServerRegion};
use vfio_user_common::guest_memory::{GuestAddress, GuestMemoryMap};

pub const PCI_VENDOR_ID: u16 = 0x1b36;
// Prototype device ID in the Red Hat/QEMU virtual-device vendor namespace.
// This value is intentionally not a registered QEMU device model ID.
pub const PCI_DEVICE_ID: u16 = 0x00ff;
pub const PCI_CONFIG_SPACE_SIZE: usize = 4096;
pub const PCI_REGION_COUNT: usize = VFIO_PCI_CONFIG_REGION_INDEX as usize + 1;
pub const FRAMEBUFFER_BYTES_PER_PIXEL: u32 = 4;

const PCI_BAR_RANGE: std::ops::Range<usize> = 0x10..0x28;
const PCI_ROM_ADDRESS_RANGE: std::ops::Range<usize> = 0x30..0x34;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FramebufferGeometry {
    pub gpa: u64,
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    pub fourcc: u32,
    pub size: u64,
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum GeometryError {
    #[error("framebuffer width and height must be non-zero")]
    Empty,
    #[error("unsupported framebuffer format 0x{0:08x}; only XRGB8888 is supported")]
    UnsupportedFormat(u32),
    #[error("framebuffer row size overflow")]
    RowSizeOverflow,
    #[error("framebuffer stride {stride} is smaller than the {minimum}-byte pixel row")]
    InvalidStride { stride: u32, minimum: u64 },
    #[error("framebuffer size overflow")]
    SizeOverflow,
    #[error("framebuffer GPA range overflow")]
    AddressOverflow,
}

impl FramebufferGeometry {
    pub fn new(
        gpa: u64,
        width: u32,
        height: u32,
        stride: u32,
        fourcc: u32,
    ) -> Result<Self, GeometryError> {
        if width == 0 || height == 0 {
            return Err(GeometryError::Empty);
        }
        if fourcc != DRM_FORMAT_XRGB8888 {
            return Err(GeometryError::UnsupportedFormat(fourcc));
        }

        let minimum = u64::from(width)
            .checked_mul(u64::from(FRAMEBUFFER_BYTES_PER_PIXEL))
            .ok_or(GeometryError::RowSizeOverflow)?;
        if u64::from(stride) < minimum {
            return Err(GeometryError::InvalidStride { stride, minimum });
        }

        let size = u64::from(stride)
            .checked_mul(u64::from(height))
            .ok_or(GeometryError::SizeOverflow)?;
        gpa.checked_add(size)
            .ok_or(GeometryError::AddressOverflow)?;

        Ok(Self {
            gpa,
            width,
            height,
            stride,
            fourcc,
            size,
        })
    }

    fn ramfb_config(self) -> RamfbConfig {
        RamfbConfig {
            address: self.gpa,
            fourcc: self.fourcc,
            flags: 0,
            width: self.width,
            height: self.height,
            stride: self.stride,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MappingRange {
    pub iova: u64,
    pub size: u64,
    pub readable: bool,
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum MappingRangeError {
    #[error("DMA mapping address range overflow")]
    MappingOverflow,
    #[error("framebuffer address range overflow")]
    FramebufferOverflow,
}

pub fn mapping_covers_framebuffer(
    mapping: MappingRange,
    framebuffer_gpa: u64,
    framebuffer_size: u64,
) -> Result<bool, MappingRangeError> {
    let mapping_end = mapping
        .iova
        .checked_add(mapping.size)
        .ok_or(MappingRangeError::MappingOverflow)?;
    let framebuffer_end = framebuffer_gpa
        .checked_add(framebuffer_size)
        .ok_or(MappingRangeError::FramebufferOverflow)?;

    Ok(mapping.readable && mapping.iova <= framebuffer_gpa && framebuffer_end <= mapping_end)
}

#[derive(Clone)]
pub struct DmaFramebuffer {
    geometry: FramebufferGeometry,
    memory: GuestMemoryMap,
}

impl DmaFramebuffer {
    pub fn new(geometry: FramebufferGeometry) -> Self {
        Self {
            geometry,
            memory: GuestMemoryMap::new(),
        }
    }

    pub fn geometry(&self) -> FramebufferGeometry {
        self.geometry
    }

    pub fn mapping_count(&self) -> usize {
        self.memory.mapping_count()
    }

    pub fn has_framebuffer_mapping(&self) -> bool {
        let Ok(size) = usize::try_from(self.geometry.size) else {
            return false;
        };
        self.memory.contains(GuestAddress(self.geometry.gpa), size)
    }

    pub fn clear_mappings(&self) {
        self.memory.clear();
    }

    fn add_mapping(
        &self,
        flags: DmaMapFlags,
        offset: u64,
        iova: u64,
        size: u64,
        file: Option<File>,
    ) -> io::Result<()> {
        if size == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "DMA mapping size is zero",
            ));
        }
        iova.checked_add(size).ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidInput, "DMA mapping range overflows")
        })?;

        let readable = flags.contains(DmaMapFlags::READ);
        let writable = flags.contains(DmaMapFlags::WRITE);
        if !readable {
            return Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "DMA mapping is not readable",
            ));
        }

        let range = MappingRange {
            iova,
            size,
            readable,
        };
        let covers = mapping_covers_framebuffer(range, self.geometry.gpa, self.geometry.size)
            .map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error))?;
        info!(
            "DMA_MAP iova=0x{iova:016x} size=0x{size:x} offset=0x{offset:x} permissions={}{} covers_framebuffer={}",
            if readable { "read" } else { "" },
            if writable { "/write" } else { "" },
            if covers { "yes" } else { "no" },
        );

        self.memory
            .map(flags, offset, GuestAddress(iova), size, file)
    }

    fn remove_mapping(&self, flags: DmaUnmapFlags, iova: u64, size: u64) -> io::Result<()> {
        self.memory
            .unmap(flags, GuestAddress(iova), size)
            .map(|_| ())
    }

    fn read_pixels(&self) -> io::Result<Vec<u8>> {
        let size = usize::try_from(self.geometry.size).map_err(|_| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "framebuffer size does not fit the host address space",
            )
        })?;
        self.memory
            .read_vec(GuestAddress(self.geometry.gpa), size)
            .inspect_err(|error| warn!("cannot read mapped framebuffer: {error}"))
    }
}

impl FramebufferSource for DmaFramebuffer {
    fn config(&self) -> Option<RamfbConfig> {
        self.has_framebuffer_mapping()
            .then(|| self.geometry.ramfb_config())
    }

    fn read_framebuffer(&self) -> Option<Vec<u8>> {
        self.read_pixels().ok()
    }
}

pub struct MinimalPciBackend {
    config: [u8; PCI_CONFIG_SPACE_SIZE],
    framebuffer: DmaFramebuffer,
}

impl MinimalPciBackend {
    pub fn new(framebuffer: DmaFramebuffer) -> Self {
        let mut config = [0u8; PCI_CONFIG_SPACE_SIZE];
        Self::restore_static_config(&mut config);
        Self {
            config,
            framebuffer,
        }
    }

    fn restore_static_config(config: &mut [u8; PCI_CONFIG_SPACE_SIZE]) {
        config[0..2].copy_from_slice(&PCI_VENDOR_ID.to_le_bytes());
        config[2..4].copy_from_slice(&PCI_DEVICE_ID.to_le_bytes());
        // An unclassified PCI function is intentionally harmless: it exists
        // only to establish the vfio-user DMA mapping transport.
        config[0x08..0x0c].copy_from_slice(&[0, 0, 0, 0xff]);
        config[0x0e] = 0;

        // BARs and the expansion ROM are not implemented. PCI sizing probes
        // write all ones, so these fields must be restored to zero rather than
        // echoing the probe value back to Cloud Hypervisor.
        config[PCI_BAR_RANGE].fill(0);
        config[PCI_ROM_ADDRESS_RANGE].fill(0);
    }

    pub fn regions() -> Vec<ServerRegion> {
        (0..PCI_REGION_COUNT)
            .map(|index| ServerRegion {
                region_info: vfio_region_info {
                    argsz: std::mem::size_of::<vfio_region_info>() as u32,
                    flags: if index == VFIO_PCI_CONFIG_REGION_INDEX as usize {
                        vfio_bindings::bindings::vfio::VFIO_REGION_INFO_FLAG_READ
                            | vfio_bindings::bindings::vfio::VFIO_REGION_INFO_FLAG_WRITE
                    } else {
                        0
                    },
                    index: index as u32,
                    cap_offset: 0,
                    size: if index == VFIO_PCI_CONFIG_REGION_INDEX as usize {
                        PCI_CONFIG_SPACE_SIZE as u64
                    } else {
                        0
                    },
                    offset: 0,
                },
                sparse_areas: Vec::new(),
                mmap_fd: None,
            })
            .collect()
    }

    /// Cloud Hypervisor probes the standard PCI interrupt indices even when
    /// the endpoint advertises no usable interrupt vectors.
    pub fn irqs() -> Vec<IrqInfo> {
        (0..=vfio_bindings::bindings::vfio::VFIO_PCI_REQ_IRQ_INDEX)
            .map(|index| IrqInfo {
                index,
                flags: 0,
                count: 0,
            })
            .collect()
    }

    fn checked_config_range(offset: u64, length: usize) -> io::Result<std::ops::Range<usize>> {
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
}

impl ServerBackend for MinimalPciBackend {
    fn region_read(&mut self, region: u32, offset: u64, data: &mut [u8]) -> io::Result<()> {
        if region != VFIO_PCI_CONFIG_REGION_INDEX {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "device has no BAR regions",
            ));
        }
        let range = Self::checked_config_range(offset, data.len())?;
        data.copy_from_slice(&self.config[range]);
        Ok(())
    }

    fn region_write(&mut self, region: u32, offset: u64, data: &[u8]) -> io::Result<()> {
        if region != VFIO_PCI_CONFIG_REGION_INDEX {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "device has no BAR regions",
            ));
        }
        let range = Self::checked_config_range(offset, data.len())?;
        self.config[range].copy_from_slice(data);
        Self::restore_static_config(&mut self.config);
        Ok(())
    }

    fn dma_map(
        &mut self,
        flags: DmaMapFlags,
        offset: u64,
        address: u64,
        size: u64,
        fd: Option<File>,
    ) -> io::Result<()> {
        self.framebuffer
            .add_mapping(flags, offset, address, size, fd)
    }

    fn dma_unmap(&mut self, flags: DmaUnmapFlags, address: u64, size: u64) -> io::Result<()> {
        self.framebuffer.remove_mapping(flags, address, size)
    }

    fn reset(&mut self) -> io::Result<()> {
        self.framebuffer.clear_mappings();
        Ok(())
    }

    fn set_irqs(
        &mut self,
        _index: u32,
        _flags: u32,
        _start: u32,
        _count: u32,
        _fds: Vec<File>,
    ) -> io::Result<()> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "minimal transport device has no interrupts",
        ))
    }
}

pub fn framebuffer_checksum(data: &[u8]) -> u64 {
    data.iter().fold(0xcbf2_9ce4_8422_2325, |hash, byte| {
        (hash ^ u64::from(*byte)).wrapping_mul(0x0000_0100_0000_01b3)
    })
}

#[cfg(test)]
mod tests {
    use std::fs::OpenOptions;
    use std::io::{Seek, SeekFrom, Write};
    use std::thread;

    use super::*;

    const FB_GPA: u64 = 0x1000;
    const FB_SIZE: u64 = 0x1000;

    fn range(iova: u64, size: u64) -> MappingRange {
        MappingRange {
            iova,
            size,
            readable: true,
        }
    }

    #[test]
    fn mapping_containment_boundaries() {
        assert!(mapping_covers_framebuffer(range(FB_GPA, FB_SIZE), FB_GPA, FB_SIZE).unwrap());
        assert!(mapping_covers_framebuffer(range(FB_GPA, 0x2000), FB_GPA, FB_SIZE).unwrap());
        assert!(mapping_covers_framebuffer(range(0, 0x3000), FB_GPA, FB_SIZE).unwrap());
        assert!(mapping_covers_framebuffer(range(0, 0x2000), FB_GPA, FB_SIZE).unwrap());
        assert!(!mapping_covers_framebuffer(range(0x1800, 0x1000), FB_GPA, FB_SIZE).unwrap());
        assert!(!mapping_covers_framebuffer(range(0, 0x1800), FB_GPA, FB_SIZE).unwrap());
    }

    #[test]
    fn mapping_containment_checks_overflow_and_permissions() {
        assert_eq!(
            mapping_covers_framebuffer(range(u64::MAX, 2), 0, 1),
            Err(MappingRangeError::MappingOverflow)
        );
        assert_eq!(
            mapping_covers_framebuffer(range(0, u64::MAX), u64::MAX, 2),
            Err(MappingRangeError::FramebufferOverflow)
        );
        assert!(
            !mapping_covers_framebuffer(
                MappingRange {
                    iova: 0,
                    size: 0x3000,
                    readable: false,
                },
                FB_GPA,
                FB_SIZE,
            )
            .unwrap()
        );
    }

    #[test]
    fn geometry_is_validated() {
        let geometry =
            FramebufferGeometry::new(0x2000, 1024, 768, 4096, DRM_FORMAT_XRGB8888).unwrap();
        assert_eq!(geometry.width, 1024);
        assert_eq!(geometry.height, 768);
        assert_eq!(geometry.stride, 4096);
        assert_eq!(geometry.size, 3_145_728);
        assert_eq!(FRAMEBUFFER_BYTES_PER_PIXEL, 4);

        assert!(matches!(
            FramebufferGeometry::new(0, 10, 10, 39, DRM_FORMAT_XRGB8888),
            Err(GeometryError::InvalidStride { .. })
        ));
        assert_eq!(
            FramebufferGeometry::new(u64::MAX, 1, 1, 4, DRM_FORMAT_XRGB8888),
            Err(GeometryError::AddressOverflow)
        );
    }

    fn temp_file(size: u64) -> File {
        let path = std::env::temp_dir().join(format!(
            "vfio-user-simplefb-test-{}-{:?}",
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
        file.seek(SeekFrom::Start(FB_GPA)).unwrap();
        file.write_all(&vec![0x5a; FB_SIZE as usize]).unwrap();
        std::fs::remove_file(path).unwrap();
        file
    }

    fn test_framebuffer() -> DmaFramebuffer {
        DmaFramebuffer::new(
            FramebufferGeometry::new(FB_GPA, 1024, 1, 4096, DRM_FORMAT_XRGB8888).unwrap(),
        )
    }

    #[test]
    fn finds_framebuffer_among_multiple_dma_regions() {
        let framebuffer = test_framebuffer();
        framebuffer
            .add_mapping(
                DmaMapFlags::READ_WRITE,
                0,
                0,
                0x1000,
                Some(temp_file(0x4000)),
            )
            .unwrap();
        framebuffer
            .add_mapping(
                DmaMapFlags::READ_WRITE,
                0x1000,
                0x1000,
                0x3000,
                Some(temp_file(0x4000)),
            )
            .unwrap();

        assert_eq!(framebuffer.mapping_count(), 2);
        assert_eq!(
            framebuffer.read_framebuffer().unwrap(),
            vec![0x5a; FB_SIZE as usize]
        );
    }

    #[test]
    fn partial_unmap_invalidates_the_whole_mapping() {
        let framebuffer = test_framebuffer();
        framebuffer
            .add_mapping(
                DmaMapFlags::READ_WRITE,
                0,
                0,
                0x4000,
                Some(temp_file(0x4000)),
            )
            .unwrap();
        framebuffer
            .remove_mapping(DmaUnmapFlags::empty(), FB_GPA, FB_SIZE)
            .unwrap();
        assert_eq!(framebuffer.mapping_count(), 0);
        assert!(framebuffer.read_framebuffer().is_none());
    }

    #[test]
    fn pci_identity_and_absent_regions_are_hard_wired() {
        let mut backend = MinimalPciBackend::new(test_framebuffer());

        backend
            .region_write(VFIO_PCI_CONFIG_REGION_INDEX, 0, &[0; 4])
            .unwrap();
        let mut identity = [0; 4];
        backend
            .region_read(VFIO_PCI_CONFIG_REGION_INDEX, 0, &mut identity)
            .unwrap();
        assert_eq!(
            identity,
            [
                PCI_VENDOR_ID.to_le_bytes()[0],
                PCI_VENDOR_ID.to_le_bytes()[1],
                PCI_DEVICE_ID.to_le_bytes()[0],
                PCI_DEVICE_ID.to_le_bytes()[1],
            ]
        );

        backend
            .region_write(VFIO_PCI_CONFIG_REGION_INDEX, 0x10, &[0xff; 0x18])
            .unwrap();
        let mut bars = [0xff; 0x18];
        backend
            .region_read(VFIO_PCI_CONFIG_REGION_INDEX, 0x10, &mut bars)
            .unwrap();
        assert_eq!(bars, [0; 0x18]);

        backend
            .region_write(VFIO_PCI_CONFIG_REGION_INDEX, 0x30, &[0xff; 4])
            .unwrap();
        let mut rom = [0xff; 4];
        backend
            .region_read(VFIO_PCI_CONFIG_REGION_INDEX, 0x30, &mut rom)
            .unwrap();
        assert_eq!(rom, [0; 4]);

        backend
            .region_write(VFIO_PCI_CONFIG_REGION_INDEX, 4, &[7, 0])
            .unwrap();
        let mut command = [0; 2];
        backend
            .region_read(VFIO_PCI_CONFIG_REGION_INDEX, 4, &mut command)
            .unwrap();
        assert_eq!(command, [7, 0]);
    }
}
