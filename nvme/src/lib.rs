// Copyright 2020 Cloud Hypervisor Authors
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Minimal NVMe 1.4 controller implementation as a PCI device.
//!
//! Supports one namespace backed by a raw disk image file. Handles Read, Write,
//! Flush, and the required admin commands (Create/Delete SQ, Create CQ,
//! Identify, Get/Set Features, Async Event Request).

use std::any::Any;
use std::fs::OpenOptions;
use std::io::{Read, Seek, SeekFrom, Write};
use std::path::PathBuf;
use std::sync::{Arc, Barrier, Mutex};

use byteorder::{ByteOrder, LittleEndian};
use log::{error, info, warn};
use pci::{
    BarReprogrammingParams, MsixCap, MsixConfig, MaybeMutInterruptSourceGroup, PciBarConfiguration,
    PciBarPrefetchable, PciBarRegionType, PciClassCode, PciConfiguration, PciDevice,
    PciDeviceError, PciHeaderType, PciMassStorageSubclass, PciProgrammingInterface, PciSubclass,
};
use thiserror::Error;
use uuid::Uuid;
use vm_allocator::{AddressAllocator, SystemAllocator};
use vm_device::interrupt::{InterruptIndex, InterruptManager, InterruptSourceGroup, MsiIrqGroupConfig};
use vm_device::{BusDevice, Resource};
use vm_memory::{Address, Bytes, GuestAddress, GuestAddressSpace, GuestMemoryAtomic, GuestMemoryError, GuestMemoryMmap};
use vm_memory::bitmap::AtomicBitmap;

// ---------------------------------------------------------------------------
// Error types
// ---------------------------------------------------------------------------

#[derive(Error, Debug)]
pub enum Error {
    #[error("PCI configuration error")]
    PciConfig(#[source] std::io::Error),

    #[error("BAR allocation failed for size {0}")]
    BarAllocation(u64),

    #[error("Guest memory access error")]
    GuestMemory(#[source] GuestMemoryError),

    #[error("Invalid PRP list")]
    InvalidPrpList,

    #[error("Invalid command")]
    InvalidCommand,

    #[error("Controller not ready")]
    ControllerNotReady,

    #[error("Invalid namespace")]
    InvalidNamespace,

    #[error("Disk I/O error")]
    DiskIo(#[source] std::io::Error),

    #[error("MSI-X interrupt error")]
    MsiXInterrupt(#[source] std::io::Error),

    #[error("Disk file error")]
    DiskFile(#[source] std::io::Error),

    #[error("MSI-X capability setup error")]
    MsixSetup(#[source] std::io::Error),

    #[error("Interrupt group creation error")]
    InterruptGroup(#[source] std::io::Error),
}

pub type Result<T> = std::result::Result<T, Error>;

// ---------------------------------------------------------------------------
// NVMe constants
// ---------------------------------------------------------------------------

/// NVMe page size (fixed by spec)
const NVME_PAGE_SIZE: u64 = 4096;

/// BAR0 size: 64KB to fit registers, MSI-X table, PBA, and doorbells
const BAR0_SIZE: u64 = 0x10000;

/// Doorbell stride (CAP.DSTRD = 0b011 -> 2^3 * 4 = 32 bytes)
const DBL_STRIDE: u64 = 8;

/// MSI-X table offset within BAR0 (16KB for 1024 vectors)
const MSI_X_TABLE_OFFSET: u64 = 0x3000;
const MSI_X_TABLE_SIZE: u64 = 0x4000;

/// PBA offset within BAR0 (128 bytes)
const PBA_OFFSET: u64 = 0x7000;
const PBA_SIZE: u64 = 0x80;

/// Doorbell region starts at offset 0x1000 within BAR0 (NVMe spec §3.1.25)
const DBL_BASE_OFFSET: u64 = 0x1000;

/// Maximum number of I/O submission queues allowed by NVMe spec
const NVME_MAX_IO_QUEUES: u16 = 1023;

/// Logical block size for the namespace
const LBA_SIZE: u64 = 512;

// Controller register offsets (NVMe spec §3.1, per EDK2 Nvme.h)
const CAP_OFFSET: u64 = 0x0000;   // 8 bytes
const VS_OFFSET: u64 = 0x0008;   // 4 bytes
const INTMS_OFFSET: u64 = 0x000C; // 4 bytes
const INTMC_OFFSET: u64 = 0x0010; // 4 bytes
const CC_OFFSET: u64 = 0x0014;   // 4 bytes
// 0x18-0x1B: reserved (4 bytes)
const CSTS_OFFSET: u64 = 0x001C; // 4 bytes
const NSSR_OFFSET: u64 = 0x0020; // 4 bytes
const AQA_OFFSET: u64 = 0x0024;  // 4 bytes
const ASQ_OFFSET: u64 = 0x0028;  // 8 bytes
const ACQ_OFFSET: u64 = 0x0030;  // 8 bytes

// NVMe I/O command opcodes
const NVME_CMD_FLUSH: u8 = 0x00;
const NVME_CMD_WRITE: u8 = 0x01;
const NVME_CMD_READ: u8 = 0x02;
const NVME_CMD_WRITE_UNCORRECTABLE: u8 = 0x04;
const NVME_CMD_WRITE_ZEROES: u8 = 0x08;
const NVME_CMD_COMPARE: u8 = 0x05;
const NVME_CMD_VERIFY: u8 = 0x0C;

// NVMe Admin command opcodes
const NVME_ADMIN_DELETE_SQ: u8 = 0x00;
const NVME_ADMIN_CREATE_SQ: u8 = 0x01;
const NVME_ADMIN_GET_LOG_PAGE: u8 = 0x02;
const NVME_ADMIN_DELETE_CQ: u8 = 0x04;
const NVME_ADMIN_CREATE_CQ: u8 = 0x05;
const NVME_ADMIN_IDENTIFY: u8 = 0x06;
const NVME_ADMIN_ABORT_CMD: u8 = 0x08;
const NVME_ADMIN_SET_FEATURES: u8 = 0x09;
const NVME_ADMIN_GET_FEATURES: u8 = 0x0A;
const NVME_ADMIN_ASYNC_EVENT_REQ: u8 = 0x0C;

// NVMe completion status codes (§5.1.2, §6.1.2)
const NVME_SC_SUCCESS: u16 = 0x0;
const NVME_SC_INVALID_OPCODE: u16 = 0x1;
const NVME_SC_INVALID_FIELD: u16 = 0x2;
const NVME_SC_CMD_ID_CONFLICT: u16 = 0x9;
const NVME_SC_DATA_SGL_LEN: u16 = 0xD;
const NVME_SC_INVALID_NS: u16 = 0xC;
const NVME_SC_INTERNAL_ERROR: u16 = 0x11;
const NVME_SC_ASYNC_EVT_REQ_LIMIT: u16 = 0x52;

// ---------------------------------------------------------------------------
// NVMe data structures
// ---------------------------------------------------------------------------

/// NVMe Submission Queue Entry (64 bytes = 16 DWORDs)
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
struct NvmeSqe {
    /// DWORD 0: opcode (bits 0-7), flags (bits 8-15), command ID (bits 16-31)
    dword0: u32,
    /// DWORD 1: FUSE (bits 0-15), reserved (bits 16-31)
    dword1: u32,
    /// DWORD 2-3: Namespace ID (32 bits)
    nsid: u32,
    /// DWORD 4: Reserved
    reserved1: u32,
    /// DWORD 5: CDW10 (command-specific)
    cdw10: u32,
    /// DWORD 6: CDW11 (command-specific)
    cdw11: u32,
    /// DWORD 7: CDW12 (reserved)
    cdw12: u32,
    /// DWORD 8: CDW13 (reserved)
    cdw13: u32,
    /// DWORD 9: CDW14 (reserved)
    cdw14: u32,
    /// DWORD 10: PRP Entry 1 (lower 32 bits)
    prp1_lo: u32,
    /// DWORD 11: PRP Entry 1 (upper 32 bits)
    prp1_hi: u32,
    /// DWORD 12: PRP Entry 2 (lower 32 bits)
    prp2_lo: u32,
    /// DWORD 13: PRP Entry 2 (upper 32 bits)
    prp2_hi: u32,
    /// DWORD 14-15: Reserved (or DSM for Deallocate command)
    reserved2: [u8; 8],
}

impl NvmeSqe {
    fn opcode(&self) -> u8 {
        (self.dword0 & 0xFF) as u8
    }

    fn command_id(&self) -> u16 {
        (self.dword0 >> 16) as u16
    }

    fn flags(&self) -> u8 {
        ((self.dword0 >> 8) & 0xFF) as u8
    }
}

/// NVMe Completion Queue Entry (16 bytes = 4 DWORDs)
/// Layout matches EDK2's NVME_CQ struct interpretation.
#[repr(C, align(16))]
#[derive(Clone, Copy, Debug, Default)]
struct NvmeCqe {
    /// DW0: Result
    dword0: u32,
    /// DW1: Reserved in EDK2 struct
    dword1: u32,
    /// DW2 lower: SQ Head (16 bits)
    sqhd: u16,
    /// DW2 upper: SQ ID (16 bits)
    sqid: u16,
    /// DW3 lower: Command ID (16 bits)
    cid: u16,
    /// DW3 upper: Phase Tag (bit 0), SC (bits 1-8), Sct (bits 9-11), etc.
    status_fields: u16,
}

impl NvmeCqe {
    fn new(cid: u16, phase: bool, sc: u16, sq_id: u16, sq_head: u16, result: u32) -> Self {
        // EDK2's NVME_CQ struct: Pt is bit 0 of upper half of DW3
        let status_fields = (sc as u16) << 1 | (if phase { 1u16 } else { 0 });

        NvmeCqe {
            dword0: result,
            dword1: 0,
            sqhd: sq_head,
            sqid: sq_id,
            cid,
            status_fields,
        }
    }
}

/// NVMe queue state
#[derive(Clone, Debug)]
struct NvmeQueue {
    /// Guest physical address of the submission queue ring
    sq_addr: Option<GuestAddress>,
    /// Guest physical address of the completion queue ring
    cq_addr: Option<GuestAddress>,
    /// Queue size (number of entries, must be power of 2)
    queue_size: u16,
    /// Submission queue head (points to next command to process)
    sq_head: u16,
    /// Submission queue tail (points to next free slot)
    sq_tail: u16,
    /// Completion queue tail (points to next free slot)
    cq_tail: u16,
    /// Total CQEs written (for spec-compliant phase tag: PT = (cq_count / queue_size) & 1)
    cq_count: u32,
    /// Completion queue phase bit
    cq_phase: bool,
    /// Whether this queue is allocated
    allocated: bool,
}

impl NvmeQueue {
    fn new() -> Self {
        NvmeQueue {
            sq_addr: None,
            cq_addr: None,
            queue_size: 0,
            sq_head: 0,
            sq_tail: 0,
            cq_tail: 0,
            cq_count: 0,
            cq_phase: false,
            allocated: false,
        }
    }
}

/// Enterprise NVMHCI programming interface (PI=0x02).
/// Required because EDK2 NvmExpressDxe only accepts this PI value.
struct NvmehciProgrammingInterface;

impl PciProgrammingInterface for NvmehciProgrammingInterface {
    fn get_register_value(&self) -> u8 {
        0x02
    }
}

// ---------------------------------------------------------------------------
// NVMe Controller
// ---------------------------------------------------------------------------

pub struct NvmeController {
    /// Device identifier
    id: String,

    /// PCI configuration space
    configuration: PciConfiguration,

    /// Allocated BAR regions
    bar_regions: Vec<PciBarConfiguration>,

    /// BAR0 base address in guest physical address space
    bar0_base: Option<GuestAddress>,

    /// Interrupt manager for MSI-X
    interrupt_manager: Arc<dyn InterruptManager<GroupConfig = MsiIrqGroupConfig>>,

    /// MSI-X interrupt source group
    interrupt_group: Option<Arc<dyn InterruptSourceGroup>>,

    /// MSI-X configuration
    msix_config: Arc<Mutex<MsixConfig>>,

    /// MSI-X PCI capability (holds table/PBA offsets set by guest)
    msix_cap: MsixCap,

    /// MSI-X capability register index in config space
    msix_cap_reg_idx: Option<usize>,

    /// Guest memory for DMA access
    guest_memory: GuestMemoryAtomic<GuestMemoryMmap<AtomicBitmap>>,

    /// Admin submission/completion queue pair
    admin_queue: NvmeQueue,

    /// I/O submission/completion queue pairs (indexed by queue ID - 1)
    io_queues: Vec<NvmeQueue>,

    /// Controller Configuration (CC.EN)
    cc_en: bool,

    /// Full CC register value (for read-back)
    cc: u32,

    /// INTMS register (Interrupt Message Set)
    intms: u32,

    /// INTMC register (Interrupt Message Clear)
    intmc: u32,

    /// Number of namespaces
    num_namespaces: u32,

    /// Namespace capacity in LBAs
    namespace_capacity: u64,

    /// Backing disk file handle
    disk_file: std::fs::File,

    /// Whether the disk is read-only
    disk_readonly: bool,

    /// Namespace UUID (for Identify data)
    namespace_uuid: Uuid,

    /// PCI BDF for MSI-X setup
    pci_device_bdf: u8,

    /// Maximum number of I/O queues (derived from vCPU count)
    max_io_queues: u16,

    /// Number of MSI-X vectors (1 admin + max_io_queues)
    msix_vectors: u16,

    /// Partial 64-bit register write buffers (firmware may write as two 32-bit accesses)
    asq_lo: u32,
    asq_hi: u32,
    acq_lo: u32,
    acq_hi: u32,
}

impl NvmeController {
    pub fn new(
        id: String,
        disk_path: PathBuf,
        disk_readonly: bool,
        interrupt_manager: Arc<dyn InterruptManager<GroupConfig = MsiIrqGroupConfig>>,
        guest_memory: GuestMemoryAtomic<GuestMemoryMmap<AtomicBitmap>>,
        pci_device_bdf: u8,
        num_vcpus: u32,
    ) -> Result<Self> {
        let disk_file = OpenOptions::new()
            .read(true)
            .write(!disk_readonly)
            .open(&disk_path)
            .map_err(Error::DiskFile)?;

        let disk_size = disk_file
            .metadata()
            .map_err(Error::DiskFile)?
            .len();

        let namespace_capacity = disk_size / LBA_SIZE;

        // Limit I/O queues to vCPU count to avoid creating too many EventFds
        let max_io_queues = (num_vcpus as u16).min(NVME_MAX_IO_QUEUES);
        let msix_vectors = 1 + max_io_queues;

        // Create MSI-X interrupt group
        let interrupt_source_group: Arc<dyn InterruptSourceGroup> = interrupt_manager
            .create_group(MsiIrqGroupConfig {
                base: 0,
                count: msix_vectors as InterruptIndex,
            })
            .map_err(Error::InterruptGroup)?;

        let msix_config = Arc::new(Mutex::new(
            MsixConfig::new(
                msix_vectors,
                MaybeMutInterruptSourceGroup::Immutable(interrupt_source_group.clone()),
                pci_device_bdf as u32,
                None,
            )
            .map_err(|e| Error::MsixSetup(std::io::Error::new(std::io::ErrorKind::Other, e)))?,
        ));

        let msix_config_clone = msix_config.clone();

        let mut configuration = PciConfiguration::new(
            0x1AF4,
            0x2000,
            0x0,
            PciClassCode::MassStorage,
            &PciMassStorageSubclass::NvmController as &dyn PciSubclass,
            Some(&NvmehciProgrammingInterface as &dyn PciProgrammingInterface),
            PciHeaderType::Device,
            0x1AF4,
            0x2000,
            Some(msix_config_clone),
            None,
        );

        // Setup MSI-X capability
        // BAR0 layout: 0x0000-0x0FFF registers, 0x1000-0x1FFF doorbells,
        // 0x2000-0x5FFF MSI-X table, 0x6000+ PBA
        let msix_cap = MsixCap::new(
            0,            // table BAR indicator (BAR0)
            msix_vectors,
            0x2000,       // table offset in BAR0
            0,            // PBA BAR indicator (BAR0)
            0x6000,       // PBA offset in BAR0
        );
        let msix_cap_offset = configuration
            .add_capability(&msix_cap)
            .map_err(|e| Error::PciConfig(std::io::Error::new(std::io::ErrorKind::Other, e)))?;
        let msix_cap_reg_idx = Some((msix_cap_offset / 4) as usize);

        Ok(NvmeController {
            id,
            configuration,
            bar_regions: Vec::new(),
            bar0_base: None,
            interrupt_manager,
            interrupt_group: Some(interrupt_source_group),
            msix_config,
            msix_cap,
            msix_cap_reg_idx,
            guest_memory,
            admin_queue: {
                let mut q = NvmeQueue::new();
                // Start with phase=true so EDK2 (which polls for Pt change from
                // initial 0) detects the first admin completions.
                q.cq_phase = true;
                q
            },
            io_queues: vec![NvmeQueue::new(); max_io_queues as usize],
            cc_en: false,
            cc: 0,
            intms: 0,
            intmc: 0,
            num_namespaces: 1,
            namespace_capacity,
            disk_file,
            disk_readonly,
            namespace_uuid: Uuid::new_v4(),
            pci_device_bdf,
            max_io_queues,
            msix_vectors,
            asq_lo: 0,
            asq_hi: 0,
            acq_lo: 0,
            acq_hi: 0,
        })
    }

    // -----------------------------------------------------------------------
    // Register access
    // -----------------------------------------------------------------------

    fn read_register(&self, offset: u64, data: &mut [u8]) {
        // Handle 64-bit registers that may be read as two 32-bit accesses
        if offset >= CAP_OFFSET && offset < CAP_OFFSET + 8 {
             let cap: u64 = 0x0040_0020_0000_01FF;
            let cap_offset = offset - CAP_OFFSET;
            if cap_offset == 0 && data.len() >= 8 {
                LittleEndian::write_u64(data, cap);
                info!("NVMe read CAP[64] -> 0x{:x}", cap);
            } else if cap_offset == 0 && data.len() >= 4 {
                let val = (cap & 0xffff_ffff) as u32;
                LittleEndian::write_u32(data, val);
                info!("NVMe read CAP[0..4] -> 0x{:x}", val);
            } else if cap_offset == 4 && data.len() >= 4 {
                let val = (cap >> 32) as u32;
                LittleEndian::write_u32(data, val);
                info!("NVMe read CAP[4..8] -> 0x{:x}", val);
            } else {
                info!("NVMe read CAP partial offset=0x{:x} len={}", offset, data.len());
                let bytes = cap.to_le_bytes();
                for (i, b) in data.iter_mut().enumerate() {
                    if cap_offset as usize + i < 8 {
                        *b = bytes[cap_offset as usize + i];
                    }
                }
            }
            return;
        }

        if offset >= ASQ_OFFSET && offset < ASQ_OFFSET + 8 {
            let addr = self.admin_queue.sq_addr.map(|ga| ga.raw_value()).unwrap_or(0);
            let reg_offset = offset - ASQ_OFFSET;
            if reg_offset == 0 && data.len() >= 8 {
                LittleEndian::write_u64(data, addr);
            } else if reg_offset == 0 && data.len() >= 4 {
                LittleEndian::write_u32(data, (addr & 0xffff_ffff) as u32);
            } else if reg_offset == 4 && data.len() >= 4 {
                LittleEndian::write_u32(data, (addr >> 32) as u32);
            } else {
                let bytes = addr.to_le_bytes();
                for (i, b) in data.iter_mut().enumerate() {
                    if reg_offset as usize + i < 8 {
                        *b = bytes[reg_offset as usize + i];
                    }
                }
            }
            return;
        }

        if offset >= ACQ_OFFSET && offset < ACQ_OFFSET + 8 {
            let addr = self.admin_queue.cq_addr.map(|ga| ga.raw_value()).unwrap_or(0);
            let reg_offset = offset - ACQ_OFFSET;
            if reg_offset == 0 && data.len() >= 8 {
                LittleEndian::write_u64(data, addr);
            } else if reg_offset == 0 && data.len() >= 4 {
                LittleEndian::write_u32(data, (addr & 0xffff_ffff) as u32);
            } else if reg_offset == 4 && data.len() >= 4 {
                LittleEndian::write_u32(data, (addr >> 32) as u32);
            } else {
                let bytes = addr.to_le_bytes();
                for (i, b) in data.iter_mut().enumerate() {
                    if reg_offset as usize + i < 8 {
                        *b = bytes[reg_offset as usize + i];
                    }
                }
            }
            return;
        }

        match offset {
            VS_OFFSET => {
                if data.len() >= 4 {
                    LittleEndian::write_u32(data, 0x1400);
                }
            }
            INTMS_OFFSET | INTMC_OFFSET => {
                if data.len() >= 4 {
                    LittleEndian::write_u32(data, 0);
                }
            }
            CC_OFFSET => {
                if data.len() >= 4 {
                    LittleEndian::write_u32(data, self.cc);
                }
            }
            CSTS_OFFSET => {
                // CSTS.RDY = 1 when controller is ready (cc_en && no fatal error)
                let csts: u32 = if self.cc_en { 1 } else { 0 };
                if data.len() >= 4 {
                    LittleEndian::write_u32(data, csts);
                }
            }
            AQA_OFFSET => {
                let sqs = if self.admin_queue.allocated {
                    (self.admin_queue.queue_size - 1) as u32
                } else {
                    0
                };
                let cqs = sqs;
                let aqa: u32 = (cqs << 16) | sqs;
                if data.len() >= 4 {
                    LittleEndian::write_u32(data, aqa);
                }
            }
            _ => {
                if offset < DBL_BASE_OFFSET {
                    warn!("Unexpected register read at offset 0x{:x}", offset);
                }
                data.fill(0);
            }
        }
    }

    fn write_register(&mut self, offset: u64, data: &[u8]) {
        if data.len() >= 8 {
            info!(
                "NVMe write_register offset=0x{:x} data=0x{:016x}",
                offset,
                LittleEndian::read_u64(data)
            );
        } else {
            info!(
                "NVMe write_register offset=0x{:x} data=0x{:08x}",
                offset,
                if data.len() >= 4 {
                    LittleEndian::read_u32(data)
                } else if data.len() >= 2 {
                    let mut aligned = [0u8; 4];
                    aligned[..data.len()].copy_from_slice(data);
                    LittleEndian::read_u32(&aligned)
                } else {
                    data[0] as u32
                }
            );
        }
        match offset {
            CC_OFFSET => {
                let cc = if data.len() >= 4 {
                    LittleEndian::read_u32(data)
                } else {
                    let mut aligned = [0u8; 4];
                    aligned[..data.len()].copy_from_slice(data);
                    LittleEndian::read_u32(&aligned)
                };

                self.cc = cc;
                let new_en = (cc & 0x1) != 0;

                if new_en != self.cc_en {
                    if new_en {
                        info!("NVMe controller enabled");
                    } else {
                        info!("NVMe controller disabled, resetting queues");
                        self.admin_queue.sq_head = 0;
                        self.admin_queue.sq_tail = 0;
                        self.admin_queue.cq_tail = 0;
                        self.admin_queue.cq_count = 0;
                        self.admin_queue.cq_phase = true;
                        for queue in self.io_queues.iter_mut() {
                            *queue = NvmeQueue::new();
                        }
                    }
                    self.cc_en = new_en;
                }
            }
            INTMS_OFFSET => {
                let mask = if data.len() >= 4 {
                    LittleEndian::read_u32(data)
                } else {
                    let mut aligned = [0u8; 4];
                    aligned[..data.len()].copy_from_slice(data);
                    LittleEndian::read_u32(&aligned)
                };
                self.intms |= mask;
                info!("INTMS write: 0x{:x} -> intms=0x{:x}", mask, self.intms);
            }
            INTMC_OFFSET => {
                let mask = if data.len() >= 4 {
                    LittleEndian::read_u32(data)
                } else {
                    let mut aligned = [0u8; 4];
                    aligned[..data.len()].copy_from_slice(data);
                    LittleEndian::read_u32(&aligned)
                };
                self.intms &= !mask;
                info!("INTMC write: 0x{:x} -> intms=0x{:x}", mask, self.intms);
            }
            AQA_OFFSET => {
                let aqa = if data.len() >= 4 {
                    LittleEndian::read_u32(data)
                } else {
                    let mut aligned = [0u8; 4];
                    aligned[..data.len()].copy_from_slice(data);
                    LittleEndian::read_u32(&aligned)
                };
                let sqs = (aqa & 0xFFFF) as u16;
                let cqs = ((aqa >> 16) & 0xFFFF) as u16;
                info!("NVMe AQA write: SQS={}, CQS={}", sqs, cqs);
                self.admin_queue.queue_size = (sqs + 1).max(cqs + 1) as u16;
            }
            0x0028..=0x002C => {
                let write_len = data.len();
                let reg_offset = offset - 0x0028;
                if write_len == 8 && reg_offset == 0 {
                    let addr = LittleEndian::read_u64(data);
                    self.admin_queue.sq_addr = Some(GuestAddress(addr));
                    info!("NVMe ASQ write[64]: 0x{:x}", addr);
                } else {
                    let val = if data.len() >= 4 {
                        LittleEndian::read_u32(data)
                    } else {
                        let mut aligned = [0u8; 4];
                        aligned[..data.len()].copy_from_slice(data);
                        LittleEndian::read_u32(&aligned)
                    };
                    if reg_offset == 0 {
                        self.asq_lo = val;
                        let addr = (self.asq_hi as u64) << 32 | self.asq_lo as u64;
                        if addr != 0 {
                            self.admin_queue.sq_addr = Some(GuestAddress(addr));
                            info!("NVMe ASQ write[lo]: 0x{:x}", addr);
                        }
                    } else {
                        self.asq_hi = val;
                        let addr = (self.asq_hi as u64) << 32 | self.asq_lo as u64;
                        if addr != 0 {
                            self.admin_queue.sq_addr = Some(GuestAddress(addr));
                            info!("NVMe ASQ write[hi]: 0x{:x}", addr);
                        }
                    }
                }
            }
            0x0030..=0x0034 => {
                let write_len = data.len();
                let reg_offset = offset - 0x0030;
                if write_len == 8 && reg_offset == 0 {
                    let addr = LittleEndian::read_u64(data);
                    self.admin_queue.cq_addr = Some(GuestAddress(addr));
                    info!("NVMe ACQ write[64]: 0x{:x}", addr);
                } else {
                    let val = if data.len() >= 4 {
                        LittleEndian::read_u32(data)
                    } else {
                        let mut aligned = [0u8; 4];
                        aligned[..data.len()].copy_from_slice(data);
                        LittleEndian::read_u32(&aligned)
                    };
                    if reg_offset == 0 {
                        self.acq_lo = val;
                        let addr = (self.acq_hi as u64) << 32 | self.acq_lo as u64;
                        if addr != 0 {
                            self.admin_queue.cq_addr = Some(GuestAddress(addr));
                            info!("NVMe ACQ write[lo]: 0x{:x}", addr);
                        }
                    } else {
                        self.acq_hi = val;
                        let addr = (self.acq_hi as u64) << 32 | self.acq_lo as u64;
                        if addr != 0 {
                            self.admin_queue.cq_addr = Some(GuestAddress(addr));
                            info!("ACQ write[hi]: 0x{:x}", addr);
                        }
                    }
                }
            }
            _ => {
                if offset < DBL_BASE_OFFSET {
                    warn!("Unexpected register write at offset 0x{:x}", offset);
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Doorbell processing
    // -----------------------------------------------------------------------

    fn process_sq_doorbell(&mut self, sqid: u16, new_tail: u16) {
        info!("NVMe process_sq_doorbell sqid={} tail={} cc_en={}", sqid, new_tail, self.cc_en);
        if !self.cc_en {
            return;
        }

        let queue = if sqid == 0 {
            &mut self.admin_queue
        } else if sqid <= self.max_io_queues {
            &mut self.io_queues[sqid as usize - 1]
        } else {
            warn!("Invalid SQ ID {}", sqid);
            return;
        };

        info!("NVMe process_sq_doorbell sqid={} allocated={} sq_addr={:?} cq_addr={:?}", sqid, queue.allocated, queue.sq_addr, queue.cq_addr);
        if sqid == 0 {
            if queue.sq_addr.is_none() || queue.cq_addr.is_none() || queue.queue_size == 0 {
                info!("NVMe admin queue not ready");
                return;
            }
        } else if !queue.allocated {
            return;
        }

        queue.sq_tail = new_tail;
        info!("NVMe SQ{} doorbell: tail={}, head={}", sqid, new_tail, queue.sq_head);

        if sqid == 0 {
            self.process_admin_queue();
        } else {
            self.process_io_queue(sqid);
        }
    }

    fn process_cq_doorbell(&mut self, cqid: u16, new_head: u16) {
        if !self.cc_en {
            return;
        }

        let _queue = if cqid == 0 {
            &mut self.admin_queue
        } else if cqid <= self.max_io_queues {
            &mut self.io_queues[cqid as usize - 1]
        } else {
            return;
        };

        info!("CQ{} doorbell: head={}", cqid, new_head);
    }

    // -----------------------------------------------------------------------
    // Queue processing
    // -----------------------------------------------------------------------

    fn process_admin_queue(&mut self) {
        info!("NVMe process_admin_queue sq_addr={:?} cq_addr={:?}", self.admin_queue.sq_addr, self.admin_queue.cq_addr);
        let sq_addr = self.admin_queue.sq_addr;
        let cq_addr = self.admin_queue.cq_addr;
        if sq_addr.is_none() || cq_addr.is_none() {
            return;
        }

        let guest_mem = self.guest_memory.memory();

        while self.admin_queue.sq_head != self.admin_queue.sq_tail {
            let sqe_idx = self.admin_queue.sq_head % self.admin_queue.queue_size;
            let sqe_addr = sq_addr.unwrap().unchecked_add(sqe_idx as u64 * 64);
            info!("NVMe admin SQE idx={} addr=0x{:x}", sqe_idx, sqe_addr.raw_value());

            let mut sqe_bytes = [0u8; 64];
            if let Err(e) = guest_mem.read(&mut sqe_bytes, sqe_addr) {
                error!("Failed to read admin SQE at 0x{:x}: {}", sqe_addr.raw_value(), e);
                return;
            }

            let sqe = parse_sqe(&sqe_bytes);
            info!("NVMe admin SQE bytes[0..64]: {:02x?}", &sqe_bytes);
            info!("NVMe admin command: opcode=0x{:02x}, CID={}, NSID={}, CDW10=0x{:08x}", sqe.opcode(), sqe.command_id(), sqe.nsid, sqe.cdw10);

            let (status, result) = self.handle_admin_command(&sqe);
            info!("NVMe admin completion: CID={}, status=0x{:04x}, result=0x{:08x}", sqe.command_id(), status, result);

            // Advance SQ head
            self.admin_queue.sq_head = (self.admin_queue.sq_head + 1) % self.admin_queue.queue_size;

            // Enqueue completion
            self.enqueue_completion(
                0,
                sqe.command_id(),
                status,
                0,
                result,
            );
        }
    }

    fn process_io_queue(&mut self, sqid: u16) {
        let idx = sqid as usize - 1;
        let sq_addr = self.io_queues[idx].sq_addr;
        let cq_addr = self.io_queues[idx].cq_addr;
        info!(
            "NVMe process_io_queue sqid={} sq_addr={:?} cq_addr={:?} allocated={}",
            sqid,
            sq_addr,
            cq_addr,
            self.io_queues[idx].allocated
        );
        if sq_addr.is_none() || cq_addr.is_none() {
            return;
        }

        let guest_mem = self.guest_memory.memory();

        while self.io_queues[idx].sq_head != self.io_queues[idx].sq_tail {
            let sqe_idx = self.io_queues[idx].sq_head % self.io_queues[idx].queue_size;
            let sqe_addr = sq_addr.unwrap().unchecked_add(sqe_idx as u64 * 64);

            let mut sqe_bytes = [0u8; 64];
            if let Err(e) = guest_mem.read(&mut sqe_bytes, sqe_addr) {
                error!("Failed to read I/O SQE at 0x{:x}: {}", sqe_addr.raw_value(), e);
                return;
            }

            let sqe = parse_sqe(&sqe_bytes);
        info!(
            "I/O command: SQ={}, opcode=0x{:02x}, CID={}, NSID={}, flags=0x{:02x}",
            sqid,
            sqe.opcode(),
            sqe.command_id(),
            sqe.nsid,
            sqe.flags()
        );

            let (status, result) = self.handle_io_command(&sqe);

            // Advance SQ head
            self.io_queues[idx].sq_head =
                (self.io_queues[idx].sq_head + 1) % self.io_queues[idx].queue_size;

            // Enqueue completion
            self.enqueue_completion(
                sqid,
                sqe.command_id(),
                status,
                sqid,
                result,
            );
        }
    }

    // -----------------------------------------------------------------------
    // Admin command handlers
    // -----------------------------------------------------------------------

    fn handle_admin_command(&mut self, sqe: &NvmeSqe) -> (u16, u32) {
        match sqe.opcode() {
            NVME_ADMIN_DELETE_SQ => self.admin_delete_sq(sqe),
            NVME_ADMIN_CREATE_SQ => self.admin_create_sq(sqe),
            NVME_ADMIN_CREATE_CQ => self.admin_create_cq(sqe),
            NVME_ADMIN_GET_LOG_PAGE => self.admin_get_log_page(sqe),
            NVME_ADMIN_IDENTIFY => self.admin_identify(sqe),
            NVME_ADMIN_GET_FEATURES => self.admin_get_features(sqe),
            NVME_ADMIN_SET_FEATURES => self.admin_set_features(sqe),
            NVME_ADMIN_ABORT_CMD => self.admin_abort(sqe),
            NVME_ADMIN_ASYNC_EVENT_REQ => self.admin_async_event_request(sqe),
            _ => {
                warn!("Unsupported admin opcode 0x{:02x}", sqe.opcode());
                (NVME_SC_INVALID_OPCODE, 0)
            }
        }
    }

    fn admin_delete_sq(&mut self, sqe: &NvmeSqe) -> (u16, u32) {
        let sqid = sqe.command_id();
        if sqid == 0 || sqid > self.max_io_queues {
            return (NVME_SC_INVALID_FIELD, 0);
        }
        if !self.io_queues[sqid as usize - 1].allocated {
            return (NVME_SC_INVALID_FIELD, 0);
        }
        self.io_queues[sqid as usize - 1] = NvmeQueue::new();
        info!("Deleted SQ {}", sqid);
        (NVME_SC_SUCCESS, 0)
    }

    fn admin_create_sq(&mut self, sqe: &NvmeSqe) -> (u16, u32) {
        let sqid = (sqe.cdw10 & 0xFFFF) as u16;
        let qsize = ((sqe.cdw10 >> 16) & 0xFFFF) as u16;
        let pc = sqe.cdw11 & 0x1;
        let cqid = (sqe.cdw11 >> 16) as u16;

        info!(
            "NVMe Create SQ: SQID={} QSIZE={} PC={} CQID={}",
            sqid, qsize, pc, cqid
        );

        if sqid == 0 || sqid > self.max_io_queues {
            info!("NVMe Create SQ rejected: SQID out of range");
            return (NVME_SC_INVALID_FIELD, 0);
        }
        if self.io_queues[sqid as usize - 1].allocated {
            info!("NVMe Create SQ rejected: SQ already allocated");
            return (NVME_SC_INVALID_FIELD, 0);
        }

        // For simplicity, require CQ ID == SQ ID
        if cqid != sqid {
            info!(
                "NVMe Create SQ rejected: CQID {} != SQID {}",
                cqid, sqid
            );
            return (NVME_SC_INVALID_FIELD, 0);
        }

        let queue = &mut self.io_queues[sqid as usize - 1];
        queue.sq_addr = Some(GuestAddress(sqe.prp1()));
        queue.queue_size = qsize + 1;
        queue.allocated = true;
        queue.cq_phase = pc != 0;

        info!(
            "NVMe Created SQ {} -> CQ {}, size={}, PC={}",
            sqid,
            cqid,
            qsize + 1,
            pc
        );
        (NVME_SC_SUCCESS, 0)
    }

    fn admin_create_cq(&mut self, sqe: &NvmeSqe) -> (u16, u32) {
        let cqid = (sqe.cdw10 & 0xFFFF) as u16;
        let qsize = ((sqe.cdw10 >> 16) as u16) + 1;
        let pc = sqe.cdw11 & 0x1;

        if cqid == 0 || cqid > self.max_io_queues {
            return (NVME_SC_INVALID_FIELD, 0);
        }

        let queue = &mut self.io_queues[cqid as usize - 1];
        queue.cq_addr = Some(GuestAddress(sqe.prp1()));
        queue.queue_size = qsize;
        queue.cq_phase = pc != 0;

        info!("NVMe Created CQ {}, size={}, PC={}", cqid, qsize, pc);
        (NVME_SC_SUCCESS, 0)
    }

    fn admin_get_log_page(&mut self, _sqe: &NvmeSqe) -> (u16, u32) {
        (NVME_SC_SUCCESS, 0)
    }

    fn admin_identify(&mut self, sqe: &NvmeSqe) -> (u16, u32) {
        if sqe.prp1() == 0 {
            return (NVME_SC_INVALID_FIELD, 0);
        }

        let cns = (sqe.cdw10 & 0xFF) as u8;
        let nsid = if sqe.nsid == 0 { 1 } else { sqe.nsid };
        info!("NVMe Identify: CNS=0x{:02x}, NSID={}, PRP1=0x{:x}, PRP2=0x{:x}", cns, nsid, sqe.prp1(), sqe.prp2());
        let mut data = [0u8; 4096];

        match cns {
            0x00 => {
                // Identify Namespace or Namespace Type - NSID=0 means only namespace
                self.fill_identify_namespace(&mut data);
            }
            0x01 => {
                // Identify Controller
                self.fill_identify_controller(&mut data);
            }
            0x02 => {
                // Identify Namespace by NSID
                if nsid != 1 {
                    return (NVME_SC_INVALID_NS, 0);
                }
                self.fill_identify_namespace(&mut data);
            }
            0x03 => {
                // Identify Namespace List - return list with NSID=1
                data[0] = 1;
            }
            0x04 => {
                // Identify Namespace Type
                self.fill_identify_namespace(&mut data);
            }
            _ => {
                warn!("Unsupported Identify CNS 0x{:02x}", cns);
                return (NVME_SC_INVALID_FIELD, 0);
            }
        }

        info!("NVMe Identify: writing {} bytes to PRP1=0x{:x}", data.len(), sqe.prp1());
        if let Err(e) = self.write_prp(sqe.prp1(), sqe.prp2(), &data) {
            info!("NVMe Identify: write_prp FAILED: {}", e);
            error!("Failed to write Identify data: {}", e);
            return (NVME_SC_INTERNAL_ERROR, 0);
        }
        info!("NVMe Identify: write_prp OK");

        (NVME_SC_SUCCESS, 0)
    }

    fn fill_identify_controller(&self, data: &mut [u8; 4096]) {
        // Per NVMe 1.4 spec §5.15 / QEMU NvmeIdCtrl struct

        // Offset 0x000: PCI Vendor ID
        LittleEndian::write_u16(&mut data[0x000..0x002], 0x1AF4);

        // Offset 0x002: PCI Subsystem Vendor ID
        LittleEndian::write_u16(&mut data[0x002..0x004], 0x1AF4);

        // Offset 0x004: Serial Number (20 bytes, ASCII)
        let serial = b"CLOUDHV             ";
        data[0x004..0x018].copy_from_slice(serial);

        // Offset 0x018: Model Number (40 bytes, ASCII)
        let model = b"CLOUDHV-NVME                            ";
        data[0x018..0x040].copy_from_slice(model);

        // Offset 0x040: Firmware revision (8 bytes, ASCII)
        let fw = b"1.0.0   ";
        data[0x040..0x048].copy_from_slice(fw);

        // Offset 0x048: Recommended Arbitration Burst
        data[0x048] = 2;

        // Offset 0x049: IEEE Identifier (3 bytes)
        data[0x049..0x04C].copy_from_slice(&[0x14, 0x00, 0x00]);

        // Offset 0x04C: Command Management Interface Capabilities
        data[0x04C] = 0;

        // Offset 0x04D: Maximum Data Transfer Size
        data[0x04D] = 4; // 2^4 = 16 pages = 64KB

        // Offset 0x04E: Controller ID
        data[0x04E] = 1;

        // Offset 0x050: Version
        LittleEndian::write_u32(&mut data[0x050..0x054], 0x14);

        // Offset 0x100: Optional Admin Command Support
        LittleEndian::write_u16(&mut data[0x100..0x102], 0x7);

        // Offset 0x102: Autonomous Power State Transition
        data[0x102] = 3;

        // Offset 0x103: Async Event Request Limit
        data[0x103] = 8;

        // Offset 0x104: Firmware updates
        data[0x104] = 7;

        // Offset 0x105: Log Page Attributes
        data[0x105] = 0x1;

        // Offset 0x106: Error Log Page Entries
        data[0x106] = 8;

        // Offset 0x107: Number of Power State Support Structures
        data[0x107] = 1;

        // Offset 0x118: Total NVM Capacity (16 bytes, in bytes)
        LittleEndian::write_u64(&mut data[0x118..0x120], self.namespace_capacity * LBA_SIZE);

        // Offset 0x128: Replay Protected Memory Block Support
        LittleEndian::write_u32(&mut data[0x128..0x12C], 0);

        // Offset 0x200: SQ Entry Size - min=64B(6), max=64B(6)
        data[0x200] = 0x66;

        // Offset 0x201: CQ Entry Size - min=16B(2), max=16B(2)
        data[0x201] = 0x22;

        // Offset 0x202: Maximum Outstanding Commands
        LittleEndian::write_u16(&mut data[0x202..0x204], 65535);

        // Offset 0x204: Number of Namespaces (NN)
        LittleEndian::write_u32(&mut data[0x204..0x208], self.num_namespaces);

        // Offset 0x208: Optional NVM Command Support
        LittleEndian::write_u16(&mut data[0x208..0x20A], 0x3);

        // Offset 0x20A: Fused Operation Support
        LittleEndian::write_u16(&mut data[0x20A..0x20C], 0x3);

        // Offset 0x210: Volatile Write Cache
        data[0x210] = 1;
    }

    fn fill_identify_namespace(&self, data: &mut [u8; 4096]) {
        // Per NVMe 1.4 spec §6.14 / QEMU NvmeIdNs struct

        // Offset 0: Namespace Size (NSZE) - number of LBAs
        LittleEndian::write_u64(&mut data[0..8], self.namespace_capacity);

        // Offset 8: Namespace Capabilities (NCAP)
        LittleEndian::write_u64(&mut data[8..16], self.namespace_capacity);

        // Offset 16: Namespace Usage (NUSE)
        LittleEndian::write_u64(&mut data[16..24], self.namespace_capacity);

        // Offset 25: Number of Logical Block Formats (NLBAF) = count-1
        data[25] = 0; // 1 format supported

        // Offset 26: Format and Data Layout Support (FLBAS)
        // Bits 0-3: LBAF index (0), Bits 4-7: METAD/DSF/MC/MOD (all 0)
        data[26] = 0; // No metadata, use LBAF[0]

        // Offset 27: Metadata Capability (MC)
        data[27] = 0; // No metadata

        // Offset 128: LBAF[0] - MS=0 (no metadata), DS=9 (512B), RP=1 (good)
        // LBAF struct: MS(uint16_t), DS(uint8_t), RP(uint8_t)
        data[128] = 0; // MS low byte (metadata size = 0)
        data[129] = 0; // MS high byte
        data[130] = 9; // DS = log2(512) = 9
        data[131] = 1; // RP = 1 (better relative performance)
    }

    fn admin_get_features(&mut self, sqe: &NvmeSqe) -> (u16, u32) {
        let feat_id = (sqe.cdw10 & 0xFF) as u8;
        info!("Get features: feat_id={}", feat_id);
        (NVME_SC_SUCCESS, 0)
    }

    fn admin_set_features(&mut self, sqe: &NvmeSqe) -> (u16, u32) {
        let feat_id = (sqe.cdw10 & 0xFF) as u8;
        info!("Set features: feat_id={}", feat_id);
        (NVME_SC_SUCCESS, 0)
    }

    fn admin_abort(&mut self, _sqe: &NvmeSqe) -> (u16, u32) {
        info!("Abort command");
        (NVME_SC_SUCCESS, 0)
    }

    fn admin_async_event_request(&mut self, _sqe: &NvmeSqe) -> (u16, u32) {
        info!("Async event request");
        (NVME_SC_SUCCESS, 0)
    }

    // -----------------------------------------------------------------------
    // I/O command handlers
    // -----------------------------------------------------------------------

    fn handle_io_command(&mut self, sqe: &NvmeSqe) -> (u16, u32) {
        match sqe.opcode() {
            NVME_CMD_FLUSH => self.io_flush(sqe),
            NVME_CMD_READ => self.io_read(sqe),
            NVME_CMD_WRITE => self.io_write(sqe),
            NVME_CMD_WRITE_UNCORRECTABLE => {
                warn!("Write Uncorrectable not implemented");
                (NVME_SC_INVALID_OPCODE, 0)
            }
            NVME_CMD_WRITE_ZEROES => {
                warn!("Write Zeroes not implemented");
                (NVME_SC_INVALID_OPCODE, 0)
            }
            NVME_CMD_COMPARE => {
                warn!("Compare not implemented");
                (NVME_SC_INVALID_OPCODE, 0)
            }
            NVME_CMD_VERIFY => {
                warn!("Verify not implemented");
                (NVME_SC_INVALID_OPCODE, 0)
            }
            _ => {
                warn!("Unsupported I/O opcode 0x{:02x}", sqe.opcode());
                (NVME_SC_INVALID_OPCODE, 0)
            }
        }
    }

    fn io_flush(&mut self, _sqe: &NvmeSqe) -> (u16, u32) {
        info!("Flush");
        if !self.disk_readonly {
            if let Err(e) = self.disk_file.flush() {
                error!("Flush failed: {}", e);
                return (NVME_SC_INTERNAL_ERROR, 0);
            }
        }
        (NVME_SC_SUCCESS, 0)
    }

    fn io_read(&mut self, sqe: &NvmeSqe) -> (u16, u32) {
        // NVMe READ: CDW10=SLBA[31:0], CDW11=SLBA[63:32], CDW12=NLB[15:0]|Control[31:16]
        let slba = sqe.cdw10 as u64 | ((sqe.cdw11 as u64) << 32);
        let length = (sqe.cdw12 & 0xFFFF) as u32 + 1;
        let nsid = if sqe.nsid == 0 { 1 } else { sqe.nsid };
        info!("Read: NSID={} SLBA={} length={} PRP1=0x{:x} PRP2=0x{:x}", nsid, slba, length, sqe.prp1(), sqe.prp2());

        if nsid != 1 {
            return (NVME_SC_INVALID_NS, 0);
        }

        let data_len = (length as u64) * LBA_SIZE;
        let mut data = vec![0u8; data_len as usize];

        let offset = slba * LBA_SIZE;
        if offset + data_len > self.namespace_capacity * LBA_SIZE {
            warn!("Read beyond namespace capacity");
            return (NVME_SC_INVALID_FIELD, 0);
        }

        // Read from disk file
        if let Err(e) = self.disk_file.seek(SeekFrom::Start(offset)) {
            error!("Seek failed: {}", e);
            return (NVME_SC_INTERNAL_ERROR, 0);
        }

        if let Err(e) = self.disk_file.read_exact(&mut data) {
            error!("Read failed: {}", e);
            return (NVME_SC_INTERNAL_ERROR, 0);
        }

        if let Err(e) = self.write_prp(sqe.prp1(), sqe.prp2(), &data) {
            warn!("PRP too small for read: {}; writing partial data", e);
            // Write whatever fits in the PRP region
            let cap = self.prp_capacity(sqe.prp1(), sqe.prp2());
            if cap > 0 {
                let _ = self.write_prp(sqe.prp1(), sqe.prp2(), &data[..cap as usize]);
            }
        }
        info!("Read SLBA={} wrote {} bytes, first 16: {:02x?}", slba, data.len(), &data[..std::cmp::min(16, data.len())]);

        (NVME_SC_SUCCESS, 0)
    }

    fn io_write(&mut self, sqe: &NvmeSqe) -> (u16, u32) {
        if self.disk_readonly {
            return (NVME_SC_INVALID_FIELD, 0);
        }

        // NVMe WRITE: CDW10=SLBA[31:0], CDW11=SLBA[63:32], CDW12=NLB[15:0]|Control[31:16]
        let slba = sqe.cdw10 as u64 | ((sqe.cdw11 as u64) << 32);
        let length = (sqe.cdw12 & 0xFFFF) as u32 + 1;
        let nsid = if sqe.nsid == 0 { 1 } else { sqe.nsid };
        info!("Write: NSID={} SLBA={} length={}", nsid, slba, length);

        if nsid != 1 {
            return (NVME_SC_INVALID_NS, 0);
        }

        let data_len = (length as u64) * LBA_SIZE;
        let mut data = vec![0u8; data_len as usize];

        let offset = slba * LBA_SIZE;
        if offset + data_len > self.namespace_capacity * LBA_SIZE {
            warn!("Write beyond namespace capacity");
            return (NVME_SC_INVALID_FIELD, 0);
        }

        if let Err(e) = self.read_prp(sqe.prp1(), sqe.prp2(), &mut data) {
            error!("Failed to read PRP for write: {}", e);
            return (NVME_SC_INTERNAL_ERROR, 0);
        }

        // Write to disk file
        if let Err(e) = self.disk_file.seek(SeekFrom::Start(offset)) {
            error!("Seek failed: {}", e);
            return (NVME_SC_INTERNAL_ERROR, 0);
        }

        if let Err(e) = self.disk_file.write_all(&data) {
            error!("Write failed: {}", e);
            return (NVME_SC_INTERNAL_ERROR, 0);
        }

        (NVME_SC_SUCCESS, 0)
    }

    // -----------------------------------------------------------------------
    // PRP helpers
    // -----------------------------------------------------------------------

    fn read_prp(&self, prp1: u64, prp2: u64, buf: &mut [u8]) -> Result<()> {
        if prp1 == 0 {
            return Ok(());
        }

        let guest_mem = self.guest_memory.memory();
        let mut offset = 0;

        // PRP1: First physical region page
        let prp1_page_offset = prp1 % NVME_PAGE_SIZE;
        let prp1_remaining = NVME_PAGE_SIZE - prp1_page_offset;
        let to_copy = std::cmp::min(buf.len() as u64, prp1_remaining);

        if to_copy > 0 {
            let mut page_buf = vec![0u8; to_copy as usize];
            guest_mem
                .read(&mut page_buf[..], GuestAddress(prp1))
                .map_err(Error::GuestMemory)?;
            buf[..to_copy as usize].copy_from_slice(&page_buf);
            offset += to_copy as usize;
        }

        if offset >= buf.len() {
            return Ok(());
        }

        // PRP2: Either a second page or a PRP list
        if prp2 == 0 {
            // If PRP1 was not page-aligned, the next page is used automatically
            if prp1_page_offset != 0 {
                let next_page = prp1 - prp1_page_offset + NVME_PAGE_SIZE;
                let remaining = buf.len() - offset;
                if remaining as u64 > NVME_PAGE_SIZE {
                    return Err(Error::InvalidPrpList);
                }
                if remaining > 0 {
                    let mut page_buf = vec![0u8; remaining];
                    guest_mem
                        .read(&mut page_buf[..], GuestAddress(next_page))
                        .map_err(Error::GuestMemory)?;
                    buf[offset..].copy_from_slice(&page_buf);
                }
                return Ok(());
            }
            return Err(Error::InvalidPrpList);
        }

        // NVMe spec: if remaining data fits in one page, PRP2 is a direct page;
        // otherwise PRP2 points to a PRP list
        let remaining_after_prp1 = buf.len() - offset;
        if remaining_after_prp1 as u64 <= NVME_PAGE_SIZE {
            // PRP2 is a direct second page
            if remaining_after_prp1 > 0 {
                let mut page_buf = vec![0u8; remaining_after_prp1];
                guest_mem
                    .read(&mut page_buf[..], GuestAddress(prp2))
                    .map_err(Error::GuestMemory)?;
                buf[offset..].copy_from_slice(&page_buf);
            }
        } else {
            // PRP2 is a PRP list pointer
            let mut prp_list_addr = prp2;
            let mut remaining = remaining_after_prp1;

            while remaining > 0 {
                let mut prp_entry_bytes = [0u8; 8];
                guest_mem
                    .read(&mut prp_entry_bytes, GuestAddress(prp_list_addr))
                    .map_err(Error::GuestMemory)?;
                let prp_entry = LittleEndian::read_u64(&prp_entry_bytes);

                if prp_entry == 0 {
                    break;
                }

                let page_size = std::cmp::min(remaining as u64, NVME_PAGE_SIZE);
                let mut page_buf = vec![0u8; page_size as usize];
                guest_mem
                    .read(&mut page_buf[..], GuestAddress(prp_entry))
                    .map_err(Error::GuestMemory)?;
                buf[offset..offset + page_size as usize].copy_from_slice(&page_buf);
                offset += page_size as usize;
                remaining -= page_size as usize;

                prp_list_addr += 8;
            }
        }

        Ok(())
    }

    /// Calculate how many bytes the PRP can hold
    fn prp_capacity(&self, prp1: u64, prp2: u64) -> u64 {
        if prp1 == 0 {
            return 0;
        }
        let prp1_page_offset = prp1 % NVME_PAGE_SIZE;
        let mut capacity = NVME_PAGE_SIZE - prp1_page_offset;
        if prp2 == 0 {
            // Only overflow page if PRP1 was not page-aligned
            if prp1_page_offset != 0 {
                capacity += NVME_PAGE_SIZE;
            }
        } else if prp2 & 1 == 0 {
            // Second physical region page
            capacity += NVME_PAGE_SIZE;
        }
        // Note: PRP lists are not counted (would need to walk the list)
        capacity
    }

    fn write_prp(&self, prp1: u64, prp2: u64, buf: &[u8]) -> Result<()> {
        if prp1 == 0 {
            return Ok(());
        }

        let guest_mem = self.guest_memory.memory();
        let mut offset = 0;

        // PRP1: First physical region page
        let prp1_page_offset = prp1 % NVME_PAGE_SIZE;
        let prp1_remaining = NVME_PAGE_SIZE - prp1_page_offset;
        let to_copy = std::cmp::min(buf.len() as u64, prp1_remaining);

        if to_copy > 0 {
            guest_mem
                .write(&buf[..to_copy as usize], GuestAddress(prp1))
                .map_err(Error::GuestMemory)?;
            offset += to_copy as usize;
        }

        if offset >= buf.len() {
            return Ok(());
        }

        // PRP2: Either a second page or a PRP list
        if prp2 == 0 {
            // If PRP1 was not page-aligned, the remaining data goes to the next page
            if prp1_page_offset != 0 {
                let next_page = prp1 - prp1_page_offset + NVME_PAGE_SIZE;
                let remaining = buf.len() - offset;
                if remaining as u64 > NVME_PAGE_SIZE {
                    return Err(Error::InvalidPrpList);
                }
                guest_mem
                    .write(&buf[offset..], GuestAddress(next_page))
                    .map_err(Error::GuestMemory)?;
                return Ok(());
            }
            return Err(Error::InvalidPrpList);
        }

        // NVMe spec: if remaining data fits in one page, PRP2 is a direct page pointer;
        // otherwise PRP2 points to a PRP list. No bit-0 convention.
        let remaining_after_prp1 = buf.len() - offset;
        if remaining_after_prp1 as u64 <= NVME_PAGE_SIZE {
            // PRP2 is a direct second page
            if remaining_after_prp1 > 0 {
                guest_mem
                    .write(&buf[offset..], GuestAddress(prp2))
                    .map_err(Error::GuestMemory)?;
            }
        } else {
            // PRP2 is a PRP list pointer
            let mut prp_list_addr = prp2;
            let mut remaining = remaining_after_prp1;

            while remaining > 0 {
                let mut prp_entry_bytes = [0u8; 8];
                guest_mem
                    .read(&mut prp_entry_bytes, GuestAddress(prp_list_addr))
                    .map_err(Error::GuestMemory)?;
                let prp_entry = LittleEndian::read_u64(&prp_entry_bytes);

                if prp_entry == 0 {
                    break;
                }

                let page_size = std::cmp::min(remaining as u64, NVME_PAGE_SIZE);
                guest_mem
                    .write(&buf[offset..offset + page_size as usize], GuestAddress(prp_entry))
                    .map_err(Error::GuestMemory)?;
                offset += page_size as usize;
                remaining -= page_size as usize;

                prp_list_addr += 8;
            }
        }

        Ok(())
    }

    // -----------------------------------------------------------------------
    // Completion queue
    // -----------------------------------------------------------------------

    fn enqueue_completion(&mut self, cqid: u16, cid: u16, status: u16, sq_id: u16, result: u32) {
        let queue = if cqid == 0 {
            &self.admin_queue
        } else {
            &self.io_queues[cqid as usize - 1]
        };

        if queue.cq_addr.is_none() {
            return;
        }

        let guest_mem = self.guest_memory.memory();
        let cq_addr = queue.cq_addr.unwrap();

        // EDK2's sync PassThru (NvmExpressPassthru.c:868) advances Cqh ^= 1 after
        // each completion and toggles Pt ^= 1 when Cqh wraps to 0. For a queue of
        // size 2: CID=0 at slot 0, CID=1 at slot 1, CID=2 at slot 0 with Pt toggled.
        // We must use tail-based slot calculation AND flip phase on wrap to match.
        let (cq_offset, phase) = if cqid == 0 {
            let tail = self.admin_queue.cq_tail;
            let offset = (tail % queue.queue_size) as u64 * 16;
            (offset, queue.cq_phase)
        } else {
            let queue = &self.io_queues[cqid as usize - 1];
            let tail = queue.cq_tail;
            let offset = (tail % queue.queue_size) as u64 * 16;
            (offset, queue.cq_phase)
        };

        let cqe_addr = GuestAddress(cq_addr.0 + cq_offset);
        info!("NVMe enqueue_completion cqid={} cid={} status=0x{:04x} cq_addr=0x{:x} offset={} cqe_addr=0x{:x} phase={}", cqid, cid, status, cq_addr.raw_value(), cq_offset, cqe_addr.raw_value(), phase);

        let cqe = NvmeCqe::new(cid, phase, status, sq_id, 0, result);
        let mut cqe_bytes = [0u8; 16];
        // SAFETY: NvmeCqe is a simple struct with no padding issues, and we're
        // copying exactly 16 bytes which matches the struct size.
        unsafe {
            std::ptr::copy_nonoverlapping(
                &cqe as *const NvmeCqe as *const u8,
                cqe_bytes.as_mut_ptr(),
                16,
            );
        }
        info!("NVMe CQE bytes: {:02x?}", &cqe_bytes);

        if let Err(e) = guest_mem.write(&cqe_bytes, cqe_addr) {
            error!("Failed to write CQE: {}", e);
            return;
        }

        // Advance CQ tail and flip phase on wrap.
        // EDK2's sync PassThru (NvmExpressPassthru.c:868) does:
        //   if ((Cqh ^= 1) == 0) Pt ^= 1;
        // which toggles Pt when Cqh wraps. We match this by flipping phase on wrap.
        if cqid == 0 {
            let old_tail = self.admin_queue.cq_tail;
            self.admin_queue.cq_tail = (old_tail + 1) % self.admin_queue.queue_size;
            if self.admin_queue.cq_tail < old_tail {
                self.admin_queue.cq_phase ^= true;
            }
        } else {
            let queue = &mut self.io_queues[cqid as usize - 1];
            let old_tail = queue.cq_tail;
            queue.cq_tail = (old_tail + 1) % queue.queue_size;
            if queue.cq_tail < old_tail {
                queue.cq_phase ^= true;
            }
        }

        // Trigger MSI-X interrupt if not masked
        let msix = self.msix_config.lock().unwrap();
        let interrupt_masked = msix.masked()
            || msix.table_entries[cqid as usize].masked()
            || (self.intms & (1 << cqid)) == 0;
        drop(msix);

        if !interrupt_masked {
            if let Some(ref group) = self.interrupt_group {
                if let Err(e) = group.trigger(cqid as InterruptIndex) {
                    error!("Failed to trigger MSI-X interrupt: {}", e);
                }
            }
        }
    }
}

/// Parse SQE bytes into NvmeSqe struct (EDK2-compatible layout)
fn parse_sqe(bytes: &[u8; 64]) -> NvmeSqe {
    NvmeSqe {
        dword0: LittleEndian::read_u32(&bytes[0..4]),
        dword1: LittleEndian::read_u32(&bytes[4..8]),
        nsid: LittleEndian::read_u32(&bytes[8..12]),
        reserved1: LittleEndian::read_u32(&bytes[12..16]),
        cdw10: LittleEndian::read_u32(&bytes[40..44]),
        cdw11: LittleEndian::read_u32(&bytes[44..48]),
        cdw12: LittleEndian::read_u32(&bytes[48..52]),
        cdw13: LittleEndian::read_u32(&bytes[52..56]),
        cdw14: LittleEndian::read_u32(&bytes[56..60]),
        prp1_lo: LittleEndian::read_u32(&bytes[24..28]),
        prp1_hi: LittleEndian::read_u32(&bytes[28..32]),
        prp2_lo: LittleEndian::read_u32(&bytes[32..36]),
        prp2_hi: LittleEndian::read_u32(&bytes[36..40]),
        reserved2: [0; 8],
    }
}

impl NvmeSqe {
    fn prp1(&self) -> u64 {
        self.prp1_lo as u64 | ((self.prp1_hi as u64) << 32)
    }

    fn prp2(&self) -> u64 {
        self.prp2_lo as u64 | ((self.prp2_hi as u64) << 32)
    }
}

// ---------------------------------------------------------------------------
// BusDevice implementation
// ---------------------------------------------------------------------------

impl BusDevice for NvmeController {
    fn read(&mut self, base: u64, offset: u64, data: &mut [u8]) {
        if offset < DBL_BASE_OFFSET {
            self.read_register(offset, data);
        } else {
            let tbl_offset = self.msix_cap.table_offset() as u64;
            let tbl_size = self.msix_cap.table_size() as u64 * 16;
            let pba_offset = self.msix_cap.pba_offset() as u64;
            let pba_size = ((self.msix_cap.table_size() as u64 / 64) + 1) * 8;

            if tbl_offset <= offset && offset < tbl_offset + tbl_size {
                let table_offset = offset - tbl_offset;
                self.msix_config.lock().unwrap().read_table(table_offset, data);
            } else if pba_offset <= offset && offset < pba_offset + pba_size {
                let pba_offset_rel = offset - pba_offset;
                self.msix_config.lock().unwrap().read_pba(pba_offset_rel, data);
            } else {
                warn!("Unexpected BAR0 read at offset 0x{:x}", offset);
                data.fill(0);
            }
        }
    }

    fn write(&mut self, base: u64, offset: u64, data: &[u8]) -> Option<Arc<Barrier>> {
        info!("NVMe BAR write offset=0x{:x} len={} val=0x{:08x}", offset, data.len(), if data.len() >= 4 { LittleEndian::read_u32(data) } else { 0u32 });
        if offset < DBL_BASE_OFFSET {
            self.write_register(offset, data);
        } else {
            let tbl_offset = self.msix_cap.table_offset() as u64;
            let tbl_size = self.msix_cap.table_size() as u64 * 16;
            let pba_offset = self.msix_cap.pba_offset() as u64;
            let pba_size = ((self.msix_cap.table_size() as u64 / 64) + 1) * 8;

            if tbl_offset <= offset && offset < tbl_offset + tbl_size {
                let table_offset = offset - tbl_offset;
                self.msix_config.lock().unwrap().write_table(table_offset, data);
            } else if pba_offset <= offset && offset < pba_offset + pba_size {
                let pba_offset_rel = offset - pba_offset;
                self.msix_config.lock().unwrap().write_pba(pba_offset_rel, data);
            } else if offset >= DBL_BASE_OFFSET {
                let db_offset = offset - DBL_BASE_OFFSET;
                let sqid = (db_offset / DBL_STRIDE) as u16;
                let new_value = if data.len() >= 2 {
                    LittleEndian::read_u16(data)
                } else {
                    let mut aligned = [0u8; 2];
                    aligned[..data.len()].copy_from_slice(data);
                    LittleEndian::read_u16(&aligned)
                };

                if (db_offset % DBL_STRIDE) < 4 {
                    info!("NVMe SQ{} doorbell tail={}", sqid, new_value);
                    self.process_sq_doorbell(sqid, new_value);
                } else {
                    info!("NVMe CQ{} doorbell head={}", sqid, new_value);
                    self.process_cq_doorbell(sqid, new_value);
                }
            }
        }
        None
    }
}

// ---------------------------------------------------------------------------
// PciDevice implementation
// ---------------------------------------------------------------------------

impl PciDevice for NvmeController {
    fn allocate_bars(
        &mut self,
        _allocator: &mut SystemAllocator,
        mmio32_allocator: &mut AddressAllocator,
        _mmio64_allocator: &mut AddressAllocator,
        _resources: Option<Vec<Resource>>,
    ) -> std::result::Result<Vec<PciBarConfiguration>, PciDeviceError> {
        let bar0_addr = mmio32_allocator
            .allocate(None, BAR0_SIZE, None)
            .ok_or(PciDeviceError::IoAllocationFailed(BAR0_SIZE))?;
        info!("NVMe BAR0 address 0x{:x}", bar0_addr.0);

        let bar0 = PciBarConfiguration::default()
            .set_index(0)
            .set_address(bar0_addr.raw_value())
            .set_size(BAR0_SIZE)
            .set_region_type(PciBarRegionType::Memory32BitRegion)
            .set_prefetchable(PciBarPrefetchable::NotPrefetchable);

        self.configuration
            .add_pci_bar(&bar0)
            .map_err(|e| PciDeviceError::IoRegistrationFailed(bar0_addr.raw_value(), e))?;

        self.bar0_base = Some(bar0_addr);
        self.bar_regions.push(bar0);

        Ok(self.bar_regions.clone())
    }

    fn free_bars(
        &mut self,
        _allocator: &mut SystemAllocator,
        mmio32_allocator: &mut AddressAllocator,
        _mmio64_allocator: &mut AddressAllocator,
    ) -> std::result::Result<(), PciDeviceError> {
        if let Some(bar0_addr) = self.bar0_base {
            mmio32_allocator.free(bar0_addr, BAR0_SIZE);
        }
        Ok(())
    }

    fn write_config_register(
        &mut self,
        reg_idx: usize,
        offset: u64,
        data: &[u8],
    ) -> (Vec<BarReprogrammingParams>, Option<Arc<Barrier>>) {
        let bar_reprogramming = self.configuration.write_config_register(reg_idx, offset, data);

        // Forward MSI-X capability writes to our MsixCap
        if let Some(msix_reg_idx) = self.msix_cap_reg_idx {
            if reg_idx == msix_reg_idx {
                // First dword of MSI-X capability (bytes 0-3)
                // Message Control is at byte offset 2 within the capability
                if offset == 2 && data.len() == 2 {
                    self.msix_cap.set_msg_ctl(LittleEndian::read_u16(data));
                } else if offset == 0 && data.len() == 4 {
                    self.msix_cap.set_msg_ctl((LittleEndian::read_u32(data) >> 16) as u16);
                }
            } else if reg_idx == msix_reg_idx + 1 {
                // Second dword of MSI-X capability (bytes 4-7) = Table Offset
                if offset == 0 && data.len() == 4 {
                    self.msix_cap.table = LittleEndian::read_u32(data);
                }
            } else if reg_idx == msix_reg_idx + 2 {
                // Third dword of MSI-X capability (bytes 8-11) = PBA Offset
                if offset == 0 && data.len() == 4 {
                    self.msix_cap.pba = LittleEndian::read_u32(data);
                }
            }
        }

        (bar_reprogramming, None)
    }

    fn read_config_register(&mut self, reg_idx: usize) -> u32 {
        self.configuration.read_reg(reg_idx)
    }

    fn read_bar(&mut self, base: u64, offset: u64, data: &mut [u8]) {
        info!(
            "NVMe read_bar: base=0x{:x} offset=0x{:x} bar0_base={:?}",
            base,
            offset,
            self.bar0_base.map(|b| b.raw_value())
        );
        if self.bar0_base.map(|b| b.raw_value()) == Some(base) {
            self.read(base, offset, data);
        } else {
            warn!(
                "Unexpected BAR read at base 0x{:x} offset 0x{:x} (expected 0x{:x})",
                base,
                offset,
                self.bar0_base.map(|b| b.raw_value()).unwrap_or(0)
            );
            data.fill(0);
        }
    }

    fn write_bar(&mut self, base: u64, offset: u64, data: &[u8]) -> Option<Arc<Barrier>> {
        info!(
            "NVMe write_bar: base=0x{:x} offset=0x{:x} bar0_base={:?}",
            base,
            offset,
            self.bar0_base.map(|b| b.raw_value())
        );
        if self.bar0_base.map(|b| b.raw_value()) == Some(base) {
            self.write(base, offset, data)
        } else {
            warn!(
                "Unexpected BAR write at base 0x{:x} offset 0x{:x} (expected 0x{:x})",
                base,
                offset,
                self.bar0_base.map(|b| b.raw_value()).unwrap_or(0)
            );
            None
        }
    }

    fn restore_bar_addr(&mut self, params: &BarReprogrammingParams) {
        self.configuration.restore_bar_addr(params);
    }

    fn move_bar(&mut self, _old_base: u64, new_base: u64) -> std::result::Result<(), std::io::Error> {
        self.bar0_base = Some(GuestAddress(new_base));
        info!("NVMe BAR0 moved to 0x{:x}", new_base);
        Ok(())
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn id(&self) -> Option<String> {
        Some(self.id.clone())
    }
}

// ---------------------------------------------------------------------------
// Migration traits (no-op for now)
// ---------------------------------------------------------------------------

impl vm_migration::Pausable for NvmeController {}
impl vm_migration::Snapshottable for NvmeController {}
impl vm_migration::Transportable for NvmeController {}
impl vm_migration::Migratable for NvmeController {}
