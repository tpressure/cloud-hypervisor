The easiest overall solution is to inject the virtio storage driver into Windows boot.wim; that requires no new Cloud Hypervisor device. Microsoft explicitly supports adding drivers to offline Windows PE images using DISM. The driver should also
  be added to install.wim so the installed system can boot from the virtio disk. Microsoft DISM documentation
  (https://learn.microsoft.com/en-us/windows-hardware/manufacture/desktop/add-and-remove-drivers-to-an-offline-windows-image?view=windows-11)

  If the original Windows ISO must remain untouched, I recommend implementing a minimal NVMe disk, backed by a GPT/FAT image containing the extracted virtio drivers.

  Why NVMe:

  - Windows supplies stornvme.sys starting with Windows 8.1 and Windows Server 2012 R2. Microsoft StorNVMe documentation (https://learn.microsoft.com/en-us/windows-hardware/drivers/storage/nvme-features-supported-by-stornvme)
  - It avoids implementing AHCI, ATA, ATAPI and CD-ROM/MMC command sets.
  - It avoids the much larger USB/xHCI stack.
  - It uses Cloud Hypervisor’s existing PCI, BAR, MSI-X and guest-memory infrastructure.
  - Cloud Hypervisor already supports external VFIO-user NVMe devices, providing a useful reference/prototyping path in docs/vfio-user.md:42.

  ## Proposed implementation plan

  1. Prove the approach before writing a built-in device.
      - Extract only the required x64 viostor driver files from the virtio driver ISO.
      - Create a small GPT/FAT32 raw image containing them.
      - Attach it as an SPDK VFIO-user NVMe device through the existing --user-device support.
      - Confirm that the exact Windows installer sees the FAT volume, loads viostor, and then discovers the existing virtio system disk.

  2. Add transport selection to disk configuration.

     Proposed syntax:

     --disk path=windows.iso,readonly=on
     --disk path=virtio-drivers.img,transport=nvme,readonly=on

     transport would default to virtio, preserving all existing command lines and API configurations.

  3. Implement a deliberately small NVMe controller.

     Initial scope:
      - One PCI function and one namespace.
      - NVMe PCI class 01/08/02.
      - BAR0 controller registers and doorbells.
      - MSI-X interrupts.
      - Admin queues and I/O queues.
      - Identify controller/namespace.
      - Create/delete submission and completion queues.
      - Get/set features needed by StorNVMe.
      - Read, write and flush commands.
      - Correct PRP and PRP-list handling.
      - Reset and controller-disable behavior.

     Even though the driver image is conceptually read-only, supporting writes initially may improve compatibility with Windows volume mounting. The host image can be copied fresh for each installation.

  4. Integrate lifecycle handling.
      - Cold-plug initially; hotplug can follow separately.
      - Pause/resume and reset.
      - Snapshot queue/controller state deterministically.
      - Drain in-flight requests before migration.
      - Validate backing-file size and identity during restore.
      - Strictly bounds-check every guest DMA address and disk offset.

  5. Add focused tests.
      - PCI identification and register reset values.
      - Controller enable/disable sequencing.
      - Queue wraparound and completion phase bits.
      - Identify structures.
      - PRP-list traversal and malformed guest addresses.
      - End-of-disk and read-only behavior.
      - Snapshot/restore with pending and completed queues.

  6. Windows acceptance test.

     Using the unchanged Windows installation ISO:
      - Reach Windows Setup through VNC.
      - Open “Load driver”.
      - See and browse the NVMe-backed FAT volume.
      - Load viostor.
      - Confirm the virtio system disk appears.
      - Partition it and begin installation.
      - Reboot far enough to confirm the installed system can still access its virtio boot disk.

  I would avoid IDE/ATAPI, AHCI, floppy and USB for this task: each brings a larger legacy or protocol stack than the narrow NVMe subset Windows requires. No files were changed during this planning pass.

