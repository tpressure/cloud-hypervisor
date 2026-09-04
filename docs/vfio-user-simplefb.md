# External RAMFB VNC backend

`vfio-user-simplefb` serves the firmware RAM framebuffer from a process
separate from Cloud Hypervisor. The vfio-user PCI function is only a transport
endpoint which causes Cloud Hypervisor to export shared guest RAM. It has no
BARs, interrupts, display registers, or guest driver.

The prototype endpoint uses PCI ID `1b36:00ff`, an unregistered device ID in
the Red Hat/QEMU virtual-device namespace. It deliberately avoids IDs belonging
to real QEMU device models and should be replaced by an assigned ID before any
production use.

The guest-visible display remains EDK2 `QemuRamfbDxe` and UEFI GOP. Linux and
Windows continue to use their firmware framebuffer paths.

## Memory layout

With the 4 GiB configuration in `run_ramfb.sh`, Cloud Hypervisor creates these
normal guest-memory-zone regions:

```text
0x0000000000000000..0x00000000bfffffff  3 GiB low RAM
0x0000000100000000..0x000000013fffffff  1 GiB high RAM
```

The supplied EDK2 build deterministically allocates its 1024x768 framebuffer
at `0x00000000beb00000` in the low region:

```text
0x00000000beb00000..0x00000000bedfffff  3 MiB framebuffer
```

EDK2 allocates these pages with `AllocateReservedPages()`. They therefore have
ordinary Cloud Hypervisor guest-RAM backing while being reserved from guest OS
allocation. The active format is DRM `XRGB8888` (B, G, R, unused byte order on
little-endian hosts), with a 4096-byte stride and a 3,145,728-byte active size.

The GPA is allocated by firmware rather than compiled into Cloud Hypervisor.
Confirm the `Ramfb: Framebuffer at ...` firmware log whenever the firmware,
memory size, or device topology changes, then pass the reported address to
`--fb-gpa`. The default in `run_ramfb.sh` is specific to the supplied firmware
and 4 GiB VM layout.

## Build and run

Build both external devices and Cloud Hypervisor:

```bash
cargo build --release -p vfio-usb-hid -p vfio_user_simplefb
cargo build --release -p cloud-hypervisor --features kvm,fw_cfg
```

The convenience script launches the USB HID and framebuffer endpoints first,
waits for their sockets, and then launches Cloud Hypervisor:

```bash
./run_ramfb.sh
```

The equivalent daemon command is:

```bash
target/release/vfio_user_simplefb \
    --socket /tmp/ch-vm.simplefb.sock \
    --input-socket /tmp/ch-vm.input.sock \
    --fb-gpa 0xBEB00000 \
    --width 1024 \
    --height 768 \
    --stride 4096 \
    --format xrgb8888 \
    --vnc unix:/tmp/ch-vm.vnc.sock
```

Cloud Hypervisor must use file-backed shared memory. `--display ramfb` enables
the existing fw_cfg item needed by `QemuRamfbDxe`; it does not start a VNC
server in Cloud Hypervisor:

```bash
target/release/cloud-hypervisor \
    --kernel /home/gonzo/opencode/edk2/Build/CloudHvX64/DEBUG_GCC5/FV/CLOUDHV.fd \
    --disk path=oracular-server-cloudimg-amd64.raw \
           path=/tmp/ubuntu-cloudinit.img \
    --cpus boot=4 \
    --memory size=4096M,shared=on \
    --console tty \
    --seccomp log \
    --api-socket /tmp/ch-api.sock \
    --display ramfb \
    --user-device socket=/tmp/ch-vm.simplefb.sock,id=simplefb-transport \
    --user-device socket=/tmp/ch-vm.usb-hid.sock,id=usb-hid
```

See [External vfio-user USB HID input](vfio-usb-hid.md) for the separate UHCI
device, host-input socket, and guest discovery details.

Use `--checksum-interval-ms 1000` on the daemon for a memory-visibility test.
Changing checksums prove that firmware or the guest is updating the directly
mapped framebuffer; no pixel data is carried in vfio-user messages.

## DMA and lifetime handling

For the configuration above, Cloud Hypervisor sends identity mappings for the
two memory-zone regions. The daemon validates the full framebuffer range
against every received IOVA range and mmaps the passed shared-memory file
descriptor. It does not assume that the framebuffer begins at a mapping base.

Framebuffer snapshots hold a mapping-state read lock while copying pixels.
`VFIO_USER_DMA_UNMAP` takes the write lock, waits for active copies, and removes
every mapping overlapping the unmap range. A partial unmap conservatively
invalidates the complete local mapping. Reset and disconnect also stop further
framebuffer access.

After Cloud Hypervisor has supplied its DMA regions, the daemon exits with an
actionable error if none contains the configured framebuffer range. This most
commonly means that `--fb-gpa` does not match the address reported by firmware.

## Manual compatibility tests

Linux:

1. Boot an unmodified Linux image with the commands above.
2. Confirm the EDK2 screen and kernel output over VNC.
3. Confirm the guest binds its normal firmware framebuffer path (`simpledrm`,
   `efifb`, or the distribution's equivalent) and has no driver bound to the
   transport PCI function.

Windows:

1. Boot the existing unmodified Windows image with the same daemon and shared
   memory options.
2. Confirm UEFI output followed by Windows basic-display output over VNC.
3. Confirm that no driver is installed for the transport PCI function.

## Limitations

- The current firmware chooses the GPA dynamically. Automatic GPA discovery
  would require a separate configuration channel or a fixed-address EDK2
  allocation; neither belongs in generic vfio-user code.
- Framebuffer geometry is explicit daemon configuration. A firmware or guest
  GOP mode change is not discovered automatically, so width, height, stride,
  and format must match the active firmware mode.
- VNC input is forwarded over a separate host-only Unix socket to
  `vfio-usb-hid`; the framebuffer PCI function contains no input registers.
- The server accepts one Cloud Hypervisor connection per process lifetime and
  one VNC client at a time.
- The inherited VNC backend has no authentication. Prefer the Unix listener;
  protect any TCP listener with appropriate network controls.
- Full frames are compared and copied at up to 25 frames per second. More
  granular hashing is a possible optimization and does not require a guest
  protocol change.
