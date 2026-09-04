# External vfio-user USB HID input

`vfio-usb-hid` is a separate process which presents a conventional PCI UHCI
USB 1.1 host controller to a Cloud Hypervisor guest. Two permanently attached
low-speed devices provide a boot-protocol HID keyboard and mouse. Linux and
Windows use their standard PCI USB and USB HID drivers; no guest driver is
needed.

PS/2 input is deliberately not used. An external vfio-user process exposes a
PCI function, while i8042 is a platform device with I/O ports and interrupt
routing outside that PCI function. A standards-based PCI USB controller keeps
the device boundary self-contained.

UHCI was selected instead of OHCI because it is the preferred smallest USB
controller for the required control and interrupt transfers. Runtime testing
confirmed that Cloud Hypervisor forwards vfio-user I/O BAR accesses and INTx,
so UHCI's two potential integration constraints are not blockers.

## Architecture

```text
                         stock guest
                    +---------+---------+
                    |                   |
               firmware FB          USB HID
                    |              keyboard/mouse
                    |                   |
                    |                PCI UHCI
                    |                   |
                    +---------+---------+
                              |
                       Cloud Hypervisor
                              |
                    generic vfio-user only
                       /              \
                      v                v
          vfio_user_simplefb       vfio-usb-hid
                 ^                     ^
                 |                     |
       VNC framebuffer          host input socket
                 |                     |
                 +--- VNC client ------+
```

Cloud Hypervisor has no VNC, USB, keyboard, or mouse coupling between the two
devices. It connects each PCI function through the existing `--user-device`
option. The separate host-input Unix socket is owned by `vfio-usb-hid`.

## Guest-visible controller

The prototype PCI identity is `1b36:00fe`. The device ID is currently an
unregistered value in the Red Hat/QEMU virtual-device namespace and must be
assigned before production use. The PCI class is `0c0300`:

```text
base class    0x0c  serial bus controller
subclass      0x03  USB controller
programming   0x00  UHCI
revision      0x01
interrupt pin INTA
```

BAR4 is a 32-byte PCI I/O BAR containing the standard UHCI register layout:

```text
0x00  USBCMD       16 bit
0x02  USBSTS       16 bit
0x04  USBINTR      16 bit
0x06  FRNUM        16 bit
0x08  FLBASEADD    32 bit
0x0c  SOFMOD        8 bit
0x10  PORTSC1      16 bit
0x12  PORTSC2      16 bit
```

The controller uses one level-triggered, maskable vfio-user INTx eventfd. It
implements run/halt, host reset, frame advancement, status and interrupt
enables, and two low-speed root ports. Port 1 contains the keyboard and port 2
contains the mouse.

When running, a 1 ms timer processes one entry in the guest's 1024-entry UHCI
frame list. Queue heads and transfer descriptors are fetched iteratively. The
initial implementation supports SETUP, IN, and OUT transactions for control
and interrupt endpoints. An interrupt-IN TD remains active on NAK and is
retried when the schedule reaches it again.

## DMA safety

Cloud Hypervisor supplies guest-memory mappings with `VFIO_USER_DMA_MAP` and
`VFIO_USER_DMA_UNMAP`. The shared `vfio_user_common::GuestMemoryMap` owns those
mmaps and validates every complete GPA range before an access. A read lock is
held for each copy, so unmap waits for an in-flight access and subsequent
schedule accesses fail rather than following stale host pointers.

Frame entries, queue heads, transfer descriptors, and transfer buffers are all
read through this abstraction. Address arithmetic is checked, descriptor
walking is iterative, cycles are detected, and each frame is limited to 256
schedule entries. Invalid DMA stops the controller with the UHCI host-controller
error and halted status instead of accessing memory outside an active mapping.

## USB HID devices

Both devices use USB 1.1 device descriptors with an 8-byte endpoint zero,
vendor `1b36`, one configuration, and no strings. The keyboard product ID is
`0100`; the mouse product ID is `0101`.

The keyboard interface has class/subclass/protocol `03/01/01` and one
interrupt-IN endpoint, address `0x81`, maximum packet size 8, interval 10 ms.
Its 63-byte report descriptor describes eight modifiers, five keyboard LEDs,
and six simultaneous key usages. Its boot and report protocols both use the
standard eight-byte boot-keyboard report. More than six normal keys produces
the HID ErrorRollOver report. Num Lock, Caps Lock, and Scroll Lock output bits
from `SET_REPORT` are retained.

The mouse interface has class/subclass/protocol `03/01/02` and one interrupt-IN
endpoint, address `0x81`, maximum packet size 4, interval 10 ms. Its 52-byte
report descriptor describes three buttons, relative X/Y, and a vertical wheel.
Report protocol returns four bytes; boot protocol returns buttons and X/Y in
three bytes. Motion and wheel values are accumulated and split into signed
8-bit reports without dropping the remainder.

The endpoint-zero implementation supports the standard enumeration requests,
HID report/idle/protocol requests, and delayed application of `SET_ADDRESS`
until its status stage completes.

## Host input protocol

The input socket carries fixed 16-byte binary records. Every record contains
the ASCII magic `VHID`, protocol version 1, an event kind, a little-endian
payload length, and an eight-byte payload. The event model is independent of
VNC, USB, and vfio-user:

```text
Key          USB usage, pressed
Mouse        dx:i16, dy:i16, wheel:i16, buttons:u8
ReleaseAll
KeyboardLeds num-lock, caps-lock, scroll-lock (reserved reverse direction)
```

Reads tolerate stream fragmentation and writes use complete-record semantics.
`vfio_user_simplefb` retries the connection every 250 ms, so framebuffer output
continues if the input process is absent. Connecting, VNC disconnect, IPC
disconnect, and frontend shutdown all release keys, modifiers, mouse buttons,
and pending motion to prevent stuck input.

RFB/X11 keysyms are translated to USB HID usages in the simplefb process. The
mapping covers letters, digits, punctuation, modifiers, navigation keys,
F1-F12, lock keys, and numeric-keypad keysyms. The USB process remains the
authoritative keyboard state owner.

RFB pointer positions are absolute, but the HID mouse is relative. The first
event for each VNC connection establishes a baseline; later positions become
signed deltas. Large deltas are split first into IPC records and then into HID
reports without clamping away motion. RFB wheel-button transitions become
transient wheel deltas, and RFB's left/middle/right ordering is translated to
the HID left/right/middle bit layout.

## Build and launch

Build the two external devices and Cloud Hypervisor:

```bash
cargo build --release -p vfio-usb-hid -p vfio_user_simplefb
cargo build --release -p cloud-hypervisor --features kvm,fw_cfg
```

Start the USB controller first:

```bash
target/release/vfio-usb-hid \
    --socket /tmp/ch-vm.usb-hid.sock \
    --input-socket /tmp/ch-vm.input.sock
```

Start the framebuffer and VNC frontend independently:

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

Replace the example framebuffer GPA with the address reported by the selected
EDK2 build. Then attach both generic vfio-user functions:

```bash
target/release/cloud-hypervisor \
    --kernel /path/to/CLOUDHV.fd \
    --disk path=/path/to/guest.raw \
    --memory size=4096M,shared=on \
    --display ramfb \
    --user-device socket=/tmp/ch-vm.simplefb.sock,id=simplefb-transport \
    --user-device socket=/tmp/ch-vm.usb-hid.sock,id=usb-hid
```

The repository's `run_ramfb.sh` performs this startup ordering, waits for all
three sockets, and cleans up both device processes.

## Verification and limitations

A stock Ubuntu kernel was used to verify the complete PCI, I/O BAR, DMA, and
INTx path. `uhci_hcd` created a two-port bus, enumerated `1b36:0100` and
`1b36:0101` as low-speed devices, and `hid-generic` bound them as a keyboard
and mouse.

The model intentionally provides only one UHCI controller, two root ports, and
the two fixed HID devices. It does not implement USB hubs, hotplug, bulk or
isochronous data devices, EHCI/xHCI companion controllers, passthrough, or
migration state. Windows and EDK2 input still require manual compatibility
testing. The input protocol defines a keyboard-LED record, but LED state is not
currently sent back to the VNC frontend.
