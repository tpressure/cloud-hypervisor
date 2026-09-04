#!/bin/bash
# Run Cloud Hypervisor with the external vfio-user RAMFB VNC backend.
# Boots a VM with EDK2 firmware (QemuRamfbDxe + QemuFwCfgLib) that writes
# the framebuffer config to the fw_cfg etc/ramfb item. Pixel access is
# provided to vfio-user-simplefb through shared guest-memory DMA mappings.
#
# Usage:
#   ./run_ramfb.sh              # Start VM with VNC on Unix socket
#   ./run_ramfb.sh --tcp 5900   # Start VM with VNC on TCP port 5900
#   ./run_ramfb.sh --viewer     # Start VM and launch vncviewer

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CH_BIN="${SCRIPT_DIR}/target/release/cloud-hypervisor"
FB_DAEMON="${SCRIPT_DIR}/target/release/vfio_user_simplefb"
USB_DAEMON="${SCRIPT_DIR}/target/release/vfio-usb-hid"

# Use EDK2 firmware built from gardenlinux branch (has QemuRamfbDxe + QemuFwCfgLib).
# Falls back to CLOUDHV.fd in repo root if the build artifact is not present.
EDK2_FIRMWARE="/home/gonzo/opencode/edk2/Build/CloudHvX64/DEBUG_GCC5/FV/CLOUDHV.fd"
FALLBACK_FIRMWARE="${SCRIPT_DIR}/CLOUDHV.fd"

if [ -f "$EDK2_FIRMWARE" ]; then
    FIRMWARE="$EDK2_FIRMWARE"
else
    echo "Error: No EDK2 firmware found."
    echo "Expected at: $EDK2_FIRMWARE"
    echo "Build EDK2 firmware with QemuRamfbDxe enabled:"
    echo "  cd /home/gonzo/opencode/edk2 && git checkout gardenlinux"
    echo "  source edksetup.sh"
    echo "  build -a X64 -p OvmfPkg/CloudHv/CloudHvX64.dsc -b DEBUG -t GCC5"
    exit 1
fi

DISK="${SCRIPT_DIR}/oracular-server-cloudimg-amd64.raw"
CLOUDINIT="/tmp/ubuntu-cloudinit.img"
VNC_SOCKET="/tmp/ch-vm.vnc.sock"
VFIO_USER_SOCKET="/tmp/ch-vm.simplefb.sock"
USB_VFIO_USER_SOCKET="/tmp/ch-vm.usb-hid.sock"
INPUT_SOCKET="/tmp/ch-vm.input.sock"
API_SOCKET="/tmp/ch-api.sock"

# Default VNC display parameters (can be overridden by firmware)
VNC_WIDTH="${VNC_WIDTH:-1024}"
VNC_HEIGHT="${VNC_HEIGHT:-768}"
FB_GPA="${FB_GPA:-0xBEB00000}"
VNC_MODE="unix:${VNC_SOCKET}"
VNC_VIEWER_TARGET="Unix:${VNC_SOCKET}"
VIEWER=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --tcp)
            VNC_MODE="tcp:${2}"
            VNC_VIEWER_TARGET="localhost::${2}"
            shift 2
            ;;
        --viewer)
            VIEWER=true
            shift
            ;;
        --width)
            VNC_WIDTH="$2"
            shift 2
            ;;
        --height)
            VNC_HEIGHT="$2"
            shift 2
            ;;
        --fb-gpa)
            FB_GPA="$2"
            shift 2
            ;;
        --firmware)
            FIRMWARE="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [--tcp <port>] [--viewer] [--width <w>] [--height <h>] [--fb-gpa <gpa>] [--firmware <path>]"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Check prerequisites
if [ ! -f "$CH_BIN" ]; then
    echo "Error: cloud-hypervisor binary not found at $CH_BIN"
    echo "Run: cargo build --release --features kvm,fw_cfg"
    exit 1
fi

if [ ! -f "$FB_DAEMON" ]; then
    echo "Error: vfio-user-simplefb binary not found at $FB_DAEMON"
    echo "Run: cargo build --release -p vfio_user_simplefb"
    exit 1
fi

if [ ! -f "$USB_DAEMON" ]; then
    echo "Error: vfio-usb-hid binary not found at $USB_DAEMON"
    echo "Run: cargo build --release -p vfio-usb-hid"
    exit 1
fi

if [ ! -f "$DISK" ]; then
    echo "Error: Disk image not found at $DISK"
    exit 1
fi

# Clean up stale sockets
rm -f "$VNC_SOCKET" "$VFIO_USER_SOCKET" "$USB_VFIO_USER_SOCKET" "$INPUT_SOCKET" "$API_SOCKET"

cleanup() {
    if [ -n "${VIEWER_PID:-}" ]; then
        kill "$VIEWER_PID" 2>/dev/null || true
        wait "$VIEWER_PID" 2>/dev/null || true
    fi
    if [ -n "${FB_DAEMON_PID:-}" ]; then
        kill "$FB_DAEMON_PID" 2>/dev/null || true
        wait "$FB_DAEMON_PID" 2>/dev/null || true
    fi
    if [ -n "${USB_DAEMON_PID:-}" ]; then
        kill "$USB_DAEMON_PID" 2>/dev/null || true
        wait "$USB_DAEMON_PID" 2>/dev/null || true
    fi
    rm -f "$VNC_SOCKET" "$VFIO_USER_SOCKET" "$USB_VFIO_USER_SOCKET" "$INPUT_SOCKET" "$API_SOCKET"
}
trap cleanup EXIT INT TERM

echo "=== Cloud Hypervisor RAMFB VNC ==="
echo "Firmware:   $FIRMWARE"
echo "Disk:       $DISK"
echo "Cloud-init: $CLOUDINIT"
echo "VNC:        $VNC_MODE"
echo "Display:    ${VNC_WIDTH}x${VNC_HEIGHT}"
echo "FB GPA:     ${FB_GPA}"
echo ""
echo "To connect with VNC viewer:"
echo "  vncviewer ${VNC_VIEWER_TARGET}"
echo ""
echo "Note: --display ramfb enables fw_cfg for QemuRamfbDxe."
echo ""
echo "Starting external USB HID daemon..."
echo ""

"$USB_DAEMON" \
    --socket "$USB_VFIO_USER_SOCKET" \
    --input-socket "$INPUT_SOCKET" &
USB_DAEMON_PID=$!

for _ in $(seq 1 50); do
    if [ -S "$USB_VFIO_USER_SOCKET" ] && [ -S "$INPUT_SOCKET" ]; then
        break
    fi
    if ! kill -0 "$USB_DAEMON_PID" 2>/dev/null; then
        echo "Error: vfio-usb-hid exited before creating its sockets"
        exit 1
    fi
    sleep 0.1
done

if [ ! -S "$USB_VFIO_USER_SOCKET" ] || [ ! -S "$INPUT_SOCKET" ]; then
    echo "Error: timed out waiting for vfio-usb-hid sockets"
    exit 1
fi

echo "Starting external framebuffer daemon..."
echo ""

"$FB_DAEMON" \
    --socket "$VFIO_USER_SOCKET" \
    --input-socket "$INPUT_SOCKET" \
    --fb-gpa "$FB_GPA" \
    --width "$VNC_WIDTH" \
    --height "$VNC_HEIGHT" \
    --stride "$((VNC_WIDTH * 4))" \
    --format xrgb8888 \
    --vnc "$VNC_MODE" &
FB_DAEMON_PID=$!

for _ in $(seq 1 50); do
    if [ -S "$VFIO_USER_SOCKET" ]; then
        break
    fi
    if ! kill -0 "$FB_DAEMON_PID" 2>/dev/null; then
        echo "Error: vfio-user-simplefb exited before creating its socket"
        exit 1
    fi
    sleep 0.1
done

if [ ! -S "$VFIO_USER_SOCKET" ]; then
    echo "Error: timed out waiting for $VFIO_USER_SOCKET"
    exit 1
fi

if [ "$VIEWER" = true ]; then
    if ! command -v vncviewer >/dev/null 2>&1; then
        echo "Error: --viewer requested but vncviewer is not installed"
        exit 1
    fi
    vncviewer "$VNC_VIEWER_TARGET" &
    VIEWER_PID=$!
fi

echo "Starting VM..."
echo ""

"$CH_BIN" \
    --kernel "$FIRMWARE" \
    --disk "path=$DISK" "path=$CLOUDINIT" \
    --cpus boot=4 \
    --memory size=4096M,shared=on \
    --console tty \
    --seccomp log \
    --api-socket "$API_SOCKET" \
    --display ramfb \
    --user-device "socket=${VFIO_USER_SOCKET},id=simplefb-transport" \
    --user-device "socket=${USB_VFIO_USER_SOCKET},id=usb-hid"
