// Copyright 2017 The Chromium OS Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE-BSD-3-Clause file.
//
// SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause

use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Barrier, Mutex};
use std::thread;
use std::time::Instant;

use log::{debug, error, info, warn};
use vm_device::BusDevice;
use vm_device::interrupt::InterruptSourceGroup;
use vmm_sys_util::eventfd::EventFd;

// I/O port offsets relative to base 0x60
const OFFSET_DATA: u64 = 0;
const OFFSET_PORT_B: u64 = 1;
const OFFSET_TIMER_LATCH: u64 = 2;
const OFFSET_MODE_SELECT: u64 = 3;
const OFFSET_COMMAND: u64 = 4;

// PS/2 command codes
const CMD_READ_PORT_A: u8 = 0x20;
const CMD_READ_PORT_B: u8 = 0x21;

// CTR (Controller Configuration Register) bits
const CTR_KBD_INT: u8 = 1 << 0;
const CTR_AUX_INT: u8 = 1 << 1;
const CTR_SYSTEM_FLAG: u8 = 1 << 2;
const CTR_KBD_DISABLE: u8 = 1 << 4;
const CTR_AUX_DISABLE: u8 = 1 << 5;
const CTR_XLATE: u8 = 1 << 6; // Scancode translation enable (Set 2 → Set 1)
const CMD_WRITE_PORT_A: u8 = 0x60;
const CMD_TEST_CONTROLLER: u8 = 0xAA;
const CMD_DISABLE_SECONDARY: u8 = 0xA7;
const CMD_DISABLE_KEYBOARD: u8 = 0xAD;
const CMD_ENABLE_KEYBOARD: u8 = 0xAE;
const CMD_ENABLE_SECONDARY: u8 = 0xA8;
const CMD_TEST_SECONDARY: u8 = 0xA9;
const CMD_READ_KBD_INPUT: u8 = 0xD0;
const CMD_WRITE_KBD_CTRL: u8 = 0xD1;
const CMD_WRITE_KBD_OUTPUT: u8 = 0xD2;
const CMD_WRITE_SECONDARY_OUTPUT: u8 = 0xD3;
const CMD_WRITE_TO_AUX: u8 = 0xD4;
const CMD_SELF_TEST: u8 = 0xF0;
const CMD_RESET_CONTROLLER: u8 = 0xFE;
const CMD_TEST_KEYBOARD: u8 = 0xAB;

// PS/2 status register bits
const STATUS_OUTPUT_BUFFER_FULL: u8 = 1 << 0;
#[allow(dead_code)]
const STATUS_INPUT_BUFFER_FULL: u8 = 1 << 1;
const STATUS_SYSTEM_FLAG: u8 = 1 << 2;
#[allow(dead_code)]
const STATUS_COMMAND_DATA: u8 = 1 << 3;
const STATUS_KEYLOCK: u8 = 1 << 4;
const STATUS_AUX_OUTPUT_BUFFER_FULL: u8 = 1 << 5;
#[allow(dead_code)]
const STATUS_PARITY_ERROR: u8 = 1 << 6;
#[allow(dead_code)]
const STATUS_RTC: u8 = 1 << 7;

// PS/2 keyboard IRQ line
#[allow(dead_code)]
const KBD_IRQ: u8 = 1;
// PS/2 mouse IRQ line
#[allow(dead_code)]
const AUX_IRQ: u8 = 12;

// Maximum data buffer size
const MAX_DATA_BUFFER: usize = 32;

/// Input event types that can be forwarded to the guest via PS/2.
#[derive(Debug, Clone, Copy)]
pub enum InputEvent {
    /// Keyboard event with a virtual key code and pressed state.
    Keyboard { key: u32, pressed: bool },
    /// Mouse event with button state and X/Y movement.
    Mouse { buttons: u8, dx: i16, dy: i16 },
}

/// Maps X11 keysyms to PS/2 keyboard scan codes.
struct KeyboardMap;

impl KeyboardMap {
    /// Generate scan codes from an X11 keysym (US keyboard layout).
    /// Returns a Vec of bytes to send to the PS/2 data port.
    ///
    /// Returns (make_code, needs_e0_prefix). Keys in the dedicated arrow/navigation
    /// cluster and KP_Enter/KP_Divide require the 0xE0 extension prefix.
    fn scan_codes(key: u32, pressed: bool, send_set1: bool) -> Vec<u8> {
        let (make_code, e0) = match key {
            // Letters (lowercase X11 keysyms) -> PS/2 Set 1
            0x61 => (0x1E, false), // a
            0x62 => (0x30, false), // b
            0x63 => (0x2E, false), // c
            0x64 => (0x20, false), // d
            0x65 => (0x12, false), // e
            0x66 => (0x21, false), // f
            0x67 => (0x22, false), // g
            0x68 => (0x23, false), // h
            0x69 => (0x17, false), // i
            0x6A => (0x24, false), // j
            0x6B => (0x25, false), // k
            0x6C => (0x26, false), // l
            0x6D => (0x32, false), // m
            0x6E => (0x31, false), // n
            0x6F => (0x18, false), // o
            0x70 => (0x19, false), // p
            0x71 => (0x10, false), // q
            0x72 => (0x13, false), // r
            0x73 => (0x1F, false), // s
            0x74 => (0x14, false), // t
            0x75 => (0x16, false), // u
            0x76 => (0x2F, false), // v
            0x77 => (0x11, false), // w
            0x78 => (0x2D, false), // x
            0x79 => (0x15, false), // y
            0x7A => (0x2C, false), // z

            // Uppercase letters (same scan codes, shift handled by guest OS)
            0x41 => (0x1E, false), // A
            0x42 => (0x30, false), // B
            0x43 => (0x2E, false), // C
            0x44 => (0x20, false), // D
            0x45 => (0x12, false), // E
            0x46 => (0x21, false), // F
            0x47 => (0x22, false), // G
            0x48 => (0x23, false), // H
            0x49 => (0x17, false), // I
            0x4A => (0x24, false), // J
            0x4B => (0x25, false), // K
            0x4C => (0x26, false), // L
            0x4D => (0x32, false), // M
            0x4E => (0x31, false), // N
            0x4F => (0x18, false), // O
            0x50 => (0x19, false), // P
            0x51 => (0x10, false), // Q
            0x52 => (0x13, false), // R
            0x53 => (0x1F, false), // S
            0x54 => (0x14, false), // T
            0x55 => (0x16, false), // U
            0x56 => (0x2F, false), // V
            0x57 => (0x11, false), // W
            0x58 => (0x2D, false), // X
            0x59 => (0x15, false), // Y
            0x5A => (0x2C, false), // Z

            // Digits
            0x30 => (0x0B, false), // 0
            0x31 => (0x02, false), // 1
            0x32 => (0x03, false), // 2
            0x33 => (0x04, false), // 3
            0x34 => (0x05, false), // 4
            0x35 => (0x06, false), // 5
            0x36 => (0x07, false), // 6
            0x37 => (0x08, false), // 7
            0x38 => (0x09, false), // 8
            0x39 => (0x0A, false), // 9

            // Punctuation / symbols
            0x20 => (0x39, false), // Space
            0x21 => (0x02, false), // Exclamation (!)
            0x60 => (0x29, false), // Grave (`)
            0x2D => (0x0C, false), // Minus (-)
            0x3D => (0x0D, false), // Equals (=)
            0x5B => (0x1A, false), // Left Bracket ([)
            0x5D => (0x1B, false), // Right Bracket (])
            0x5C => (0x2B, false), // Backslash (\)
            0x3B => (0x27, false), // Semicolon (;)
            0x27 => (0x28, false), // Quote (')
            0x2C => (0x33, false), // Comma (,)
            0x2E => (0x34, false), // Period (.)
            0x2F => (0x35, false), // Slash (/)

            // Control keys
            0xFF1B => (0x01, false), // Escape
            0xFF09 => (0x0F, false), // Tab
            0xFF0D => (0x1C, false), // Return/Enter
            0xFFE1 => (0x2A, false), // Shift_L
            0xFFE2 => (0x36, false), // Shift_R
            0xFFE3 => (0x1D, false), // Control_L
            0xFFE4 => (0x1D, true),  // Control_R
            0xFFE9 => (0x38, false), // Alt_L
            0xFFEA => (0x38, true),  // Alt_R (AltGr)
            0xFFE5 => (0x3A, false), // Caps_Lock
            0xFF08 => (0x0E, false), // BackSpace

            // Dedicated navigation cluster (E0-prefixed in Set 1)
            0xFFFF => (0x53, true), // Delete
            0xFF67 => (0x53, true), // Delete (alternate)
            0xFF50 => (0x47, true), // Home
            0xFF57 => (0x4F, true), // End
            0xFF55 => (0x49, true), // Prior/PageUp
            0xFF56 => (0x51, true), // Next/PageDown
            0xFF52 => (0x48, true), // Up
            0xFF54 => (0x50, true), // Down
            0xFF51 => (0x4B, true), // Left
            0xFF53 => (0x4D, true), // Right
            0xFF63 => (0x52, true), // Insert

            // F keys
            0xFFBE => (0x3B, false), // F1
            0xFFBF => (0x3C, false), // F2
            0xFFC0 => (0x3D, false), // F3
            0xFFC1 => (0x3E, false), // F4
            0xFFC2 => (0x3F, false), // F5
            0xFFC3 => (0x40, false), // F6
            0xFFC4 => (0x41, false), // F7
            0xFFC5 => (0x42, false), // F8
            0xFFC6 => (0x43, false), // F9
            0xFFC7 => (0x44, false), // F10
            0xFFC8 => (0x45, false), // F11
            0xFFC9 => (0x46, false), // F12

            // Numpad (Set 1 codes, share with navigation keys)
            0xFF90 => (0x52, false), // KP_0
            0xFF91 => (0x4F, false), // KP_1
            0xFF92 => (0x50, false), // KP_2
            0xFF93 => (0x51, false), // KP_3
            0xFF94 => (0x4B, false), // KP_4
            0xFF95 => (0x4C, false), // KP_5
            0xFF96 => (0x4D, false), // KP_6
            0xFF97 => (0x47, false), // KP_7
            0xFF98 => (0x48, false), // KP_8
            0xFF99 => (0x49, false), // KP_9
            0xFFAE => (0x53, false), // KP_Decimal
            0xFF8D => (0x1C, true),  // KP_Enter (E0-prefixed)
            0xFF6B => (0x4E, false), // KP_Add
            0xFF6D => (0x4A, false), // KP_Subtract
            0xFF6A => (0x37, false), // KP_Multiply
            0xFF6F => (0x35, true),  // KP_Divide (E0-prefixed)
            _ => (0x00, false),      // Unknown key
        };

        if make_code == 0 {
            return Vec::new();
        }

        if send_set1 {
            if pressed {
                if e0 {
                    vec![0xE0, make_code]
                } else {
                    vec![make_code]
                }
            } else {
                let break_code = make_code | 0x80;
                if e0 {
                    vec![0xE0, break_code]
                } else {
                    vec![break_code]
                }
            }
        } else {
            let Some(make_code) = Self::set2_make_code(make_code, e0) else {
                return Vec::new();
            };

            match (pressed, e0) {
                (true, true) => vec![0xE0, make_code],
                (true, false) => vec![make_code],
                (false, true) => vec![0xE0, 0xF0, make_code],
                (false, false) => vec![0xF0, make_code],
            }
        }
    }

    /// Translate the set-1 make codes above to their set-2 equivalent.
    ///
    /// Input normally passes through the i8042 translation engine.  Firmware
    /// can disable that engine, however, in which case the keyboard has to
    /// provide raw set-2 bytes instead.
    fn set2_make_code(set1: u8, e0: bool) -> Option<u8> {
        let set2 = if e0 {
            match set1 {
                0x1C => 0x5A,
                0x1D => 0x14,
                0x35 => 0x4A,
                0x38 => 0x11,
                0x47 => 0x6C,
                0x48 => 0x75,
                0x49 => 0x7D,
                0x4B => 0x6B,
                0x4D => 0x74,
                0x4F => 0x69,
                0x50 => 0x72,
                0x51 => 0x7A,
                0x52 => 0x70,
                0x53 => 0x71,
                _ => return None,
            }
        } else {
            match set1 {
                0x01 => 0x76,
                0x02 => 0x16,
                0x03 => 0x1E,
                0x04 => 0x26,
                0x05 => 0x25,
                0x06 => 0x2E,
                0x07 => 0x36,
                0x08 => 0x3D,
                0x09 => 0x3E,
                0x0A => 0x46,
                0x0B => 0x45,
                0x0C => 0x4E,
                0x0D => 0x55,
                0x0E => 0x66,
                0x0F => 0x0D,
                0x10 => 0x15,
                0x11 => 0x1D,
                0x12 => 0x24,
                0x13 => 0x2D,
                0x14 => 0x2C,
                0x15 => 0x35,
                0x16 => 0x3C,
                0x17 => 0x43,
                0x18 => 0x44,
                0x19 => 0x4D,
                0x1A => 0x54,
                0x1B => 0x5B,
                0x1C => 0x5A,
                0x1D => 0x14,
                0x1E => 0x1C,
                0x1F => 0x1B,
                0x20 => 0x23,
                0x21 => 0x2B,
                0x22 => 0x34,
                0x23 => 0x33,
                0x24 => 0x3B,
                0x25 => 0x42,
                0x26 => 0x4B,
                0x27 => 0x4C,
                0x28 => 0x52,
                0x29 => 0x0E,
                0x2A => 0x12,
                0x2B => 0x5D,
                0x2C => 0x1A,
                0x2D => 0x22,
                0x2E => 0x21,
                0x2F => 0x2A,
                0x30 => 0x32,
                0x31 => 0x31,
                0x32 => 0x3A,
                0x33 => 0x41,
                0x34 => 0x49,
                0x35 => 0x4A,
                0x36 => 0x59,
                0x37 => 0x7C,
                0x38 => 0x11,
                0x39 => 0x29,
                0x3A => 0x58,
                0x3B => 0x05,
                0x3C => 0x06,
                0x3D => 0x04,
                0x3E => 0x0C,
                0x3F => 0x03,
                0x40 => 0x0B,
                0x41 => 0x83,
                0x42 => 0x0A,
                0x43 => 0x01,
                0x44 => 0x09,
                0x45 => 0x78,
                0x46 => 0x07,
                0x47 => 0x6C,
                0x48 => 0x75,
                0x49 => 0x7D,
                0x4A => 0x7B,
                0x4B => 0x6B,
                0x4C => 0x73,
                0x4D => 0x74,
                0x4E => 0x79,
                0x4F => 0x69,
                0x50 => 0x72,
                0x51 => 0x7A,
                0x52 => 0x70,
                0x53 => 0x71,
                _ => return None,
            }
        };

        Some(set2)
    }
}

/// A i8042 PS/2 controller that emulates keyboard and mouse input.
///
/// This device handles PS/2 protocol commands, keyboard scan code generation,
/// and mouse packet generation. Input events are received via `input_channel`
/// and forwarded to the guest through the PS/2 data port.
struct PendingMouseEvent {
    buttons: u8,
    buttons_dirty: bool,
    dx: i32,
    dy: i32,
}

pub struct I8042Device {
    reset_evt: EventFd,
    vcpus_kill_signalled: Arc<AtomicBool>,
    vcpus_pause_signalled: Arc<AtomicBool>,

    // PS/2 status register
    status: u8,

    // Output data buffer (data to send to guest on read of 0x60)
    output_buffer: VecDeque<u8>,
    // Whether each output byte originated from the auxiliary (mouse) port.
    output_buffer_aux: VecDeque<bool>,

    // Port A register (keyboard/mouse presence and status)
    // Bit 0: keyboard output buffer full / keyboard present
    // Bit 4: mouse output buffer full / mouse present
    port_a: u8,

    // Port B register value
    port_b: u8,

    // Controller configuration register (CTR)
    // Bit 1: KBD INT (keyboard interrupt enable)
    // Bit 2: SYS FLAG (system flag)
    // Bit 3: KBD DIS (keyboard disable)
    // Bit 4: AUX DIS (auxiliary disable)
    // Bit 5: AUX INT (auxiliary interrupt enable)
    // Bit 6: XLATE (scancode translation: 1 = Set 2→Set 1, 0 = raw Set 2)
    // Bit 7: CPU INT (CPU interrupt enable)
    ctr: u8,

    // Control register (bit 6 = keylock, must be 0 for atkbd to load)
    control_reg: u8,

    // Keyboard state
    keyboard_enabled: bool,
    keyboard_scancode_set: u8,
    // Tracks currently pressed keys + last event time to filter VNC autorepeat and recover from missed up events
    pressed_keys: HashMap<u32, (bool, Instant)>,
    // Mouse (secondary) state
    mouse_enabled: bool,
    mouse_reporting_enabled: bool,
    mouse_resolution: u8,
    mouse_sample_rate: u8,
    mouse_scaling_21: bool,
    mouse_buttons: u8,
    pending_mouse_events: VecDeque<PendingMouseEvent>,

    // Pending command that needs data from input buffer
    pending_command: Option<u8>,
    // Pending mouse command that needs a parameter.
    pending_mouse_command: Option<u8>,
    // Pending keyboard command that needs a parameter.
    pending_keyboard_command: Option<u8>,

    // Channel to receive input events from VNC or other sources
    input_channel: Option<Arc<Mutex<VecDeque<InputEvent>>>>,

    // IRQs for the primary keyboard and auxiliary mouse ports.
    keyboard_irq: Option<Arc<dyn InterruptSourceGroup>>,
    mouse_irq: Option<Arc<dyn InterruptSourceGroup>>,
}

impl I8042Device {
    /// Constructs a i8042 device.
    ///
    /// `reset_evt` - EventFd to signal on guest reset request.
    /// `vcpus_kill_signalled` - Flag indicating vCPUs have been signaled to stop.
    /// `vcpus_pause_signalled` - Flag indicating vCPUs have been signaled to pause.
    /// `input_channel` - Optional shared queue for receiving input events from VNC.
    /// `irq` - Optional IRQ source for keyboard interrupt.
    pub fn new(
        reset_evt: EventFd,
        vcpus_kill_signalled: Arc<AtomicBool>,
        vcpus_pause_signalled: Arc<AtomicBool>,
        input_channel: Option<Arc<Mutex<VecDeque<InputEvent>>>>,
        keyboard_irq: Option<Arc<dyn InterruptSourceGroup>>,
        mouse_irq: Option<Arc<dyn InterruptSourceGroup>>,
    ) -> I8042Device {
        I8042Device {
            reset_evt,
            vcpus_kill_signalled,
            vcpus_pause_signalled,
            status: STATUS_SYSTEM_FLAG | STATUS_KEYLOCK,
            output_buffer: VecDeque::new(),
            output_buffer_aux: VecDeque::new(),
            port_a: 0x00,
            port_b: 0x23,
            ctr: CTR_KBD_INT | CTR_AUX_INT | CTR_SYSTEM_FLAG | CTR_XLATE,
            control_reg: 0x00,
            keyboard_enabled: true,
            keyboard_scancode_set: 2,
            pressed_keys: HashMap::new(),
            mouse_enabled: true,
            mouse_reporting_enabled: false,
            mouse_resolution: 2,
            mouse_sample_rate: 100,
            mouse_scaling_21: false,
            mouse_buttons: 0,
            pending_mouse_events: VecDeque::new(),
            pending_command: None,
            pending_mouse_command: None,
            pending_keyboard_command: None,
            input_channel,
            keyboard_irq,
            mouse_irq,
        }
    }

    /// Get a clone of the IRQ source group, if present.
    pub fn keyboard_irq(&self) -> Option<Arc<dyn InterruptSourceGroup>> {
        self.keyboard_irq.as_ref().map(Arc::clone)
    }

    /// Get a clone of the auxiliary mouse IRQ source group, if present.
    pub fn mouse_irq(&self) -> Option<Arc<dyn InterruptSourceGroup>> {
        self.mouse_irq.as_ref().map(Arc::clone)
    }

    /// Check whether the output buffer is nearly full.
    /// Returns true when the buffer has fewer than 4 free slots,
    /// giving the caller a chance to back off before events are dropped.
    pub fn output_buffer_near_full(&self) -> bool {
        self.output_buffer.len() >= MAX_DATA_BUFFER - 4
    }

    /// Update output-buffer status bits for the byte currently visible to the guest.
    fn update_output_buffer_status(&mut self) {
        if self.output_buffer.is_empty() {
            self.status &= !(STATUS_OUTPUT_BUFFER_FULL | STATUS_AUX_OUTPUT_BUFFER_FULL);
        } else {
            self.status |= STATUS_OUTPUT_BUFFER_FULL;
            if self.output_buffer_aux.front().copied().unwrap_or(false) {
                self.status |= STATUS_AUX_OUTPUT_BUFFER_FULL;
            } else {
                self.status &= !STATUS_AUX_OUTPUT_BUFFER_FULL;
            }
        }
    }

    fn trigger_output_irq(&self) {
        let (irq_enabled, irq) = if self.output_buffer_aux.front().copied().unwrap_or(false) {
            (
                self.ctr & (CTR_AUX_INT | CTR_AUX_DISABLE) == CTR_AUX_INT,
                &self.mouse_irq,
            )
        } else {
            (
                self.ctr & (CTR_KBD_INT | CTR_KBD_DISABLE) == CTR_KBD_INT,
                &self.keyboard_irq,
            )
        };

        if irq_enabled && let Some(irq) = irq {
            if let Err(error) = irq.trigger(0) {
                warn!("i8042: failed to trigger IRQ: {error}");
            }
        }
    }

    /// Push a byte to the output buffer and trigger the matching IRQ if data was added.
    fn push_output(&mut self, byte: u8) {
        self.push_output_with_source(byte, false);
    }

    /// Push a byte from the auxiliary mouse port to the output buffer.
    fn push_aux_output(&mut self, byte: u8) {
        self.push_output_with_source(byte, true);
    }

    fn push_output_with_source(&mut self, byte: u8, aux: bool) {
        if self.output_buffer.len() < MAX_DATA_BUFFER {
            let was_empty = self.output_buffer.is_empty();
            self.output_buffer.push_back(byte);
            self.output_buffer_aux.push_back(aux);
            if was_empty {
                self.update_output_buffer_status();
                self.trigger_output_irq();
            }
        } else {
            debug!("i8042: output buffer full, dropping byte 0x{byte:02x}");
        }
    }

    /// Process a keyboard input event and generate scan codes.
    /// Returns true if scan codes were added to the output buffer.
    pub fn process_keyboard_event(&mut self, key: u32, pressed: bool) -> bool {
        if !self.keyboard_enabled {
            return false;
        }

        // Strict state machine: reject duplicate down/up events.
        // TigerVNC resends "down" events on autorepeat; we only want one down + one up.
        if let Some(currently_down) = self.pressed_keys.get(&key).map(|&(down, _)| down) {
            if currently_down == pressed {
                info!(
                    "i8042: filtering duplicate keysym=0x{:x} pressed={}",
                    key, pressed
                );
                return false;
            }
        }

        // VNC input is injected after the controller's translation stage.  It
        // therefore has to match the format currently visible at port 0x60.
        let send_set1 = self.ctr & CTR_XLATE != 0 || self.keyboard_scancode_set == 1;
        let scan_codes = KeyboardMap::scan_codes(key, pressed, send_set1);
        if !scan_codes.is_empty() {
            self.pressed_keys.insert(key, (pressed, Instant::now()));
            info!(
                "i8042: keysym=0x{:x} pressed={} -> scan_codes=[{:02x?}]",
                key, pressed, scan_codes
            );
            self.push_output_bytes_no_irq(&scan_codes);
            true
        } else {
            info!("i8042: unknown keysym=0x{:x} pressed={}", key, pressed);
            false
        }
    }

    /// Push bytes to output buffer without triggering IRQ.
    fn push_output_bytes_no_irq(&mut self, bytes: &[u8]) {
        self.push_output_bytes_no_irq_with_source(bytes, false);
    }

    /// Push auxiliary bytes to the output buffer without triggering an IRQ.
    fn push_aux_output_bytes_no_irq(&mut self, bytes: &[u8]) {
        self.push_output_bytes_no_irq_with_source(bytes, true);
    }

    fn push_output_bytes_no_irq_with_source(&mut self, bytes: &[u8], aux: bool) {
        for &b in bytes {
            if self.output_buffer.len() < MAX_DATA_BUFFER {
                self.output_buffer.push_back(b);
                self.output_buffer_aux.push_back(aux);
            }
        }
        self.update_output_buffer_status();
    }

    /// Process a mouse input event and generate one or more 3-byte mouse packets.
    ///
    /// Standard PS/2 mouse packet format:
    /// Byte 0: bits 0-2 = buttons (L, R, M), bit 3 = 1 (always), bit 4 = X sign,
    ///         bit 5 = Y sign, bit 6 = X overflow, bit 7 = Y overflow
    /// Byte 1: X movement (signed, 8-bit with overflow flag)
    /// Byte 2: Y movement (signed, 8-bit with overflow flag)
    pub fn process_mouse_event(&mut self, buttons: u8, dx: i16, dy: i16) -> bool {
        if !self.mouse_enabled || !self.mouse_reporting_enabled {
            return false;
        }
        let buttons = buttons & 0x07;
        let buttons_changed = self.mouse_buttons != buttons;
        self.mouse_buttons = buttons;
        if dx != 0 || dy != 0 || buttons_changed {
            if let Some(event) = self.pending_mouse_events.back_mut()
                && event.buttons == buttons
            {
                event.dx = event.dx.saturating_add(i32::from(dx));
                event.dy = event.dy.saturating_add(i32::from(dy));
                event.buttons_dirty |= buttons_changed;
            } else {
                self.pending_mouse_events.push_back(PendingMouseEvent {
                    buttons,
                    buttons_dirty: buttons_changed,
                    dx: i32::from(dx),
                    dy: i32::from(dy),
                });
            }
        }
        self.flush_mouse_packets()
    }

    fn flush_mouse_packets(&mut self) -> bool {
        if !self.mouse_enabled || !self.mouse_reporting_enabled {
            return false;
        }
        let mut added = false;
        while self.output_buffer.len() <= MAX_DATA_BUFFER - 3 {
            let Some(event) = self.pending_mouse_events.front_mut() else {
                break;
            };
            let x_byte = event.dx.clamp(i8::MIN.into(), i8::MAX.into()) as i8;
            let y_byte = event.dy.clamp(i8::MIN.into(), i8::MAX.into()) as i8;

            let mut byte0 = 0x08 | event.buttons;
            if x_byte.is_negative() {
                byte0 |= 0x10;
            }
            if y_byte.is_negative() {
                byte0 |= 0x20;
            }

            self.push_aux_output_bytes_no_irq(&[byte0, x_byte as u8, y_byte as u8]);
            let event = self.pending_mouse_events.front_mut().unwrap();
            event.dx -= i32::from(x_byte);
            event.dy -= i32::from(y_byte);
            event.buttons_dirty = false;
            if event.dx == 0 && event.dy == 0 && !event.buttons_dirty {
                self.pending_mouse_events.pop_front();
            }
            added = true;
        }
        added
    }

    fn clear_pending_mouse_movement(&mut self) {
        self.pending_mouse_events.clear();
    }

    /// Drain pending input events from the input channel.
    fn drain_input_channel(&mut self) {
        let events = if let Some(channel) = &self.input_channel {
            let mut queue = channel.lock().unwrap();
            queue.drain(..).collect::<Vec<_>>()
        } else {
            Vec::new()
        };

        for event in events {
            match event {
                InputEvent::Keyboard { key, pressed } => {
                    self.process_keyboard_event(key, pressed);
                }
                InputEvent::Mouse { buttons, dx, dy } => {
                    self.process_mouse_event(buttons, dx, dy);
                }
            }
        }
    }

    /// Handle a command written to the command port (0x64).
    fn handle_command(&mut self, cmd: u8) {
        debug!("i8042: cmd write 0x{cmd:02x}");
        match cmd {
            CMD_RESET_CONTROLLER => {
                info!("i8042 reset signalled");
                if let Err(e) = self.reset_evt.write(1) {
                    error!("Error triggering i8042 reset event: {e}");
                }
                while !self.vcpus_kill_signalled.load(Ordering::SeqCst)
                    && !self.vcpus_pause_signalled.load(Ordering::SeqCst)
                {
                    thread::sleep(std::time::Duration::from_millis(1));
                }
            }
            CMD_TEST_CONTROLLER => {
                self.push_output(0x55);
            }
            CMD_SELF_TEST => {
                self.push_output(0x55);
            }
            CMD_DISABLE_KEYBOARD => {
                debug!("i8042: keyboard disabled");
                self.keyboard_enabled = false;
                self.port_a &= !0x01;
                self.ctr |= CTR_KBD_DISABLE;
            }
            CMD_ENABLE_KEYBOARD => {
                debug!("i8042: keyboard enabled");
                self.keyboard_enabled = true;
                self.port_a |= 0x01;
                self.ctr &= !CTR_KBD_DISABLE;
            }
            CMD_DISABLE_SECONDARY => {
                debug!("i8042: mouse/secondary disabled");
                self.mouse_enabled = false;
                self.clear_pending_mouse_movement();
                self.port_a &= !0x10;
                self.ctr |= CTR_AUX_DISABLE;
            }
            CMD_ENABLE_SECONDARY => {
                debug!("i8042: mouse/secondary enabled");
                self.mouse_enabled = true;
                self.port_a |= 0x10;
                self.ctr &= !CTR_AUX_DISABLE;
            }
            CMD_TEST_KEYBOARD => {
                self.push_output(0x00);
            }
            CMD_READ_PORT_A => {
                // Linux uses 0x20 to read the CTR during i8042 initialization.
                // It checks the XLATE bit (0x40) to determine whether the controller
                // translates Set 2 → Set 1 scancodes. If XLATE is clear, atkbd uses
                // the Set 2 keycode table, causing Set 1 scancodes to be misinterpreted.
                self.push_output(self.ctr);
            }
            CMD_WRITE_PORT_A => {
                self.pending_command = Some(cmd);
            }
            CMD_TEST_SECONDARY => {
                // The auxiliary-port self test succeeds, allowing the guest to
                // discover the PS/2 mouse before issuing device commands.
                self.push_output(0x00);
            }
            CMD_READ_PORT_B => {
                self.push_output(self.port_b);
            }
            CMD_READ_KBD_INPUT => {
                self.push_output(self.control_reg);
            }
            CMD_WRITE_KBD_OUTPUT => {
                self.pending_command = Some(cmd);
            }
            CMD_WRITE_SECONDARY_OUTPUT => {
                self.pending_command = Some(cmd);
            }
            CMD_WRITE_TO_AUX => {
                self.pending_command = Some(cmd);
            }
            CMD_WRITE_KBD_CTRL => {
                self.pending_command = Some(cmd);
            }
            _ => {
                warn!("i8042: unknown command 0x{cmd:02x}");
            }
        }
    }

    /// Handle data written to the data port (0x60).
    fn handle_data_write(&mut self, data: u8) {
        debug!("i8042: data write 0x{data:02x}");
        if let Some(cmd) = self.pending_command {
            match cmd {
                CMD_WRITE_KBD_OUTPUT => {
                    self.push_output(data);
                }
                CMD_WRITE_SECONDARY_OUTPUT => {
                    self.push_aux_output(data);
                }
                CMD_WRITE_TO_AUX => {
                    self.handle_mouse_command(data);
                }
                CMD_WRITE_PORT_A => {
                    self.ctr = data;
                }
                CMD_WRITE_KBD_CTRL => {
                    self.control_reg = data;
                }
                _ => {}
            }
            self.pending_command = None;
        } else {
            self.handle_keyboard_command(data);
        }
    }

    fn handle_keyboard_command(&mut self, command: u8) {
        if let Some(pending_command) = self.pending_keyboard_command.take() {
            self.push_output(0xFA);
            if pending_command == 0xF0 {
                if command == 0 {
                    self.push_output(self.keyboard_scancode_set);
                } else if (1..=3).contains(&command) {
                    self.keyboard_scancode_set = command;
                }
            }
            return;
        }

        match command {
            // Set LEDs, typematic rate, and scan-code set each take one parameter.
            0xED | 0xF0 | 0xF3 => {
                self.pending_keyboard_command = Some(command);
                self.push_output(0xFA);
            }
            // Reset returns ACK followed by the keyboard self-test result.
            0xFF => {
                self.keyboard_enabled = true;
                self.push_output(0xFA);
                self.push_output(0xAA);
            }
            // Identify an AT-compatible keyboard.
            0xF2 => {
                self.push_output(0xFA);
                self.push_output(0xAB);
                self.push_output(0x83);
            }
            0xF4 => {
                self.keyboard_enabled = true;
                self.push_output(0xFA);
            }
            0xF5 | 0xF6 => {
                self.keyboard_enabled = false;
                self.push_output(0xFA);
            }
            // Echo is used by some firmware probes.
            0xEE => self.push_output(0xEE),
            _ => {
                debug!("i8042: unimplemented keyboard command 0x{command:02x}");
                self.push_output(0xFA);
            }
        }
    }

    fn handle_mouse_command(&mut self, command: u8) {
        if let Some(pending_command) = self.pending_mouse_command.take() {
            self.push_aux_output(0xFA);
            match pending_command {
                0xE8 => self.mouse_resolution = command & 0x03,
                0xF3 => self.mouse_sample_rate = command,
                _ => unreachable!("only mouse commands with a parameter are pending"),
            }
            return;
        }

        match command {
            // Set resolution and sample rate each take one parameter.
            0xE8 | 0xF3 => {
                self.pending_mouse_command = Some(command);
                self.push_aux_output(0xFA);
            }
            // Set scaling. The standard mouse uses 1:1 scaling by default;
            // no packet conversion is needed for the relative VNC events.
            0xE6 => {
                self.mouse_scaling_21 = false;
                self.push_aux_output(0xFA);
            }
            0xE7 => {
                self.mouse_scaling_21 = true;
                self.push_aux_output(0xFA);
            }
            // Get status: ACK, status flags, resolution, and sample rate.
            // psmouse uses this during its standard-device probe.
            0xE9 => {
                let status = (u8::from(self.mouse_reporting_enabled) << 5)
                    | (u8::from(self.mouse_scaling_21) << 4);
                self.push_aux_output(0xFA);
                self.push_aux_output(status);
                self.push_aux_output(self.mouse_resolution);
                self.push_aux_output(self.mouse_sample_rate);
            }
            // Select stream mode. This is the normal operating mode.
            0xEA => self.push_aux_output(0xFA),
            // Poll requests one packet even while reporting is disabled.
            0xEB => {
                self.push_aux_output(0xFA);
                self.push_aux_output(0x08);
                self.push_aux_output(0x00);
                self.push_aux_output(0x00);
            }
            // Reset wrap mode and echo are used by generic PS/2 probes.
            0xEC => self.push_aux_output(0xFA),
            0xEE => self.push_aux_output(0xEE),
            0xF0 => self.push_aux_output(0xFA),
            // Set defaults and disable data reporting.
            0xF6 => {
                self.mouse_reporting_enabled = false;
                self.mouse_resolution = 2;
                self.mouse_sample_rate = 100;
                self.mouse_scaling_21 = false;
                self.clear_pending_mouse_movement();
                self.push_aux_output(0xFA);
            }
            // Enable/disable data reporting.
            0xF4 => {
                self.mouse_reporting_enabled = true;
                self.push_aux_output(0xFA);
            }
            0xF5 => {
                self.mouse_reporting_enabled = false;
                self.clear_pending_mouse_movement();
                self.push_aux_output(0xFA);
            }
            // Get device ID for a standard three-button PS/2 mouse.
            0xF2 => {
                self.push_aux_output(0xFA);
                self.push_aux_output(0x00);
            }
            // Reset returns ACK, self-test pass, and the standard device ID.
            0xFF => {
                self.mouse_reporting_enabled = false;
                self.mouse_resolution = 2;
                self.mouse_sample_rate = 100;
                self.mouse_scaling_21 = false;
                self.clear_pending_mouse_movement();
                self.push_aux_output(0xFA);
                self.push_aux_output(0xAA);
                self.push_aux_output(0x00);
            }
            _ => {
                debug!("i8042: unimplemented mouse command 0x{command:02x}");
                self.push_aux_output(0xFA);
            }
        }
    }
}

// i8042 device is located at I/O port 0x60. We implement:
// - Port 0x60 (offset 0): PS/2 data port
// - Port 0x61 (offset 1): Port B register
// - Port 0x62 (offset 2): Timer latch
// - Port 0x63 (offset 3): Mode select
// - Port 0x64 (offset 4): Command/status register
impl BusDevice for I8042Device {
    fn read(&mut self, _base: u64, offset: u64, data: &mut [u8]) {
        if data.len() != 1 {
            warn!("Invalid read size on i8042 device: {}", data.len());
            return;
        }

        match offset {
            OFFSET_DATA => {
                self.drain_input_channel();
                if let Some(byte) = self.output_buffer.pop_front() {
                    self.output_buffer_aux.pop_front();
                    data[0] = byte;
                    debug!("i8042: guest read 0x{byte:02x}");
                    self.flush_mouse_packets();
                    self.update_output_buffer_status();
                    if !self.output_buffer.is_empty() {
                        // More data is available from the source at the front of the queue.
                        self.trigger_output_irq();
                    }
                } else {
                    data[0] = 0xFF;
                }
            }
            OFFSET_PORT_B => {
                data[0] = self.port_b;
            }
            OFFSET_TIMER_LATCH => {
                data[0] = 0x00;
            }
            OFFSET_MODE_SELECT => {
                data[0] = 0x00;
            }
            OFFSET_COMMAND => {
                data[0] = self.status | STATUS_SYSTEM_FLAG | STATUS_KEYLOCK;
            }
            _ => {
                debug!("i8042: read from unknown offset {offset}");
                data[0] = 0x00;
            }
        }
    }

    fn write(&mut self, _base: u64, offset: u64, data: &[u8]) -> Option<Arc<Barrier>> {
        if data.len() != 1 {
            warn!("Invalid write size on i8042 device: {}", data.len());
            return None;
        }

        match offset {
            OFFSET_DATA => {
                self.handle_data_write(data[0]);
            }
            OFFSET_PORT_B => {
                self.port_b = data[0];
            }
            OFFSET_TIMER_LATCH => {
                debug!("i8042: write to timer latch: 0x{:02x}", data[0]);
            }
            OFFSET_MODE_SELECT => {
                debug!("i8042: write to mode select: 0x{:02x}", data[0]);
            }
            OFFSET_COMMAND => {
                self.handle_command(data[0]);
            }
            _ => {
                debug!(
                    "i8042: write to unknown offset {}: 0x{:02x}",
                    offset, data[0]
                );
            }
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_device() -> I8042Device {
        I8042Device {
            reset_evt: EventFd::new(0).unwrap(),
            vcpus_kill_signalled: Arc::new(AtomicBool::new(false)),
            vcpus_pause_signalled: Arc::new(AtomicBool::new(false)),
            status: STATUS_SYSTEM_FLAG | STATUS_KEYLOCK,
            output_buffer: VecDeque::new(),
            output_buffer_aux: VecDeque::new(),
            port_a: 0x00,
            port_b: 0x20,
            ctr: CTR_KBD_INT | CTR_AUX_INT | CTR_SYSTEM_FLAG | CTR_XLATE,
            control_reg: 0x00,
            keyboard_enabled: true,
            keyboard_scancode_set: 2,
            pressed_keys: HashMap::new(),
            mouse_enabled: true,
            mouse_reporting_enabled: false,
            mouse_resolution: 2,
            mouse_sample_rate: 100,
            mouse_scaling_21: false,
            mouse_buttons: 0,
            pending_mouse_events: VecDeque::new(),
            pending_command: None,
            pending_mouse_command: None,
            pending_keyboard_command: None,
            input_channel: None,
            keyboard_irq: None,
            mouse_irq: None,
        }
    }

    #[test]
    fn test_port_b_read() {
        let mut dev = make_device();
        let mut data = [0u8];
        dev.read(0, OFFSET_PORT_B, &mut data);
        assert_eq!(data[0], 0x20);
    }

    #[test]
    fn test_port_b_write() {
        let mut dev = make_device();
        dev.write(0, OFFSET_PORT_B, &[0x40]);
        let mut data = [0u8];
        dev.read(0, OFFSET_PORT_B, &mut data);
        assert_eq!(data[0], 0x40);
    }

    #[test]
    fn test_status_read() {
        let mut dev = make_device();
        let mut data = [0u8];
        dev.read(0, OFFSET_COMMAND, &mut data);
        assert!(data[0] & STATUS_SYSTEM_FLAG != 0);
    }

    #[test]
    fn test_keyboard_enable_disable() {
        let mut dev = make_device();

        dev.write(0, OFFSET_COMMAND, &[CMD_DISABLE_KEYBOARD]);
        assert!(!dev.keyboard_enabled);
        assert_eq!(dev.port_a & 0x01, 0);
        assert_ne!(dev.ctr & CTR_KBD_DISABLE, 0);

        dev.write(0, OFFSET_COMMAND, &[CMD_ENABLE_KEYBOARD]);
        assert!(dev.keyboard_enabled);
        assert_eq!(dev.port_a & 0x01, 0x01);
        assert_eq!(dev.ctr & CTR_KBD_DISABLE, 0);
    }

    #[test]
    fn test_keyboard_enable_disable_data_port() {
        // Test 0xF5 (DISABLE) and 0xF4 (ENABLE) written directly to data port.
        // These are the commands the Linux atkbd driver sends during init:
        // atkbd_deactivate() sends 0xF5, atkbd_activate() sends 0xF4.
        let mut dev = make_device();

        // 0xF5 = DISABLE typing (atkbd_deactivate)
        dev.write(0, OFFSET_DATA, &[0xF5]);
        assert!(!dev.keyboard_enabled);

        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0xFA); // keyboard ACK

        // 0xF4 = ENABLE typing (atkbd_activate)
        dev.write(0, OFFSET_DATA, &[0xF4]);
        assert!(dev.keyboard_enabled);

        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0xFA); // keyboard ACK
    }

    #[test]
    fn test_mouse_enable_disable() {
        let mut dev = make_device();

        dev.write(0, OFFSET_COMMAND, &[CMD_DISABLE_SECONDARY]);
        assert!(!dev.mouse_enabled);
        assert_eq!(dev.port_a & 0x10, 0);

        dev.write(0, OFFSET_COMMAND, &[CMD_ENABLE_SECONDARY]);
        assert!(dev.mouse_enabled);
        assert_eq!(dev.port_a & 0x10, 0x10);
    }

    #[test]
    fn test_mouse_port_self_test() {
        let mut dev = make_device();

        dev.write(0, OFFSET_COMMAND, &[CMD_TEST_SECONDARY]);

        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0x00);
    }

    #[test]
    fn test_keyboard_port_self_test() {
        let mut dev = make_device();

        dev.write(0, OFFSET_COMMAND, &[CMD_TEST_KEYBOARD]);

        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0x00);
    }

    #[test]
    fn test_keyboard_reset() {
        let mut dev = make_device();

        dev.write(0, OFFSET_DATA, &[0xFF]);

        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0xFA);
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0xAA);
    }

    #[test]
    fn test_keyboard_scan_code_set_query() {
        let mut dev = make_device();

        dev.write(0, OFFSET_DATA, &[0xF0]);
        dev.write(0, OFFSET_DATA, &[0x00]);

        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0xFA);
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0xFA);
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0x02);
    }

    #[test]
    fn test_test_controller() {
        let mut dev = make_device();
        dev.write(0, OFFSET_COMMAND, &[CMD_TEST_CONTROLLER]);

        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0x55);
    }

    #[test]
    fn test_read_port_a() {
        let mut dev = make_device();
        dev.ctr = 0xAB;

        dev.write(0, OFFSET_COMMAND, &[CMD_READ_PORT_A]);
        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0xAB);
    }

    #[test]
    fn test_read_port_b() {
        let mut dev = make_device();
        dev.port_b = 0x42;

        dev.write(0, OFFSET_COMMAND, &[CMD_READ_PORT_B]);
        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0x42);
    }

    #[test]
    fn test_keyboard_scan_codes() {
        let mut dev = make_device();

        // X11 keysym 0x71 ('q') -> PS/2 scan code 0x10
        dev.process_keyboard_event(0x71, true);
        assert_eq!(dev.output_buffer.len(), 1);
        assert_eq!(dev.output_buffer[0], 0x10);

        dev.process_keyboard_event(0x71, false);
        assert_eq!(dev.output_buffer.len(), 2);
        assert_eq!(dev.output_buffer[1], 0x90);
    }

    #[test]
    fn test_exclamation_scan_codes() {
        let mut dev = make_device();

        // X11 keysym 0x21 ('!') uses the physical '1' key. The VNC client
        // sends Shift as a separate keyboard event.
        dev.process_keyboard_event(0x21, true);
        dev.process_keyboard_event(0x21, false);
        assert_eq!(dev.output_buffer, VecDeque::from([0x02, 0x82]));

        let mut dev = make_device();
        dev.ctr &= !CTR_XLATE;
        dev.process_keyboard_event(0x21, true);
        dev.process_keyboard_event(0x21, false);
        assert_eq!(dev.output_buffer, VecDeque::from([0x16, 0xF0, 0x16]));
    }

    #[test]
    fn test_keyboard_uses_set2_when_translation_is_disabled() {
        let mut dev = make_device();
        dev.ctr &= !CTR_XLATE;

        // q: set 2 make 0x15 and break 0xf0, 0x15.
        dev.process_keyboard_event(0x71, true);
        dev.process_keyboard_event(0x71, false);
        assert_eq!(dev.output_buffer, VecDeque::from([0x15, 0xF0, 0x15]));

        // The dedicated cursor keys retain their E0 prefix in set 2.
        dev.process_keyboard_event(0xFF52, true);
        assert_eq!(
            dev.output_buffer,
            VecDeque::from([0x15, 0xF0, 0x15, 0xE0, 0x75])
        );
    }

    #[test]
    fn test_extended_keyboard_scan_codes() {
        let mut dev = make_device();

        // X11 keysym 0xFF53 (Right) uses an E0-prefixed Set 1 scan code.
        dev.process_keyboard_event(0xFF53, true);
        dev.process_keyboard_event(0xFF53, false);

        assert_eq!(
            dev.output_buffer.make_contiguous(),
            &[0xE0, 0x4D, 0xE0, 0xCD]
        );
    }

    #[test]
    fn test_keyboard_disabled() {
        let mut dev = make_device();
        dev.keyboard_enabled = false;

        dev.process_keyboard_event(0x71, true); // X11 keysym 'q'
        assert_eq!(dev.output_buffer.len(), 0);
    }

    #[test]
    fn test_mouse_packet() {
        let mut dev = make_device();
        dev.mouse_reporting_enabled = true;

        dev.process_mouse_event(0x01, 10, -5);

        assert_eq!(dev.output_buffer.len(), 3);
        let packet = dev.output_buffer.make_contiguous();
        assert_eq!(packet[0] & 0x08, 0x08);
        assert_eq!(packet[0] & 0x01, 0x01);
        assert_eq!(packet[1], 10);
        assert_eq!(packet[2], (-5i8) as u8);
        assert!(dev.output_buffer_aux.iter().all(|&aux| aux));
        assert_ne!(dev.status & STATUS_AUX_OUTPUT_BUFFER_FULL, 0);

        let mut data = [0u8];
        for _ in 0..3 {
            dev.read(0, OFFSET_DATA, &mut data);
        }
        assert_eq!(dev.status & STATUS_AUX_OUTPUT_BUFFER_FULL, 0);
    }

    #[test]
    fn test_mouse_large_movement_is_split() {
        let mut dev = make_device();
        dev.mouse_reporting_enabled = true;

        dev.process_mouse_event(0x00, 300, 0);

        assert_eq!(dev.output_buffer.len(), 9);
        let packet = dev.output_buffer.make_contiguous();
        assert_eq!(packet[0] & 0xc0, 0);
        assert_eq!(packet[3] & 0xc0, 0);
        assert_eq!(packet[6] & 0xc0, 0);
        let movement: i16 = packet
            .chunks_exact(3)
            .map(|packet| i16::from(packet[1] as i8))
            .sum();
        assert_eq!(movement, 300);
    }

    #[test]
    fn test_mouse_disabled() {
        let mut dev = make_device();
        dev.mouse_enabled = false;
        dev.mouse_reporting_enabled = true;

        dev.process_mouse_event(0x01, 10, 5);
        assert_eq!(dev.output_buffer.len(), 0);
    }

    #[test]
    fn test_mouse_enable_data_reporting() {
        let mut dev = make_device();

        dev.write(0, OFFSET_COMMAND, &[CMD_WRITE_TO_AUX]);
        dev.write(0, OFFSET_DATA, &[0xF4]);
        assert!(dev.mouse_reporting_enabled);
        assert_eq!(dev.output_buffer, VecDeque::from([0xFA]));
        assert_eq!(dev.output_buffer_aux, VecDeque::from([true]));

        dev.output_buffer.clear();
        dev.output_buffer_aux.clear();
        dev.process_mouse_event(0, 1, 1);
        assert_eq!(dev.output_buffer.len(), 3);
    }

    #[test]
    fn test_mouse_get_status() {
        let mut dev = make_device();

        dev.write(0, OFFSET_COMMAND, &[CMD_WRITE_TO_AUX]);
        dev.write(0, OFFSET_DATA, &[0xE9]);

        assert_eq!(dev.output_buffer, VecDeque::from([0xFA, 0x00, 0x02, 100]));
        assert!(dev.output_buffer_aux.iter().all(|&aux| aux));
    }

    #[test]
    fn test_input_channel() {
        let channel = Arc::new(Mutex::new(VecDeque::new()));
        let mut dev = I8042Device {
            input_channel: Some(channel.clone()),
            ..make_device()
        };

        // X11 keysym 0x71 ('q') -> PS/2 scan code 0x10
        channel.lock().unwrap().push_back(InputEvent::Keyboard {
            key: 0x71,
            pressed: true,
        });

        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0x10);
    }

    #[test]
    fn test_data_port_empty_read() {
        let mut dev = make_device();
        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0xFF);
    }

    #[test]
    fn test_write_keyboard_output() {
        let mut dev = make_device();

        dev.write(0, OFFSET_COMMAND, &[CMD_WRITE_KBD_OUTPUT]);
        dev.write(0, OFFSET_DATA, &[0x55]);

        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0x55);
    }

    #[test]
    fn test_write_secondary_output() {
        let mut dev = make_device();

        dev.write(0, OFFSET_COMMAND, &[CMD_WRITE_SECONDARY_OUTPUT]);
        dev.write(0, OFFSET_DATA, &[0xAA]);

        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0xAA);
    }

    #[test]
    fn test_output_buffer_full() {
        let mut dev = make_device();
        for _ in 0..MAX_DATA_BUFFER {
            dev.push_output(0x42);
        }
        assert_eq!(dev.output_buffer.len(), MAX_DATA_BUFFER);

        dev.push_output(0xFF);
        assert_eq!(dev.output_buffer.len(), MAX_DATA_BUFFER);
    }

    #[test]
    fn test_mouse_movement_waits_for_buffer_space() {
        let mut dev = make_device();
        dev.mouse_reporting_enabled = true;
        for _ in 0..MAX_DATA_BUFFER {
            dev.push_output(0x42);
        }

        assert!(!dev.process_mouse_event(0, 100, 0));
        assert_eq!(dev.pending_mouse_events.front().unwrap().dx, 100);

        let mut data = [0u8];
        for _ in 0..3 {
            dev.read(0, OFFSET_DATA, &mut data);
        }
        assert!(dev.pending_mouse_events.is_empty());
        assert_eq!(dev.output_buffer.len(), MAX_DATA_BUFFER);
        assert_eq!(
            &dev.output_buffer.make_contiguous()[MAX_DATA_BUFFER - 3..],
            &[0x08, 100, 0]
        );
    }

    #[test]
    fn test_mouse_backlog_preserves_button_transitions() {
        let mut dev = make_device();
        dev.mouse_reporting_enabled = true;
        for _ in 0..MAX_DATA_BUFFER {
            dev.push_output(0x42);
        }

        dev.process_mouse_event(0, 100, 0);
        dev.process_mouse_event(1, 0, 0);
        dev.process_mouse_event(1, -100, 0);
        assert_eq!(dev.pending_mouse_events.len(), 2);

        let mut data = [0u8];
        for _ in 0..MAX_DATA_BUFFER {
            dev.read(0, OFFSET_DATA, &mut data);
        }
        let packets = dev.output_buffer.iter().copied().collect::<Vec<_>>();
        assert_eq!(packets, &[0x08, 100, 0, 0x19, (-100i8) as u8, 0]);
        for _ in 0..packets.len() {
            dev.read(0, OFFSET_DATA, &mut data);
        }
        assert!(dev.pending_mouse_events.is_empty());
    }
}
