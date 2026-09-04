// Copyright © 2026 Cyberus Technology GmbH
//
// SPDX-License-Identifier: Apache-2.0

use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, mpsc};
use std::time::{Duration, Instant};
use std::{io, thread};

use display::vnc::VncInputEvent;
use log::{debug, info};
use vfio_user_common::input_protocol::{InputEvent, write_event};

pub struct InputBridge {
    stop: Arc<AtomicBool>,
    handle: Option<thread::JoinHandle<()>>,
}

impl InputBridge {
    pub fn spawn(path: PathBuf, receiver: mpsc::Receiver<VncInputEvent>) -> io::Result<Self> {
        let stop = Arc::new(AtomicBool::new(false));
        let worker_stop = Arc::clone(&stop);
        let handle = thread::Builder::new()
            .name("simplefb-input-ipc".to_string())
            .spawn(move || run_bridge(&path, &receiver, &worker_stop))?;
        Ok(Self {
            stop,
            handle: Some(handle),
        })
    }

    pub fn stop(mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

impl Drop for InputBridge {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

fn run_bridge(path: &Path, receiver: &mpsc::Receiver<VncInputEvent>, stop: &AtomicBool) {
    let mut translator = VncInputTranslator::default();
    let mut stream = None;
    let mut next_connect = Instant::now();

    while !stop.load(Ordering::Relaxed) {
        if stream.is_none() && Instant::now() >= next_connect {
            match UnixStream::connect(path) {
                Ok(mut connected) => {
                    if write_event(&mut connected, InputEvent::ReleaseAll).is_ok() {
                        info!("host input: connected to {}", path.display());
                        stream = Some(connected);
                        translator.reset();
                    }
                }
                Err(error) => debug!(
                    "host input: {} is unavailable; retrying: {error}",
                    path.display()
                ),
            }
            next_connect = Instant::now() + Duration::from_millis(250);
        }

        match receiver.recv_timeout(Duration::from_millis(50)) {
            Ok(event) => {
                for event in translator.translate(&event) {
                    let Some(connected) = stream.as_mut() else {
                        continue;
                    };
                    if let Err(error) = write_event(connected, event) {
                        debug!("host input: connection lost: {error}");
                        stream = None;
                        next_connect = Instant::now();
                        translator.reset();
                        break;
                    }
                }
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        }
    }

    if let Some(mut connected) = stream {
        let _ = write_event(&mut connected, InputEvent::ReleaseAll);
    }
}

#[derive(Default)]
struct VncInputTranslator {
    pointer_position: Option<(u32, u32)>,
    buttons: u8,
}

impl VncInputTranslator {
    fn reset(&mut self) {
        self.pointer_position = None;
        self.buttons = 0;
    }

    fn translate(&mut self, event: &VncInputEvent) -> Vec<InputEvent> {
        match event {
            VncInputEvent::Keyboard { key, down } => {
                keysym_to_usage(*key).map_or_else(Vec::new, |usage| {
                    vec![InputEvent::Key {
                        usage,
                        pressed: *down,
                    }]
                })
            }
            VncInputEvent::MouseButton { button, down } if *button < 3 => {
                // RFB orders buttons as left, middle, right while the HID
                // descriptor exposes left, right, middle.
                let mask = [1, 4, 2][usize::from(*button)];
                if *down {
                    self.buttons |= mask;
                } else {
                    self.buttons &= !mask;
                }
                vec![mouse_event(0, 0, 0, self.buttons)]
            }
            VncInputEvent::MouseButton {
                button: 3,
                down: true,
            } => {
                vec![mouse_event(0, 0, 1, self.buttons)]
            }
            VncInputEvent::MouseButton {
                button: 4,
                down: true,
            } => {
                vec![mouse_event(0, 0, -1, self.buttons)]
            }
            VncInputEvent::MouseButton { .. } => Vec::new(),
            VncInputEvent::PointerMove { dx, dy } => {
                vec![mouse_event(*dx, *dy, 0, self.buttons)]
            }
            VncInputEvent::PointerPosition { x, y } => {
                let Some((previous_x, previous_y)) = self.pointer_position.replace((*x, *y)) else {
                    return Vec::new();
                };
                split_motion(
                    i64::from(*x) - i64::from(previous_x),
                    i64::from(*y) - i64::from(previous_y),
                    self.buttons,
                )
            }
            VncInputEvent::ReleaseAll => {
                self.reset();
                vec![InputEvent::ReleaseAll]
            }
        }
    }
}

fn mouse_event(dx: i16, dy: i16, wheel: i16, buttons: u8) -> InputEvent {
    InputEvent::Mouse {
        dx,
        dy,
        wheel,
        buttons,
    }
}

fn split_motion(mut dx: i64, mut dy: i64, buttons: u8) -> Vec<InputEvent> {
    let mut events = Vec::new();
    while dx != 0 || dy != 0 {
        let part_x = dx.clamp(i64::from(i16::MIN), i64::from(i16::MAX)) as i16;
        let part_y = dy.clamp(i64::from(i16::MIN), i64::from(i16::MAX)) as i16;
        events.push(mouse_event(part_x, part_y, 0, buttons));
        dx -= i64::from(part_x);
        dy -= i64::from(part_y);
    }
    events
}

pub fn keysym_to_usage(keysym: u32) -> Option<u8> {
    let usage = match keysym {
        0x61..=0x7a => 0x04 + (keysym as u8 - b'a'),
        0x41..=0x5a => 0x04 + (keysym as u8 - b'A'),
        0x31..=0x39 => 0x1e + (keysym as u8 - b'1'),
        0x30 => 0x27,
        0xff0d | 0x0d => 0x28,
        0xff1b | 0x1b => 0x29,
        0xff08 | 0x08 => 0x2a,
        0xff09 | 0x09 => 0x2b,
        0x20 => 0x2c,
        0x2d | 0x5f => 0x2d,
        0x3d | 0x2b => 0x2e,
        0x5b | 0x7b => 0x2f,
        0x5d | 0x7d => 0x30,
        0x5c | 0x7c => 0x31,
        0x3b | 0x3a => 0x33,
        0x27 | 0x22 => 0x34,
        0x60 | 0x7e => 0x35,
        0x2c | 0x3c => 0x36,
        0x2e | 0x3e => 0x37,
        0x2f | 0x3f => 0x38,
        0xffe5 | 0xffe6 => 0x39,
        0xffbe..=0xffc9 => 0x3a + (keysym - 0xffbe) as u8,
        0xff14 => 0x47,
        0xff13 => 0x48,
        0xff63 => 0x49,
        0xff50 => 0x4a,
        0xff55 => 0x4b,
        0xffff => 0x4c,
        0xff57 => 0x4d,
        0xff56 => 0x4e,
        0xff53 => 0x4f,
        0xff51 => 0x50,
        0xff54 => 0x51,
        0xff52 => 0x52,
        0xff7f => 0x53,
        0xffaf => 0x54,
        0xffaa => 0x55,
        0xffad => 0x56,
        0xffab => 0x57,
        0xff8d => 0x58,
        0xff9c => 0x59,
        0xff99 => 0x5a,
        0xff9b => 0x5b,
        0xff96 => 0x5c,
        0xff9d => 0x5d,
        0xff98 => 0x5e,
        0xff95 => 0x5f,
        0xff97 => 0x60,
        0xff9a => 0x61,
        0xffb1..=0xffb9 => 0x59 + (keysym - 0xffb1) as u8,
        0xffb0 | 0xff9e => 0x62,
        0xffae | 0xffac | 0xff9f => 0x63,
        0xffbd => 0x67,
        0xffe3 => 0xe0,
        0xffe1 => 0xe1,
        0xffe9 => 0xe2,
        0xffe7 | 0xffeb => 0xe3,
        0xffe4 => 0xe4,
        0xffe2 => 0xe5,
        0xffea | 0xfe03 | 0xff7e => 0xe6,
        0xffe8 | 0xffec => 0xe7,
        _ => return None,
    };
    Some(usage)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maps_common_keysyms_to_hid_usages() {
        assert_eq!(keysym_to_usage('a' as u32), Some(0x04));
        assert_eq!(keysym_to_usage('Z' as u32), Some(0x1d));
        assert_eq!(keysym_to_usage(0xffe3), Some(0xe0));
        assert_eq!(keysym_to_usage(0xffe2), Some(0xe5));
        assert_eq!(keysym_to_usage(0xff52), Some(0x52));
        assert_eq!(keysym_to_usage(0xffc9), Some(0x45));
        assert_eq!(keysym_to_usage(0xffb0), Some(0x62));
        assert_eq!(keysym_to_usage(0x0101_f642), None);
    }

    #[test]
    fn absolute_pointer_uses_a_per_client_baseline() {
        let mut translator = VncInputTranslator::default();
        assert!(
            translator
                .translate(&VncInputEvent::PointerPosition { x: 100, y: 100 })
                .is_empty()
        );
        assert_eq!(
            translator.translate(&VncInputEvent::PointerPosition { x: 125, y: 80 }),
            [mouse_event(25, -20, 0, 0)]
        );
        assert_eq!(
            translator.translate(&VncInputEvent::ReleaseAll),
            [InputEvent::ReleaseAll]
        );
        assert!(
            translator
                .translate(&VncInputEvent::PointerPosition { x: 500, y: 500 })
                .is_empty()
        );
    }

    #[test]
    fn wheel_is_transient_and_large_motion_is_split() {
        let mut translator = VncInputTranslator::default();
        assert_eq!(
            translator.translate(&VncInputEvent::MouseButton {
                button: 3,
                down: true,
            }),
            [mouse_event(0, 0, 1, 0)]
        );
        assert!(
            translator
                .translate(&VncInputEvent::MouseButton {
                    button: 3,
                    down: false,
                })
                .is_empty()
        );
        let events = split_motion(40_000, -40_000, 1);
        assert_eq!(events.len(), 2);
        let (sum_x, sum_y) = events.iter().fold((0i64, 0i64), |(x, y), event| {
            if let InputEvent::Mouse { dx, dy, .. } = event {
                (x + i64::from(*dx), y + i64::from(*dy))
            } else {
                unreachable!()
            }
        });
        assert_eq!((sum_x, sum_y), (40_000, -40_000));
    }

    #[test]
    fn rfb_middle_and_right_buttons_use_hid_bit_order() {
        let mut translator = VncInputTranslator::default();
        assert_eq!(
            translator.translate(&VncInputEvent::MouseButton {
                button: 1,
                down: true,
            }),
            [mouse_event(0, 0, 0, 4)]
        );
        assert_eq!(
            translator.translate(&VncInputEvent::MouseButton {
                button: 2,
                down: true,
            }),
            [mouse_event(0, 0, 0, 6)]
        );
    }
}
