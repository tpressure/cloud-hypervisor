// Copyright © 2026 Cyberus Technology GmbH
//
// SPDX-License-Identifier: Apache-2.0

use std::os::unix::net::{UnixListener, UnixStream};
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use std::{io, thread};

use log::{info, warn};
use vfio_user_common::input_protocol::{InputEvent, read_event};

use crate::UhciController;

pub fn spawn_input_server(
    path: &Path,
    controller: Arc<Mutex<UhciController>>,
    stop: Arc<AtomicBool>,
) -> io::Result<thread::JoinHandle<()>> {
    let listener = UnixListener::bind(path)?;
    listener.set_nonblocking(true)?;
    info!("host input: listening on {}", path.display());
    thread::Builder::new()
        .name("uhci-input-ipc".to_string())
        .spawn(move || run_input_server(&listener, &controller, &stop))
}

fn run_input_server(
    listener: &UnixListener,
    controller: &Arc<Mutex<UhciController>>,
    stop: &AtomicBool,
) {
    while !stop.load(Ordering::Relaxed) {
        match listener.accept() {
            Ok((stream, _)) => {
                info!("host input: frontend connected");
                if let Err(error) = handle_client(stream, controller, stop)
                    && !matches!(
                        error.kind(),
                        io::ErrorKind::UnexpectedEof
                            | io::ErrorKind::ConnectionReset
                            | io::ErrorKind::BrokenPipe
                    )
                {
                    warn!("host input: frontend disconnected after error: {error}");
                }
                controller.lock().unwrap().release_all_input();
                info!("host input: frontend disconnected; released input state");
            }
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(50));
            }
            Err(error) => {
                warn!("host input: accept failed: {error}");
                thread::sleep(Duration::from_millis(100));
            }
        }
    }
    controller.lock().unwrap().release_all_input();
}

fn handle_client(
    mut stream: UnixStream,
    controller: &Arc<Mutex<UhciController>>,
    stop: &AtomicBool,
) -> io::Result<()> {
    stream.set_read_timeout(Some(Duration::from_millis(100)))?;
    while !stop.load(Ordering::Relaxed) {
        match read_event(&mut stream) {
            Ok(Some(event)) => apply_event(controller, event),
            Ok(None) => return Ok(()),
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) => {}
            Err(error) => return Err(error),
        }
    }
    Ok(())
}

fn apply_event(controller: &Arc<Mutex<UhciController>>, event: InputEvent) {
    let mut controller = controller.lock().unwrap();
    match event {
        InputEvent::Key { usage, pressed } => controller.input_key(usage, pressed),
        InputEvent::Mouse {
            dx,
            dy,
            wheel,
            buttons,
        } => controller.input_mouse(i32::from(dx), i32::from(dy), wheel, buttons),
        InputEvent::ReleaseAll => controller.release_all_input(),
        InputEvent::KeyboardLeds { .. } => {
            warn!("host input: ignored frontend-to-device keyboard LED event");
        }
    }
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use vfio_user_common::input_protocol::write_event;

    use super::*;
    use crate::uhci::InterruptLine;

    fn wait_for<T>(mut poll: impl FnMut() -> Option<T>) -> T {
        let deadline = Instant::now() + Duration::from_secs(1);
        loop {
            if let Some(value) = poll() {
                return value;
            }
            assert!(
                Instant::now() < deadline,
                "timed out waiting for input event"
            );
            thread::sleep(Duration::from_millis(10));
        }
    }

    #[test]
    fn input_events_update_controller_state() {
        let controller = Arc::new(Mutex::new(UhciController::new(InterruptLine::new())));
        apply_event(
            &controller,
            InputEvent::Key {
                usage: 4,
                pressed: true,
            },
        );
        assert_eq!(
            controller.lock().unwrap().take_keyboard_report().unwrap(),
            [0, 0, 4, 0, 0, 0, 0, 0]
        );
        apply_event(
            &controller,
            InputEvent::Mouse {
                dx: 4,
                dy: -2,
                wheel: 1,
                buttons: 1,
            },
        );
        assert_eq!(
            controller.lock().unwrap().take_mouse_report().unwrap(),
            [1, 4, 254, 1]
        );
        apply_event(&controller, InputEvent::ReleaseAll);
        assert_eq!(
            controller.lock().unwrap().take_keyboard_report().unwrap(),
            [0; 8]
        );
        assert_eq!(
            controller.lock().unwrap().take_mouse_report().unwrap(),
            [0, 0, 0, 0]
        );
        assert_eq!(controller.lock().unwrap().keyboard_leds(), 0);
    }

    #[test]
    fn unix_socket_framing_and_disconnect_release_input() {
        let path = std::env::temp_dir().join(format!(
            "vfio-usb-hid-input-test-{}-{:?}.sock",
            std::process::id(),
            thread::current().id()
        ));
        let controller = Arc::new(Mutex::new(UhciController::new(InterruptLine::new())));
        let stop = Arc::new(AtomicBool::new(false));
        let handle = spawn_input_server(&path, Arc::clone(&controller), Arc::clone(&stop)).unwrap();
        let mut stream = UnixStream::connect(&path).unwrap();

        write_event(
            &mut stream,
            InputEvent::Key {
                usage: 4,
                pressed: true,
            },
        )
        .unwrap();
        write_event(
            &mut stream,
            InputEvent::Mouse {
                dx: 7,
                dy: -3,
                wheel: 1,
                buttons: 1,
            },
        )
        .unwrap();
        assert_eq!(
            wait_for(|| controller.lock().unwrap().take_keyboard_report()),
            [0, 0, 4, 0, 0, 0, 0, 0]
        );
        assert_eq!(
            wait_for(|| controller.lock().unwrap().take_mouse_report()),
            [1, 7, 253, 1]
        );

        drop(stream);
        assert_eq!(
            wait_for(|| controller.lock().unwrap().take_keyboard_report()),
            [0; 8]
        );
        assert_eq!(
            wait_for(|| controller.lock().unwrap().take_mouse_report()),
            [0, 0, 0, 0]
        );

        stop.store(true, Ordering::Relaxed);
        handle.join().unwrap();
        std::fs::remove_file(path).unwrap();
    }
}
