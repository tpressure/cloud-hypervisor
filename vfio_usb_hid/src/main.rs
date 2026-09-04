// Copyright © 2026 Cyberus Technology GmbH
//
// SPDX-License-Identifier: Apache-2.0

use std::error::Error;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use clap::{Arg, Command};
use log::{info, warn};
use vfio_usb_hid::VfioUsbHidBackend;
use vfio_usb_hid::input::spawn_input_server;
use vfio_user::Server;

fn remove_socket(path: &Path) {
    match std::fs::remove_file(path) {
        Ok(()) => {}
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
        Err(error) => warn!("failed to remove socket {}: {error}", path.display()),
    }
}

struct SocketGuard(PathBuf);

impl Drop for SocketGuard {
    fn drop(&mut self) {
        remove_socket(&self.0);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let matches = Command::new("vfio-usb-hid")
        .version(env!("CARGO_PKG_VERSION"))
        .about("Serve a PCI UHCI controller and USB HID devices over vfio-user")
        .arg_required_else_help(true)
        .arg(
            Arg::new("socket")
                .long("socket")
                .value_name("PATH")
                .help("vfio-user Unix socket")
                .required(true),
        )
        .arg(
            Arg::new("input-socket")
                .long("input-socket")
                .value_name("PATH")
                .help("host input Unix socket")
                .required(true),
        )
        .get_matches();

    let socket = PathBuf::from(matches.get_one::<String>("socket").unwrap());
    let input_socket = PathBuf::from(matches.get_one::<String>("input-socket").unwrap());
    remove_socket(&socket);
    remove_socket(&input_socket);
    let server = Server::new(
        &socket,
        true,
        VfioUsbHidBackend::irqs(),
        VfioUsbHidBackend::regions(),
    )?;
    let _socket_guard = SocketGuard(socket.clone());
    let _input_socket_guard = SocketGuard(input_socket.clone());
    info!("vfio-user UHCI: listening on {}", socket.display());
    let mut backend = VfioUsbHidBackend::new();
    let controller = backend.controller();
    let memory = backend.memory();
    let stop = Arc::new(AtomicBool::new(false));
    let input = spawn_input_server(&input_socket, Arc::clone(&controller), Arc::clone(&stop))?;
    let scheduler_stop = Arc::clone(&stop);
    let scheduler = thread::Builder::new()
        .name("uhci-frame-timer".to_string())
        .spawn(move || {
            let interval = Duration::from_millis(1);
            let mut deadline = Instant::now() + interval;
            while !scheduler_stop.load(Ordering::Relaxed) {
                let now = Instant::now();
                if now < deadline {
                    thread::sleep(deadline - now);
                }
                controller.lock().unwrap().tick(&memory);
                deadline += interval;
                if deadline < Instant::now() {
                    deadline = Instant::now() + interval;
                }
            }
        })?;
    let result = server.run(&mut backend);
    stop.store(true, Ordering::Relaxed);
    let _ = scheduler.join();
    let _ = input.join();
    result.map_err(Into::into)
}

fn main() {
    if let Err(error) = run() {
        eprintln!("vfio-usb-hid: {error}");
        let mut source = error.source();
        while let Some(current) = source {
            eprintln!("  caused by: {current}");
            source = current.source();
        }
        std::process::exit(1);
    }
}
