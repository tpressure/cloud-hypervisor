// Copyright © 2026 Cyberus Technology GmbH
//
// SPDX-License-Identifier: Apache-2.0

use std::error::Error;
use std::path::{Path, PathBuf};
use std::sync::{Arc, mpsc};
use std::thread;
use std::time::{Duration, Instant};

use clap::{Arg, Command};
use display::framebuffer::FramebufferSource;
use display::ramfb::DRM_FORMAT_XRGB8888;
use display::vnc::{VncListenerType, VncServer, VncServerConfig};
use log::{info, warn};
use signal_hook::consts::{SIGINT, SIGTERM};
use signal_hook::iterator::Signals;
use vfio_user::Server;
use vfio_user_simplefb::{
    DmaFramebuffer, FramebufferGeometry, MinimalPciBackend, framebuffer_checksum,
};

fn parse_u64(value: &str) -> Result<u64, String> {
    if let Some(hex) = value.strip_prefix("0x") {
        u64::from_str_radix(hex, 16).map_err(|error| error.to_string())
    } else {
        value.parse::<u64>().map_err(|error| error.to_string())
    }
}

fn parse_format(value: &str) -> Result<u32, String> {
    match value.to_ascii_lowercase().as_str() {
        "xrgb8888" | "bgrx8888" => Ok(DRM_FORMAT_XRGB8888),
        _ => Err(format!(
            "unsupported format {value:?}; expected xrgb8888 or bgrx8888"
        )),
    }
}

fn parse_vnc_listener(value: &str) -> Result<VncListenerType, String> {
    if let Some(path) = value.strip_prefix("unix:") {
        if path.is_empty() {
            return Err("VNC Unix socket path is empty".to_string());
        }
        return Ok(VncListenerType::Unix {
            path: path.to_string(),
        });
    }

    let tcp_address = value.strip_prefix("tcp:").unwrap_or(value);
    let port = tcp_address
        .rsplit_once(':')
        .map_or(tcp_address, |(_, port)| port)
        .parse::<u16>()
        .map_err(|error| format!("invalid VNC TCP listener {value:?}: {error}"))?;
    Ok(VncListenerType::Tcp { port })
}

fn remove_socket(path: &Path) {
    match std::fs::remove_file(path) {
        Ok(()) => {}
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
        Err(error) => warn!("failed to remove socket {}: {error}", path.display()),
    }
}

struct SocketGuard(Option<PathBuf>);

impl Drop for SocketGuard {
    fn drop(&mut self) {
        if let Some(path) = &self.0 {
            remove_socket(path);
        }
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let matches = Command::new("vfio-user-simplefb")
        .version(env!("CARGO_PKG_VERSION"))
        .about("Serve a RAMFB guest-memory framebuffer over VNC using vfio-user DMA mappings")
        .arg_required_else_help(true)
        .arg(
            Arg::new("socket")
                .long("socket")
                .value_name("PATH")
                .help("vfio-user Unix socket")
                .required(true),
        )
        .arg(
            Arg::new("fb-gpa")
                .long("fb-gpa")
                .value_name("GPA")
                .help("guest physical framebuffer address (decimal or 0x-prefixed hex)")
                .value_parser(parse_u64)
                .required(true),
        )
        .arg(
            Arg::new("width")
                .long("width")
                .value_parser(clap::value_parser!(u32))
                .default_value("1024"),
        )
        .arg(
            Arg::new("height")
                .long("height")
                .value_parser(clap::value_parser!(u32))
                .default_value("768"),
        )
        .arg(
            Arg::new("stride")
                .long("stride")
                .value_parser(clap::value_parser!(u32))
                .help("bytes per scan line; defaults to width * 4"),
        )
        .arg(
            Arg::new("format")
                .long("format")
                .value_parser(parse_format)
                .default_value("xrgb8888"),
        )
        .arg(
            Arg::new("vnc")
                .long("vnc")
                .value_name("tcp:PORT|ADDR:PORT|unix:PATH")
                .help("VNC listener")
                .value_parser(parse_vnc_listener)
                .required(true),
        )
        .arg(
            Arg::new("checksum-interval-ms")
                .long("checksum-interval-ms")
                .value_parser(clap::value_parser!(u64))
                .default_value("0")
                .help("periodically log an FNV-1a framebuffer checksum; 0 disables it"),
        )
        .get_matches();

    let socket = PathBuf::from(matches.get_one::<String>("socket").unwrap());
    let gpa = *matches.get_one::<u64>("fb-gpa").unwrap();
    let width = *matches.get_one::<u32>("width").unwrap();
    let height = *matches.get_one::<u32>("height").unwrap();
    let stride = matches.get_one::<u32>("stride").copied().unwrap_or(
        width
            .checked_mul(4)
            .ok_or("default framebuffer stride overflows")?,
    );
    let fourcc = *matches.get_one::<u32>("format").unwrap();
    let vnc_listener = matches.get_one::<VncListenerType>("vnc").unwrap().clone();
    let checksum_interval = *matches.get_one::<u64>("checksum-interval-ms").unwrap();

    let geometry = FramebufferGeometry::new(gpa, width, height, stride, fourcc)?;
    let framebuffer = DmaFramebuffer::new(geometry);
    let surface: Arc<dyn FramebufferSource> = Arc::new(framebuffer.clone());
    let (input_sender, input_receiver) = mpsc::channel();
    // Input needs a separate external transport. Disconnect this channel so
    // unhandled VNC input cannot accumulate in an unbounded queue.
    drop(input_receiver);
    let vnc_server = VncServer::new(
        Arc::clone(&surface),
        VncServerConfig {
            listener: vnc_listener.clone(),
        },
        input_sender,
    );
    let vnc_handle = vnc_server.spawn()?;
    let _vnc_socket_guard = SocketGuard(match &vnc_listener {
        VncListenerType::Unix { path } => Some(PathBuf::from(path)),
        VncListenerType::Tcp { .. } => None,
    });

    let (checksum_stop, checksum_handle) = if checksum_interval == 0 {
        (None, None)
    } else {
        let checksum_surface = Arc::clone(&surface);
        let (stop_sender, stop_receiver) = mpsc::channel();
        let handle = thread::Builder::new()
            .name("simplefb-checksum".to_string())
            .spawn(move || {
                loop {
                    if let Some(data) = checksum_surface.read_framebuffer() {
                        info!(
                            "framebuffer gpa=0x{gpa:016x} size={} checksum=0x{:016x}",
                            data.len(),
                            framebuffer_checksum(&data)
                        );
                    }
                    match stop_receiver.recv_timeout(Duration::from_millis(checksum_interval)) {
                        Ok(()) | Err(mpsc::RecvTimeoutError::Disconnected) => break,
                        Err(mpsc::RecvTimeoutError::Timeout) => {}
                    }
                }
            })?;
        (Some(stop_sender), Some(handle))
    };

    info!(
        "framebuffer gpa=0x{:016x} size={} geometry={}x{} stride={} format=XRGB8888",
        geometry.gpa, geometry.size, geometry.width, geometry.height, geometry.stride
    );
    let server = Server::new(
        &socket,
        false,
        MinimalPciBackend::irqs(),
        MinimalPciBackend::regions(),
    )?;
    let _vfio_socket_guard = SocketGuard(Some(socket.clone()));
    info!("vfio-user: listening on {}", socket.display());

    let (result_sender, result_receiver) = mpsc::sync_channel(1);
    let mapping_monitor = framebuffer.clone();
    let mut backend = MinimalPciBackend::new(framebuffer);
    thread::Builder::new()
        .name("simplefb-vfio-user".to_string())
        .spawn(move || {
            let result = server.run(&mut backend).map_err(|error| error.to_string());
            let _ = result_sender.send(result);
        })?;

    let mut signals = Signals::new([SIGINT, SIGTERM])?;
    let mapping_deadline = Instant::now() + Duration::from_secs(5);
    let result = loop {
        match result_receiver.recv_timeout(Duration::from_millis(100)) {
            Ok(result) => break result,
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                break Err("vfio-user worker exited without a result".to_string());
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
        }
        if let Some(signal) = signals.pending().next() {
            info!("received signal {signal}; shutting down");
            break Ok(());
        }
        if Instant::now() >= mapping_deadline
            && !mapping_monitor.has_framebuffer_mapping()
            && mapping_monitor.mapping_count() > 0
        {
            let mapping_count = mapping_monitor.mapping_count();
            break Err(format!(
                "framebuffer GPA range 0x{:x}..0x{:x} is not contained in any of the {} readable DMA mappings; verify --fb-gpa and geometry",
                geometry.gpa,
                geometry.gpa + geometry.size,
                mapping_count,
            ));
        }
    };

    vnc_server.stop();
    if let Some(stop) = checksum_stop {
        let _ = stop.send(());
    }
    let _ = vnc_handle.join();
    if let Some(handle) = checksum_handle {
        let _ = handle.join();
    }
    result.map_err(Into::into)
}

fn main() {
    if let Err(error) = run() {
        eprintln!("vfio-user-simplefb: {error}");
        let mut source = error.source();
        while let Some(current) = source {
            eprintln!("  caused by: {current}");
            source = current.source();
        }
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_numeric_arguments() {
        assert_eq!(parse_u64("0xBEB00000").unwrap(), 0xbeb0_0000);
        assert_eq!(parse_u64("4096").unwrap(), 4096);
        assert_eq!(parse_format("bgrx8888").unwrap(), DRM_FORMAT_XRGB8888);
    }

    #[test]
    fn parses_vnc_listeners() {
        assert!(matches!(
            parse_vnc_listener("tcp:5900").unwrap(),
            VncListenerType::Tcp { port: 5900 }
        ));
        assert!(matches!(
            parse_vnc_listener("0.0.0.0:5901").unwrap(),
            VncListenerType::Tcp { port: 5901 }
        ));
        assert!(matches!(
            parse_vnc_listener("unix:/tmp/vnc.sock").unwrap(),
            VncListenerType::Unix { .. }
        ));
    }
}
