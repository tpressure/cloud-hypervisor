use std::io::{Read, Result, Write};
use std::net::{TcpListener, TcpStream};
use std::os::unix::net::{UnixListener, UnixStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::thread;
use std::time::Duration;

use log::{debug, error, info, warn};

use crate::framebuffer::FramebufferSource;

const MAX_CLIENT_MESSAGE_SIZE: usize = 16 * 1024 * 1024;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct PixelFormat {
    bits_per_pixel: u8,
    depth: u8,
    big_endian: bool,
    red_max: u16,
    green_max: u16,
    blue_max: u16,
    red_shift: u8,
    green_shift: u8,
    blue_shift: u8,
}

const XRGB8888_FORMAT: PixelFormat = PixelFormat {
    bits_per_pixel: 32,
    depth: 24,
    big_endian: false,
    red_max: 255,
    green_max: 255,
    blue_max: 255,
    red_shift: 16,
    green_shift: 8,
    blue_shift: 0,
};

impl PixelFormat {
    fn from_set_pixel_format(message: &[u8]) -> Result<Self> {
        let format = Self {
            bits_per_pixel: message[4],
            depth: message[5],
            big_endian: message[6] != 0,
            red_max: u16::from_be_bytes([message[8], message[9]]),
            green_max: u16::from_be_bytes([message[10], message[11]]),
            blue_max: u16::from_be_bytes([message[12], message[13]]),
            red_shift: message[14],
            green_shift: message[15],
            blue_shift: message[16],
        };

        if message[7] == 0 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "color-map pixel formats are not supported",
            ));
        }
        if !matches!(format.bits_per_pixel, 8 | 16 | 32)
            || format.depth == 0
            || format.depth > format.bits_per_pixel
            || format.red_max == 0
            || format.green_max == 0
            || format.blue_max == 0
        {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "invalid true-color pixel format",
            ));
        }

        let pixel_mask = if format.bits_per_pixel == 32 {
            u64::from(u32::MAX)
        } else {
            (1u64 << format.bits_per_pixel) - 1
        };
        let masks = [
            u64::from(format.red_max) << format.red_shift,
            u64::from(format.green_max) << format.green_shift,
            u64::from(format.blue_max) << format.blue_shift,
        ];
        if masks.iter().any(|mask| mask & !pixel_mask != 0)
            || masks[0] & masks[1] != 0
            || masks[0] & masks[2] != 0
            || masks[1] & masks[2] != 0
        {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "pixel format color fields overlap or exceed the pixel size",
            ));
        }

        Ok(format)
    }

    fn bytes_per_pixel(self) -> usize {
        usize::from(self.bits_per_pixel / 8)
    }
}

#[derive(Clone, Copy, Debug)]
struct FramebufferUpdateRequest {
    incremental: bool,
}

struct ClientState {
    pixel_format: PixelFormat,
    framebuffer_request: Option<FramebufferUpdateRequest>,
    pointer_button_mask: u8,
}

impl Default for ClientState {
    fn default() -> Self {
        Self {
            pixel_format: XRGB8888_FORMAT,
            framebuffer_request: None,
            pointer_button_mask: 0,
        }
    }
}

/// VNC input event types
#[derive(Debug, Clone)]
pub enum VncInputEvent {
    Keyboard { key: u32, down: bool },
    MouseButton { button: u8, down: bool },
    PointerMove { dx: i16, dy: i16 },
    PointerPosition { x: u32, y: u32 },
    ReleaseAll,
}

/// Unified stream type for TCP and Unix sockets
pub enum VncStream {
    Tcp(TcpStream),
    Unix(UnixStream),
}

impl VncStream {
    fn set_nonblocking(&self, nonblocking: bool) -> Result<()> {
        match self {
            VncStream::Tcp(s) => s.set_nonblocking(nonblocking),
            VncStream::Unix(s) => s.set_nonblocking(nonblocking),
        }
    }

    fn set_read_timeout(&self, dur: Option<Duration>) -> Result<()> {
        match self {
            VncStream::Tcp(s) => s.set_read_timeout(dur),
            VncStream::Unix(s) => s.set_read_timeout(dur),
        }
    }

    fn as_raw_fd(&self) -> std::os::unix::io::RawFd {
        match self {
            VncStream::Tcp(s) => std::os::unix::io::AsRawFd::as_raw_fd(s),
            VncStream::Unix(s) => std::os::unix::io::AsRawFd::as_raw_fd(s),
        }
    }
}

impl Read for VncStream {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        match self {
            VncStream::Tcp(s) => s.read(buf),
            VncStream::Unix(s) => s.read(buf),
        }
    }
}

impl Write for VncStream {
    fn write(&mut self, buf: &[u8]) -> Result<usize> {
        match self {
            VncStream::Tcp(s) => s.write(buf),
            VncStream::Unix(s) => s.write(buf),
        }
    }

    fn flush(&mut self) -> Result<()> {
        match self {
            VncStream::Tcp(s) => s.flush(),
            VncStream::Unix(s) => s.flush(),
        }
    }
}

/// Unified listener type
pub enum VncListener {
    Tcp(TcpListener),
    Unix(UnixListener),
}

impl VncListener {
    fn bind(config: &VncServerConfig) -> Result<Self> {
        match &config.listener {
            VncListenerType::Tcp { port } => {
                let listener = TcpListener::bind(format!("0.0.0.0:{port}"))?;
                info!("vnc: listening on TCP 0.0.0.0:{port}");
                Ok(Self::Tcp(listener))
            }
            VncListenerType::Unix { path } => {
                let _ = std::fs::remove_file(path);
                let listener = UnixListener::bind(path)?;
                info!("vnc: listening on Unix socket {path}");
                Ok(Self::Unix(listener))
            }
        }
    }

    fn accept(&self) -> Result<(VncStream, String)> {
        match self {
            VncListener::Tcp(l) => {
                let (stream, addr) = l.accept()?;
                Ok((VncStream::Tcp(stream), addr.to_string()))
            }
            VncListener::Unix(l) => {
                let (stream, addr) = l.accept()?;
                Ok((
                    VncStream::Unix(stream),
                    addr.as_pathname()
                        .map(|p| p.to_string_lossy().to_string())
                        .unwrap_or_else(|| "unix".to_string()),
                ))
            }
        }
    }

    fn set_nonblocking(&self, nonblocking: bool) -> Result<()> {
        match self {
            VncListener::Tcp(l) => l.set_nonblocking(nonblocking),
            VncListener::Unix(l) => l.set_nonblocking(nonblocking),
        }
    }
}

/// VNC listener configuration
#[derive(Debug, Clone)]
pub enum VncListenerType {
    Tcp { port: u16 },
    Unix { path: String },
}

/// VNC server configuration
#[derive(Clone)]
pub struct VncServerConfig {
    pub listener: VncListenerType,
}

/// VNC server that polls the framebuffer and serves a single client.
pub struct VncServer {
    surface: Arc<dyn FramebufferSource>,
    config: VncServerConfig,
    input_sender: mpsc::Sender<VncInputEvent>,
    running: Arc<AtomicBool>,
    on_disconnect: Mutex<Option<Box<dyn Fn() + Send>>>,
}

impl VncServer {
    pub fn new(
        surface: Arc<dyn FramebufferSource>,
        config: VncServerConfig,
        input_sender: mpsc::Sender<VncInputEvent>,
    ) -> Self {
        Self {
            surface,
            config,
            input_sender,
            running: Arc::new(AtomicBool::new(false)),
            on_disconnect: Mutex::new(None),
        }
    }

    pub fn set_on_disconnect<F>(&self, f: F)
    where
        F: Fn() + Send + 'static,
    {
        *self.on_disconnect.lock().unwrap() = Some(Box::new(f));
    }

    pub fn spawn(&self) -> std::io::Result<thread::JoinHandle<()>> {
        let surface = Arc::clone(&self.surface);
        let listener = VncListener::bind(&self.config)?;
        let input_sender = self.input_sender.clone();
        let running = Arc::clone(&self.running);
        let on_disconnect = self
            .on_disconnect
            .lock()
            .map_err(|e| std::io::Error::other(format!("mutex poisoned: {e}")))?
            .take();

        self.running.store(true, Ordering::SeqCst);

        thread::Builder::new()
            .name("ch-vnc-server".to_string())
            .spawn(move || {
                run_vnc_server(surface, listener, input_sender, running, on_disconnect);
            })
            .map_err(|e| {
                error!("vnc: failed to spawn server thread: {e}");
                std::io::Error::other(format!("failed to spawn thread: {e}"))
            })
    }

    pub fn stop(&self) {
        self.running.store(false, Ordering::SeqCst);
    }
}

fn run_vnc_server(
    surface: Arc<dyn FramebufferSource>,
    listener: VncListener,
    input_sender: mpsc::Sender<VncInputEvent>,
    running: Arc<AtomicBool>,
    on_disconnect: Option<Box<dyn Fn() + Send>>,
) {
    listener.set_nonblocking(true).ok();

    while running.load(Ordering::SeqCst) {
        if surface.is_initialized() {
            debug!("vnc: surface initialized, breaking wait loop");
            break;
        }
        thread::sleep(Duration::from_millis(100));
    }

    if !running.load(Ordering::SeqCst) {
        return;
    }

    loop {
        if !running.load(Ordering::SeqCst) {
            break;
        }

        match listener.accept() {
            Ok((stream, peer)) => {
                info!("vnc: client connected from {peer}");
                match handle_client(stream, &surface, &input_sender, &running, &on_disconnect) {
                    Ok(()) => info!("vnc: client disconnected from {peer}"),
                    Err(e) => warn!("vnc: handle_client error from {peer}: {e}"),
                }
                let _ = input_sender.send(VncInputEvent::ReleaseAll);
                if let Some(callback) = &on_disconnect {
                    callback();
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(100));
            }
            Err(e) => {
                error!("vnc: accept failed: {e}");
                thread::sleep(Duration::from_millis(100));
            }
        }
    }
}

fn handle_client(
    stream: VncStream,
    surface: &Arc<dyn FramebufferSource>,
    input_sender: &mpsc::Sender<VncInputEvent>,
    running: &Arc<AtomicBool>,
    _on_disconnect: &Option<Box<dyn Fn() + Send>>,
) -> Result<()> {
    let stream = Arc::new(Mutex::new(stream));

    {
        let mut s = stream.lock().unwrap();
        s.set_nonblocking(false)?;
        s.set_read_timeout(Some(Duration::from_secs(5))).ok();

        let use_rfb38 = handshake(&mut *s)?;
        security(&mut *s, use_rfb38)?;

        let config = surface.config().ok_or_else(|| {
            std::io::Error::new(
                std::io::ErrorKind::NotConnected,
                "framebuffer DMA mapping is unavailable",
            )
        })?;
        let width = config.width;
        let height = config.height;
        debug!("vnc: surface config width={} height={}", width, height);
        server_init(&mut *s, width, height)?;
        client_init(&mut *s)?;
    }

    {
        let s = stream.lock().unwrap();
        s.set_nonblocking(true)?;
        s.set_read_timeout(None).ok();
    }

    info!("vnc: client authenticated and connected");

    // RFB framebuffer updates are sent in response to client requests. Waiting
    // for the first request also lets the client select its pixel format before
    // any pixel data is transmitted.
    let mut last_data: Option<Vec<u8>> = None;
    let mut client_state = ClientState::default();

    let fb_interval = Duration::from_millis(40); // 25 FPS for framebuffer updates
    let input_interval = Duration::from_millis(100); // 10 Hz for input polling
    let mut input_buffer = Vec::new();

    while running.load(Ordering::SeqCst) {
        let sleep_duration = {
            let mut s = stream.lock().unwrap();

            // Check for client input FIRST (before FBU to avoid blocking)
            let fd = s.as_raw_fd();
            let mut poll_fd = libc::pollfd {
                fd,
                events: libc::POLLIN,
                revents: 0,
            };
            let polled = {
                // SAFETY: fd is a valid file descriptor from a live VncStream.
                // poll_fd is properly initialized with valid fields.
                // The pointer is valid for reads of 1 element.
                (unsafe { libc::poll(&mut poll_fd, 1, 0) }) > 0
            };
            if polled && (poll_fd.revents & (libc::POLLHUP | libc::POLLERR | libc::POLLNVAL)) != 0 {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::ConnectionReset,
                    "Client disconnected (poll hangup/error)",
                ));
            }
            let readable = polled && (poll_fd.revents & libc::POLLIN) != 0;

            if readable {
                // Keep non-blocking. Incomplete RFB messages remain buffered until
                // the rest of the message arrives.
                info!("vnc: socket readable, draining input");
                check_client_input(&mut *s, input_sender, &mut input_buffer, &mut client_state)?;
            }

            let mut fb_sent = false;
            match (client_state.framebuffer_request, surface.read_framebuffer()) {
                (Some(request), Some(current_data)) => {
                    let has_change = match &last_data {
                        None => true,
                        Some(prev) => prev != &current_data,
                    };
                    debug!(
                        "vnc: read_framebuffer returned {} bytes, has_change={}",
                        current_data.len(),
                        has_change
                    );

                    if has_change || !request.incremental {
                        // Check if socket is writable before sending FBU
                        let mut poll_out = libc::pollfd {
                            fd: s.as_raw_fd(),
                            events: libc::POLLOUT,
                            revents: 0,
                        };
                        let writable = {
                            // SAFETY: fd is valid from live VncStream
                            (unsafe { libc::poll(&mut poll_out, 1, 0) }) > 0
                        } && (poll_out.revents & libc::POLLOUT) != 0;

                        if writable {
                            let config = surface.config().ok_or_else(|| {
                                std::io::Error::new(
                                    std::io::ErrorKind::NotConnected,
                                    "framebuffer DMA mapping was removed",
                                )
                            })?;
                            // Framebuffer updates can be much larger than the socket send
                            // buffer, especially when sent through nova-novncproxy. The
                            // stream is normally non-blocking, so write_all() can otherwise
                            // return WouldBlock after sending only part of an RFB message.
                            // Send a complete update synchronously for now; a future
                            // non-blocking implementation should retain and drain partial
                            // framebuffer updates on POLLOUT.
                            s.set_nonblocking(false)?;
                            let send_result = send_framebuffer_update(
                                &mut *s,
                                &current_data,
                                config.width,
                                config.height,
                                config.stride,
                                client_state.pixel_format,
                            );
                            s.set_nonblocking(true)?;

                            if send_result.is_err() {
                                warn!(
                                    "vnc: failed to send framebuffer update, client disconnected"
                                );
                                return Err(std::io::Error::new(
                                    std::io::ErrorKind::BrokenPipe,
                                    "Client disconnected",
                                ));
                            } else {
                                last_data = Some(current_data);
                                client_state.framebuffer_request = None;
                                fb_sent = true;
                            }
                        }
                    }
                }
                (Some(_), None) => {
                    debug!("vnc: read_framebuffer returned None");
                }
                (None, _) => {}
            }
            if fb_sent { fb_interval } else { input_interval }
        }; // drop lock

        thread::sleep(sleep_duration);
    }

    Ok(())
}

fn handshake<RW: Read + Write>(rw: &mut RW) -> Result<bool> {
    // Advertise RFB 3.8 for list-based security negotiation.
    rw.write_all(b"RFB 003.008\n")?;
    rw.flush()?;

    let mut client_version = [0u8; 12];
    rw.read_exact(&mut client_version)?;

    let client_ver_str = String::from_utf8_lossy(&client_version);
    info!(
        "vnc: client version: {} (bytes: {:02x?}",
        client_ver_str.trim(),
        client_version
    );

    // Parse client version string: "RFB 003.XXX\n" or multi-version "RFB 003.089\n003.079\n..."
    // Check if client supports RFB 3.8 (003.008)
    let uses_rfb38 = client_ver_str.contains("003.008");

    if uses_rfb38 {
        info!("vnc: using RFB 3.8 protocol");
    } else if client_ver_str.contains("003.003") {
        info!("vnc: using RFB 3.3 protocol");
    } else {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!("Unsupported RFB version: {}", client_ver_str.trim()),
        ));
    }

    Ok(uses_rfb38)
}

fn security<RW: Read + Write>(rw: &mut RW, use_rfb38: bool) -> Result<()> {
    if use_rfb38 {
        // RFB 3.8 security negotiation:
        // Server sends: 1-byte count + count x 1-byte security types
        // Client responds with: 1-byte selected security type
        // Server sends: 4-byte security result (0=OK)
        rw.write_all(&[1])?; // count = 1 (1 byte)
        rw.write_all(&[1])?; // type 1 = None (no authentication)
        rw.flush()?;
        info!(
            "vnc: sent RFB 3.8 security types (count=1, type=1=None), waiting for client response"
        );
    } else {
        // RFB 3.3 security negotiation:
        // Server sends: 4-byte security type directly (no count byte)
        // - type 1 = VNC Authentication (requires password challenge, which we don't support)
        // - For backward compatibility with clients that only support 3.3, we still need to
        //   handle this, but it will fail without proper VNC auth implementation.
        //   Since we advertise 3.8 first, most clients will negotiate to 3.8.
        rw.write_all(&[0, 0, 0, 1])?; // type 1 = VNC Authentication (RFB 3.3 format, no count)
        rw.flush()?;
        info!("vnc: sent RFB 3.3 security type (1=VNCAuth), waiting for client response");
    }

    let mut selected = [0u8; 1];
    match rw.read_exact(&mut selected) {
        Ok(()) => {}
        Err(e) => {
            info!("vnc: read_exact for security type failed: {e}");
            return Err(e);
        }
    }

    let selected_type = u32::from(selected[0]);
    debug!("vnc: client selected security type: {selected_type}");

    if use_rfb38 {
        if selected_type != 1 {
            rw.write_all(&[0, 0, 0, 1])?; // 1 security reason
            rw.write_all(b"Invalid security type")?;
            rw.flush()?;
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "Invalid security type",
            ));
        }
        // Auth OK - send 4-byte security result
        rw.write_all(&[0, 0, 0, 0])?; // result 0 = OK
        rw.flush()?;
    } else {
        if selected_type != 1 {
            rw.write_all(&[0, 0, 0, 1])?;
            rw.write_all(b"Invalid security type")?;
            rw.flush()?;
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "Invalid security type",
            ));
        }
        // For RFB 3.3, type 1 = VNC Auth, which requires sending a 16-byte challenge.
        // We don't support VNC auth, so this path will fail. Clients should negotiate to 3.8.
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "VNC authentication not supported; client must support RFB 3.8",
        ));
    }

    Ok(())
}

fn server_init<W: Write>(writer: &mut W, width: u32, height: u32) -> Result<()> {
    let name = b"Cloud Hypervisor";
    let name_len = name.len() as u32;

    // RFB ServerInit: width and height are CARD16 (2 bytes each)
    writer.write_all(&((width as u16).to_be_bytes()))?;
    writer.write_all(&((height as u16).to_be_bytes()))?;

    // Pixel format: XRGB8888 (16 bytes)
    // bits_per_pixel=32, depth=24, big_endian=0, true_color=1
    // red_max=255, green_max=255, blue_max=255
    // red_shift=16, green_shift=8, blue_shift=0
    // Note: TigerVNC reads shift values as 1 byte each + 3 padding bytes
    writer.write_all(&[32, 24, 0, 1, 0, 255, 0, 255, 0, 255, 16, 8, 0, 0, 0, 0])?;

    writer.write_all(&name_len.to_be_bytes())?;
    writer.write_all(name)?;
    writer.flush()?;

    Ok(())
}

fn client_init<R: Read>(reader: &mut R) -> Result<()> {
    let mut shared = [0u8; 1];
    reader.read_exact(&mut shared)?;

    let shared_flag = shared[0];
    debug!("vnc: client shared flag: {shared_flag}");

    Ok(())
}

fn send_framebuffer_update<W: Write>(
    writer: &mut W,
    data: &[u8],
    width: u32,
    height: u32,
    stride: u32,
    pixel_format: PixelFormat,
) -> Result<()> {
    let source_row_size = width.checked_mul(4).ok_or_else(|| {
        std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "framebuffer width overflow",
        )
    })?;
    let fb_size = stride.checked_mul(height).ok_or_else(|| {
        std::io::Error::new(std::io::ErrorKind::InvalidData, "framebuffer size overflow")
    })?;

    if stride < source_row_size || data.len() < fb_size as usize {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "framebuffer data is smaller than its dimensions and stride",
        ));
    }

    // Framebuffer update message:
    // message-type=0 (1 byte), padding (1 byte), number-of-rects (CARD16)
    writer.write_all(&[0])?;
    writer.write_all(&[0])?;
    writer.write_all(&[0, 1])?;

    // Rectangle: x, y, width, height (all CARD16), encoding (CARD32)
    writer.write_all(&[0, 0])?; // x = 0
    writer.write_all(&[0, 0])?; // y = 0
    writer.write_all(&((width as u16).to_be_bytes()))?; // width CARD16
    writer.write_all(&((height as u16).to_be_bytes()))?; // height CARD16
    writer.write_all(&[0, 0, 0, 0])?; // encoding = 0 (RAW)

    let source_row_size = source_row_size as usize;
    let stride = stride as usize;
    for y in 0..height as usize {
        let row = &data[y * stride..y * stride + source_row_size];
        write_pixels(writer, row, pixel_format)?;
    }
    writer.flush()?;

    Ok(())
}

fn write_pixels<W: Write>(writer: &mut W, source: &[u8], format: PixelFormat) -> Result<()> {
    if format == XRGB8888_FORMAT {
        return writer.write_all(source);
    }

    let mut converted = Vec::with_capacity(source.len() / 4 * format.bytes_per_pixel());
    for pixel in source.chunks_exact(4) {
        // DRM_FORMAT_XRGB8888 is stored as B, G, R, X on little-endian hosts.
        let red = scale_color(pixel[2], format.red_max);
        let green = scale_color(pixel[1], format.green_max);
        let blue = scale_color(pixel[0], format.blue_max);
        let value =
            (red << format.red_shift) | (green << format.green_shift) | (blue << format.blue_shift);
        let bytes = if format.big_endian {
            value.to_be_bytes()
        } else {
            value.to_le_bytes()
        };
        if format.big_endian {
            converted.extend_from_slice(&bytes[4 - format.bytes_per_pixel()..]);
        } else {
            converted.extend_from_slice(&bytes[..format.bytes_per_pixel()]);
        }
    }
    writer.write_all(&converted)
}

fn scale_color(color: u8, maximum: u16) -> u32 {
    (u32::from(color) * u32::from(maximum) + 127) / 255
}

fn check_client_input<R: Read>(
    reader: &mut R,
    input_sender: &mpsc::Sender<VncInputEvent>,
    input_buffer: &mut Vec<u8>,
    client_state: &mut ClientState,
) -> Result<()> {
    let mut read_buffer = [0u8; 4096];

    loop {
        match reader.read(&mut read_buffer) {
            Ok(0) => {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::UnexpectedEof,
                    "client disconnected",
                ));
            }
            Ok(size) => {
                input_buffer.extend_from_slice(&read_buffer[..size]);
                if input_buffer.len() > MAX_CLIENT_MESSAGE_SIZE {
                    return Err(std::io::Error::new(
                        std::io::ErrorKind::InvalidData,
                        "client input buffer exceeds the maximum message size",
                    ));
                }
            }
            Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => break,
            Err(error) if error.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(error) => return Err(error),
        }
    }

    drain_client_input(input_buffer, input_sender, client_state)
}

fn drain_client_input(
    input_buffer: &mut Vec<u8>,
    input_sender: &mpsc::Sender<VncInputEvent>,
    client_state: &mut ClientState,
) -> Result<()> {
    loop {
        let Some(message_size) = client_message_size(input_buffer)? else {
            return Ok(());
        };
        if input_buffer.len() < message_size {
            return Ok(());
        }

        handle_client_message(&input_buffer[..message_size], input_sender, client_state)?;
        input_buffer.drain(..message_size);
    }
}

fn client_message_size(input_buffer: &[u8]) -> Result<Option<usize>> {
    if input_buffer.is_empty() {
        return Ok(None);
    }

    let message_size = match input_buffer[0] {
        // SetPixelFormat: type + 3 bytes padding + 16-byte pixel format.
        0 => 20,
        // SetColorMapEntries: type + padding + first color + count + colors.
        1 => {
            if input_buffer.len() < 6 {
                return Ok(None);
            }
            variable_message_size(
                6,
                u16::from_be_bytes([input_buffer[4], input_buffer[5]]) as usize,
                6,
            )?
        }
        // SetEncodings: type + padding + count + encodings.
        2 => {
            if input_buffer.len() < 4 {
                return Ok(None);
            }
            variable_message_size(
                4,
                u16::from_be_bytes([input_buffer[2], input_buffer[3]]) as usize,
                4,
            )?
        }
        // FramebufferUpdateRequest: type + incremental + x/y/width/height.
        3 => 10,
        // KeyEvent: type + down flag + padding + keysym.
        4 => 8,
        // PointerEvent: type + button mask + x/y.
        5 => 6,
        // ClientCutText: type + padding + text length + text.
        6 => {
            if input_buffer.len() < 8 {
                return Ok(None);
            }
            variable_message_size(
                8,
                u32::from_be_bytes([
                    input_buffer[4],
                    input_buffer[5],
                    input_buffer[6],
                    input_buffer[7],
                ]) as usize,
                1,
            )?
        }
        // SetColourValues: type + first color + count + color values.
        9 => {
            if input_buffer.len() < 5 {
                return Ok(None);
            }
            variable_message_size(
                5,
                u16::from_be_bytes([input_buffer[3], input_buffer[4]]) as usize,
                6,
            )?
        }
        // QEMU extended client message: type + 12-byte payload.
        0xFF => 13,
        message_type => {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("unsupported client message type 0x{message_type:02x}"),
            ));
        }
    };

    Ok(Some(message_size))
}

fn variable_message_size(header_size: usize, count: usize, item_size: usize) -> Result<usize> {
    let message_size = count
        .checked_mul(item_size)
        .and_then(|items_size| header_size.checked_add(items_size))
        .filter(|&size| size <= MAX_CLIENT_MESSAGE_SIZE)
        .ok_or_else(|| {
            std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "client message exceeds the maximum supported size",
            )
        })?;

    Ok(message_size)
}

fn handle_client_message(
    message: &[u8],
    input_sender: &mpsc::Sender<VncInputEvent>,
    client_state: &mut ClientState,
) -> Result<()> {
    match message[0] {
        0 => {
            let pixel_format = PixelFormat::from_set_pixel_format(message)?;
            info!("vnc: client selected pixel format {pixel_format:?}");
            client_state.pixel_format = pixel_format;
        }
        1 => {
            let first_idx = u16::from_be_bytes([message[2], message[3]]);
            let n_entries = u16::from_be_bytes([message[4], message[5]]);
            debug!("vnc: SetColorMapEntries first={first_idx} n={n_entries}");
        }
        2 => {
            let count = u16::from_be_bytes([message[2], message[3]]);
            debug!("vnc: SetEncodings count={count}");
        }
        3 => {
            let incremental = message[1] != 0;
            let x = u16::from_be_bytes([message[2], message[3]]);
            let y = u16::from_be_bytes([message[4], message[5]]);
            let width = u16::from_be_bytes([message[6], message[7]]);
            let height = u16::from_be_bytes([message[8], message[9]]);
            debug!(
                "vnc: FramebufferUpdateRequest incremental={incremental} x={x} y={y} width={width} height={height}"
            );
            client_state.framebuffer_request = Some(FramebufferUpdateRequest { incremental });
        }
        4 => {
            let down = message[1] != 0;
            let key = u32::from_be_bytes([message[4], message[5], message[6], message[7]]);

            let _ = input_sender.send(VncInputEvent::Keyboard { key, down });
            info!("vnc: key event keysym=0x{key:x} down={down}");
        }
        5 => {
            let mask = message[1];
            let x = u16::from_be_bytes([message[2], message[3]]) as u32;
            let y = u16::from_be_bytes([message[4], message[5]]) as u32;

            let changed_buttons = client_state.pointer_button_mask ^ mask;
            for (bit, button) in [(1, 0), (2, 1), (4, 2), (8, 3), (16, 4)] {
                if changed_buttons & bit != 0 {
                    let _ = input_sender.send(VncInputEvent::MouseButton {
                        button,
                        down: mask & bit != 0,
                    });
                }
            }
            client_state.pointer_button_mask = mask;
            let _ = input_sender.send(VncInputEvent::PointerPosition { x, y });

            debug!("vnc: pointer event mask={mask} x={x} y={y}");
        }
        6 => {
            let length = u32::from_be_bytes([message[4], message[5], message[6], message[7]]);
            debug!("vnc: ClientCutText length={length}");
        }
        9 => {
            let first_color = u16::from_be_bytes([message[1], message[2]]);
            let num_colors = u16::from_be_bytes([message[3], message[4]]);
            debug!("vnc: SetColourValues first={first_color} num={num_colors}");
        }
        0xFF => {
            let sub_type = message[1];
            if sub_type == 0 {
                // QEMU Extended KeyEvent: sub-type + down-flag(U16) + keysym(U32) + keycode(U32).
                let down = u16::from_be_bytes([message[2], message[3]]) != 0;
                let key = u32::from_be_bytes([message[4], message[5], message[6], message[7]]);
                let _ = input_sender.send(VncInputEvent::Keyboard { key, down });
                info!("vnc: qemu key event keysym=0x{key:x} down={down}");
            } else {
                info!("vnc: qemu extended msg sub-type={sub_type}");
            }
        }
        _ => unreachable!("client_message_size validates message types"),
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;
    use std::io::Cursor;

    use super::*;

    struct MockSocket {
        read_buf: Cursor<Vec<u8>>,
        write_buf: Vec<u8>,
    }

    impl MockSocket {
        fn new() -> Self {
            Self {
                read_buf: Cursor::new(Vec::new()),
                write_buf: Vec::new(),
            }
        }

        fn queue_read_data(&mut self, data: &[u8]) {
            self.read_buf = Cursor::new(data.to_vec());
        }

        fn written_data(&self) -> &[u8] {
            &self.write_buf
        }
    }

    impl Read for MockSocket {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            self.read_buf.read(buf)
        }
    }

    impl Write for MockSocket {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.write_buf.extend_from_slice(buf);
            Ok(buf.len())
        }

        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    struct NonBlockingReader {
        chunks: VecDeque<Vec<u8>>,
    }

    impl NonBlockingReader {
        fn new(chunks: impl IntoIterator<Item = Vec<u8>>) -> Self {
            Self {
                chunks: chunks.into_iter().collect(),
            }
        }

        fn queue_read_data(&mut self, data: Vec<u8>) {
            self.chunks.push_back(data);
        }
    }

    impl Read for NonBlockingReader {
        fn read(&mut self, buffer: &mut [u8]) -> std::io::Result<usize> {
            let Some(chunk) = self.chunks.pop_front() else {
                return Err(std::io::ErrorKind::WouldBlock.into());
            };
            assert!(chunk.len() <= buffer.len());
            buffer[..chunk.len()].copy_from_slice(&chunk);
            Ok(chunk.len())
        }
    }

    #[test]
    fn test_rfb38_handshake() {
        let mut sock = MockSocket::new();
        sock.queue_read_data(b"RFB 003.008\n");
        let result = handshake(&mut sock);
        assert!(result.is_ok(), "handshake should succeed for RFB 3.8");
        assert!(result.unwrap(), "should negotiate RFB 3.8");
        assert!(sock.written_data().starts_with(b"RFB 003.008\n"));
    }

    #[test]
    fn test_rfb33_handshake() {
        let mut sock = MockSocket::new();
        sock.queue_read_data(b"RFB 003.003\n");
        let result = handshake(&mut sock);
        assert!(result.is_ok());
        assert!(!result.unwrap(), "should fall back to RFB 3.3");
    }

    #[test]
    fn test_rfb38_security_noauth() {
        let mut sock = MockSocket::new();
        sock.queue_read_data(&[1]);
        security(&mut sock, true).expect("security should succeed for type 1 (None)");
        let w = sock.written_data();
        assert_eq!(&w[0..1], &[1]); // count=1
        assert_eq!(&w[1..2], &[1]); // type=None (1 byte in RFB 3.8)
        assert_eq!(&w[2..6], &[0, 0, 0, 0]); // OK (4-byte result)
    }

    #[test]
    fn test_full_rfb38_handshake() {
        let mut sock = MockSocket::new();

        sock.queue_read_data(b"RFB 003.008\n");
        let use_rfb38 = handshake(&mut sock).expect("handshake failed");
        assert!(use_rfb38);

        sock.queue_read_data(&[1]);
        security(&mut sock, use_rfb38).expect("security failed");

        sock.queue_read_data(&[]);
        server_init(&mut sock, 800, 600).expect("server_init failed");

        sock.queue_read_data(&[1]);
        client_init(&mut sock).expect("client_init failed");

        let w = sock.written_data();
        assert!(w.starts_with(b"RFB 003.008\n"));
        // Security types start at offset 12 (after "RFB 003.008\n")
        assert_eq!(&w[12..13], &[1]); // count=1
        assert_eq!(&w[13..14], &[1]); // type=None
        assert_eq!(&w[14..18], &[0, 0, 0, 0]); // OK
        // Server init starts at offset 18
        assert_eq!(&w[18..20], &(800u16).to_be_bytes());
        assert_eq!(&w[20..22], &(600u16).to_be_bytes());
    }

    #[test]
    fn test_fragmented_key_event_is_retained() {
        let (input_sender, input_receiver) = mpsc::channel();
        let mut reader = NonBlockingReader::new([vec![4, 1, 0, 0, 0, 0]]);
        let mut input_buffer = Vec::new();
        let mut client_state = ClientState::default();

        check_client_input(
            &mut reader,
            &input_sender,
            &mut input_buffer,
            &mut client_state,
        )
        .unwrap();
        assert!(input_receiver.try_recv().is_err());
        assert_eq!(input_buffer, vec![4, 1, 0, 0, 0, 0]);

        reader.queue_read_data(vec![0, 0x66]); // X11 keysym 'f'
        check_client_input(
            &mut reader,
            &input_sender,
            &mut input_buffer,
            &mut client_state,
        )
        .unwrap();

        assert!(matches!(
            input_receiver.recv().unwrap(),
            VncInputEvent::Keyboard {
                key: 0x66,
                down: true,
            }
        ));
        assert!(input_buffer.is_empty());
    }

    #[test]
    fn test_set_encodings_does_not_consume_following_key_event() {
        let (input_sender, input_receiver) = mpsc::channel();
        let mut client_state = ClientState::default();
        let mut input_buffer = vec![
            2, 0, 0, 1, 0, 0, 0, 0, // SetEncodings with one Raw encoding.
            4, 1, 0, 0, 0, 0, 0, 0x66, // KeyEvent for 'f'.
        ];

        drain_client_input(&mut input_buffer, &input_sender, &mut client_state).unwrap();

        assert!(matches!(
            input_receiver.recv().unwrap(),
            VncInputEvent::Keyboard {
                key: 0x66,
                down: true,
            }
        ));
        assert!(input_buffer.is_empty());
    }

    #[test]
    fn test_set_pixel_format_is_twenty_bytes() {
        let (input_sender, input_receiver) = mpsc::channel();
        let mut client_state = ClientState::default();
        let mut input_buffer = vec![
            // SetPixelFormat selecting little-endian RGB565.
            0, 0, 0, 0, 16, 16, 0, 1, 0, 31, 0, 63, 0, 31, 11, 5, 0, 0, 0, 0,
            // SetEncodings with one Raw encoding.
            2, 0, 0, 1, 0, 0, 0, 0, // KeyEvent for 'f'.
            4, 1, 0, 0, 0, 0, 0, 0x66,
        ];

        drain_client_input(&mut input_buffer, &input_sender, &mut client_state).unwrap();

        assert_eq!(client_state.pixel_format.bits_per_pixel, 16);
        assert_eq!(client_state.pixel_format.red_shift, 11);
        assert!(matches!(
            input_receiver.recv().unwrap(),
            VncInputEvent::Keyboard {
                key: 0x66,
                down: true,
            }
        ));
        assert!(input_buffer.is_empty());
    }

    #[test]
    fn test_rgb565_conversion() {
        let format = PixelFormat {
            bits_per_pixel: 16,
            depth: 16,
            big_endian: false,
            red_max: 31,
            green_max: 63,
            blue_max: 31,
            red_shift: 11,
            green_shift: 5,
            blue_shift: 0,
        };
        let mut output = Vec::new();

        // Source pixels are blue, green, and red byte order (XRGB8888).
        write_pixels(
            &mut output,
            &[0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 0],
            format,
        )
        .unwrap();

        assert_eq!(output, [0x00, 0xf8, 0xe0, 0x07, 0x1f, 0x00]);
    }

    #[test]
    fn test_pointer_sends_only_changed_buttons() {
        let (input_sender, input_receiver) = mpsc::channel();
        let mut client_state = ClientState::default();

        handle_client_message(&[5, 0, 0, 10, 0, 20], &input_sender, &mut client_state).unwrap();
        assert!(matches!(
            input_receiver.recv().unwrap(),
            VncInputEvent::PointerPosition { x: 10, y: 20 }
        ));
        assert!(input_receiver.try_recv().is_err());

        handle_client_message(&[5, 1, 0, 11, 0, 21], &input_sender, &mut client_state).unwrap();
        assert!(matches!(
            input_receiver.recv().unwrap(),
            VncInputEvent::MouseButton {
                button: 0,
                down: true,
            }
        ));
        assert!(matches!(
            input_receiver.recv().unwrap(),
            VncInputEvent::PointerPosition { x: 11, y: 21 }
        ));
        assert!(input_receiver.try_recv().is_err());
    }
}
