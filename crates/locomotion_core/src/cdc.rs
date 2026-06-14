use std::ffi::CString;
use std::io;
use std::os::fd::RawFd;
use std::path::Path;
use std::time::{Duration, Instant};

use thiserror::Error;

#[derive(Debug, Error)]
pub enum CdcError {
    #[error("USB CDC fd is gone or no longer usable: {0}")]
    Disconnected(io::Error),
    #[error("USB CDC port is not open")]
    NotOpen,
    #[error("USB CDC write timeout after {0:.3}s")]
    WriteTimeout(f64),
    #[error("path contains interior NUL byte: {0}")]
    BadPath(String),
    #[error("io failed: {0}")]
    Io(#[from] io::Error),
}

#[derive(Debug)]
pub struct CdcSerial {
    pub path: String,
    pub baudrate: i32,
    pub read_chunk_size: usize,
    fd: Option<RawFd>,
}

impl CdcSerial {
    pub fn new(path: impl Into<String>, baudrate: i32) -> Self {
        Self {
            path: path.into(),
            baudrate,
            read_chunk_size: 4096,
            fd: None,
        }
    }

    pub fn is_open(&self) -> bool {
        self.fd.is_some()
    }

    pub fn open(&mut self) -> Result<(), CdcError> {
        if self.fd.is_some() {
            return Ok(());
        }
        let c_path =
            CString::new(self.path.clone()).map_err(|_| CdcError::BadPath(self.path.clone()))?;
        let fd = unsafe {
            libc::open(
                c_path.as_ptr(),
                libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK,
            )
        };
        if fd < 0 {
            let err = io::Error::last_os_error();
            raise_if_cdc_disconnected(err)?;
            return Err(CdcError::Io(io::Error::last_os_error()));
        }
        if let Err(err) = configure_raw_serial(fd, self.baudrate) {
            unsafe {
                libc::close(fd);
            }
            return Err(err);
        }
        self.fd = Some(fd);
        Ok(())
    }

    pub fn close(&mut self) -> Result<(), CdcError> {
        let Some(fd) = self.fd.take() else {
            return Ok(());
        };
        let rc = unsafe { libc::close(fd) };
        if rc < 0 {
            let err = io::Error::last_os_error();
            if !is_recoverable_cdc_errno(err.raw_os_error()) {
                return Err(CdcError::Io(err));
            }
        }
        Ok(())
    }

    pub fn read_available(&mut self) -> Result<Vec<u8>, CdcError> {
        let fd = self.require_fd()?;
        let mut buf = vec![0_u8; self.read_chunk_size];
        let n = unsafe { libc::read(fd, buf.as_mut_ptr().cast(), buf.len()) };
        if n >= 0 {
            buf.truncate(n as usize);
            return Ok(buf);
        }
        let err = io::Error::last_os_error();
        if matches!(err.raw_os_error(), Some(libc::EAGAIN)) {
            return Ok(Vec::new());
        }
        raise_if_cdc_disconnected(err)?;
        Err(CdcError::Io(io::Error::last_os_error()))
    }

    pub fn wait_readable(&self, timeout_s: f64) -> Result<bool, CdcError> {
        let fd = self.require_fd()?;
        poll_fd(fd, libc::POLLIN, timeout_s)
    }

    pub fn write_all(&mut self, data: &[u8], timeout_s: f64) -> Result<(), CdcError> {
        let fd = self.require_fd()?;
        let timeout_s = timeout_s.max(0.0);
        let deadline = Instant::now() + Duration::from_secs_f64(timeout_s);
        let mut offset = 0;
        while offset < data.len() {
            let n = unsafe { libc::write(fd, data[offset..].as_ptr().cast(), data.len() - offset) };
            if n > 0 {
                offset += n as usize;
                continue;
            }
            if n < 0 {
                let err = io::Error::last_os_error();
                if !matches!(err.raw_os_error(), Some(libc::EAGAIN)) {
                    raise_if_cdc_disconnected(err)?;
                    return Err(CdcError::Io(io::Error::last_os_error()));
                }
            }
            if Instant::now() >= deadline {
                return Err(CdcError::WriteTimeout(timeout_s));
            }
            let remaining = deadline.saturating_duration_since(Instant::now());
            let wait_s = remaining.min(Duration::from_millis(1)).as_secs_f64();
            let _ = poll_fd(fd, libc::POLLOUT, wait_s)?;
        }
        Ok(())
    }

    fn require_fd(&self) -> Result<RawFd, CdcError> {
        self.fd.ok_or(CdcError::NotOpen)
    }
}

impl Drop for CdcSerial {
    fn drop(&mut self) {
        let _ = self.close();
    }
}

pub fn configure_raw_serial(fd: RawFd, baudrate: i32) -> Result<(), CdcError> {
    let mut attrs = std::mem::MaybeUninit::<libc::termios>::uninit();
    if unsafe { libc::tcgetattr(fd, attrs.as_mut_ptr()) } < 0 {
        let err = io::Error::last_os_error();
        raise_if_cdc_disconnected(err)?;
        return Err(CdcError::Io(io::Error::last_os_error()));
    }
    let mut attrs = unsafe { attrs.assume_init() };
    unsafe {
        libc::cfmakeraw(&mut attrs);
    }
    attrs.c_cflag |= libc::CLOCAL | libc::CREAD;
    #[cfg(any(target_os = "linux", target_os = "android"))]
    {
        attrs.c_cflag &= !libc::CRTSCTS;
    }
    let speed = baud_to_termios(baudrate);
    unsafe {
        libc::cfsetispeed(&mut attrs, speed);
        libc::cfsetospeed(&mut attrs, speed);
    }
    if unsafe { libc::tcsetattr(fd, libc::TCSANOW, &attrs) } < 0 {
        let err = io::Error::last_os_error();
        raise_if_cdc_disconnected(err)?;
        return Err(CdcError::Io(io::Error::last_os_error()));
    }
    unsafe {
        libc::tcflush(fd, libc::TCIOFLUSH);
    }
    Ok(())
}

#[cfg(any(target_os = "linux", target_os = "android"))]
pub fn baud_to_termios(baudrate: i32) -> libc::speed_t {
    match baudrate {
        9600 => libc::B9600,
        19200 => libc::B19200,
        38400 => libc::B38400,
        57600 => libc::B57600,
        115200 => libc::B115200,
        230400 => libc::B230400,
        460800 => libc::B460800,
        921600 => libc::B921600,
        _ => libc::B921600,
    }
}

#[cfg(not(any(target_os = "linux", target_os = "android")))]
pub fn baud_to_termios(baudrate: i32) -> libc::speed_t {
    match baudrate {
        9600 => libc::B9600,
        19200 => libc::B19200,
        38400 => libc::B38400,
        57600 => libc::B57600,
        115200 => libc::B115200,
        _ => libc::B115200,
    }
}

pub fn default_cdc_port() -> String {
    let by_id_dir = Path::new("/dev/serial/by-id");
    for pattern in ["STMicroelectronics", "STM32"] {
        if let Ok(entries) = std::fs::read_dir(by_id_dir) {
            let mut matches: Vec<_> = entries
                .filter_map(Result::ok)
                .filter(|entry| entry.file_name().to_string_lossy().contains(pattern))
                .map(|entry| entry.path())
                .collect();
            matches.sort();
            if let Some(path) = matches.into_iter().next() {
                return path.to_string_lossy().into_owned();
            }
        }
    }
    for prefix in ["/dev/ttyACM", "/dev/ttyUSB"] {
        if let Ok(entries) = std::fs::read_dir("/dev") {
            let mut matches: Vec<_> = entries
                .filter_map(Result::ok)
                .map(|entry| entry.path())
                .filter(|path| path.to_string_lossy().starts_with(prefix))
                .collect();
            matches.sort();
            if let Some(path) = matches.into_iter().next() {
                return path.to_string_lossy().into_owned();
            }
        }
    }
    "/dev/ttyACM0".to_string()
}

pub fn resolve_cdc_port(value: impl AsRef<str>) -> String {
    let port = value.as_ref().trim();
    if port.eq_ignore_ascii_case("auto") {
        default_cdc_port()
    } else {
        port.to_string()
    }
}

pub fn cdc_port_disappeared(port: &str) -> bool {
    port.starts_with("/dev/") && !Path::new(port).exists()
}

fn poll_fd(fd: RawFd, events: i16, timeout_s: f64) -> Result<bool, CdcError> {
    let mut pfd = libc::pollfd {
        fd,
        events,
        revents: 0,
    };
    let timeout_ms = (timeout_s.max(0.0) * 1000.0).ceil() as i32;
    let rc = unsafe { libc::poll(&mut pfd, 1, timeout_ms) };
    if rc >= 0 {
        return Ok(rc > 0 && pfd.revents & events != 0);
    }
    let err = io::Error::last_os_error();
    raise_if_cdc_disconnected(err)?;
    Err(CdcError::Io(io::Error::last_os_error()))
}

fn raise_if_cdc_disconnected(err: io::Error) -> Result<(), CdcError> {
    if is_recoverable_cdc_errno(err.raw_os_error()) {
        Err(CdcError::Disconnected(err))
    } else {
        Ok(())
    }
}

fn is_recoverable_cdc_errno(errno: Option<i32>) -> bool {
    matches!(
        errno,
        Some(libc::EBADF)
            | Some(libc::EIO)
            | Some(libc::ENODEV)
            | Some(libc::ENOENT)
            | Some(libc::ENXIO)
            | Some(libc::EPIPE)
    )
}
