use std::os::unix::net::UnixDatagram;
use std::path::{Path, PathBuf};

use locomotion_core::cdc::{CdcError, CdcSerial, resolve_cdc_port};
use locomotion_core::protocol::{PolicyTargetFrame, ProtocolError, pack_policy_target};
use thiserror::Error;

pub const DEFAULT_SIM_SOCKET_PATH: &str = "/tmp/se3_sim_loop.sock";

pub trait Sender {
    fn send(&mut self, target: &PolicyTargetFrame) -> Result<(), SenderError>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SenderMode {
    Usb,
    SimSocket,
    Both,
}

#[derive(Debug, Clone)]
pub struct SenderConfig {
    pub mode: SenderMode,
    pub usb_port: String,
    pub baudrate: i32,
    pub write_timeout_s: f64,
    pub socket_path: PathBuf,
}

impl Default for SenderConfig {
    fn default() -> Self {
        Self {
            mode: SenderMode::Usb,
            usb_port: "auto".to_string(),
            baudrate: 921600,
            write_timeout_s: 0.02,
            socket_path: PathBuf::from(DEFAULT_SIM_SOCKET_PATH),
        }
    }
}

#[derive(Debug, Error)]
pub enum SenderError {
    #[error("{0}")]
    Protocol(#[from] ProtocolError),
    #[error("{0}")]
    Cdc(#[from] CdcError),
    #[error("io failed: {0}")]
    Io(#[from] std::io::Error),
}

pub fn pack_target_frame(target: &PolicyTargetFrame) -> Result<Vec<u8>, SenderError> {
    Ok(pack_policy_target(target)?)
}

pub struct UsbSender {
    serial: CdcSerial,
    write_timeout_s: f64,
}

impl UsbSender {
    pub fn new(
        port: impl AsRef<str>,
        baudrate: i32,
        write_timeout_s: f64,
    ) -> Result<Self, SenderError> {
        let mut serial = CdcSerial::new(resolve_cdc_port(port), baudrate);
        serial.open()?;
        Ok(Self {
            serial,
            write_timeout_s,
        })
    }
}

impl Sender for UsbSender {
    fn send(&mut self, target: &PolicyTargetFrame) -> Result<(), SenderError> {
        let packet = pack_target_frame(target)?;
        self.serial.write_all(&packet, self.write_timeout_s)?;
        Ok(())
    }
}

pub struct SocketSender {
    socket: UnixDatagram,
    target_path: PathBuf,
}

impl SocketSender {
    pub fn new(target_path: impl Into<PathBuf>) -> Result<Self, SenderError> {
        Ok(Self {
            socket: UnixDatagram::unbound()?,
            target_path: target_path.into(),
        })
    }

    pub fn target_path(&self) -> &Path {
        &self.target_path
    }
}

impl Sender for SocketSender {
    fn send(&mut self, target: &PolicyTargetFrame) -> Result<(), SenderError> {
        let packet = pack_target_frame(target)?;
        self.socket.send_to(&packet, &self.target_path)?;
        Ok(())
    }
}

struct MultiSender {
    senders: Vec<Box<dyn Sender>>,
}

impl Sender for MultiSender {
    fn send(&mut self, target: &PolicyTargetFrame) -> Result<(), SenderError> {
        for sender in &mut self.senders {
            sender.send(target)?;
        }
        Ok(())
    }
}

pub fn build_sender(config: &SenderConfig) -> Result<Box<dyn Sender>, SenderError> {
    match config.mode {
        SenderMode::Usb => Ok(Box::new(UsbSender::new(
            &config.usb_port,
            config.baudrate,
            config.write_timeout_s,
        )?)),
        SenderMode::SimSocket => Ok(Box::new(SocketSender::new(config.socket_path.clone())?)),
        SenderMode::Both => Ok(Box::new(MultiSender {
            senders: vec![
                Box::new(UsbSender::new(
                    &config.usb_port,
                    config.baudrate,
                    config.write_timeout_s,
                )?),
                Box::new(SocketSender::new(config.socket_path.clone())?),
            ],
        })),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::time::Duration;

    fn sample_target() -> PolicyTargetFrame {
        PolicyTargetFrame {
            seq: 42,
            joint_pos: [1.0, 2.0, 3.0, 4.0],
            wheel_vel: [5.0, 6.0],
        }
    }

    #[test]
    fn sender_packet_uses_policy_target_wire_format() {
        let packet = pack_target_frame(&sample_target()).unwrap();
        let expected = pack_policy_target(&sample_target()).unwrap();
        assert_eq!(packet, expected);
    }

    #[test]
    fn socket_sender_sends_complete_datagram() {
        let path = std::env::temp_dir().join(format!(
            "se3-control-socket-test-{}.sock",
            std::process::id()
        ));
        let _ = fs::remove_file(&path);
        let receiver = UnixDatagram::bind(&path).unwrap();
        receiver
            .set_read_timeout(Some(Duration::from_secs(1)))
            .unwrap();
        let mut sender = SocketSender::new(&path).unwrap();
        let target = sample_target();
        sender.send(&target).unwrap();

        let mut buf = [0_u8; 256];
        let len = receiver.recv(&mut buf).unwrap();
        assert_eq!(&buf[..len], pack_policy_target(&target).unwrap().as_slice());

        drop(receiver);
        let _ = fs::remove_file(path);
    }
}
