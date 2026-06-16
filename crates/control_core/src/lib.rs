//! Control runtime support.

pub mod sender;

pub use sender::{
    DEFAULT_SIM_SOCKET_PATH, Sender, SenderConfig, SenderError, SenderMode, SocketSender,
    UsbSender, build_sender, pack_target_frame,
};
