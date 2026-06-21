//! Locomotion policy runtime configuration, error types, and command source.

use std::fmt;
use std::path::PathBuf;

use se3_command::{Command, CommandSourceKind};
use thiserror::Error;

use crate::cdc::CdcError;
use crate::ort_policy::OrtPolicyError;
use crate::policy_io::{PolicyActionDecoderConfig, PolicyIoError};
use crate::protocol::ProtocolError;
use crate::robot::RobotConfig;
use crate::runtime_constants::DEFAULT_CDC_PORT;

#[derive(Debug, Error)]
pub enum LocomotionPolicyError {
    #[error("{0}")]
    Cdc(#[from] CdcError),
    #[error("{0}")]
    Protocol(#[from] ProtocolError),
    #[error("{0}")]
    OrtPolicy(#[from] OrtPolicyError),
    #[error("{0}")]
    PolicyIo(#[from] PolicyIoError),
    #[error("unsupported checkpoint type: {0}")]
    UnsupportedCheckpoint(PathBuf),
    #[error("sim transport failed to bind client socket {client_socket_path}")]
    SimSocketBind {
        client_socket_path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error(
        "sim transport failed to connect to sim_loop socket {sim_socket_path}; start se3-sim-loop first with the same --socket-path, or pass a matching --sim-socket-path (client socket: {client_socket_path})"
    )]
    SimSocketConnect {
        sim_socket_path: PathBuf,
        client_socket_path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("io failed: {0}")]
    Io(#[from] std::io::Error),
    #[error("json failed: {0}")]
    Json(#[from] serde_json::Error),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LocomotionTransport {
    Cdc,
    Sim,
}

#[derive(Debug, Clone)]
pub struct LocomotionPolicyConfig {
    pub checkpoint: PathBuf,
    pub ort_ep: String,
    pub command_source: CommandSourceKind,
    pub fixed_command: Command,
    pub robot_cfg: RobotConfig,
    pub action_decoder: PolicyActionDecoderConfig,
    pub transport: LocomotionTransport,
    pub port: String,
    pub sim_socket_path: PathBuf,
    pub sim_client_socket_path: PathBuf,
    pub baudrate: i32,
    pub device: String,
    pub max_steps: usize,
    pub dry_run: bool,
    pub print_every: usize,
    pub telemetry_log: Option<PathBuf>,
    pub telemetry_log_every: usize,
    pub telemetry_flush_every: usize,
}

impl Default for LocomotionPolicyConfig {
    fn default() -> Self {
        let robot_cfg = RobotConfig::default();
        Self {
            checkpoint: PathBuf::new(),
            ort_ep: "auto".to_string(),
            command_source: CommandSourceKind::Fixed,
            fixed_command: Command::idle(robot_cfg.default_base_height as f32),
            action_decoder: PolicyActionDecoderConfig {
                robot_cfg: robot_cfg.clone(),
                ..PolicyActionDecoderConfig::default()
            },
            robot_cfg,
            transport: LocomotionTransport::Cdc,
            port: DEFAULT_CDC_PORT.to_string(),
            sim_socket_path: PathBuf::from("/tmp/se3_sim_loop.sock"),
            sim_client_socket_path: PathBuf::from("/tmp/se3_locomotion.sock"),
            baudrate: 921600,
            device: "auto".to_string(),
            max_steps: 0,
            dry_run: false,
            print_every: 50,
            telemetry_log: None,
            telemetry_log_every: 1,
            telemetry_flush_every: 25,
        }
    }
}

#[derive(Debug, Clone)]
pub struct RuntimeStats {
    pub steps: usize,
    pub state_frames: usize,
    pub action_frames: usize,
    pub timeout_frames: usize,
    pub nonfinite_frames: usize,
    pub last_state_seq: u32,
    pub last_action: [f32; 6],
    pub last_target_joint_pos: [f32; 4],
    pub last_target_wheel_vel: [f32; 2],
    pub last_state_joint_pos: [f32; 4],
    pub last_state_target_joint_pos: [f32; 4],
    pub last_state_hip_torque: [f32; 4],
    pub last_state_wheel_torque: [f32; 2],
    pub last_state_wheel_motor_torque: [f32; 2],
    pub policy_inference_frames: usize,
    pub last_policy_inference_ms: f64,
    pub total_policy_inference_ms: f64,
    pub max_policy_inference_ms: f64,
    pub last_step_policy_inference_ms: Option<f64>,
    pub last_action_flags: u32,
    pub last_state_output_enabled: u8,
}

impl Default for RuntimeStats {
    fn default() -> Self {
        Self {
            steps: 0,
            state_frames: 0,
            action_frames: 0,
            timeout_frames: 0,
            nonfinite_frames: 0,
            last_state_seq: 0,
            last_action: [0.0; 6],
            last_target_joint_pos: [0.0; 4],
            last_target_wheel_vel: [0.0; 2],
            last_state_joint_pos: [0.0; 4],
            last_state_target_joint_pos: [0.0; 4],
            last_state_hip_torque: [0.0; 4],
            last_state_wheel_torque: [0.0; 2],
            last_state_wheel_motor_torque: [0.0; 2],
            policy_inference_frames: 0,
            last_policy_inference_ms: 0.0,
            total_policy_inference_ms: 0.0,
            max_policy_inference_ms: 0.0,
            last_step_policy_inference_ms: None,
            last_action_flags: 0,
            last_state_output_enabled: 0,
        }
    }
}

impl fmt::Display for RuntimeStats {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "steps={} state={} action={} timeout={} nonfinite={} inf_ms_avg={:.2}",
            self.steps,
            self.state_frames,
            self.action_frames,
            self.timeout_frames,
            self.nonfinite_frames,
            if self.policy_inference_frames > 0 {
                self.total_policy_inference_ms / self.policy_inference_frames as f64
            } else {
                0.0
            }
        )
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct RuntimeCommandSample {
    pub command: Command,
    pub source: CommandSourceKind,
    pub active: bool,
    pub device_id: Option<usize>,
    pub device_name: Option<String>,
}

impl RuntimeCommandSample {
    pub fn fixed(command: Command) -> Self {
        Self {
            command,
            source: CommandSourceKind::Fixed,
            active: true,
            device_id: None,
            device_name: None,
        }
    }

    pub fn xinput_idle(command: Command) -> Self {
        Self {
            command,
            source: CommandSourceKind::XInput,
            active: false,
            device_id: None,
            device_name: None,
        }
    }
}

pub trait RuntimeCommandSource {
    fn sample(&mut self) -> RuntimeCommandSample;
}

#[derive(Debug)]
pub struct FixedRuntimeCommandSource {
    command: Command,
}

impl FixedRuntimeCommandSource {
    pub fn new(command: Command) -> Self {
        Self { command }
    }
}

impl RuntimeCommandSource for FixedRuntimeCommandSource {
    fn sample(&mut self) -> RuntimeCommandSample {
        RuntimeCommandSample::fixed(self.command)
    }
}
