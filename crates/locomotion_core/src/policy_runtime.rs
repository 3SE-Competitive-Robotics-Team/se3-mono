use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use std::os::unix::fs::FileTypeExt;
use std::os::unix::net::UnixDatagram;
use std::path::{Path, PathBuf};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use log::{info, warn};
use serde_json::{Map, Value, json};
use sha2::{Digest, Sha256};
use thiserror::Error;

use crate::fourbar::{output_to_policy_pos, output_to_policy_vel, policy_to_output_pos};
use crate::policy_io::{PolicyActionDecoderConfig, PolicyIoError};
use crate::{ObservationConfig, PolicyActionDecoder, RobotConfig};
use se3_command::{Command, CommandSourceKind};

use crate::cdc::{CdcError, CdcSerial, cdc_port_disappeared, resolve_cdc_port};
use crate::command::LocomotionCommand;
use crate::ort_policy::{OrtPolicyError, OrtPolicyRuntime};
use crate::policy_observation::{LocomotionObservationBuilder, synthetic_default_state};
use crate::protocol::{
    MSG_POLICY_STATE, PolicyCommandFrame, PolicyStateFrame, PolicyTargetFrame, ProtocolError,
    StreamParser, decode_policy_state, pack_policy_command, pack_policy_target,
};

// Re-export runtime constants that were historically part of this module.
pub use crate::runtime_constants::{
    ACTION_FLAG_COMMAND_INACTIVE, ACTION_FLAG_DRY_RUN, ACTION_FLAG_NONFINITE,
    ACTION_FLAG_OUTPUT_DISABLED_HOLD, ACTION_FLAG_TIMEOUT, CDC_RECONNECT_DELAY_S, DEFAULT_CDC_PORT,
    LOCOMOTION_POLICY_RATE_HZ, TELEMETRY_SCHEMA,
};
use crate::runtime_constants::{STATE_TIMEOUT_S, WRITE_TIMEOUT_S};

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

#[derive(Debug, Clone, PartialEq)]
pub struct DecodedPolicyTarget {
    pub joint_pos: [f32; 4],
    pub wheel_vel: [f32; 2],
}

#[derive(Debug, Clone)]
pub struct LocomotionActionTargetDecoder {
    pub robot_cfg: RobotConfig,
    pub command_height: f32,
    decoder: PolicyActionDecoder,
}

impl LocomotionActionTargetDecoder {
    pub fn new(command_height: f32, config: PolicyActionDecoderConfig) -> Self {
        let decoder = PolicyActionDecoder::new(config);
        let robot_cfg = decoder.robot_cfg.clone();
        Self {
            robot_cfg,
            command_height,
            decoder,
        }
    }

    pub fn decode(&self, action: [f32; 6]) -> Result<DecodedPolicyTarget, PolicyIoError> {
        let decoded = self
            .decoder
            .decode(action, Some(self.command_height), None, None)?;
        Ok(DecodedPolicyTarget {
            joint_pos: decoded.leg_target,
            wheel_vel: decoded.wheel_vel_target,
        })
    }

    pub fn clip_action(&self, action: [f32; 6]) -> [f32; 6] {
        self.decoder.clip_action(action)
    }
}

pub struct LocomotionPolicyRuntime {
    pub cfg: LocomotionPolicyConfig,
    pub obs_cfg: ObservationConfig,
    policy: LoadedPolicy,
    obs_builder: LocomotionObservationBuilder,
    target_decoder: LocomotionActionTargetDecoder,
    pub stats: RuntimeStats,
    last_action: [f32; 6],
    action_seq: u32,
    policy_memory_clean: bool,
    session_id: String,
    reset_id: usize,
    reconnect_count: usize,
    resolved_port: Option<String>,
    sim_fourbar_surrogate: bool,
    last_sample_monotonic: Option<Instant>,
    last_policy_reset_reason: String,
    checkpoint_sha256: Option<String>,
    start_monotonic: Instant,
    last_print_monotonic: Instant,
    last_print_steps: usize,
    telemetry: TelemetryLogger,
    command_source: Box<dyn RuntimeCommandSource>,
    last_command_sample: RuntimeCommandSample,
}

impl LocomotionPolicyRuntime {
    pub fn new(cfg: LocomotionPolicyConfig) -> Result<Self, LocomotionPolicyError> {
        let command_source = Box::new(FixedRuntimeCommandSource::new(cfg.fixed_command));
        Self::new_with_command_source(cfg, command_source)
    }

    pub fn new_with_command_source(
        cfg: LocomotionPolicyConfig,
        command_source: Box<dyn RuntimeCommandSource>,
    ) -> Result<Self, LocomotionPolicyError> {
        let obs_cfg = ObservationConfig::default();
        let mut policy = load_policy_runtime(&cfg.checkpoint, &cfg.ort_ep)?;
        policy.reset();
        let initial_command = LocomotionCommand::from_command(cfg.fixed_command, &cfg.robot_cfg);
        let mut obs_builder =
            LocomotionObservationBuilder::with_robot_config(cfg.robot_cfg.clone())
                .with_num_obs(policy.num_obs());
        obs_builder.set_command(initial_command);
        let target_decoder = LocomotionActionTargetDecoder::new(
            obs_builder.policy_command()[4],
            cfg.action_decoder.clone(),
        );
        let now = Instant::now();
        let session_id = format!("{}_{}", unix_time_s() as u64, std::process::id());
        let checkpoint_sha256 = sha256_file(&cfg.checkpoint)?;
        let mut telemetry = TelemetryLogger::new(
            cfg.telemetry_log.clone(),
            cfg.telemetry_log_every,
            cfg.telemetry_flush_every,
        )?;
        telemetry.write_meta(&json!({
            "schema": TELEMETRY_SCHEMA,
            "record_type": "meta",
            "session_id": session_id,
            "created_wall_time_s": unix_time_s(),
            "runtime_config": runtime_config_json(&cfg),
            "checkpoint": {
                "path": cfg.checkpoint.to_string_lossy(),
                "sha256": checkpoint_sha256,
                "iteration": policy.iteration(),
                "policy_type": policy.policy_type(),
                "num_obs": policy.num_obs(),
                "num_actions": policy.num_actions(),
                "rnn_type": policy.rnn_type(),
                "rnn_hidden_dim": policy.rnn_hidden_dim(),
                "rnn_num_layers": policy.rnn_num_layers(),
                "activation": policy.activation(),
                "ort_ep": policy.execution_provider(),
            },
            "observation_config": {
                "ang_vel_scale": obs_cfg.ang_vel_scale,
                "command_scale": obs_cfg.command_scale,
                "leg_vel_scale": obs_cfg.leg_vel_scale,
                "wheel_vel_scale": obs_cfg.wheel_vel_scale,
                "clip_value": obs_cfg.clip_value,
                "num_obs": obs_cfg.num_obs,
                "num_actions": obs_cfg.num_actions,
            },
            "command": obs_builder.policy_command(),
            "command_source": cfg.command_source.as_str(),
            "action_decoder": action_decoder_config_json(&cfg.action_decoder, target_decoder.command_height)
        }))?;
        let last_command_sample = RuntimeCommandSample::fixed(cfg.fixed_command);
        let mut runtime = Self {
            cfg,
            obs_cfg,
            policy,
            obs_builder,
            target_decoder,
            stats: RuntimeStats::default(),
            last_action: [0.0; 6],
            action_seq: 0,
            policy_memory_clean: true,
            session_id,
            reset_id: 0,
            reconnect_count: 0,
            resolved_port: None,
            sim_fourbar_surrogate: false,
            last_sample_monotonic: None,
            last_policy_reset_reason: "startup".to_string(),
            checkpoint_sha256,
            start_monotonic: now,
            last_print_monotonic: now,
            last_print_steps: 0,
            telemetry,
            command_source,
            last_command_sample,
        };
        runtime.write_event("runtime_start", json!({}))?;
        Ok(runtime)
    }

    pub fn run(&mut self) -> Result<RuntimeStats, LocomotionPolicyError> {
        info!(
            "Locomotion policy runtime: checkpoint={} iter={} type={} device={}",
            self.cfg.checkpoint.display(),
            self.policy.iteration(),
            self.policy.policy_type(),
            self.policy.execution_provider()
        );
        let result = if self.cfg.dry_run {
            self.run_dry()
        } else if self.cfg.transport == LocomotionTransport::Sim {
            self.run_sim()
        } else {
            self.run_cdc()
        };
        self.write_event("runtime_stop", json!({ "total_steps": self.stats.steps }))?;
        self.telemetry.close()?;
        result.map(|_| self.stats.clone())
    }

    fn run_dry(&mut self) -> Result<(), LocomotionPolicyError> {
        let max_steps = if self.cfg.max_steps > 0 {
            self.cfg.max_steps
        } else {
            (LOCOMOTION_POLICY_RATE_HZ * 2.0) as usize
        };
        for step in 0..max_steps {
            let loop_started = Instant::now();
            let state = synthetic_default_state(step as u32);
            self.stats.state_frames += 1;
            self.sample_command();
            let (action, flags, obs, policy_inference_ms) = if self.last_command_sample.active {
                let (action, flags, obs, policy_inference_ms) = self.act_from_state(&state)?;
                self.decode_target(action)?;
                (
                    action,
                    flags | ACTION_FLAG_DRY_RUN,
                    obs,
                    Some(policy_inference_ms),
                )
            } else {
                self.reset_policy_memory(false, "command_inactive")?;
                let (obs, obs_flags) = self.build_observation(&state)?;
                (
                    [0.0; 6],
                    obs_flags | ACTION_FLAG_DRY_RUN | ACTION_FLAG_COMMAND_INACTIVE,
                    obs,
                    None,
                )
            };
            self.record_action(&state, action, flags, policy_inference_ms);
            self.write_telemetry(
                &state,
                &obs,
                &action,
                flags,
                TelemetryTiming {
                    policy_inference_ms,
                    state_age_s: Some(0.0),
                    loop_started,
                    write_ms: None,
                },
            )?;
            self.maybe_print();
        }
        Ok(())
    }

    fn run_sim(&mut self) -> Result<(), LocomotionPolicyError> {
        let max_steps = self.cfg.max_steps;
        let period = Duration::from_secs_f64(1.0 / LOCOMOTION_POLICY_RATE_HZ);
        let period_s = period.as_secs_f64();
        self.sim_fourbar_surrogate = true;
        let state_timeout = Duration::from_secs_f64(STATE_TIMEOUT_S);
        self.reset_policy_memory(true, "sim_connect")?;
        unlink_socket_path_if_present(&self.cfg.sim_client_socket_path)?;
        let client_socket_guard = SocketPathGuard::new(self.cfg.sim_client_socket_path.clone());
        let socket = match UnixDatagram::bind(&self.cfg.sim_client_socket_path) {
            Ok(socket) => socket,
            Err(err) => {
                return Err(LocomotionPolicyError::SimSocketBind {
                    client_socket_path: self.cfg.sim_client_socket_path.clone(),
                    source: err,
                });
            }
        };
        if let Err(err) = socket.connect(&self.cfg.sim_socket_path) {
            return Err(LocomotionPolicyError::SimSocketConnect {
                sim_socket_path: self.cfg.sim_socket_path.clone(),
                client_socket_path: self.cfg.sim_client_socket_path.clone(),
                source: err,
            });
        }
        socket.set_read_timeout(Some(state_timeout))?;
        self.resolved_port = Some(self.cfg.sim_socket_path.to_string_lossy().into_owned());
        self.write_event(
            "sim_open",
            json!({
                "sim_socket_path": self.cfg.sim_socket_path,
                "sim_client_socket_path": self.cfg.sim_client_socket_path,
            }),
        )?;
        let handshake_state = synthetic_default_state(0);
        let handshake = self.make_hold_target_packet(&handshake_state)?;
        socket.send(&handshake)?;
        let mut parser = StreamParser::default();
        let mut buf = [0_u8; 4096];
        let mut latest_state: Option<PolicyStateFrame> = None;
        let mut latest_state_time = Instant::now();
        let mut next_tick = Instant::now() + period;
        while max_steps == 0 || self.stats.steps < max_steps {
            let loop_started = Instant::now();
            let now = Instant::now();
            if now >= next_tick {
                next_tick += period;
                let Some(state) = latest_state.as_ref() else {
                    self.stats.steps += 1;
                    self.stats.timeout_frames += 1;
                    self.maybe_print();
                    continue;
                };
                let age_s = now
                    .saturating_duration_since(latest_state_time)
                    .as_secs_f64();
                let StepOutput {
                    action,
                    flags,
                    obs,
                    policy_inference_ms,
                    packet,
                    command_packet,
                } = self.step_from_state(state, age_s)?;
                let write_started = Instant::now();
                write_step_output_packets(
                    command_packet.as_deref(),
                    packet.as_deref(),
                    |packet| socket.send(packet).map(|_| ()),
                )?;
                let write_ms = write_started.elapsed().as_secs_f64() * 1000.0;
                self.record_action(state, action, flags, policy_inference_ms);
                self.write_telemetry(
                    state,
                    &obs,
                    &action,
                    flags,
                    TelemetryTiming {
                        policy_inference_ms,
                        state_age_s: Some(age_s),
                        loop_started,
                        write_ms: Some(write_ms),
                    },
                )?;
                self.maybe_print();
                continue;
            }
            let wait_s = (next_tick.saturating_duration_since(now).as_secs_f64())
                .min(period_s)
                .max(0.0);
            socket.set_read_timeout(Some(Duration::from_secs_f64(wait_s)))?;
            let n = match socket.recv(&mut buf) {
                Ok(n) => n,
                Err(err)
                    if matches!(
                        err.kind(),
                        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                    ) =>
                {
                    0
                }
                Err(err) => return Err(err.into()),
            };
            if n > 0 {
                (latest_state, latest_state_time) =
                    self.read_sim_states(&mut parser, &buf[..n], latest_state, latest_state_time)?;
            }
        }
        client_socket_guard.cleanup()?;
        Ok(())
    }

    fn read_sim_states(
        &mut self,
        parser: &mut StreamParser,
        data: &[u8],
        mut latest_state: Option<PolicyStateFrame>,
        mut latest_state_time: Instant,
    ) -> Result<(Option<PolicyStateFrame>, Instant), LocomotionPolicyError> {
        for message in parser.feed(data) {
            if message.msg_type != MSG_POLICY_STATE {
                continue;
            }
            let mut state = decode_policy_state(&message)?;
            if self.sim_fourbar_surrogate {
                state = sim_output_state_to_policy_state(state);
            }
            latest_state_time = Instant::now();
            self.stats.state_frames += 1;
            self.stats.last_state_seq = state.seq;
            latest_state = Some(state);
        }
        Ok((latest_state, latest_state_time))
    }

    fn run_cdc(&mut self) -> Result<(), LocomotionPolicyError> {
        let max_steps = self.cfg.max_steps;
        let period = Duration::from_secs_f64(1.0 / LOCOMOTION_POLICY_RATE_HZ);
        let period_s = period.as_secs_f64();
        self.sim_fourbar_surrogate = false;

        while max_steps == 0 || self.stats.steps < max_steps {
            self.reset_policy_memory(true, "cdc_connect")?;
            let mut parser = StreamParser::default();
            let mut latest_state: Option<PolicyStateFrame> = None;
            let mut latest_state_time = Instant::now();
            let mut next_tick = Instant::now();
            let port = resolve_cdc_port(&self.cfg.port);
            self.resolved_port = Some(port.clone());
            let loop_result = (|| -> Result<(), LocomotionPolicyError> {
                let mut serial = CdcSerial::new(&port, self.cfg.baudrate);
                serial.open()?;
                info!("USB CDC open: port={port} baudrate={}", self.cfg.baudrate);
                self.write_event(
                    "cdc_open",
                    json!({ "port": port, "reconnect_count": self.reconnect_count }),
                )?;
                while max_steps == 0 || self.stats.steps < max_steps {
                    let loop_started = Instant::now();
                    let now = Instant::now();
                    let wait_s = (next_tick.saturating_duration_since(now).as_secs_f64())
                        .min(period_s)
                        .max(0.0);
                    if serial.wait_readable(wait_s)? {
                        (latest_state, latest_state_time) = self.read_states(
                            &mut serial,
                            &mut parser,
                            latest_state,
                            latest_state_time,
                        )?;
                    }
                    if cdc_port_disappeared(&port) {
                        return Err(CdcError::Disconnected(std::io::Error::new(
                            std::io::ErrorKind::NotFound,
                            format!("USB CDC port disappeared: {port}"),
                        ))
                        .into());
                    }

                    let now = Instant::now();
                    if now < next_tick {
                        continue;
                    }
                    next_tick += period;
                    let Some(state) = latest_state.as_ref() else {
                        self.stats.steps += 1;
                        self.stats.timeout_frames += 1;
                        self.maybe_print();
                        continue;
                    };
                    let age_s = now
                        .saturating_duration_since(latest_state_time)
                        .as_secs_f64();
                    let StepOutput {
                        action,
                        flags,
                        obs,
                        policy_inference_ms,
                        packet,
                        command_packet,
                    } = self.step_from_state(state, age_s)?;
                    let write_started = Instant::now();
                    write_step_output_packets(
                        command_packet.as_deref(),
                        packet.as_deref(),
                        |packet| serial.write_all(packet, WRITE_TIMEOUT_S),
                    )?;
                    let write_ms = write_started.elapsed().as_secs_f64() * 1000.0;
                    self.record_action(state, action, flags, policy_inference_ms);
                    self.write_telemetry(
                        state,
                        &obs,
                        &action,
                        flags,
                        TelemetryTiming {
                            policy_inference_ms,
                            state_age_s: Some(age_s),
                            loop_started,
                            write_ms: Some(write_ms),
                        },
                    )?;
                    self.maybe_print();
                }
                Ok(())
            })();

            match loop_result {
                Ok(()) => return Ok(()),
                Err(LocomotionPolicyError::Cdc(
                    err @ (CdcError::Disconnected(_) | CdcError::WriteTimeout(_)),
                )) => {
                    let error_text = err.to_string();
                    self.reconnect_count += 1;
                    self.reset_policy_memory(true, "cdc_disconnected")?;
                    self.write_event(
                        "cdc_disconnected",
                        json!({
                            "port": port,
                            "reconnect_count": self.reconnect_count,
                            "error": error_text,
                        }),
                    )?;
                    warn!(
                        "USB CDC disconnected: port={port}; reconnecting in {CDC_RECONNECT_DELAY_S:.1}s"
                    );
                    thread::sleep(Duration::from_secs_f64(CDC_RECONNECT_DELAY_S));
                }
                Err(err) => return Err(err),
            }
        }
        Ok(())
    }

    fn read_states(
        &mut self,
        serial: &mut CdcSerial,
        parser: &mut StreamParser,
        mut latest_state: Option<PolicyStateFrame>,
        mut latest_state_time: Instant,
    ) -> Result<(Option<PolicyStateFrame>, Instant), LocomotionPolicyError> {
        loop {
            let data = serial.read_available()?;
            if data.is_empty() {
                return Ok((latest_state, latest_state_time));
            }
            for message in parser.feed(&data) {
                if message.msg_type != MSG_POLICY_STATE {
                    continue;
                }
                let state = decode_policy_state(&message)?;
                latest_state_time = Instant::now();
                self.stats.state_frames += 1;
                self.stats.last_state_seq = state.seq;
                latest_state = Some(state);
            }
        }
    }

    fn build_observation(
        &self,
        state: &PolicyStateFrame,
    ) -> Result<(Vec<f32>, u32), LocomotionPolicyError> {
        let result = self.obs_builder.build(state, self.last_action)?;
        let flags = if result.had_nonfinite_input {
            ACTION_FLAG_NONFINITE
        } else {
            0
        };
        Ok((result.obs, flags))
    }

    fn step_from_state(
        &mut self,
        state: &PolicyStateFrame,
        age_s: f64,
    ) -> Result<StepOutput, LocomotionPolicyError> {
        let mut flags = 0_u32;
        let policy_inference_ms;
        let obs;
        let action;
        let packet;
        let command_packet;
        self.sample_command();
        if !self.last_command_sample.active {
            self.reset_policy_memory(false, "command_inactive")?;
            let (built_obs, obs_flags) = self.build_observation(state)?;
            obs = built_obs;
            action = [0.0; 6];
            flags |= obs_flags | ACTION_FLAG_COMMAND_INACTIVE;
            if obs_flags & ACTION_FLAG_NONFINITE != 0 {
                self.stats.nonfinite_frames += 1;
            }
            policy_inference_ms = None;
            packet = Some(self.make_hold_target_packet(state)?);
            command_packet = Some(self.make_command_packet()?);
        } else if age_s > STATE_TIMEOUT_S {
            self.reset_policy_memory(false, "state_timeout")?;
            action = [0.0; 6];
            flags |= ACTION_FLAG_TIMEOUT;
            let (built_obs, obs_flags) = self.build_observation(state)?;
            obs = built_obs;
            flags |= obs_flags;
            if obs_flags & ACTION_FLAG_NONFINITE != 0 {
                self.stats.nonfinite_frames += 1;
            }
            policy_inference_ms = None;
            packet = Some(self.make_hold_target_packet(state)?);
            command_packet = Some(self.make_command_packet()?);
        } else if state.output_enabled == 0 {
            self.reset_policy_memory(false, "output_disabled")?;
            let (built_obs, obs_flags) = self.build_observation(state)?;
            obs = built_obs;
            action = [0.0; 6];
            flags |= obs_flags | ACTION_FLAG_OUTPUT_DISABLED_HOLD;
            if obs_flags & ACTION_FLAG_NONFINITE != 0 {
                self.stats.nonfinite_frames += 1;
            }
            policy_inference_ms = None;
            packet = Some(self.make_hold_target_packet(state)?);
            command_packet = Some(self.make_command_packet()?);
        } else {
            let result = self.act_from_state(state)?;
            action = result.0;
            flags = result.1;
            obs = result.2;
            policy_inference_ms = Some(result.3);
            packet = Some(self.make_target_packet(action)?);
            command_packet = Some(self.make_command_packet()?);
        }
        Ok(StepOutput {
            action,
            flags,
            obs,
            policy_inference_ms,
            packet,
            command_packet,
        })
    }

    fn sample_command(&mut self) {
        let sample = self.command_source.sample();
        let command =
            LocomotionCommand::from_command(sample.command, &self.target_decoder.robot_cfg);
        self.target_decoder.command_height = command.chassis.height_m;
        self.obs_builder.set_command(command);
        self.last_command_sample = sample;
    }

    fn act_from_state(
        &mut self,
        state: &PolicyStateFrame,
    ) -> Result<([f32; 6], u32, Vec<f32>, f64), LocomotionPolicyError> {
        let (obs, mut flags) = self.build_observation(state)?;
        let started = Instant::now();
        let action_vec = self.policy.act(&obs)?;
        let policy_inference_ms = started.elapsed().as_secs_f64() * 1000.0;
        self.record_policy_inference(policy_inference_ms);
        let mut action = [0.0_f32; 6];
        for (dst, src) in action.iter_mut().zip(action_vec) {
            if src.is_finite() {
                *dst = src;
            } else {
                flags |= ACTION_FLAG_NONFINITE;
                *dst = 0.0;
            }
        }
        if flags & ACTION_FLAG_NONFINITE != 0 {
            self.stats.nonfinite_frames += 1;
        }
        self.policy_memory_clean = false;
        Ok((action, flags, obs, policy_inference_ms))
    }

    fn record_policy_inference(&mut self, policy_inference_ms: f64) {
        self.stats.policy_inference_frames += 1;
        self.stats.last_policy_inference_ms = policy_inference_ms;
        self.stats.total_policy_inference_ms += policy_inference_ms;
        self.stats.max_policy_inference_ms =
            self.stats.max_policy_inference_ms.max(policy_inference_ms);
    }

    fn avg_policy_inference_ms(&self) -> f64 {
        if self.stats.policy_inference_frames == 0 {
            0.0
        } else {
            self.stats.total_policy_inference_ms / self.stats.policy_inference_frames as f64
        }
    }

    fn reset_policy_memory(
        &mut self,
        force: bool,
        reason: &str,
    ) -> Result<(), LocomotionPolicyError> {
        if self.policy_memory_clean && !force {
            return Ok(());
        }
        self.policy.reset();
        self.last_action = [0.0; 6];
        self.stats.last_action = self.last_action;
        self.stats.last_step_policy_inference_ms = None;
        self.policy_memory_clean = true;
        self.reset_id += 1;
        self.last_policy_reset_reason = reason.to_string();
        self.write_event(
            "policy_reset",
            json!({ "reset_id": self.reset_id, "reason": reason }),
        )?;
        Ok(())
    }

    fn make_target_packet(&mut self, action: [f32; 6]) -> Result<Vec<u8>, LocomotionPolicyError> {
        let target = self.decode_target(action)?;
        let joint_pos = self.target_joint_pos_for_transport(target.joint_pos);
        let frame = PolicyTargetFrame {
            seq: self.action_seq,
            joint_pos,
            wheel_vel: target.wheel_vel,
        };
        self.action_seq = self.action_seq.wrapping_add(1);
        Ok(pack_policy_target(&frame)?)
    }

    fn make_hold_target_packet(
        &mut self,
        state: &PolicyStateFrame,
    ) -> Result<Vec<u8>, LocomotionPolicyError> {
        let joint_pos = state.joint_pos;
        let wheel_vel = [0.0, 0.0];
        self.stats.last_target_joint_pos = joint_pos;
        self.stats.last_target_wheel_vel = wheel_vel;
        let transport_joint_pos = self.target_joint_pos_for_transport(joint_pos);
        let frame = PolicyTargetFrame {
            seq: self.action_seq,
            joint_pos: transport_joint_pos,
            wheel_vel,
        };
        self.action_seq = self.action_seq.wrapping_add(1);
        Ok(pack_policy_target(&frame)?)
    }

    fn make_command_packet(&self) -> Result<Vec<u8>, LocomotionPolicyError> {
        let frame = PolicyCommandFrame {
            seq: self.action_seq.wrapping_sub(1),
            command: self.obs_builder.policy_command(),
        };
        Ok(pack_policy_command(&frame)?)
    }

    fn target_joint_pos_for_transport(&self, policy_joint_pos: [f32; 4]) -> [f32; 4] {
        if self.sim_fourbar_surrogate {
            policy_to_output_pos(policy_joint_pos.map(|v| v as f64)).map(|v| v as f32)
        } else {
            policy_joint_pos
        }
    }

    fn decode_target(
        &mut self,
        action: [f32; 6],
    ) -> Result<DecodedPolicyTarget, LocomotionPolicyError> {
        let target = self.target_decoder.decode(action)?;
        self.stats.last_target_joint_pos = target.joint_pos;
        self.stats.last_target_wheel_vel = target.wheel_vel;
        Ok(target)
    }

    fn record_action(
        &mut self,
        state: &PolicyStateFrame,
        action: [f32; 6],
        flags: u32,
        policy_inference_ms: Option<f64>,
    ) {
        self.stats.steps += 1;
        self.stats.action_frames += 1;
        self.stats.last_state_seq = state.seq;
        self.stats.last_action_flags = flags;
        self.stats.last_state_output_enabled = state.output_enabled;
        self.stats.last_step_policy_inference_ms = policy_inference_ms;
        self.last_action = self.target_decoder.clip_action(action);
        self.stats.last_action = self.last_action;
        self.stats.last_state_joint_pos = state.joint_pos;
        self.stats.last_state_target_joint_pos = state.target_joint_pos;
        self.stats.last_state_hip_torque = state.hip_torque;
        self.stats.last_state_wheel_torque = state.wheel_torque;
        self.stats.last_state_wheel_motor_torque = state.wheel_motor_torque;
        if flags & ACTION_FLAG_TIMEOUT != 0 {
            self.stats.timeout_frames += 1;
        }
    }

    fn write_telemetry(
        &mut self,
        state: &PolicyStateFrame,
        obs: &[f32],
        action: &[f32; 6],
        flags: u32,
        timing: TelemetryTiming,
    ) -> Result<(), LocomotionPolicyError> {
        let now = Instant::now();
        let loop_dt_ms = self
            .last_sample_monotonic
            .map(|last| now.saturating_duration_since(last).as_secs_f64() * 1000.0);
        self.last_sample_monotonic = Some(now);
        let nx_target = self.stats.last_target_joint_pos;
        let stm_target = state.target_joint_pos;
        let joint_pos = state.joint_pos;
        let mut record = Map::new();
        record.insert("schema".to_string(), json!(TELEMETRY_SCHEMA));
        record.insert("record_type".to_string(), json!("sample"));
        record.insert("session_id".to_string(), json!(self.session_id));
        record.insert("reset_id".to_string(), json!(self.reset_id));
        record.insert("reconnect_count".to_string(), json!(self.reconnect_count));
        record.insert("resolved_port".to_string(), json!(self.resolved_port));
        record.insert("wall_time_s".to_string(), json!(unix_time_s()));
        record.insert(
            "runtime_uptime_s".to_string(),
            json!(self.start_monotonic.elapsed().as_secs_f64()),
        );
        record.insert(
            "loop_work_ms".to_string(),
            json!(timing.loop_started.elapsed().as_secs_f64() * 1000.0),
        );
        record.insert("loop_dt_ms".to_string(), json!(loop_dt_ms));
        record.insert(
            "state_age_ms_nx".to_string(),
            json!(timing.state_age_s.map(|v| v * 1000.0)),
        );
        record.insert("write_ms".to_string(), json!(timing.write_ms));
        record.insert("rate_hz".to_string(), json!(LOCOMOTION_POLICY_RATE_HZ));
        record.insert(
            "sample_period_ms".to_string(),
            json!(1000.0 / LOCOMOTION_POLICY_RATE_HZ),
        );
        record.insert("step".to_string(), json!(self.stats.steps));
        record.insert("state_seq".to_string(), json!(state.seq));
        record.insert("tick_ms".to_string(), json!(state.tick_ms));
        record.insert("target_seq".to_string(), json!(state.target_seq));
        record.insert("target_age_ms".to_string(), json!(state.target_age_ms));
        record.insert("target_valid".to_string(), json!(state.target_valid));
        record.insert("rc_switch_r".to_string(), json!(state.rc_switch_r));
        record.insert("output_enabled".to_string(), json!(state.output_enabled));
        record.insert("flags".to_string(), json!(flags));
        record.insert("flag_names".to_string(), json!(action_flag_names(flags)));
        record.insert(
            "policy_inference_ms".to_string(),
            json!(timing.policy_inference_ms),
        );
        record.insert(
            "policy_inference_ms_last".to_string(),
            json!(self.stats.last_policy_inference_ms),
        );
        record.insert(
            "policy_inference_ms_avg".to_string(),
            json!(self.avg_policy_inference_ms()),
        );
        record.insert(
            "policy_inference_ms_max".to_string(),
            json!(self.stats.max_policy_inference_ms),
        );
        record.insert(
            "policy_inference_frames".to_string(),
            json!(self.stats.policy_inference_frames),
        );
        record.insert("target_mode".to_string(), json!(target_mode_name(flags)));
        record.insert(
            "last_policy_reset_reason".to_string(),
            json!(self.last_policy_reset_reason),
        );
        record.insert(
            "command".to_string(),
            json!(self.obs_builder.policy_command()),
        );
        record.insert(
            "command_source".to_string(),
            json!(self.last_command_sample.source.as_str()),
        );
        record.insert(
            "command_active".to_string(),
            json!(self.last_command_sample.active),
        );
        record.insert(
            "command_device_id".to_string(),
            json!(self.last_command_sample.device_id),
        );
        record.insert(
            "command_device_name".to_string(),
            json!(self.last_command_sample.device_name),
        );
        record.insert(
            "checkpoint_sha256".to_string(),
            json!(self.checkpoint_sha256),
        );
        record.insert("obs".to_string(), json!(obs));
        record.insert("action".to_string(), json!(action));
        record.insert("clipped_action".to_string(), json!(self.last_action));
        record.insert("nx_target_joint_pos".to_string(), json!(nx_target));
        record.insert(
            "nx_target_active".to_string(),
            json!(policy_active_angles(nx_target)),
        );
        record.insert(
            "nx_target_wheel_vel".to_string(),
            json!(self.stats.last_target_wheel_vel),
        );
        record.insert("stm_target_joint_pos".to_string(), json!(stm_target));
        record.insert(
            "stm_target_active".to_string(),
            json!(policy_active_angles(stm_target)),
        );
        record.insert("joint_pos".to_string(), json!(joint_pos));
        record.insert("joint_vel".to_string(), json!(state.joint_vel));
        record.insert(
            "joint_active".to_string(),
            json!(policy_active_angles(joint_pos)),
        );
        record.insert(
            "joint_pos_error_nx_target".to_string(),
            json!(wrap_angle_vec4(sub4(nx_target, joint_pos))),
        );
        record.insert(
            "joint_pos_error_stm_target".to_string(),
            json!(wrap_angle_vec4(sub4(stm_target, joint_pos))),
        );
        record.insert("wheel_pos".to_string(), json!(state.wheel_pos));
        record.insert("wheel_vel".to_string(), json!(state.wheel_vel));
        record.insert("hip_torque".to_string(), json!(state.hip_torque));
        record.insert("wheel_torque".to_string(), json!(state.wheel_torque));
        record.insert(
            "wheel_motor_torque".to_string(),
            json!(state.wheel_motor_torque),
        );
        record.insert(
            "base_ang_vel_body".to_string(),
            json!(state.base_ang_vel_body),
        );
        record.insert(
            "projected_gravity".to_string(),
            json!(state.projected_gravity),
        );
        self.telemetry
            .write(self.stats.steps, &Value::Object(record))?;
        Ok(())
    }

    fn write_event(
        &mut self,
        event: &str,
        fields: serde_json::Value,
    ) -> Result<(), LocomotionPolicyError> {
        let mut record = json!({
            "schema": TELEMETRY_SCHEMA,
            "record_type": "event",
            "event": event,
            "session_id": self.session_id,
            "reset_id": self.reset_id,
            "reconnect_count": self.reconnect_count,
            "wall_time_s": unix_time_s(),
            "runtime_uptime_s": self.start_monotonic.elapsed().as_secs_f64(),
            "step": self.stats.steps,
            "state_seq": self.stats.last_state_seq,
            "resolved_port": self.resolved_port,
        });
        if let (Some(dst), Some(src)) = (record.as_object_mut(), fields.as_object()) {
            for (key, value) in src {
                dst.insert(key.clone(), value.clone());
            }
        }
        self.telemetry.write_event(&record)?;
        Ok(())
    }

    fn maybe_print(&mut self) {
        if self.cfg.print_every == 0 || !self.stats.steps.is_multiple_of(self.cfg.print_every) {
            return;
        }
        let now = Instant::now();
        let interval_s = now
            .saturating_duration_since(self.last_print_monotonic)
            .as_secs_f64()
            .max(1.0e-3);
        let total_s = self.start_monotonic.elapsed().as_secs_f64().max(1.0e-3);
        let recent_fps = (self.stats.steps - self.last_print_steps) as f64 / interval_s;
        let avg_fps = self.stats.steps as f64 / total_s;
        self.last_print_monotonic = now;
        self.last_print_steps = self.stats.steps;
        let policy_ms = self
            .stats
            .last_step_policy_inference_ms
            .map(|v| format!("{v:.3}"))
            .unwrap_or_else(|| "--".to_string());
        let flag_names = action_flag_names(self.stats.last_action_flags);
        let flags_text = if flag_names.is_empty() {
            "none".to_string()
        } else {
            flag_names.join(",")
        };
        let target_mode = target_mode_name(self.stats.last_action_flags);
        info!(
            "step={} states={} actions={} last_state={} timeouts={} nonfinite={} mode={} output={} flags={} fps={:.1}/{:.1} policy_ms={}/{:.3}/{:.3} policy_n={} action4=[{}] target4=[{}] stm_target4=[{}] joint4=[{}] err4=[{}] torque4=[{}] wheel_motor_torque=[{}]",
            self.stats.steps,
            self.stats.state_frames,
            self.stats.action_frames,
            self.stats.last_state_seq,
            self.stats.timeout_frames,
            self.stats.nonfinite_frames,
            target_mode,
            self.stats.last_state_output_enabled,
            flags_text,
            recent_fps,
            avg_fps,
            policy_ms,
            self.avg_policy_inference_ms(),
            self.stats.max_policy_inference_ms,
            self.stats.policy_inference_frames,
            fmt_values(&self.stats.last_action[..4]),
            fmt_values(&self.stats.last_target_joint_pos),
            fmt_values(&self.stats.last_state_target_joint_pos),
            fmt_values(&self.stats.last_state_joint_pos),
            fmt_values(&wrap_angle_vec4(sub4(
                self.stats.last_state_target_joint_pos,
                self.stats.last_state_joint_pos
            ))),
            fmt_values(&self.stats.last_state_hip_torque),
            fmt_values(&self.stats.last_state_wheel_motor_torque),
        );
    }
}

fn write_step_output_packets<F, E>(
    command_packet: Option<&[u8]>,
    packet: Option<&[u8]>,
    mut write: F,
) -> Result<(), E>
where
    F: FnMut(&[u8]) -> Result<(), E>,
{
    if let Some(command_packet) = command_packet {
        write(command_packet)?;
    }
    if let Some(packet) = packet {
        write(packet)?;
    }
    Ok(())
}

pub fn load_policy_runtime(
    checkpoint: &Path,
    ort_ep: &str,
) -> Result<LoadedPolicy, LocomotionPolicyError> {
    match checkpoint.extension().and_then(|value| value.to_str()) {
        Some("onnx") => Ok(LoadedPolicy::Ort(Box::new(OrtPolicyRuntime::new(
            checkpoint, ort_ep,
        )?))),
        _ => Err(LocomotionPolicyError::UnsupportedCheckpoint(
            checkpoint.to_path_buf(),
        )),
    }
}

struct TelemetryTiming {
    policy_inference_ms: Option<f64>,
    state_age_s: Option<f64>,
    loop_started: Instant,
    write_ms: Option<f64>,
}

struct StepOutput {
    action: [f32; 6],
    flags: u32,
    obs: Vec<f32>,
    policy_inference_ms: Option<f64>,
    packet: Option<Vec<u8>>,
    command_packet: Option<Vec<u8>>,
}

pub enum LoadedPolicy {
    Ort(Box<OrtPolicyRuntime>),
    #[cfg(test)]
    Noop,
}

impl LoadedPolicy {
    fn reset(&mut self) {
        match self {
            Self::Ort(policy) => policy.reset(),
            #[cfg(test)]
            Self::Noop => {}
        }
    }

    fn act(&mut self, obs: &[f32]) -> Result<Vec<f32>, LocomotionPolicyError> {
        match self {
            Self::Ort(policy) => Ok(policy.act(obs)?),
            #[cfg(test)]
            Self::Noop => Ok(vec![0.0; 6]),
        }
    }

    fn iteration(&self) -> &str {
        match self {
            Self::Ort(policy) => &policy.iteration,
            #[cfg(test)]
            Self::Noop => "test",
        }
    }

    fn policy_type(&self) -> String {
        match self {
            Self::Ort(policy) => policy.policy_type(),
            #[cfg(test)]
            Self::Noop => "test".to_string(),
        }
    }

    fn num_obs(&self) -> usize {
        match self {
            Self::Ort(policy) => policy.num_obs,
            #[cfg(test)]
            Self::Noop => ObservationConfig::default().num_obs,
        }
    }

    fn num_actions(&self) -> usize {
        match self {
            Self::Ort(policy) => policy.num_actions,
            #[cfg(test)]
            Self::Noop => ObservationConfig::default().num_actions,
        }
    }

    fn rnn_type(&self) -> &str {
        match self {
            Self::Ort(policy) => &policy.rnn_type,
            #[cfg(test)]
            Self::Noop => "none",
        }
    }

    fn rnn_hidden_dim(&self) -> usize {
        match self {
            Self::Ort(policy) => policy.rnn_hidden_dim,
            #[cfg(test)]
            Self::Noop => 0,
        }
    }

    fn rnn_num_layers(&self) -> usize {
        match self {
            Self::Ort(policy) => policy.rnn_num_layers,
            #[cfg(test)]
            Self::Noop => 0,
        }
    }

    fn activation(&self) -> &str {
        match self {
            Self::Ort(policy) => &policy.activation,
            #[cfg(test)]
            Self::Noop => "none",
        }
    }

    fn execution_provider(&self) -> &'static str {
        match self {
            Self::Ort(policy) => policy.execution_provider.as_str(),
            #[cfg(test)]
            Self::Noop => "test",
        }
    }
}

pub struct TelemetryLogger {
    pub path: Option<PathBuf>,
    pub meta_path: Option<PathBuf>,
    every: usize,
    flush_every: usize,
    file: Option<BufWriter<File>>,
    written: usize,
}

impl TelemetryLogger {
    pub fn new(
        path: Option<PathBuf>,
        every: usize,
        flush_every: usize,
    ) -> Result<Self, std::io::Error> {
        let Some(path) = path else {
            return Ok(Self {
                path: None,
                meta_path: None,
                every: every.max(1),
                flush_every: flush_every.max(1),
                file: None,
                written: 0,
            });
        };
        let resolved = resolve_telemetry_log_path(path)?;
        let meta_path = resolved.with_extension("meta.json");
        let file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&resolved)?;
        info!("telemetry log: {}", resolved.display());
        info!("telemetry meta: {}", meta_path.display());
        Ok(Self {
            path: Some(resolved),
            meta_path: Some(meta_path),
            every: every.max(1),
            flush_every: flush_every.max(1),
            file: Some(BufWriter::new(file)),
            written: 0,
        })
    }

    pub fn write_meta(&mut self, meta: &serde_json::Value) -> Result<(), LocomotionPolicyError> {
        let Some(path) = self.meta_path.as_ref() else {
            return Ok(());
        };
        let mut file = File::create(path)?;
        serde_json::to_writer_pretty(&mut file, meta)?;
        file.write_all(b"\n")?;
        Ok(())
    }

    pub fn write(
        &mut self,
        step: usize,
        record: &serde_json::Value,
    ) -> Result<(), LocomotionPolicyError> {
        if self.file.is_none() || !step.is_multiple_of(self.every) {
            return Ok(());
        }
        self.write_line(record)
    }

    pub fn write_event(&mut self, record: &serde_json::Value) -> Result<(), LocomotionPolicyError> {
        if self.file.is_none() {
            return Ok(());
        }
        self.write_line(record)?;
        if let Some(file) = self.file.as_mut() {
            file.flush()?;
        }
        Ok(())
    }

    fn write_line(&mut self, record: &serde_json::Value) -> Result<(), LocomotionPolicyError> {
        let Some(file) = self.file.as_mut() else {
            return Ok(());
        };
        serde_json::to_writer(&mut *file, record)?;
        file.write_all(b"\n")?;
        self.written += 1;
        if self.written.is_multiple_of(self.flush_every) {
            file.flush()?;
        }
        Ok(())
    }

    pub fn close(&mut self) -> Result<(), std::io::Error> {
        if let Some(file) = self.file.as_mut() {
            file.flush()?;
        }
        self.file = None;
        Ok(())
    }
}

pub fn telemetry_log_path(value: Option<String>) -> Option<PathBuf> {
    let text = value?.trim().to_string();
    if matches!(
        text.to_ascii_lowercase().as_str(),
        "" | "0" | "false" | "none" | "off"
    ) {
        None
    } else {
        Some(PathBuf::from(text))
    }
}

pub fn env_int(name: &str, default: usize) -> usize {
    std::env::var(name)
        .ok()
        .and_then(|value| value.trim().parse().ok())
        .unwrap_or(default)
}

fn resolve_telemetry_log_path(path: PathBuf) -> Result<PathBuf, std::io::Error> {
    if path.extension().is_some_and(|suffix| suffix == "jsonl") {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        return Ok(path);
    }
    std::fs::create_dir_all(&path)?;
    Ok(path.join(format!(
        "locomotion_telemetry_{}.jsonl",
        unix_time_s() as u64
    )))
}

fn unlink_socket_path_if_present(path: &Path) -> Result<(), std::io::Error> {
    let meta = match std::fs::symlink_metadata(path) {
        Ok(meta) => meta,
        Err(err) if err.kind() == std::io::ErrorKind::NotFound => return Ok(()),
        Err(err) => return Err(err),
    };
    if !meta.file_type().is_socket() {
        return Err(std::io::Error::new(
            std::io::ErrorKind::AlreadyExists,
            format!("refusing to unlink non-socket path: {}", path.display()),
        ));
    }
    std::fs::remove_file(path)
}

struct SocketPathGuard {
    path: PathBuf,
}

impl SocketPathGuard {
    fn new(path: PathBuf) -> Self {
        Self { path }
    }

    fn cleanup(&self) -> Result<(), std::io::Error> {
        unlink_socket_path_if_present(&self.path)
    }
}

impl Drop for SocketPathGuard {
    fn drop(&mut self) {
        let _ = self.cleanup();
    }
}

fn sha256_file(path: &Path) -> Result<Option<String>, std::io::Error> {
    if !path.is_file() {
        return Ok(None);
    }
    let mut file = File::open(path)?;
    let mut hasher = Sha256::new();
    let mut buf = [0_u8; 1024 * 1024];
    loop {
        let n = std::io::Read::read(&mut file, &mut buf)?;
        if n == 0 {
            break;
        }
        hasher.update(&buf[..n]);
    }
    let digest = hasher.finalize();
    Ok(Some(hex_lower(&digest)))
}

fn runtime_config_json(cfg: &LocomotionPolicyConfig) -> serde_json::Value {
    json!({
        "checkpoint": cfg.checkpoint,
        "ort_ep": cfg.ort_ep,
        "command_source": cfg.command_source.as_str(),
        "fixed_command": cfg.fixed_command.chassis.map(|command| command.to_policy_command()),
        "robot_default_base_height": cfg.robot_cfg.default_base_height,
        "action_decoder": cfg.action_decoder,
        "transport": match cfg.transport {
            LocomotionTransport::Cdc => "cdc",
            LocomotionTransport::Sim => "sim",
        },
        "port": cfg.port,
        "sim_socket_path": cfg.sim_socket_path,
        "sim_client_socket_path": cfg.sim_client_socket_path,
        "baudrate": cfg.baudrate,
        "device": cfg.device,
        "rate_hz": LOCOMOTION_POLICY_RATE_HZ,
        "state_timeout_s": STATE_TIMEOUT_S,
        "write_timeout_s": WRITE_TIMEOUT_S,
        "max_steps": cfg.max_steps,
        "dry_run": cfg.dry_run,
        "print_every": cfg.print_every,
        "telemetry_log": cfg.telemetry_log,
        "telemetry_log_every": cfg.telemetry_log_every,
        "telemetry_flush_every": cfg.telemetry_flush_every,
    })
}

fn action_decoder_config_json(
    config: &PolicyActionDecoderConfig,
    command_height: f32,
) -> serde_json::Value {
    let mut value = serde_json::to_value(config).unwrap_or_else(|_| json!({}));
    if let Some(object) = value.as_object_mut() {
        object.insert("command_height".to_string(), json!(command_height));
    }
    value
}

fn sim_output_state_to_policy_state(mut state: PolicyStateFrame) -> PolicyStateFrame {
    let output_pos = state.joint_pos.map(|value| value as f64);
    let output_vel = state.joint_vel.map(|value| value as f64);
    let output_target = state.target_joint_pos.map(|value| value as f64);
    state.joint_pos = output_to_policy_pos(output_pos).map(|value| value as f32);
    state.joint_vel = output_to_policy_vel(output_pos, output_vel).map(|value| value as f32);
    state.target_joint_pos = output_to_policy_pos(output_target).map(|value| value as f32);
    state
}

fn unix_time_s() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64()
}

fn wrap_angle(value: f32) -> f32 {
    (value + std::f32::consts::PI).rem_euclid(2.0 * std::f32::consts::PI) - std::f32::consts::PI
}

fn wrap_angle_vec4(values: [f32; 4]) -> [f32; 4] {
    values.map(wrap_angle)
}

fn sub4(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3]]
}

fn policy_active_angles(q: [f32; 4]) -> [f32; 2] {
    [wrap_angle(q[0] - q[1]), wrap_angle(q[3] - q[2])]
}

fn action_flag_names(flags: u32) -> Vec<&'static str> {
    let mut names = Vec::new();
    if flags & ACTION_FLAG_DRY_RUN != 0 {
        names.push("dry_run");
    }
    if flags & ACTION_FLAG_TIMEOUT != 0 {
        names.push("timeout");
    }
    if flags & ACTION_FLAG_NONFINITE != 0 {
        names.push("nonfinite");
    }
    if flags & ACTION_FLAG_OUTPUT_DISABLED_HOLD != 0 {
        names.push("output_disabled_hold");
    }
    if flags & ACTION_FLAG_COMMAND_INACTIVE != 0 {
        names.push("command_inactive");
    }
    names
}

fn target_mode_name(flags: u32) -> &'static str {
    if flags & ACTION_FLAG_COMMAND_INACTIVE != 0 {
        "command_inactive"
    } else if flags & (ACTION_FLAG_OUTPUT_DISABLED_HOLD | ACTION_FLAG_TIMEOUT) != 0 {
        "hold_current"
    } else {
        "policy"
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

fn fmt_values(values: &[f32]) -> String {
    values
        .iter()
        .map(|value| format!("{value:+.3}"))
        .collect::<Vec<_>>()
        .join(" ")
}

fn hex_lower(bytes: &[u8]) -> String {
    const LUT: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push(LUT[(byte >> 4) as usize] as char);
        out.push(LUT[(byte & 0x0f) as usize] as char);
    }
    out
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic, clippy::print_stdout)]
mod tests {
    use super::*;

    #[test]
    fn target_decoder_matches_python_reference() {
        let decoder = LocomotionActionTargetDecoder::new(0.22, recovery_action_decoder_config());
        let target = decoder.decode([0.2, -0.3, 0.4, -0.5, 0.6, -0.7]).unwrap();
        assert_close(
            &target.joint_pos,
            &[0.390_336_6, -0.695_675, 1.494_619, 2.429_677],
            2.0e-5,
        );
        assert_close(&target.wheel_vel, &[27.000_002, -31.5], 2.0e-5);
    }

    #[test]
    fn sim_output_state_is_converted_to_policy_state() {
        let robot = RobotConfig::default();
        let policy_pos = [
            robot.default_dof_pos[0] as f32,
            robot.default_dof_pos[1] as f32,
            robot.default_dof_pos[2] as f32,
            robot.default_dof_pos[3] as f32,
        ];
        let output_pos = policy_to_output_pos(policy_pos.map(|value| value as f64));
        let mut state = synthetic_default_state(3);
        state.joint_pos = output_pos.map(|value| value as f32);
        state.joint_vel = [0.4, -0.7, -0.2, 0.3];
        state.target_joint_pos = state.joint_pos;
        let expected_vel =
            output_to_policy_vel(output_pos, state.joint_vel.map(|value| value as f64))
                .map(|value| value as f32);

        let converted = sim_output_state_to_policy_state(state);

        assert_close(&converted.joint_pos, &policy_pos, 3.0e-5);
        assert_close(&converted.target_joint_pos, &policy_pos, 3.0e-5);
        assert_close(&converted.joint_vel, &expected_vel, 3.0e-5);
    }

    #[test]
    fn sim_transport_target_packet_uses_output_joint_coordinates() {
        let mut runtime = test_runtime_without_policy();
        runtime.sim_fourbar_surrogate = true;
        let robot = RobotConfig::default();
        let policy_pos = [
            robot.default_dof_pos[0] as f32,
            robot.default_dof_pos[1] as f32,
            robot.default_dof_pos[2] as f32,
            robot.default_dof_pos[3] as f32,
        ];

        let packet = runtime
            .make_hold_target_packet(&PolicyStateFrame {
                joint_pos: policy_pos,
                ..synthetic_default_state(11)
            })
            .unwrap();
        let messages = StreamParser::default().feed(&packet);
        let target = crate::protocol::decode_policy_target(&messages[0]).unwrap();
        let expected =
            policy_to_output_pos(policy_pos.map(|value| value as f64)).map(|value| value as f32);

        assert_close(&target.joint_pos, &expected, 3.0e-5);
        assert_close(&runtime.stats.last_target_joint_pos, &policy_pos, 0.0);
    }

    #[test]
    fn unlink_socket_path_refuses_regular_file() {
        let path = std::env::temp_dir().join(format!(
            "se3_regular_{}_{}",
            std::process::id(),
            unix_time_s()
        ));
        std::fs::write(&path, b"user data").unwrap();
        let err = unlink_socket_path_if_present(&path).unwrap_err();
        assert_eq!(err.kind(), std::io::ErrorKind::AlreadyExists);
        assert!(path.exists());
        std::fs::remove_file(path).unwrap();
    }

    fn test_runtime_without_policy() -> LocomotionPolicyRuntime {
        let now = Instant::now();
        LocomotionPolicyRuntime {
            cfg: LocomotionPolicyConfig::default(),
            obs_cfg: ObservationConfig::default(),
            policy: LoadedPolicy::Noop,
            obs_builder: LocomotionObservationBuilder::new(),
            target_decoder: LocomotionActionTargetDecoder::new(
                0.22,
                recovery_action_decoder_config(),
            ),
            stats: RuntimeStats::default(),
            last_action: [0.0; 6],
            action_seq: 0,
            policy_memory_clean: true,
            session_id: "test".to_string(),
            reset_id: 0,
            reconnect_count: 0,
            resolved_port: None,
            sim_fourbar_surrogate: false,
            last_sample_monotonic: None,
            last_policy_reset_reason: "test".to_string(),
            checkpoint_sha256: None,
            start_monotonic: now,
            last_print_monotonic: now,
            last_print_steps: 0,
            telemetry: TelemetryLogger {
                path: None,
                meta_path: None,
                every: 1,
                flush_every: 1,
                file: None,
                written: 0,
            },
            command_source: Box::new(FixedRuntimeCommandSource::new(Command::idle(0.22))),
            last_command_sample: RuntimeCommandSample::fixed(Command::idle(0.22)),
        }
    }

    fn assert_close<const N: usize>(actual: &[f32; N], expected: &[f32; N], tol: f32) {
        for (idx, (a, e)) in actual.iter().zip(expected).enumerate() {
            assert!((*a - *e).abs() <= tol, "idx {idx}: actual {a} expected {e}");
        }
    }

    fn recovery_action_decoder_config() -> PolicyActionDecoderConfig {
        PolicyActionDecoderConfig {
            height_conditioned_action_default: true,
            active_rod_semantics: true,
            ..PolicyActionDecoderConfig::default()
        }
    }
}
