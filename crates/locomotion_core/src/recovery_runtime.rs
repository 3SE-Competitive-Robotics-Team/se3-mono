#![allow(clippy::print_stdout)]

use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serde_json::{Map, Value, json};
use sha2::{Digest, Sha256};
use thiserror::Error;

use crate::policy_io::{PolicyActionDecoderConfig, PolicyIoError};
use crate::{ObservationConfig, PolicyActionDecoder, RobotConfig};

use crate::cdc::{CdcError, CdcSerial, cdc_port_disappeared, resolve_cdc_port};
use crate::ort_policy::{OrtPolicyError, OrtPolicyRuntime};
use crate::protocol::{
    MSG_POLICY_STATE, PolicyStateFrame, PolicyTargetFrame, ProtocolError, StreamParser,
    decode_policy_state, pack_policy_target,
};
use crate::recovery_observation::{RecoveryObservationBuilder, synthetic_recovery_state};

pub const DEFAULT_CDC_PORT: &str = "auto";
pub const CDC_RECONNECT_DELAY_S: f64 = 1.0;
pub const TELEMETRY_SCHEMA: &str = "se3_nx_recovery_telemetry_v2";
pub const ACTION_FLAG_DRY_RUN: u32 = 1 << 0;
pub const ACTION_FLAG_TIMEOUT: u32 = 1 << 1;
pub const ACTION_FLAG_NONFINITE: u32 = 1 << 2;
pub const ACTION_FLAG_OUTPUT_DISABLED_HOLD: u32 = 1 << 3;

#[derive(Debug, Error)]
pub enum RecoveryRuntimeError {
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
    #[error("io failed: {0}")]
    Io(#[from] std::io::Error),
    #[error("json failed: {0}")]
    Json(#[from] serde_json::Error),
}

#[derive(Debug, Clone)]
pub struct RecoveryRuntimeConfig {
    pub checkpoint: PathBuf,
    pub ort_ep: String,
    pub port: String,
    pub baudrate: i32,
    pub device: String,
    pub rate_hz: f64,
    pub state_timeout_s: f64,
    pub write_timeout_s: f64,
    pub max_steps: usize,
    pub dry_run: bool,
    pub print_every: usize,
    pub telemetry_log: Option<PathBuf>,
    pub telemetry_log_every: usize,
    pub telemetry_flush_every: usize,
}

impl Default for RecoveryRuntimeConfig {
    fn default() -> Self {
        Self {
            checkpoint: PathBuf::new(),
            ort_ep: "auto".to_string(),
            port: DEFAULT_CDC_PORT.to_string(),
            baudrate: 921600,
            device: "cpu".to_string(),
            rate_hz: 50.0,
            state_timeout_s: 0.10,
            write_timeout_s: 0.02,
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
pub struct RecoveryActionTargetDecoder {
    pub robot_cfg: RobotConfig,
    pub command_height: f32,
    decoder: PolicyActionDecoder,
}

impl RecoveryActionTargetDecoder {
    pub fn new(command_height: f32, robot_cfg: Option<RobotConfig>) -> Self {
        let robot_cfg = robot_cfg.unwrap_or_default();
        let decoder = PolicyActionDecoder::new(PolicyActionDecoderConfig {
            robot_cfg: robot_cfg.clone(),
            height_conditioned_action_default: true,
            active_rod_semantics: true,
            ..PolicyActionDecoderConfig::default()
        });
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

pub struct RecoveryRuntime {
    pub cfg: RecoveryRuntimeConfig,
    pub obs_cfg: ObservationConfig,
    policy: LoadedPolicy,
    obs_builder: RecoveryObservationBuilder,
    target_decoder: RecoveryActionTargetDecoder,
    pub stats: RuntimeStats,
    last_action: [f32; 6],
    action_seq: u32,
    policy_memory_clean: bool,
    session_id: String,
    reset_id: usize,
    reconnect_count: usize,
    resolved_port: Option<String>,
    last_sample_monotonic: Option<Instant>,
    last_policy_reset_reason: String,
    checkpoint_sha256: Option<String>,
    start_monotonic: Instant,
    last_print_monotonic: Instant,
    last_print_steps: usize,
    telemetry: TelemetryLogger,
}

impl RecoveryRuntime {
    pub fn new(cfg: RecoveryRuntimeConfig) -> Result<Self, RecoveryRuntimeError> {
        let obs_cfg = ObservationConfig::default();
        let mut policy = load_policy_runtime(&cfg.checkpoint, &cfg.ort_ep)?;
        policy.reset();
        let obs_builder = RecoveryObservationBuilder::new();
        let target_decoder = RecoveryActionTargetDecoder::new(obs_builder.command[4], None);
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
            "command": obs_builder.command,
            "action_decoder": {
                "height_conditioned_action_default": true,
                "active_rod_semantics": true,
                "command_height": target_decoder.command_height,
            }
        }))?;
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
            last_sample_monotonic: None,
            last_policy_reset_reason: "startup".to_string(),
            checkpoint_sha256,
            start_monotonic: now,
            last_print_monotonic: now,
            last_print_steps: 0,
            telemetry,
        };
        runtime.write_event("runtime_start", json!({}))?;
        Ok(runtime)
    }

    pub fn run(&mut self) -> Result<RuntimeStats, RecoveryRuntimeError> {
        println!(
            "NX recovery runtime: checkpoint={} iter={} type={} device={}",
            self.cfg.checkpoint.display(),
            self.policy.iteration(),
            self.policy.policy_type(),
            self.policy.execution_provider()
        );
        let result = if self.cfg.dry_run {
            self.run_dry()
        } else {
            self.run_cdc()
        };
        self.write_event("runtime_stop", json!({ "total_steps": self.stats.steps }))?;
        self.telemetry.close()?;
        result.map(|_| self.stats.clone())
    }

    fn run_dry(&mut self) -> Result<(), RecoveryRuntimeError> {
        let max_steps = if self.cfg.max_steps > 0 {
            self.cfg.max_steps
        } else {
            (self.cfg.rate_hz * 2.0) as usize
        };
        for step in 0..max_steps {
            let loop_started = Instant::now();
            let state = synthetic_recovery_state(step as u32);
            self.stats.state_frames += 1;
            let (action, mut flags, obs, policy_inference_ms) = self.act_from_state(&state)?;
            flags |= ACTION_FLAG_DRY_RUN;
            self.decode_target(action)?;
            self.record_action(&state, action, flags, Some(policy_inference_ms));
            self.write_telemetry(
                &state,
                &obs,
                &action,
                flags,
                Some(policy_inference_ms),
                Some(0.0),
                loop_started,
                None,
            )?;
            self.maybe_print();
        }
        Ok(())
    }

    fn run_cdc(&mut self) -> Result<(), RecoveryRuntimeError> {
        let max_steps = self.cfg.max_steps;
        let period_s = 1.0 / self.cfg.rate_hz;

        while max_steps == 0 || self.stats.steps < max_steps {
            self.reset_policy_memory(true, "cdc_connect")?;
            let mut parser = StreamParser::default();
            let mut latest_state: Option<PolicyStateFrame> = None;
            let mut latest_state_time = Instant::now();
            let mut next_tick = Instant::now();
            let port = resolve_cdc_port(&self.cfg.port);
            self.resolved_port = Some(port.clone());
            let loop_result = (|| -> Result<(), RecoveryRuntimeError> {
                let mut serial = CdcSerial::new(&port, self.cfg.baudrate);
                serial.open()?;
                println!("USB CDC open: port={port} baudrate={}", self.cfg.baudrate);
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
                    next_tick += Duration::from_secs_f64(period_s);
                    let Some(state) = latest_state.as_ref() else {
                        self.stats.steps += 1;
                        self.maybe_print();
                        continue;
                    };
                    let age_s = now
                        .saturating_duration_since(latest_state_time)
                        .as_secs_f64();
                    let mut flags = 0_u32;
                    let policy_inference_ms;
                    let obs;
                    let action;
                    let packet;
                    if age_s > self.cfg.state_timeout_s {
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
                        packet = self.make_hold_target_packet(state)?;
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
                        packet = self.make_hold_target_packet(state)?;
                    } else {
                        let result = self.act_from_state(state)?;
                        action = result.0;
                        flags = result.1;
                        obs = result.2;
                        policy_inference_ms = Some(result.3);
                        packet = self.make_target_packet(action)?;
                    }
                    let write_started = Instant::now();
                    serial.write_all(&packet, self.cfg.write_timeout_s)?;
                    let write_ms = write_started.elapsed().as_secs_f64() * 1000.0;
                    self.record_action(state, action, flags, policy_inference_ms);
                    self.write_telemetry(
                        state,
                        &obs,
                        &action,
                        flags,
                        policy_inference_ms,
                        Some(age_s),
                        loop_started,
                        Some(write_ms),
                    )?;
                    self.maybe_print();
                }
                Ok(())
            })();

            match loop_result {
                Ok(()) => return Ok(()),
                Err(RecoveryRuntimeError::Cdc(
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
                    println!(
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
    ) -> Result<(Option<PolicyStateFrame>, Instant), RecoveryRuntimeError> {
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
    ) -> Result<([f32; 32], u32), RecoveryRuntimeError> {
        let result = self.obs_builder.build(state, self.last_action)?;
        let flags = if result.had_nonfinite_input {
            ACTION_FLAG_NONFINITE
        } else {
            0
        };
        Ok((result.obs, flags))
    }

    fn act_from_state(
        &mut self,
        state: &PolicyStateFrame,
    ) -> Result<([f32; 6], u32, [f32; 32], f64), RecoveryRuntimeError> {
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
    ) -> Result<(), RecoveryRuntimeError> {
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

    fn make_target_packet(&mut self, action: [f32; 6]) -> Result<Vec<u8>, RecoveryRuntimeError> {
        let target = self.decode_target(action)?;
        let frame = PolicyTargetFrame {
            seq: self.action_seq,
            joint_pos: target.joint_pos,
            wheel_vel: target.wheel_vel,
        };
        self.action_seq = self.action_seq.wrapping_add(1);
        Ok(pack_policy_target(&frame)?)
    }

    fn make_hold_target_packet(
        &mut self,
        state: &PolicyStateFrame,
    ) -> Result<Vec<u8>, RecoveryRuntimeError> {
        let joint_pos = state.joint_pos;
        let wheel_vel = [0.0, 0.0];
        self.stats.last_target_joint_pos = joint_pos;
        self.stats.last_target_wheel_vel = wheel_vel;
        let frame = PolicyTargetFrame {
            seq: self.action_seq,
            joint_pos,
            wheel_vel,
        };
        self.action_seq = self.action_seq.wrapping_add(1);
        Ok(pack_policy_target(&frame)?)
    }

    fn decode_target(
        &mut self,
        action: [f32; 6],
    ) -> Result<DecodedPolicyTarget, RecoveryRuntimeError> {
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

    #[allow(clippy::too_many_arguments)]
    fn write_telemetry(
        &mut self,
        state: &PolicyStateFrame,
        obs: &[f32; 32],
        action: &[f32; 6],
        flags: u32,
        policy_inference_ms: Option<f64>,
        state_age_s: Option<f64>,
        loop_started: Instant,
        write_ms: Option<f64>,
    ) -> Result<(), RecoveryRuntimeError> {
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
            json!(loop_started.elapsed().as_secs_f64() * 1000.0),
        );
        record.insert("loop_dt_ms".to_string(), json!(loop_dt_ms));
        record.insert(
            "state_age_ms_nx".to_string(),
            json!(state_age_s.map(|v| v * 1000.0)),
        );
        record.insert("write_ms".to_string(), json!(write_ms));
        record.insert("rate_hz".to_string(), json!(self.cfg.rate_hz));
        record.insert(
            "sample_period_ms".to_string(),
            json!(1000.0 / self.cfg.rate_hz),
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
            json!(policy_inference_ms),
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
        record.insert(
            "target_mode".to_string(),
            json!(
                if flags & (ACTION_FLAG_OUTPUT_DISABLED_HOLD | ACTION_FLAG_TIMEOUT) != 0 {
                    "hold_current"
                } else {
                    "policy"
                }
            ),
        );
        record.insert(
            "last_policy_reset_reason".to_string(),
            json!(self.last_policy_reset_reason),
        );
        record.insert("command".to_string(), json!(self.obs_builder.command));
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
    ) -> Result<(), RecoveryRuntimeError> {
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
        let target_mode = if self.stats.last_action_flags
            & (ACTION_FLAG_OUTPUT_DISABLED_HOLD | ACTION_FLAG_TIMEOUT)
            != 0
        {
            "hold_current"
        } else {
            "policy"
        };
        println!(
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

pub fn load_policy_runtime(
    checkpoint: &Path,
    ort_ep: &str,
) -> Result<LoadedPolicy, RecoveryRuntimeError> {
    match checkpoint.extension().and_then(|value| value.to_str()) {
        Some("onnx") => Ok(LoadedPolicy::Ort(OrtPolicyRuntime::new(
            checkpoint, ort_ep,
        )?)),
        _ => Err(RecoveryRuntimeError::UnsupportedCheckpoint(
            checkpoint.to_path_buf(),
        )),
    }
}

pub enum LoadedPolicy {
    Ort(OrtPolicyRuntime),
}

impl LoadedPolicy {
    fn reset(&mut self) {
        match self {
            Self::Ort(policy) => policy.reset(),
        }
    }

    fn act(&mut self, obs: &[f32]) -> Result<Vec<f32>, RecoveryRuntimeError> {
        match self {
            Self::Ort(policy) => Ok(policy.act(obs)?),
        }
    }

    fn iteration(&self) -> &str {
        match self {
            Self::Ort(policy) => &policy.iteration,
        }
    }

    fn policy_type(&self) -> String {
        match self {
            Self::Ort(policy) => policy.policy_type(),
        }
    }

    fn num_obs(&self) -> usize {
        match self {
            Self::Ort(policy) => policy.num_obs,
        }
    }

    fn num_actions(&self) -> usize {
        match self {
            Self::Ort(policy) => policy.num_actions,
        }
    }

    fn rnn_type(&self) -> &str {
        match self {
            Self::Ort(policy) => &policy.rnn_type,
        }
    }

    fn rnn_hidden_dim(&self) -> usize {
        match self {
            Self::Ort(policy) => policy.rnn_hidden_dim,
        }
    }

    fn rnn_num_layers(&self) -> usize {
        match self {
            Self::Ort(policy) => policy.rnn_num_layers,
        }
    }

    fn activation(&self) -> &str {
        match self {
            Self::Ort(policy) => &policy.activation,
        }
    }

    fn execution_provider(&self) -> &'static str {
        match self {
            Self::Ort(policy) => policy.execution_provider.as_str(),
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
        println!("telemetry log: {}", resolved.display());
        println!("telemetry meta: {}", meta_path.display());
        Ok(Self {
            path: Some(resolved),
            meta_path: Some(meta_path),
            every: every.max(1),
            flush_every: flush_every.max(1),
            file: Some(BufWriter::new(file)),
            written: 0,
        })
    }

    pub fn write_meta(&mut self, meta: &serde_json::Value) -> Result<(), RecoveryRuntimeError> {
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
    ) -> Result<(), RecoveryRuntimeError> {
        if self.file.is_none() || !step.is_multiple_of(self.every) {
            return Ok(());
        }
        self.write_line(record)
    }

    pub fn write_event(&mut self, record: &serde_json::Value) -> Result<(), RecoveryRuntimeError> {
        if self.file.is_none() {
            return Ok(());
        }
        self.write_line(record)?;
        if let Some(file) = self.file.as_mut() {
            file.flush()?;
        }
        Ok(())
    }

    fn write_line(&mut self, record: &serde_json::Value) -> Result<(), RecoveryRuntimeError> {
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
    Ok(path.join(format!("recovery_telemetry_{}.jsonl", unix_time_s() as u64)))
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

fn runtime_config_json(cfg: &RecoveryRuntimeConfig) -> serde_json::Value {
    json!({
        "checkpoint": cfg.checkpoint,
        "ort_ep": cfg.ort_ep,
        "port": cfg.port,
        "baudrate": cfg.baudrate,
        "device": cfg.device,
        "rate_hz": cfg.rate_hz,
        "state_timeout_s": cfg.state_timeout_s,
        "write_timeout_s": cfg.write_timeout_s,
        "max_steps": cfg.max_steps,
        "dry_run": cfg.dry_run,
        "print_every": cfg.print_every,
        "telemetry_log": cfg.telemetry_log,
        "telemetry_log_every": cfg.telemetry_log_every,
        "telemetry_flush_every": cfg.telemetry_flush_every,
    })
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
    names
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
        let decoder = RecoveryActionTargetDecoder::new(0.22, None);
        let target = decoder.decode([0.2, -0.3, 0.4, -0.5, 0.6, -0.7]).unwrap();
        assert_close(
            &target.joint_pos,
            &[0.390_336_6, -0.695_675, 1.494_619, 2.429_677],
            2.0e-5,
        );
        assert_close(&target.wheel_vel, &[27.000_002, -31.5], 2.0e-5);
    }

    fn assert_close<const N: usize>(actual: &[f32; N], expected: &[f32; N], tol: f32) {
        for (idx, (a, e)) in actual.iter().zip(expected).enumerate() {
            assert!((*a - *e).abs() <= tol, "idx {idx}: actual {a} expected {e}");
        }
    }
}
