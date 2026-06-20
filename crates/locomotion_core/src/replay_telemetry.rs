use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};

use log::{info, warn};
use serde_json::{Map, Value, json};
use sha2::{Digest, Sha256};
use thiserror::Error;

use crate::policy_io::{PolicyActionDecoderConfig, PolicyIoError};
use crate::{ObservationConfig, PolicyActionDecoder, RobotConfig};

use crate::ort_policy::{OrtPolicyError, OrtPolicyRuntime};

#[derive(Debug, Clone)]
pub struct ReplayConfig {
    pub telemetry: PathBuf,
    pub checkpoint: Option<PathBuf>,
    pub ort_ep: String,
    pub action_decoder: Option<PolicyActionDecoderConfig>,
    pub meta: Option<PathBuf>,
    pub max_rows: usize,
    pub print_every: usize,
    pub report_json: Option<PathBuf>,
    pub fail_action_error: Option<f64>,
}

#[derive(Debug, Error)]
pub enum ReplayError {
    #[error("io failed: {0}")]
    Io(#[from] std::io::Error),
    #[error("json failed: {0}")]
    Json(#[from] serde_json::Error),
    #[error("{0}")]
    Policy(#[from] OrtPolicyError),
    #[error("{0}")]
    PolicyIo(#[from] PolicyIoError),
    #[error("checkpoint not found; pass --checkpoint or place the model next to the telemetry log")]
    CheckpointNotFound,
    #[error("unsupported checkpoint suffix: {0}")]
    UnsupportedCheckpoint(PathBuf),
    #[error("{key} shape mismatch: expected {expected}, got {got}")]
    ShapeMismatch {
        key: &'static str,
        expected: usize,
        got: usize,
    },
}

#[derive(Debug, Clone, Default)]
pub struct ReplayStats {
    pub total_jsonl_rows: usize,
    pub sample_rows: usize,
    pub event_rows: usize,
    pub policy_rows: usize,
    pub hold_rows: usize,
    pub reset_count: usize,
    pub action_row_max_abs_errors: Vec<f64>,
    pub target_joint_row_max_abs_errors: Vec<f64>,
    pub target_wheel_row_max_abs_errors: Vec<f64>,
    pub dt_ms: Vec<f64>,
    pub policy_inference_ms: Vec<f64>,
}

impl ReplayStats {
    pub fn summary(&self, period_ms: f64) -> Value {
        let missed = self
            .dt_ms
            .iter()
            .filter(|value| **value > period_ms * 1.5)
            .count();
        json!({
            "total_jsonl_rows": self.total_jsonl_rows,
            "sample_rows": self.sample_rows,
            "event_rows": self.event_rows,
            "policy_rows": self.policy_rows,
            "hold_rows": self.hold_rows,
            "reset_count": self.reset_count,
            "period_ms": period_ms,
            "missed_50hz_deadlines": missed,
            "action_max_abs_error": stats_value(&self.action_row_max_abs_errors),
            "target_joint_max_abs_error": stats_value(&self.target_joint_row_max_abs_errors),
            "target_wheel_max_abs_error": stats_value(&self.target_wheel_row_max_abs_errors),
            "dt_ms": stats_value(&self.dt_ms),
            "policy_inference_ms": stats_value(&self.policy_inference_ms),
        })
    }
}

pub fn replay_telemetry(cfg: ReplayConfig) -> Result<i32, ReplayError> {
    let meta = load_meta(&cfg.telemetry, cfg.meta.as_deref())?;
    let checkpoint = resolve_checkpoint(cfg.checkpoint.as_deref(), &cfg.telemetry, &meta)?;
    check_checkpoint_hash(&checkpoint, &meta)?;
    let mut policy = load_policy(&checkpoint, &cfg.ort_ep)?;
    let decoder = PolicyActionDecoder::new(resolve_action_decoder_config(
        &meta,
        cfg.action_decoder.as_ref(),
    ));
    let command_height = command_height(&meta);
    let (stats, period_ms) = replay_rows(
        &cfg.telemetry,
        &mut policy,
        &decoder,
        command_height,
        cfg.max_rows,
        cfg.print_every,
    )?;
    let summary = stats.summary(period_ms);
    print_summary(&cfg.telemetry, &checkpoint, &summary);
    if let Some(path) = cfg.report_json {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let mut file = File::create(path)?;
        serde_json::to_writer_pretty(&mut file, &summary)?;
        file.write_all(b"\n")?;
    }
    if let Some(threshold) = cfg.fail_action_error {
        let max_error = summary
            .get("action_max_abs_error")
            .and_then(|v| v.get("max"))
            .and_then(Value::as_f64)
            .unwrap_or(f64::NAN);
        if max_error.is_finite() && max_error > threshold {
            return Ok(2);
        }
    }
    Ok(0)
}

fn replay_rows(
    telemetry: &Path,
    policy: &mut OrtPolicyRuntime,
    decoder: &PolicyActionDecoder,
    command_height: f32,
    max_rows: usize,
    print_every: usize,
) -> Result<(ReplayStats, f64), ReplayError> {
    let obs_cfg = ObservationConfig::default();
    let mut stats = ReplayStats::default();
    let mut last_time_s: Option<f64> = None;
    let mut last_reset_marker: Option<i64> = None;
    let mut last_mode_policy = false;
    let mut period_ms = 20.0;

    for (line_no, row) in iter_jsonl(telemetry)? {
        stats.total_jsonl_rows += 1;
        let record_type = row
            .get("record_type")
            .and_then(Value::as_str)
            .unwrap_or_else(|| {
                if row.get("obs").is_some() {
                    "sample"
                } else {
                    "event"
                }
            });
        if record_type == "event" {
            stats.event_rows += 1;
            continue;
        }
        if record_type != "sample" || row.get("obs").is_none() {
            continue;
        }
        if max_rows > 0 && stats.sample_rows >= max_rows {
            break;
        }
        stats.sample_rows += 1;

        if let Some(row_period_ms) = optional_f64(row.get("sample_period_ms"))
            && row_period_ms > 0.0
        {
            period_ms = row_period_ms;
        }
        let time_s = row_time_s(&row);
        if let Some(last_time_s) = last_time_s {
            stats.dt_ms.push((time_s - last_time_s) * 1000.0);
        }
        last_time_s = Some(time_s);

        if let Some(policy_ms) = optional_f64(row.get("policy_inference_ms")) {
            stats.policy_inference_ms.push(policy_ms);
        }

        let target_mode = row
            .get("target_mode")
            .and_then(Value::as_str)
            .unwrap_or("policy");
        let is_policy = target_mode == "policy";
        let reset_marker = optional_i64(row.get("reset_id"));
        let reset_now = if reset_marker.is_some() && reset_marker != last_reset_marker {
            last_reset_marker = reset_marker;
            true
        } else {
            reset_marker.is_none() && is_policy && !last_mode_policy
        };
        if reset_now {
            policy.reset();
            stats.reset_count += 1;
        }

        let obs = array_f32::<32>(&row, "obs", obs_cfg.num_obs)?;
        let logged_action = array_f32::<6>(&row, "action", obs_cfg.num_actions)?;
        if is_policy {
            stats.policy_rows += 1;
            let replay_action_vec = policy.act(&obs)?;
            let replay_action = vec_to_array::<6>(replay_action_vec, "action")?;
            stats
                .action_row_max_abs_errors
                .push(max_abs_diff(&replay_action, &logged_action));
            let decoded = decoder.decode(replay_action, Some(command_height), None, None)?;
            if row.get("nx_target_joint_pos").is_some() {
                let logged_joint = array_f32::<4>(&row, "nx_target_joint_pos", 4)?;
                stats
                    .target_joint_row_max_abs_errors
                    .push(max_abs_diff(&decoded.leg_target, &logged_joint));
            }
            if row.get("nx_target_wheel_vel").is_some() {
                let logged_wheel = array_f32::<2>(&row, "nx_target_wheel_vel", 2)?;
                stats
                    .target_wheel_row_max_abs_errors
                    .push(max_abs_diff(&decoded.wheel_vel_target, &logged_wheel));
            }
        } else {
            stats.hold_rows += 1;
            policy.reset();
        }
        last_mode_policy = is_policy;
        if print_every > 0 && stats.sample_rows % print_every == 0 {
            let last_error = stats
                .action_row_max_abs_errors
                .last()
                .copied()
                .unwrap_or(f64::NAN);
            info!(
                "replayed samples={} line={} policy={} hold={} last_action_err={:.3e}",
                stats.sample_rows, line_no, stats.policy_rows, stats.hold_rows, last_error
            );
        }
    }

    Ok((stats, period_ms))
}

fn load_meta(telemetry: &Path, explicit: Option<&Path>) -> Result<Value, ReplayError> {
    let mut candidates = Vec::new();
    if let Some(path) = explicit {
        candidates.push(path.to_path_buf());
    }
    candidates.push(telemetry.with_extension("meta.json"));
    for path in candidates {
        if path.exists() {
            return Ok(serde_json::from_str(&std::fs::read_to_string(path)?)?);
        }
    }
    Ok(Value::Object(Map::new()))
}

fn resolve_checkpoint(
    explicit: Option<&Path>,
    telemetry: &Path,
    meta: &Value,
) -> Result<PathBuf, ReplayError> {
    if let Some(path) = explicit {
        let mut candidates = Vec::new();
        push_checkpoint_candidates(&mut candidates, path, telemetry);
        return Ok(candidates
            .into_iter()
            .find(|candidate| candidate.exists())
            .unwrap_or_else(|| path.to_path_buf()));
    }
    let mut candidates = Vec::new();
    if let Some(checkpoint) = meta.get("checkpoint").and_then(Value::as_object) {
        for key in ["resolved_path", "path"] {
            if let Some(value) = checkpoint.get(key).and_then(Value::as_str) {
                push_checkpoint_candidates(&mut candidates, &PathBuf::from(value), telemetry);
            }
        }
    }
    for path in candidates {
        if path.exists() {
            return Ok(path);
        }
    }
    Err(ReplayError::CheckpointNotFound)
}

fn push_checkpoint_candidates(candidates: &mut Vec<PathBuf>, path: &Path, telemetry: &Path) {
    let telemetry_dir = telemetry.parent().unwrap_or(Path::new("."));
    let file_name = path.file_name().unwrap_or_else(|| std::ffi::OsStr::new(""));
    let local_path = telemetry_dir.join(file_name);

    for candidate in checkpoint_path_variants(path) {
        push_unique(candidates, candidate);
    }
    if local_path != path {
        for candidate in checkpoint_path_variants(&local_path) {
            push_unique(candidates, candidate);
        }
    }
}

fn checkpoint_path_variants(path: &Path) -> Vec<PathBuf> {
    if path.extension().and_then(|value| value.to_str()) == Some("npz") {
        let onnx = path.with_extension("onnx");
        vec![onnx, path.to_path_buf()]
    } else {
        vec![path.to_path_buf()]
    }
}

fn push_unique(candidates: &mut Vec<PathBuf>, path: PathBuf) {
    if !candidates.iter().any(|candidate| candidate == &path) {
        candidates.push(path);
    }
}

fn check_checkpoint_hash(checkpoint: &Path, meta: &Value) -> Result<(), ReplayError> {
    let expected = meta
        .get("checkpoint")
        .and_then(Value::as_object)
        .and_then(|m| m.get("sha256"))
        .and_then(Value::as_str);
    let Some(expected) = expected.filter(|value| !value.is_empty()) else {
        return Ok(());
    };
    let actual = sha256_file(checkpoint)?;
    if actual != expected {
        warn!(
            "checkpoint sha256 mismatch: expected={expected} actual={actual} path={}",
            checkpoint.display()
        );
    }
    Ok(())
}

fn load_policy(checkpoint: &Path, ort_ep: &str) -> Result<OrtPolicyRuntime, ReplayError> {
    match checkpoint.extension().and_then(|v| v.to_str()) {
        Some("onnx") => Ok(OrtPolicyRuntime::new(checkpoint, ort_ep)?),
        _ => Err(ReplayError::UnsupportedCheckpoint(checkpoint.to_path_buf())),
    }
}

fn command_height(meta: &Value) -> f32 {
    meta.get("command")
        .and_then(Value::as_array)
        .and_then(|values| values.get(4))
        .and_then(Value::as_f64)
        .unwrap_or_else(|| RobotConfig::default().default_base_height) as f32
}

fn resolve_action_decoder_config(
    meta: &Value,
    fallback: Option<&PolicyActionDecoderConfig>,
) -> PolicyActionDecoderConfig {
    meta.get("action_decoder")
        .cloned()
        .and_then(|value| serde_json::from_value::<PolicyActionDecoderConfig>(value).ok())
        .or_else(|| fallback.cloned())
        .unwrap_or_default()
}

fn iter_jsonl(path: &Path) -> Result<Vec<(usize, Value)>, ReplayError> {
    let file = File::open(path)?;
    let reader = BufReader::new(file);
    let mut rows = Vec::new();
    for (idx, line) in reader.lines().enumerate() {
        let line = line?;
        let text = line.trim();
        if text.is_empty() {
            continue;
        }
        rows.push((idx + 1, serde_json::from_str(text)?));
    }
    Ok(rows)
}

fn array_f32<const N: usize>(
    row: &Value,
    key: &'static str,
    expected: usize,
) -> Result<[f32; N], ReplayError> {
    let values = row
        .get(key)
        .and_then(Value::as_array)
        .ok_or(ReplayError::ShapeMismatch {
            key,
            expected,
            got: 0,
        })?;
    if values.len() != expected || values.len() != N {
        return Err(ReplayError::ShapeMismatch {
            key,
            expected,
            got: values.len(),
        });
    }
    let mut out = [0.0_f32; N];
    for (dst, value) in out.iter_mut().zip(values) {
        *dst = value.as_f64().unwrap_or(0.0) as f32;
    }
    Ok(out)
}

fn vec_to_array<const N: usize>(
    values: Vec<f32>,
    key: &'static str,
) -> Result<[f32; N], ReplayError> {
    if values.len() != N {
        return Err(ReplayError::ShapeMismatch {
            key,
            expected: N,
            got: values.len(),
        });
    }
    let mut out = [0.0_f32; N];
    out.copy_from_slice(&values);
    Ok(out)
}

fn max_abs_diff<const N: usize>(a: &[f32; N], b: &[f32; N]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(a, b)| (*a as f64 - *b as f64).abs())
        .fold(0.0, f64::max)
}

fn row_time_s(row: &Value) -> f64 {
    for key in ["monotonic_time_s", "wall_time_s"] {
        if let Some(value) = optional_f64(row.get(key)) {
            return value;
        }
    }
    if let Some(tick_ms) = optional_f64(row.get("tick_ms")) {
        return tick_ms / 1000.0;
    }
    let step = optional_f64(row.get("step")).unwrap_or(0.0);
    let period_ms = optional_f64(row.get("sample_period_ms")).unwrap_or(20.0);
    step * period_ms / 1000.0
}

fn optional_f64(value: Option<&Value>) -> Option<f64> {
    let parsed = match value? {
        Value::Number(n) => n.as_f64()?,
        Value::String(s) => s.parse().ok()?,
        _ => return None,
    };
    parsed.is_finite().then_some(parsed)
}

fn optional_i64(value: Option<&Value>) -> Option<i64> {
    optional_f64(value).map(|value| value as i64)
}

fn stats_value(values: &[f64]) -> Value {
    if values.is_empty() {
        return json!({"count": 0.0, "mean": null, "p95": null, "p99": null, "max": null});
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(f64::total_cmp);
    let sum: f64 = sorted.iter().sum();
    json!({
        "count": sorted.len() as f64,
        "mean": sum / sorted.len() as f64,
        "p95": percentile(&sorted, 95.0),
        "p99": percentile(&sorted, 99.0),
        "max": sorted.last().copied().unwrap_or(f64::NAN),
    })
}

fn percentile(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return f64::NAN;
    }
    let rank = (p / 100.0) * (sorted.len() - 1) as f64;
    let lo = rank.floor() as usize;
    let hi = rank.ceil() as usize;
    if lo == hi {
        sorted[lo]
    } else {
        let t = rank - lo as f64;
        sorted[lo] + t * (sorted[hi] - sorted[lo])
    }
}

fn sha256_file(path: &Path) -> Result<String, std::io::Error> {
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
    Ok(hex_lower(&digest))
}

fn print_summary(telemetry: &Path, checkpoint: &Path, summary: &Value) {
    let action = &summary["action_max_abs_error"];
    let dt = &summary["dt_ms"];
    let policy_ms = &summary["policy_inference_ms"];
    info!("telemetry: {}", telemetry.display());
    info!("checkpoint: {}", checkpoint.display());
    info!(
        "rows: samples={} policy={} hold={} events={} resets={}",
        summary["sample_rows"],
        summary["policy_rows"],
        summary["hold_rows"],
        summary["event_rows"],
        summary["reset_count"]
    );
    info!(
        "action max abs error: mean={:.3e} p95={:.3e} max={:.3e}",
        action["mean"].as_f64().unwrap_or(f64::NAN),
        action["p95"].as_f64().unwrap_or(f64::NAN),
        action["max"].as_f64().unwrap_or(f64::NAN)
    );
    info!(
        "dt ms: mean={:.3} p95={:.3} p99={:.3} max={:.3} missed={}",
        dt["mean"].as_f64().unwrap_or(f64::NAN),
        dt["p95"].as_f64().unwrap_or(f64::NAN),
        dt["p99"].as_f64().unwrap_or(f64::NAN),
        dt["max"].as_f64().unwrap_or(f64::NAN),
        summary["missed_50hz_deadlines"]
    );
    info!(
        "logged policy ms: mean={:.3} p95={:.3} p99={:.3} max={:.3}",
        policy_ms["mean"].as_f64().unwrap_or(f64::NAN),
        policy_ms["p95"].as_f64().unwrap_or(f64::NAN),
        policy_ms["p99"].as_f64().unwrap_or(f64::NAN),
        policy_ms["max"].as_f64().unwrap_or(f64::NAN)
    );
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
