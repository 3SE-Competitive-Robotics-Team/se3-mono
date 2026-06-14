use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serde_json::{Map, Value, json};
use thiserror::Error;

use crate::{RobotConfig, policy_to_output_pos};

use crate::cdc::{CdcError, CdcSerial, resolve_cdc_port};
use crate::protocol::{
    MSG_LATENCY, MSG_POLICY_STATE, PolicyLatencyFrame, PolicyStateFrame, ProtocolError,
    StreamParser, decode_policy_latency, decode_policy_state,
};
use crate::recovery_observation::{RecoveryObservationBuilder, synthetic_recovery_state};

#[derive(Debug, Clone)]
pub struct VisualizerConfig {
    pub port: String,
    pub baudrate: i32,
    pub host: String,
    pub viewer_port: u16,
    pub synthetic: bool,
    pub local_cdc: bool,
    pub remote_url: String,
    pub remote_timeout_s: f64,
    pub rate_hz: f64,
    pub read_timeout_s: f64,
    pub no_mjcf_render: bool,
}

#[derive(Debug, Error)]
pub enum VisualizerError {
    #[error("io failed: {0}")]
    Io(#[from] std::io::Error),
    #[error("{0}")]
    Cdc(#[from] CdcError),
    #[error("{0}")]
    Protocol(#[from] ProtocolError),
    #[error("json failed: {0}")]
    Json(#[from] serde_json::Error),
    #[error("ctrlc failed: {0}")]
    CtrlC(#[from] ctrlc::Error),
}

#[derive(Debug, Default)]
struct SharedSnapshot {
    latest: Map<String, Value>,
    latest_latency: Option<Map<String, Value>>,
    latest_state: Option<PolicyStateFrame>,
    event_seq: u64,
}

impl SharedSnapshot {
    fn update(&mut self, mut snapshot: Map<String, Value>, state: Option<PolicyStateFrame>) {
        self.event_seq += 1;
        if let Some(latency) = self.latest_latency.clone() {
            attach_latency_snapshot(&mut snapshot, &latency);
        }
        snapshot.insert("_event_seq".to_string(), json!(self.event_seq));
        self.latest = snapshot;
        self.latest_state = state;
    }

    fn update_latency(&mut self, latency: PolicyLatencyFrame) {
        self.event_seq += 1;
        self.latest_latency = Some(latency_to_snapshot(&latency));
        let mut snapshot = if self.latest.is_empty() {
            let mut snapshot = Map::new();
            snapshot.insert("source".to_string(), json!("cdc"));
            snapshot.insert("connected".to_string(), json!(true));
            snapshot.insert("host_time_s".to_string(), json!(unix_time_s()));
            snapshot.insert("seq".to_string(), json!(-1));
            snapshot
        } else {
            self.latest.clone()
        };
        if let Some(latency) = self.latest_latency.clone() {
            attach_latency_snapshot(&mut snapshot, &latency);
        }
        snapshot.insert("_event_seq".to_string(), json!(self.event_seq));
        self.latest = snapshot;
    }

    fn get(&self) -> Map<String, Value> {
        self.latest.clone()
    }
}

pub fn run_visualizer(cfg: VisualizerConfig) -> Result<(), VisualizerError> {
    let shared = Arc::new(Mutex::new(SharedSnapshot::default()));
    let stop = Arc::new(AtomicBool::new(false));
    {
        let stop = stop.clone();
        ctrlc::set_handler(move || {
            stop.store(true, Ordering::SeqCst);
        })?;
    }

    if cfg.synthetic {
        spawn_synthetic_reader(shared.clone(), stop.clone(), cfg.rate_hz);
    } else if !cfg.local_cdc && !cfg.remote_url.trim().is_empty() {
        spawn_remote_reader(
            shared.clone(),
            stop.clone(),
            cfg.remote_url.clone(),
            cfg.remote_timeout_s,
        );
    } else {
        spawn_cdc_reader(
            shared.clone(),
            stop.clone(),
            cfg.port.clone(),
            cfg.baudrate,
            cfg.read_timeout_s,
        );
    }

    let listener = TcpListener::bind((cfg.host.as_str(), cfg.viewer_port))?;
    listener.set_nonblocking(true)?;
    let url_host = if cfg.host == "0.0.0.0" || cfg.host == "::" {
        "127.0.0.1"
    } else {
        cfg.host.as_str()
    };
    println!(
        "CDC visualizer listening on http://{}:{}",
        url_host, cfg.viewer_port
    );
    if cfg.no_mjcf_render {
        println!(
            "MJCF render disabled; Rust visualizer serves canvas fallback and JSON/SSE relay."
        );
    } else {
        println!(
            "MJCF render is not linked in this Rust build; /render_info reports enabled=false."
        );
    }

    while !stop.load(Ordering::SeqCst) {
        match listener.accept() {
            Ok((stream, _addr)) => {
                let shared = shared.clone();
                thread::spawn(move || {
                    if let Err(err) = handle_client(stream, shared) {
                        eprintln!("visualizer client error: {err}");
                    }
                });
            }
            Err(err) if err.kind() == std::io::ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(20));
            }
            Err(err) => return Err(err.into()),
        }
    }
    Ok(())
}

fn spawn_synthetic_reader(shared: Arc<Mutex<SharedSnapshot>>, stop: Arc<AtomicBool>, rate_hz: f64) {
    thread::spawn(move || {
        let builder = RecoveryObservationBuilder::new();
        let period = Duration::from_secs_f64(1.0 / rate_hz.max(1.0));
        let mut seq = 0_u32;
        let mut last_wall_time: Option<f64> = None;
        while !stop.load(Ordering::SeqCst) {
            let mut state = synthetic_recovery_state(seq);
            let t = seq as f32 * period.as_secs_f32();
            state.joint_pos[0] += 0.12 * t.sin();
            state.joint_pos[1] -= 0.08 * (t * 0.7).sin();
            state.joint_pos[2] -= 0.12 * (t * 0.9).sin();
            state.joint_pos[3] += 0.08 * (t * 1.1).sin();
            state.wheel_pos = [t * 2.0, -t * 2.0];
            state.wheel_vel = [2.0, -2.0];
            let now = unix_time_s();
            let frame_hz = last_wall_time.map(|last| 1.0 / (now - last).max(1.0e-6));
            last_wall_time = Some(now);
            let snapshot = state_to_snapshot(&state, &builder, "synthetic", frame_hz, None);
            shared.lock().unwrap().update(snapshot, Some(state));
            seq = seq.wrapping_add(1);
            thread::sleep(period);
        }
    });
}

fn spawn_cdc_reader(
    shared: Arc<Mutex<SharedSnapshot>>,
    stop: Arc<AtomicBool>,
    dev: String,
    baudrate: i32,
    read_timeout_s: f64,
) {
    thread::spawn(move || {
        let builder = RecoveryObservationBuilder::new();
        let mut parser = StreamParser::default();
        let mut last_wall_time: Option<f64> = None;
        while !stop.load(Ordering::SeqCst) {
            let port = resolve_cdc_port(&dev);
            let mut serial = CdcSerial::new(&port, baudrate);
            match serial.open() {
                Ok(()) => {
                    println!("CDC visualizer opened {port}");
                    while !stop.load(Ordering::SeqCst) {
                        match serial.wait_readable(read_timeout_s) {
                            Ok(true) => {}
                            Ok(false) => continue,
                            Err(err) => {
                                eprintln!("CDC wait failed: {err}");
                                break;
                            }
                        }
                        let data = match serial.read_available() {
                            Ok(data) => data,
                            Err(err) => {
                                eprintln!("CDC read failed: {err}");
                                break;
                            }
                        };
                        for message in parser.feed(&data) {
                            if message.msg_type == MSG_POLICY_STATE {
                                match decode_policy_state(&message) {
                                    Ok(state) => {
                                        let now = unix_time_s();
                                        let frame_hz = last_wall_time
                                            .map(|last| 1.0 / (now - last).max(1.0e-6));
                                        last_wall_time = Some(now);
                                        let snapshot = state_to_snapshot(
                                            &state,
                                            &builder,
                                            "cdc",
                                            frame_hz,
                                            Some(&port),
                                        );
                                        shared.lock().unwrap().update(snapshot, Some(state));
                                    }
                                    Err(err) => eprintln!("state decode failed: {err}"),
                                }
                            } else if message.msg_type == MSG_LATENCY {
                                match decode_policy_latency(&message) {
                                    Ok(latency) => shared.lock().unwrap().update_latency(latency),
                                    Err(err) => eprintln!("latency decode failed: {err}"),
                                }
                            }
                        }
                    }
                }
                Err(err) => {
                    let mut snapshot = Map::new();
                    snapshot.insert("source".to_string(), json!("cdc"));
                    snapshot.insert("connected".to_string(), json!(false));
                    snapshot.insert("host_time_s".to_string(), json!(unix_time_s()));
                    snapshot.insert("port".to_string(), json!(port));
                    snapshot.insert("error".to_string(), json!(err.to_string()));
                    shared.lock().unwrap().update(snapshot, None);
                    thread::sleep(Duration::from_secs(1));
                }
            }
        }
    });
}

fn spawn_remote_reader(
    shared: Arc<Mutex<SharedSnapshot>>,
    stop: Arc<AtomicBool>,
    remote_url: String,
    timeout_s: f64,
) {
    thread::spawn(move || {
        while !stop.load(Ordering::SeqCst) {
            let events_url = format!("{}/events", remote_url.trim_end_matches('/'));
            match read_remote_events(&events_url, timeout_s, shared.clone(), &remote_url, &stop) {
                Ok(()) => {}
                Err(err) => {
                    let mut snapshot = Map::new();
                    snapshot.insert("source".to_string(), json!("remote"));
                    snapshot.insert("connected".to_string(), json!(false));
                    snapshot.insert("host_time_s".to_string(), json!(unix_time_s()));
                    snapshot.insert("remote_url".to_string(), json!(remote_url));
                    snapshot.insert("error".to_string(), json!(err.to_string()));
                    shared.lock().unwrap().update(snapshot, None);
                    thread::sleep(Duration::from_secs(1));
                }
            }
        }
    });
}

fn read_remote_events(
    events_url: &str,
    _timeout_s: f64,
    shared: Arc<Mutex<SharedSnapshot>>,
    remote_url: &str,
    stop: &AtomicBool,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let (host, port, path) = parse_http_url(events_url)?;
    let mut stream = TcpStream::connect((host.as_str(), port))?;
    stream.write_all(
        format!(
            "GET {path} HTTP/1.1\r\nHost: {host}\r\nAccept: text/event-stream\r\nConnection: close\r\n\r\n"
        )
        .as_bytes(),
    )?;
    let mut reader = BufReader::new(stream);
    let mut line = String::new();
    let mut in_headers = true;
    while !stop.load(Ordering::SeqCst) {
        line.clear();
        if reader.read_line(&mut line)? == 0 {
            break;
        }
        let text = line.trim_end_matches(['\r', '\n']);
        if in_headers {
            if text.is_empty() {
                in_headers = false;
            }
            continue;
        }
        if let Some(data) = text.strip_prefix("data:") {
            let mut snapshot: Map<String, Value> = serde_json::from_str(data.trim())?;
            snapshot.insert("source".to_string(), json!("remote"));
            snapshot.insert(
                "remote_url".to_string(),
                json!(remote_url.trim_end_matches('/')),
            );
            let state = snapshot_to_state(&snapshot);
            shared.lock().unwrap().update(snapshot, state);
        }
    }
    Ok(())
}

fn handle_client(
    mut stream: TcpStream,
    shared: Arc<Mutex<SharedSnapshot>>,
) -> Result<(), VisualizerError> {
    let mut reader = BufReader::new(stream.try_clone()?);
    let mut first_line = String::new();
    if reader.read_line(&mut first_line)? == 0 {
        return Ok(());
    }
    let mut parts = first_line.split_whitespace();
    let method = parts.next().unwrap_or("");
    let target = parts.next().unwrap_or("/");
    if method != "GET" {
        write_response(&mut stream, 405, "text/plain", b"method not allowed")?;
        return Ok(());
    }
    let path = target.split('?').next().unwrap_or(target);
    match path {
        "/" | "/index.html" => write_response(
            &mut stream,
            200,
            "text/html; charset=utf-8",
            INDEX_HTML.as_bytes(),
        )?,
        "/snapshot" => {
            let snapshot = Value::Object(shared.lock().unwrap().get());
            let payload = serde_json::to_vec(&snapshot)?;
            write_response(&mut stream, 200, "application/json", &payload)?;
        }
        "/events" => stream_events(stream, shared)?,
        "/render_info" => {
            let payload = serde_json::to_vec(&json!({
                "enabled": false,
                "ready": false,
                "backend": "disabled",
                "model_kind": "none",
                "render_fps": null,
                "error": "MJCF rendering is not linked in the Rust visualizer",
            }))?;
            write_response(&mut stream, 200, "application/json", &payload)?;
        }
        "/render_settings" => {
            let payload = serde_json::to_vec(&json!({
                "enabled": false,
                "show_visual_model": false,
                "show_collision_model": false,
                "use_gravity_attitude": true,
                "joint_frames": {},
                "camera": {"azimuth": 135.0, "elevation": -20.0, "distance": 1.25},
            }))?;
            write_response(&mut stream, 200, "application/json", &payload)?;
        }
        "/render_stream" | "/render.png" | "/render.jpg" => {
            write_response(&mut stream, 503, "text/plain", b"render disabled")?;
        }
        _ => write_response(&mut stream, 404, "text/plain", b"not found")?,
    }
    Ok(())
}

fn stream_events(
    mut stream: TcpStream,
    shared: Arc<Mutex<SharedSnapshot>>,
) -> Result<(), VisualizerError> {
    stream.write_all(
        b"HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n",
    )?;
    let mut last_event_seq = u64::MAX;
    let start = Instant::now();
    while start.elapsed() < Duration::from_secs(3600) {
        let snapshot = shared.lock().unwrap().get();
        let event_seq = snapshot
            .get("_event_seq")
            .and_then(Value::as_u64)
            .unwrap_or(0);
        if event_seq != last_event_seq {
            let payload = serde_json::to_string(&Value::Object(snapshot))?;
            if stream
                .write_all(format!("data: {payload}\n\n").as_bytes())
                .is_err()
            {
                break;
            }
            last_event_seq = event_seq;
        }
        thread::sleep(Duration::from_millis(50));
    }
    Ok(())
}

fn write_response(
    stream: &mut TcpStream,
    status: u16,
    content_type: &str,
    payload: &[u8],
) -> Result<(), std::io::Error> {
    let reason = match status {
        200 => "OK",
        404 => "Not Found",
        405 => "Method Not Allowed",
        503 => "Service Unavailable",
        _ => "OK",
    };
    stream.write_all(
        format!(
            "HTTP/1.1 {status} {reason}\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
            payload.len()
        )
        .as_bytes(),
    )?;
    stream.write_all(payload)?;
    Ok(())
}

fn state_to_snapshot(
    state: &PolicyStateFrame,
    builder: &RecoveryObservationBuilder,
    source: &str,
    frame_hz: Option<f64>,
    port: Option<&str>,
) -> Map<String, Value> {
    let obs = builder.build(state, [0.0; 6]).ok();
    let joint_pos = state.joint_pos;
    let target_joint_pos = state.target_joint_pos;
    let joint_pos_error = wrap_angle_vec4(sub4(target_joint_pos, joint_pos));
    let joint_active = policy_active_angles(joint_pos);
    let target_active = policy_active_angles(target_joint_pos);
    let render_joint_pos = policy_to_output_pos([
        joint_pos[0] as f64,
        joint_pos[1] as f64,
        joint_pos[2] as f64,
        joint_pos[3] as f64,
    ])
    .map(|v| v as f32);
    let mut snapshot = Map::new();
    snapshot.insert("source".to_string(), json!(source));
    snapshot.insert("connected".to_string(), json!(true));
    snapshot.insert("host_time_s".to_string(), json!(unix_time_s()));
    snapshot.insert("seq".to_string(), json!(state.seq));
    snapshot.insert("tick_ms".to_string(), json!(state.tick_ms));
    snapshot.insert("target_seq".to_string(), json!(state.target_seq));
    snapshot.insert("target_age_ms".to_string(), json!(state.target_age_ms));
    snapshot.insert("target_valid".to_string(), json!(state.target_valid));
    snapshot.insert("rc_switch_r".to_string(), json!(state.rc_switch_r));
    snapshot.insert("output_enabled".to_string(), json!(state.output_enabled));
    snapshot.insert("frame_hz".to_string(), json!(frame_hz));
    snapshot.insert("base_ang_vel".to_string(), json!(state.base_ang_vel_body));
    snapshot.insert(
        "base_ang_vel_body".to_string(),
        json!(state.base_ang_vel_body),
    );
    snapshot.insert(
        "projected_gravity".to_string(),
        json!(state.projected_gravity),
    );
    snapshot.insert("joint_pos".to_string(), json!(state.joint_pos));
    snapshot.insert("render_joint_pos".to_string(), json!(render_joint_pos));
    snapshot.insert("joint_vel".to_string(), json!(state.joint_vel));
    snapshot.insert("wheel_pos".to_string(), json!(state.wheel_pos));
    snapshot.insert("wheel_vel".to_string(), json!(state.wheel_vel));
    snapshot.insert(
        "target_joint_pos".to_string(),
        json!(state.target_joint_pos),
    );
    snapshot.insert("joint_pos_error".to_string(), json!(joint_pos_error));
    snapshot.insert("joint_active".to_string(), json!(joint_active));
    snapshot.insert("target_active".to_string(), json!(target_active));
    snapshot.insert("hip_torque".to_string(), json!(state.hip_torque));
    snapshot.insert("wheel_torque".to_string(), json!(state.wheel_torque));
    snapshot.insert(
        "wheel_motor_torque".to_string(),
        json!(state.wheel_motor_torque),
    );
    snapshot.insert(
        "obs".to_string(),
        json!(obs.map(|result| observation_slices(result.obs))),
    );
    if let Some(port) = port {
        snapshot.insert("port".to_string(), json!(port));
    }
    snapshot
}

fn snapshot_to_state(snapshot: &Map<String, Value>) -> Option<PolicyStateFrame> {
    if !snapshot
        .get("connected")
        .and_then(Value::as_bool)
        .unwrap_or(false)
    {
        return None;
    }
    Some(PolicyStateFrame {
        seq: value_u32(snapshot, "seq"),
        tick_ms: value_u32(snapshot, "tick_ms"),
        target_seq: value_u32(snapshot, "target_seq"),
        target_age_ms: value_u16(snapshot, "target_age_ms"),
        target_valid: value_u8(snapshot, "target_valid"),
        rc_switch_r: value_u8(snapshot, "rc_switch_r"),
        output_enabled: value_u8(snapshot, "output_enabled"),
        base_ang_vel_body: array_from_snapshot::<3>(snapshot, "base_ang_vel", [0.0; 3]),
        projected_gravity: array_from_snapshot::<3>(
            snapshot,
            "projected_gravity",
            [0.0, 0.0, -1.0],
        ),
        joint_pos: array_from_snapshot::<4>(
            snapshot,
            "joint_pos",
            RobotConfig::default().default_dof_pos[..4]
                .try_into()
                .unwrap_or([0.0; 4])
                .map(|v| v as f32),
        ),
        joint_vel: array_from_snapshot::<4>(snapshot, "joint_vel", [0.0; 4]),
        wheel_pos: array_from_snapshot::<2>(snapshot, "wheel_pos", [0.0; 2]),
        wheel_vel: array_from_snapshot::<2>(snapshot, "wheel_vel", [0.0; 2]),
        target_joint_pos: array_from_snapshot::<4>(snapshot, "target_joint_pos", [0.0; 4]),
        hip_torque: array_from_snapshot::<4>(snapshot, "hip_torque", [0.0; 4]),
        wheel_torque: array_from_snapshot::<2>(snapshot, "wheel_torque", [0.0; 2]),
        wheel_motor_torque: array_from_snapshot::<2>(snapshot, "wheel_motor_torque", [0.0; 2]),
    })
}

fn observation_slices(obs: [f32; 32]) -> Map<String, Value> {
    let mut map = Map::new();
    map.insert("base_ang_vel[0:3]".to_string(), json!(&obs[0..3]));
    map.insert("projected_gravity[3:6]".to_string(), json!(&obs[3..6]));
    map.insert("command[6:11]".to_string(), json!(&obs[6..11]));
    map.insert("leg_pos[11:15]".to_string(), json!(&obs[11..15]));
    map.insert("leg_vel[15:19]".to_string(), json!(&obs[15..19]));
    map.insert("wheel_pos_zero[19:21]".to_string(), json!(&obs[19..21]));
    map.insert("wheel_vel[21:23]".to_string(), json!(&obs[21..23]));
    map.insert("last_action[23:29]".to_string(), json!(&obs[23..29]));
    map.insert("jump_command[29:32]".to_string(), json!(&obs[29..32]));
    map
}

fn latency_to_snapshot(latency: &PolicyLatencyFrame) -> Map<String, Value> {
    let mut snapshot = Map::new();
    snapshot.insert("policy_seq".to_string(), json!(latency.policy_seq));
    snapshot.insert(
        "rx_to_output_us".to_string(),
        json!(latency.rx_to_output_us),
    );
    snapshot.insert(
        "rx_to_output_ms".to_string(),
        json!(latency.rx_to_output_us as f64 / 1000.0),
    );
    snapshot.insert("output_enabled".to_string(), json!(latency.output_enabled));
    snapshot.insert("host_time_s".to_string(), json!(unix_time_s()));
    snapshot
}

fn attach_latency_snapshot(snapshot: &mut Map<String, Value>, latency: &Map<String, Value>) {
    snapshot.insert("latency".to_string(), Value::Object(latency.clone()));
    snapshot.insert(
        "latency_policy_seq".to_string(),
        latency.get("policy_seq").cloned().unwrap_or(json!(0)),
    );
    snapshot.insert(
        "rx_to_output_us".to_string(),
        latency.get("rx_to_output_us").cloned().unwrap_or(json!(0)),
    );
    snapshot.insert(
        "rx_to_output_ms".to_string(),
        latency
            .get("rx_to_output_ms")
            .cloned()
            .unwrap_or(json!(0.0)),
    );
    snapshot.insert(
        "latency_output_enabled".to_string(),
        latency.get("output_enabled").cloned().unwrap_or(json!(0)),
    );
}

fn parse_http_url(
    url: &str,
) -> Result<(String, u16, String), Box<dyn std::error::Error + Send + Sync>> {
    let rest = url
        .strip_prefix("http://")
        .ok_or_else(|| "only http:// remote URLs are supported by the Rust relay")?;
    let (host_port, path) = rest.split_once('/').unwrap_or((rest, ""));
    let (host, port) = if let Some((host, port)) = host_port.rsplit_once(':') {
        (host.to_string(), port.parse()?)
    } else {
        (host_port.to_string(), 80)
    };
    Ok((host, port, format!("/{}", path)))
}

fn array_from_snapshot<const N: usize>(
    snapshot: &Map<String, Value>,
    key: &str,
    fallback: [f32; N],
) -> [f32; N] {
    let Some(values) = snapshot.get(key).and_then(Value::as_array) else {
        return fallback;
    };
    if values.len() != N {
        return fallback;
    }
    let mut out = [0.0_f32; N];
    for (dst, value) in out.iter_mut().zip(values) {
        *dst = value.as_f64().unwrap_or(0.0) as f32;
    }
    out
}

fn value_u32(snapshot: &Map<String, Value>, key: &str) -> u32 {
    snapshot.get(key).and_then(Value::as_u64).unwrap_or(0) as u32
}

fn value_u16(snapshot: &Map<String, Value>, key: &str) -> u16 {
    snapshot.get(key).and_then(Value::as_u64).unwrap_or(0) as u16
}

fn value_u8(snapshot: &Map<String, Value>, key: &str) -> u8 {
    snapshot.get(key).and_then(Value::as_u64).unwrap_or(0) as u8
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

fn unix_time_s() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64()
}

const INDEX_HTML: &str = r###"<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SerialLeg CDC State</title>
  <style>
    :root { color-scheme: dark; --bg:#111418; --panel:#181d23; --line:#2b333d; --text:#e6edf3; --muted:#9da7b3; --cyan:#50d6ff; --amber:#ffc857; --red:#ff6b6b; --blue:#7aa2ff; }
    * { box-sizing: border-box; }
    body { margin:0; background:var(--bg); color:var(--text); font:13px/1.45 ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; overflow:hidden; }
    #app { display:grid; grid-template-columns:minmax(0,1fr) minmax(500px,34vw); height:100vh; }
    #viewWrap { position:relative; min-width:0; border-right:1px solid var(--line); background:#07090c; }
    #view { position:absolute; inset:0; width:100%; height:100%; display:block; }
    #hud { position:absolute; top:14px; left:14px; display:flex; gap:8px; flex-wrap:wrap; max-width:calc(100% - 28px); }
    .pill { padding:5px 8px; border:1px solid var(--line); background:rgba(24,29,35,.78); border-radius:6px; color:var(--muted); }
    .pill strong { color:var(--text); font-weight:650; }
    #side { overflow:auto; background:var(--panel); }
    .sideHeader { position:sticky; top:0; z-index:2; padding:16px 18px 12px; border-bottom:1px solid var(--line); background:rgba(24,29,35,.96); }
    #sideContent { display:grid; gap:16px; padding:14px 18px 18px; }
    .section { min-width:0; padding-top:12px; border-top:1px solid var(--line); }
    .section:first-child { padding-top:0; border-top:0; }
    h1,h2 { margin:0; font-size:14px; letter-spacing:0; font-weight:700; }
    h2 { margin-bottom:8px; color:var(--muted); font-size:12px; text-transform:uppercase; }
    .grid { display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:8px; }
    .field { min-width:0; padding:7px 8px; border:1px solid #27313b; border-radius:6px; background:#11161c; }
    .field.wide { grid-column:1 / -1; }
    .k { color:var(--muted); font-size:11px; line-height:1.2; margin-bottom:3px; }
    .v { font-variant-numeric:tabular-nums; white-space:pre-wrap; overflow-wrap:anywhere; }
    .rows { display:grid; gap:7px; }
    .barrow { display:grid; grid-template-columns:48px minmax(0,1fr) 76px; align-items:center; gap:8px; }
    .bar { height:8px; border-radius:999px; background:#0f1216; border:1px solid #27303a; overflow:hidden; position:relative; }
    .bar::before { content:""; position:absolute; left:50%; width:1px; top:0; bottom:0; background:#45515e; }
    .fill { position:absolute; top:0; bottom:0; left:50%; background:var(--cyan); }
    .fill.neg { left:auto; right:50%; background:var(--amber); }
    .value { text-align:right; font-variant-numeric:tabular-nums; color:var(--text); }
    #err { color:var(--red); margin-top:7px; min-height:16px; font-size:12px; }
    .obsList { display:grid; gap:8px; }
    .obsRow { display:grid; grid-template-columns:140px minmax(0,1fr); gap:8px; align-items:start; min-width:0; padding:7px 8px; border:1px solid #27313b; border-radius:6px; background:#11161c; }
    .obsName { color:var(--muted); font-size:11px; line-height:1.25; overflow-wrap:anywhere; }
    .obsValues { display:flex; flex-wrap:wrap; gap:4px; min-width:0; }
    .chip { min-width:54px; padding:2px 5px; border:1px solid #303945; border-radius:5px; background:#0d1116; color:var(--text); text-align:right; font-variant-numeric:tabular-nums; }
    @media (max-width:1180px){ #app{grid-template-columns:1fr;grid-template-rows:58vh 42vh;} #viewWrap{border-right:0;border-bottom:1px solid var(--line);} .obsRow{grid-template-columns:120px minmax(0,1fr);} }
  </style>
</head>
<body>
  <div id="app">
    <section id="viewWrap">
      <canvas id="view"></canvas>
      <div id="hud">
        <div class="pill">seq <strong id="hudSeq">-</strong></div>
        <div class="pill">hz <strong id="hudHz">-</strong></div>
        <div class="pill">source <strong id="hudSource">-</strong></div>
        <div class="pill">output <strong id="hudOutput">-</strong></div>
        <div class="pill">render <strong id="hudRender">canvas</strong></div>
      </div>
    </section>
    <aside id="side">
      <div class="sideHeader"><h1>SerialLeg CDC State</h1><div id="err"></div></div>
      <div id="sideContent">
        <section class="section"><h2>Comm</h2><div class="grid" id="commGrid"></div></section>
        <section class="section"><h2>Status</h2><div class="grid" id="statusGrid"></div></section>
        <section class="section"><h2>Joint Pos</h2><div class="rows" id="jointBars"></div></section>
        <section class="section"><h2>Joint Vel</h2><div class="rows" id="jointVelBars"></div></section>
        <section class="section"><h2>Wheel</h2><div class="rows" id="wheelBars"></div></section>
        <section class="section"><h2>Observation Slices</h2><div class="obsList" id="obsGrid"></div></section>
      </div>
    </aside>
  </div>
<script>
const canvas=document.getElementById("view"),ctx=canvas.getContext("2d");
const labels=["LF","LB","RF","RB"]; let snapshot=null,mouseDown=false,lastMouse=[0,0],camYaw=-0.65,camPitch=0.35,camScale=900;
function resize(){const dpr=window.devicePixelRatio||1,rect=canvas.getBoundingClientRect();canvas.width=Math.max(1,Math.floor(rect.width*dpr));canvas.height=Math.max(1,Math.floor(rect.height*dpr));ctx.setTransform(dpr,0,0,dpr,0,0);} window.addEventListener("resize",resize);resize();
canvas.addEventListener("mousedown",ev=>{mouseDown=true;lastMouse=[ev.clientX,ev.clientY];}); window.addEventListener("mouseup",()=>mouseDown=false); window.addEventListener("mousemove",ev=>{if(!mouseDown)return;const dx=ev.clientX-lastMouse[0],dy=ev.clientY-lastMouse[1];camYaw+=dx*.006;camPitch=clamp(camPitch+dy*.004,-1.1,1.1);lastMouse=[ev.clientX,ev.clientY];}); canvas.addEventListener("wheel",ev=>{ev.preventDefault();camScale=clamp(camScale*Math.exp(-ev.deltaY*.001),350,1800);},{passive:false});
function clamp(v,lo,hi){return Math.max(lo,Math.min(hi,v));} function fmt(v,n=3){if(v===null||v===undefined||Number.isNaN(Number(v)))return "-";return Number(v).toFixed(n);} function arr(v,n=3){if(!Array.isArray(v))return "-";return "["+v.map(x=>fmt(x,n)).join(", ")+"]";} function ageMs(t){if(!t)return "-";return fmt(Date.now()-Number(t)*1000,1);}
function connect(){const events=new EventSource("/events");events.onmessage=ev=>{snapshot=JSON.parse(ev.data);updatePanel(snapshot);};events.onerror=()=>document.getElementById("err").textContent="event stream reconnecting...";} connect();
function updatePanel(s){document.getElementById("err").textContent=s.error||"";document.getElementById("hudSeq").textContent=s.seq??"-";document.getElementById("hudHz").textContent=fmt(s.frame_hz,1);document.getElementById("hudSource").textContent=s.source??"-";document.getElementById("hudOutput").textContent=s.output_enabled?"on":"off"; const lat=s.latency||{}; setGrid("commGrid",[["connected",String(!!s.connected)],["source",s.source],["port",s.port],["state_hz",fmt(s.frame_hz,2)],["state_seq",s.seq],["state_age_ms",ageMs(s.host_time_s)],["target_seq",s.target_seq],["target_valid",s.target_valid],["target_age_ms",s.target_age_ms],["rx_to_output_ms",lat.rx_to_output_ms??s.rx_to_output_ms],["latency_policy_seq",lat.policy_seq??s.latency_policy_seq],["latency_age_ms",ageMs(lat.host_time_s)],["latency_output",lat.output_enabled??s.latency_output_enabled],["remote_url",s.remote_url]]); setGrid("statusGrid",[["tick_ms",s.tick_ms],["rc_switch_r",s.rc_switch_r],["output_enabled",s.output_enabled],["base_ang_vel",arr(s.base_ang_vel)],["projected_g",arr(s.projected_gravity)],["target_joint_pos",arr(s.target_joint_pos)],["joint_pos_error",arr(s.joint_pos_error)],["joint_active",arr(s.joint_active)],["target_active",arr(s.target_active)],["hip_torque",arr(s.hip_torque)],["wheel_torque",arr(s.wheel_torque)],["wheel_motor_torque",arr(s.wheel_motor_torque)]]); setBars("jointBars",labels,s.joint_pos||[],Math.PI); setBars("jointVelBars",labels,s.joint_vel||[],8); setBars("wheelBars",["L pos","R pos","L vel","R vel"],[...(s.wheel_pos||[]),...(s.wheel_vel||[])],60); setObsGrid("obsGrid",Object.entries(s.obs||{}));}
function setGrid(id,rows){const el=document.getElementById(id);el.innerHTML="";for(const [k,v] of rows){const f=document.createElement("div");f.className=Array.isArray(v)||String(v??"").length>24?"field wide":"field";f.innerHTML=`<div class="k"></div><div class="v"></div>`;f.children[0].textContent=k;f.children[1].textContent=Array.isArray(v)?arr(v):String(v??"-");el.appendChild(f);}}
function setObsGrid(id,entries){const el=document.getElementById(id);el.innerHTML="";for(const [name,values] of entries){const row=document.createElement("div");row.className="obsRow";const label=document.createElement("div");label.className="obsName";label.textContent=name;const wrap=document.createElement("div");wrap.className="obsValues";const items=Array.isArray(values)?values:[];for(const value of items){const chip=document.createElement("span");chip.className="chip";chip.textContent=fmt(value);wrap.appendChild(chip);}if(!items.length){const chip=document.createElement("span");chip.className="chip";chip.textContent="-";wrap.appendChild(chip);}row.appendChild(label);row.appendChild(wrap);el.appendChild(row);}}
function setBars(id,names,values,limit){const el=document.getElementById(id);el.innerHTML="";names.forEach((name,idx)=>{const value=Number(values[idx]||0),row=document.createElement("div");row.className="barrow";row.innerHTML=`<div class="k"></div><div class="bar"><div class="fill"></div></div><div class="value"></div>`;row.children[0].textContent=name;const fill=row.querySelector(".fill");fill.className="fill"+(value<0?" neg":"");fill.style.width=`${clamp(Math.abs(value)/limit,0,1)*50}%`;row.children[2].textContent=fmt(value);el.appendChild(row);});}
function rotX(p,a){const c=Math.cos(a),s=Math.sin(a);return[p[0],c*p[1]-s*p[2],s*p[1]+c*p[2]];} function rotY(p,a){const c=Math.cos(a),s=Math.sin(a);return[c*p[0]+s*p[2],p[1],-s*p[0]+c*p[2]];} function rotZ(p,a){const c=Math.cos(a),s=Math.sin(a);return[c*p[0]-s*p[1],s*p[0]+c*p[1],p[2]];} function add(a,b){return[a[0]+b[0],a[1]+b[1],a[2]+b[2]];}
function transformBody(p,g){g=g||[0,0,-1];const pitch=Math.asin(clamp(g[0]||0,-.9,.9)),roll=-Math.asin(clamp(g[1]||0,-.9,.9));return add(rotY(rotX(p,roll),pitch),[0,0,.22]);} function camera(p){let q=rotZ(p,camYaw);q=rotX(q,camPitch);const rect=canvas.getBoundingClientRect(),z=q[2]+1.3,f=camScale/Math.max(.35,z+1.8);return[rect.width*.5+q[0]*f,rect.height*.62-q[1]*f,z];}
function line(a,b,color,width=2){const pa=camera(a),pb=camera(b);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();ctx.moveTo(pa[0],pa[1]);ctx.lineTo(pb[0],pb[1]);ctx.stroke();} function dot(p,color,r=4){const pp=camera(p);ctx.fillStyle=color;ctx.beginPath();ctx.arc(pp[0],pp[1],r,0,Math.PI*2);ctx.fill();}
function cube(corners,color){[[0,1],[1,3],[3,2],[2,0],[4,5],[5,7],[7,6],[6,4],[0,4],[1,5],[2,6],[3,7]].forEach(([i,j])=>line(corners[i],corners[j],color,1.5));} function drawWheel(center,angle,color){const pp=camera(center),r=22;ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();ctx.arc(pp[0],pp[1],r,0,Math.PI*2);ctx.stroke();ctx.beginPath();ctx.moveTo(pp[0],pp[1]);ctx.lineTo(pp[0]+Math.cos(angle)*r,pp[1]+Math.sin(angle)*r);ctx.stroke();}
function draw(){resize();const rect=canvas.getBoundingClientRect();ctx.clearRect(0,0,rect.width,rect.height);ctx.fillStyle="#111418";ctx.fillRect(0,0,rect.width,rect.height);for(let x=-.6;x<=.6;x+=.1)line([x,-.35,0],[x,.35,0],"#222a33",1);for(let y=-.35;y<=.35;y+=.1)line([-.6,y,0],[.6,y,0],"#222a33",1);if(!snapshot||!snapshot.connected){ctx.fillStyle="#9da7b3";ctx.font="16px ui-sans-serif,system-ui";ctx.fillText("waiting for CDC state...",24,36);requestAnimationFrame(draw);return;}const q=snapshot.render_joint_pos||snapshot.joint_pos||[0,0,0,0],wp=snapshot.wheel_pos||[0,0],g=snapshot.projected_gravity||[0,0,-1],body=[];for(const x of[-.16,.16])for(const y of[-.11,.11])for(const z of[-.045,.045])body.push(transformBody([x,y,z],g));cube(body,"#7aa2ff");[{name:"L",y:-.13,front:0,back:1,wheel:0,color:"#50d6ff"},{name:"R",y:.13,front:2,back:3,wheel:1,color:"#ffc857"}].forEach(side=>{const fa=[-.08,side.y,-.045],ba=[.08,side.y,-.045],fe=[fa[0]+Math.sin(q[side.front])*.17,side.y,fa[2]-Math.cos(q[side.front])*.17],be=[ba[0]+Math.sin(q[side.back])*.17,side.y,ba[2]-Math.cos(q[side.back])*.17],wheel=[(fe[0]+be[0])*.5,side.y,Math.min(fe[2],be[2])-.065],a=transformBody(fa,g),b=transformBody(ba,g),f=transformBody(fe,g),bk=transformBody(be,g),wc=transformBody(wheel,g);line(a,f,side.color,4);line(b,bk,side.color,4);line(f,bk,"#56616f",2);line(f,wc,"#56616f",2);line(bk,wc,"#56616f",2);dot(a,"#e6edf3",3);dot(b,"#e6edf3",3);dot(f,side.color,4);dot(bk,side.color,4);drawWheel(wc,wp[side.wheel]||0,side.color);});requestAnimationFrame(draw);} draw();
</script>
</body>
</html>"###;
