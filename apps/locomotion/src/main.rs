use std::error::Error;
use std::path::PathBuf;

use clap::Parser;
use locomotion_core::recovery_runtime::{
    DEFAULT_CDC_PORT, RecoveryRuntime, RecoveryRuntimeConfig, RecoveryTransport, env_int,
    telemetry_log_path,
};
use zoo::RobotProfile;

const DEFAULT_ROBOT_ID: &str = "serial_leg_dev";

#[derive(Debug, Parser)]
#[command(about = "Run SerialLeg recovery-only policy runtime on Jetson Orin NX.")]
struct Args {
    #[arg(long, default_value = DEFAULT_ROBOT_ID)]
    robot: String,

    #[arg(long)]
    policy: Option<String>,

    #[arg(long)]
    checkpoint: Option<PathBuf>,

    #[arg(long = "ort-ep")]
    ort_ep: Option<String>,

    #[arg(long, value_parser = parse_transport, default_value = "cdc")]
    transport: RecoveryTransport,

    #[arg(long, default_value_t = default_port())]
    port: String,

    #[arg(long = "sim-socket-path")]
    sim_socket_path: Option<PathBuf>,

    #[arg(long = "sim-client-socket-path")]
    sim_client_socket_path: Option<PathBuf>,

    #[arg(long, default_value_t = 921600)]
    baudrate: i32,

    #[arg(long, default_value = "cpu")]
    device: String,

    #[arg(long = "rate-hz")]
    rate_hz: Option<f64>,

    #[arg(long = "state-timeout-s")]
    state_timeout_s: Option<f64>,

    #[arg(long = "write-timeout-s")]
    write_timeout_s: Option<f64>,

    #[arg(long = "max-steps", default_value_t = 0)]
    max_steps: usize,

    #[arg(long)]
    dry_run: bool,

    #[arg(long = "print-every", default_value_t = 50)]
    print_every: usize,

    #[arg(long = "telemetry-log")]
    telemetry_log: Option<String>,

    #[arg(long = "telemetry-log-every", default_value_t = default_telemetry_log_every())]
    telemetry_log_every: usize,

    #[arg(long = "telemetry-flush-every", default_value_t = default_telemetry_flush_every())]
    telemetry_flush_every: usize,
}

fn main() {
    if let Err(err) = run_main() {
        eprintln!("Error: {err}");
        let mut source = err.source();
        while let Some(err) = source {
            eprintln!("  caused by: {err}");
            source = err.source();
        }
        std::process::exit(1);
    }
}

fn run_main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let robot = zoo::get_robot(&args.robot)?;
    let policy = resolve_policy(&robot, args.policy.as_deref())?;
    eprintln!("selected robot={} policy={}", robot.id, policy.id);

    let checkpoint = args
        .checkpoint
        .or_else(|| std::env::var_os("SE3_RECOVERY_CHECKPOINT").map(PathBuf::from))
        .or_else(|| policy.checkpoint.clone());
    let cfg = RecoveryRuntimeConfig {
        checkpoint: checkpoint.ok_or_else(missing_checkpoint_error)?,
        ort_ep: args.ort_ep.unwrap_or_else(|| policy.ort_ep.clone()),
        transport: args.transport,
        port: args.port,
        sim_socket_path: args
            .sim_socket_path
            .unwrap_or_else(|| robot.locomotion.sim_socket_path.clone()),
        sim_client_socket_path: args
            .sim_client_socket_path
            .unwrap_or_else(|| robot.locomotion.sim_client_socket_path.clone()),
        baudrate: args.baudrate,
        device: args.device,
        rate_hz: args.rate_hz.unwrap_or(robot.locomotion.rate_hz),
        state_timeout_s: args
            .state_timeout_s
            .unwrap_or(robot.locomotion.state_timeout_s),
        write_timeout_s: args
            .write_timeout_s
            .unwrap_or(robot.locomotion.write_timeout_s),
        max_steps: args.max_steps,
        dry_run: args.dry_run,
        print_every: args.print_every,
        telemetry_log: telemetry_log_path(
            args.telemetry_log
                .or_else(|| std::env::var("SE3_TELEMETRY_LOG").ok()),
        ),
        telemetry_log_every: args.telemetry_log_every,
        telemetry_flush_every: args.telemetry_flush_every,
    };
    let mut runtime = RecoveryRuntime::new(cfg)?;
    runtime.run()?;
    Ok(())
}

fn resolve_policy<'a>(
    robot: &'a RobotProfile,
    requested_policy_id: Option<&str>,
) -> Result<&'a zoo::PolicyProfile, std::io::Error> {
    match requested_policy_id {
        Some(policy_id) => robot.policy(policy_id).map_err(|_| {
            let available = robot
                .policies
                .iter()
                .map(|policy| policy.id.as_str())
                .collect::<Vec<_>>()
                .join(", ");
            std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!(
                    "unknown policy `{policy_id}` for robot `{}` (available: [{available}])",
                    robot.id
                ),
            )
        }),
        None => robot
            .default_policy()
            .map_err(|err| std::io::Error::new(std::io::ErrorKind::InvalidInput, err.to_string())),
    }
}

fn missing_checkpoint_error() -> std::io::Error {
    std::io::Error::new(
        std::io::ErrorKind::InvalidInput,
        "missing checkpoint; provide --checkpoint, SE3_RECOVERY_CHECKPOINT, or zoo policy checkpoint",
    )
}

fn default_port() -> String {
    std::env::var("SE3_CDC_PORT").unwrap_or_else(|_| DEFAULT_CDC_PORT.to_string())
}

fn parse_transport(value: &str) -> Result<RecoveryTransport, String> {
    match value {
        "cdc" => Ok(RecoveryTransport::Cdc),
        "sim" => Ok(RecoveryTransport::Sim),
        _ => Err(format!("unsupported transport: {value}")),
    }
}

fn default_telemetry_log_every() -> usize {
    env_int("SE3_TELEMETRY_LOG_EVERY", 1)
}

fn default_telemetry_flush_every() -> usize {
    env_int("SE3_TELEMETRY_FLUSH_EVERY", 25)
}
