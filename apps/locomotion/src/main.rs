use std::error::Error;
use std::path::PathBuf;

use clap::Parser;
use locomotion_core::recovery_runtime::{
    DEFAULT_CDC_PORT, RecoveryRuntime, RecoveryRuntimeConfig, RecoveryTransport, env_int,
    telemetry_log_path,
};

#[derive(Debug, Parser)]
#[command(about = "Run SerialLeg recovery-only policy runtime on Jetson Orin NX.")]
struct Args {
    #[arg(long)]
    checkpoint: Option<PathBuf>,

    #[arg(long = "ort-ep", default_value = "auto")]
    ort_ep: String,

    #[arg(long, value_parser = parse_transport, default_value = "cdc")]
    transport: RecoveryTransport,

    #[arg(long, default_value_t = default_port())]
    port: String,

    #[arg(long = "sim-socket-path", default_value = "/tmp/se3_sim_loop.sock")]
    sim_socket_path: PathBuf,

    #[arg(
        long = "sim-client-socket-path",
        default_value = "/tmp/se3_locomotion.sock"
    )]
    sim_client_socket_path: PathBuf,

    #[arg(long, default_value_t = 921600)]
    baudrate: i32,

    #[arg(long, default_value = "cpu")]
    device: String,

    #[arg(long = "rate-hz", default_value_t = 50.0)]
    rate_hz: f64,

    #[arg(long = "state-timeout-s", default_value_t = 0.10)]
    state_timeout_s: f64,

    #[arg(long = "write-timeout-s", default_value_t = 0.02)]
    write_timeout_s: f64,

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
    let checkpoint = args
        .checkpoint
        .or_else(|| std::env::var_os("SE3_RECOVERY_CHECKPOINT").map(PathBuf::from));
    let cfg = RecoveryRuntimeConfig {
        checkpoint: checkpoint.ok_or_else(|| {
            std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "missing --checkpoint or SE3_RECOVERY_CHECKPOINT",
            )
        })?,
        ort_ep: args.ort_ep,
        transport: args.transport,
        port: args.port,
        sim_socket_path: args.sim_socket_path,
        sim_client_socket_path: args.sim_client_socket_path,
        baudrate: args.baudrate,
        device: args.device,
        rate_hz: args.rate_hz,
        state_timeout_s: args.state_timeout_s,
        write_timeout_s: args.write_timeout_s,
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
