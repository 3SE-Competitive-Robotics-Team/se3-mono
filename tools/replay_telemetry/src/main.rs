use std::path::PathBuf;
use std::process::ExitCode;

use clap::Parser;
use locomotion_core::replay_telemetry::{ReplayConfig, replay_telemetry};

#[derive(Debug, Parser)]
#[command(about = "Replay NX recovery telemetry JSONL with a local ONNX Runtime policy.")]
struct Args {
    telemetry: PathBuf,

    #[arg(long)]
    checkpoint: Option<PathBuf>,

    #[arg(long = "ort-ep", default_value = "auto")]
    ort_ep: String,

    #[arg(long)]
    meta: Option<PathBuf>,

    #[arg(long = "max-rows", default_value_t = 0)]
    max_rows: usize,

    #[arg(long = "print-every", default_value_t = 500)]
    print_every: usize,

    #[arg(long = "report-json")]
    report_json: Option<PathBuf>,

    #[arg(long = "fail-action-error")]
    fail_action_error: Option<f64>,
}

fn main() -> Result<ExitCode, Box<dyn std::error::Error>> {
    let args = Args::parse();
    let _logger_guard = se3_log::init(&se3_log::LoggerConfig::new(
        "info,locomotion_core=debug,ort=warn",
        "info,locomotion_core=debug,ort=info",
        true,
        true,
    ))?;
    let exit_code = replay_telemetry(ReplayConfig {
        telemetry: args.telemetry,
        checkpoint: args.checkpoint,
        ort_ep: args.ort_ep,
        meta: args.meta,
        max_rows: args.max_rows,
        print_every: args.print_every,
        report_json: args.report_json,
        fail_action_error: args.fail_action_error,
    })?;
    drop(_logger_guard);
    Ok(ExitCode::from(exit_code as u8))
}
