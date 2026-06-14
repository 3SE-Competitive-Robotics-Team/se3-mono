use clap::Parser;
use locomotion_core::visualize_cdc_state::{VisualizerConfig, run_visualizer};

#[derive(Debug, Parser)]
#[command(about = "Web visualizer for STM32 CDC state frames.")]
struct Args {
    #[arg(long, default_value = "auto")]
    port: String,

    #[arg(long, default_value_t = 921600)]
    baudrate: i32,

    #[arg(long, default_value = "127.0.0.1")]
    host: String,

    #[arg(long = "viewer-port", default_value_t = 8097)]
    viewer_port: u16,

    #[arg(long)]
    synthetic: bool,

    #[arg(long = "local-cdc")]
    local_cdc: bool,

    #[arg(long = "remote-url", default_value = "http://192.168.137.100:8081")]
    remote_url: String,

    #[arg(long = "remote-timeout-s", default_value_t = 10.0)]
    remote_timeout_s: f64,

    #[arg(long = "rate-hz", default_value_t = 50.0)]
    rate_hz: f64,

    #[arg(long = "read-timeout-s", default_value_t = 0.02)]
    read_timeout_s: f64,

    #[arg(long = "no-mjcf-render")]
    no_mjcf_render: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    run_visualizer(VisualizerConfig {
        port: args.port,
        baudrate: args.baudrate,
        host: args.host,
        viewer_port: args.viewer_port,
        synthetic: args.synthetic,
        local_cdc: args.local_cdc,
        remote_url: args.remote_url,
        remote_timeout_s: args.remote_timeout_s,
        rate_hz: args.rate_hz,
        read_timeout_s: args.read_timeout_s,
        no_mjcf_render: args.no_mjcf_render,
    })?;
    Ok(())
}
