use std::error::Error;
use std::path::PathBuf;

use clap::Parser;
use locomotion_core::policy_runtime::{
    DEFAULT_CDC_PORT, LocomotionPolicyConfig, LocomotionPolicyRuntime, LocomotionTransport,
    RuntimeCommandSample, RuntimeCommandSource, env_int, telemetry_log_path,
};
use log::{error, info, warn};
use se3_command::{Command, CommandSourceKind};
use se3_input::{GamepadInput, GamepadSelector, GamepadSnapshot, InputError};
use zoo::RobotProfile;

const DEFAULT_ROBOT_ID: &str = "serial_leg_dev";

#[derive(Debug, Parser)]
#[command(about = "Run SerialLeg locomotion policy runtime on Jetson Orin NX.")]
struct Args {
    #[arg(long, default_value = DEFAULT_ROBOT_ID)]
    robot: String,

    #[arg(long)]
    policy: Option<String>,

    #[arg(long)]
    checkpoint: Option<PathBuf>,

    #[arg(long = "ort-ep")]
    ort_ep: Option<String>,

    #[arg(long = "command-source", value_parser = parse_command_source)]
    command_source: Option<CommandSourceKind>,

    #[arg(long = "gamepad", default_value = "auto")]
    gamepad: String,

    #[arg(long = "list-gamepads")]
    list_gamepads: bool,

    #[arg(long, value_parser = parse_transport, default_value = "cdc")]
    transport: LocomotionTransport,

    #[arg(long, default_value_t = default_port())]
    port: String,

    #[arg(long = "sim-socket-path")]
    sim_socket_path: Option<PathBuf>,

    #[arg(long = "sim-client-socket-path")]
    sim_client_socket_path: Option<PathBuf>,

    #[arg(long, default_value_t = 921600)]
    baudrate: i32,

    #[arg(long, default_value = "auto")]
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

fn main() -> Result<(), Box<dyn Error>> {
    match run_main() {
        Ok(()) => Ok(()),
        Err(err) if se3_log::is_initialized() => {
            report_error(err.as_ref());
            se3_log::flush();
            std::process::exit(1);
        }
        Err(err) => Err(err),
    }
}

fn report_error(err: &dyn Error) {
    error!("Error: {err}");
    let mut source = err.source();
    while let Some(err) = source {
        error!("  caused by: {err}");
        source = err.source();
    }
}

fn run_main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let _logger_guard = se3_log::init(&locomotion_log_config())?;
    if args.list_gamepads {
        list_gamepads(&args.gamepad)?;
        return Ok(());
    }
    let robot = zoo::get_robot(&args.robot)?;
    let policy = resolve_policy(&robot, args.policy.as_deref())?;
    info!("selected robot={} policy={}", robot.id, policy.id);
    let command_source_kind = args.command_source.unwrap_or(robot.command.default_source);
    let command_source = build_command_source(&robot, command_source_kind, &args.gamepad)?;

    let checkpoint = args
        .checkpoint
        .or_else(|| std::env::var_os("SE3_RECOVERY_CHECKPOINT").map(PathBuf::from))
        .or_else(|| policy.checkpoint.clone());
    let cfg = LocomotionPolicyConfig {
        checkpoint: checkpoint.ok_or_else(missing_checkpoint_error)?,
        ort_ep: args.ort_ep.unwrap_or_else(|| policy.ort_ep.clone()),
        command_source: command_source_kind,
        fixed_command: robot.command.fixed,
        robot_cfg: robot.locomotion.robot_cfg.clone(),
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
    let mut runtime = LocomotionPolicyRuntime::new_with_command_source(cfg, command_source)?;
    runtime.run()?;
    Ok(())
}

fn build_command_source(
    robot: &RobotProfile,
    source: CommandSourceKind,
    gamepad_selector: &str,
) -> Result<Box<dyn RuntimeCommandSource>, Box<dyn Error>> {
    match source {
        CommandSourceKind::Fixed => Ok(Box::new(FixedCommandSource::new(robot.command.fixed))),
        CommandSourceKind::XInput => {
            let profile = robot.command.gamepad.clone().ok_or_else(|| {
                std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    format!(
                        "robot `{}` does not define a gamepad command profile",
                        robot.id
                    ),
                )
            })?;
            let selector = GamepadSelector::parse(gamepad_selector)?;
            let input = GamepadInput::new(selector)?;
            let initial_height = robot
                .command
                .fixed
                .chassis
                .map(|command| command.height_m)
                .unwrap_or(robot.locomotion.robot_cfg.default_base_height as f32);
            let state = profile.initial_state(initial_height);
            Ok(Box::new(GamepadCommandSource {
                input,
                profile,
                state,
                fallback: robot.command.fixed,
            }))
        }
    }
}

fn list_gamepads(selector: &str) -> Result<(), Box<dyn Error>> {
    let selector = GamepadSelector::parse(selector)?;
    let mut input = GamepadInput::new(selector)?;
    let mut gamepads = Vec::new();
    for _ in 0..20 {
        gamepads = input.connected_gamepads();
        if !gamepads.is_empty() {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
    if gamepads.is_empty() {
        info!("no connected gamepads");
    } else {
        for (id, name) in gamepads {
            info!("gamepad {id}: {name}");
        }
    }
    Ok(())
}

struct FixedCommandSource {
    command: Command,
}

impl FixedCommandSource {
    fn new(command: Command) -> Self {
        Self { command }
    }
}

impl RuntimeCommandSource for FixedCommandSource {
    fn sample(&mut self) -> RuntimeCommandSample {
        RuntimeCommandSample::fixed(self.command)
    }
}

struct GamepadCommandSource {
    input: GamepadInput,
    profile: zoo::GamepadCommandProfile,
    state: zoo::GamepadCommandState,
    fallback: Command,
}

impl RuntimeCommandSource for GamepadCommandSource {
    fn sample(&mut self) -> RuntimeCommandSample {
        match self.input.poll() {
            Ok(snapshot) => self.sample_from_snapshot(&snapshot),
            Err(InputError::NoConnectedGamepad) => RuntimeCommandSample::xinput_idle(self.fallback),
            Err(err) => {
                warn!("xinput sample failed: {err}");
                RuntimeCommandSample::xinput_idle(self.fallback)
            }
        }
    }
}

impl GamepadCommandSource {
    fn sample_from_snapshot(&mut self, snapshot: &GamepadSnapshot) -> RuntimeCommandSample {
        let command = self.profile.command_with_state(snapshot, &mut self.state);
        RuntimeCommandSample {
            command,
            source: CommandSourceKind::XInput,
            active: snapshot.connected && self.state.controls_enabled(),
            device_id: Some(snapshot.id),
            device_name: Some(snapshot.name.clone()),
        }
    }
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

fn parse_transport(value: &str) -> Result<LocomotionTransport, String> {
    match value {
        "cdc" => Ok(LocomotionTransport::Cdc),
        "sim" => Ok(LocomotionTransport::Sim),
        _ => Err(format!("unsupported transport: {value}")),
    }
}

fn parse_command_source(value: &str) -> Result<CommandSourceKind, String> {
    CommandSourceKind::parse(value).map_err(|err| err.to_string())
}

fn default_telemetry_log_every() -> usize {
    env_int("SE3_TELEMETRY_LOG_EVERY", 1)
}

fn default_telemetry_flush_every() -> usize {
    env_int("SE3_TELEMETRY_FLUSH_EVERY", 25)
}

fn locomotion_log_config() -> se3_log::LoggerConfig {
    se3_log::LoggerConfig::new(
        "info,locomotion=debug,locomotion_core=debug,ort=warn",
        "info,locomotion=debug,locomotion_core=debug,ort=info",
        true,
        true,
    )
}
