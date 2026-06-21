use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use log::{info, warn};
use se3_input::{GamepadInput, GamepadSelector, GamepadSnapshot, InputError};

const DEFAULT_ROBOT_ID: &str = "serial_leg_dev";

#[derive(Debug, Parser)]
#[command(about = "Debug XInput gamepad sampling and zoo command mapping.")]
struct Args {
    #[arg(long, default_value = DEFAULT_ROBOT_ID)]
    robot: String,

    #[arg(long = "gamepad", default_value = "auto")]
    gamepad: String,

    #[arg(long = "rate-hz", default_value_t = 20.0)]
    rate_hz: f64,

    #[arg(long)]
    guided: bool,

    #[arg(long = "show-all")]
    show_all: bool,

    #[arg(long = "max-samples", default_value_t = 0)]
    max_samples: usize,

    #[arg(long = "step-timeout-s", default_value_t = 12.0)]
    step_timeout_s: f64,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let _logger_guard = se3_log::init(&se3_log::LoggerConfig::new("info", "info", true, false))?;

    let robot = zoo::get_robot(&args.robot)?;
    let profile = robot.command.gamepad.clone().ok_or_else(|| {
        std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            format!(
                "robot `{}` does not define a gamepad command profile",
                robot.id
            ),
        )
    })?;
    let selector = GamepadSelector::parse(&args.gamepad)?;
    let mut input = GamepadInput::new(selector)?;
    log_connected_gamepads(&mut input);

    let initial_height = robot
        .command
        .fixed
        .chassis
        .map(|command| command.height_m)
        .unwrap_or(robot.locomotion.robot_cfg.default_base_height as f32);
    let mut state = profile.initial_state(initial_height);
    let period = Duration::from_secs_f64(1.0 / args.rate_hz.max(1.0));

    if args.guided {
        run_guided(
            &mut input,
            &profile,
            &mut state,
            period,
            Duration::from_secs_f64(args.step_timeout_s.max(1.0)),
        )?;
    } else {
        run_continuous(
            &mut input,
            &profile,
            &mut state,
            period,
            args.show_all,
            args.max_samples,
        );
    }

    Ok(())
}

fn log_connected_gamepads(input: &mut GamepadInput) {
    let gamepads = input.connected_gamepads();
    if gamepads.is_empty() {
        warn!("no connected gamepads detected yet");
        return;
    }
    for (id, name) in gamepads {
        info!("gamepad {id}: {name}");
    }
}

fn run_continuous(
    input: &mut GamepadInput,
    profile: &zoo::GamepadCommandProfile,
    state: &mut zoo::GamepadCommandState,
    period: Duration,
    show_all: bool,
    max_samples: usize,
) {
    info!(
        "continuous mode: press controls and watch raw dpad/button values plus mapped command fields"
    );
    let mut previous_signature = String::new();
    let mut sample_count = 0_usize;
    loop {
        sample_count += 1;
        match input.poll() {
            Ok(snapshot) => {
                let command = profile.command_with_state(&snapshot, state);
                let signature = sample_signature(&snapshot, command);
                if show_all || signature != previous_signature || sample_count.is_multiple_of(50) {
                    info!("sample={} {}", sample_count, signature);
                    previous_signature = signature;
                }
            }
            Err(InputError::NoConnectedGamepad) => {
                if sample_count == 1 || sample_count.is_multiple_of(20) {
                    warn!("sample={sample_count} no connected gamepad");
                }
            }
            Err(err) => warn!("sample={sample_count} xinput sample failed: {err}"),
        }
        if max_samples != 0 && sample_count >= max_samples {
            break;
        }
        thread::sleep(period);
    }
}

fn run_guided(
    input: &mut GamepadInput,
    profile: &zoo::GamepadCommandProfile,
    state: &mut zoo::GamepadCommandState,
    period: Duration,
    timeout: Duration,
) -> Result<(), Box<dyn std::error::Error>> {
    info!("guided mode: follow each prompt, then release the control before the next prompt");
    let neutral = wait_for(
        input,
        profile,
        state,
        period,
        timeout,
        "release D-pad and face buttons",
        |snapshot, _command| dpad_neutral(snapshot) && !snapshot.east && !snapshot.south,
    )?;
    info!(
        "neutral {}",
        sample_signature(&neutral.snapshot, neutral.command)
    );

    let start_height = chassis_height(neutral.command);
    let up = wait_for(
        input,
        profile,
        state,
        period,
        timeout,
        "press physical D-pad Up once",
        |snapshot, command| snapshot.dpad_y.abs() > 0.5 || chassis_height(command) != start_height,
    )?;
    let up_delta = chassis_height(up.command) - start_height;
    report_height_step("physical D-pad Up", up.snapshot.dpad_y, up_delta);
    wait_for_neutral(input, profile, state, period, timeout)?;

    let before_down = chassis_height(profile.command_with_state(&neutral.snapshot, state));
    let down = wait_for(
        input,
        profile,
        state,
        period,
        timeout,
        "press physical D-pad Down once",
        |snapshot, command| snapshot.dpad_y.abs() > 0.5 || chassis_height(command) != before_down,
    )?;
    let down_delta = chassis_height(down.command) - before_down;
    report_height_step("physical D-pad Down", down.snapshot.dpad_y, down_delta);
    wait_for_neutral(input, profile, state, period, timeout)?;

    let left = wait_for(
        input,
        profile,
        state,
        period,
        timeout,
        "press physical D-pad Left",
        |snapshot, command| snapshot.dpad_x.abs() > 0.5 || chassis_roll(command).abs() > 1.0e-6,
    )?;
    info!(
        "physical D-pad Left: raw_dpad_x={:+.1} cmd_roll={:+.3}",
        left.snapshot.dpad_x,
        chassis_roll(left.command)
    );
    wait_for_neutral(input, profile, state, period, timeout)?;

    let right = wait_for(
        input,
        profile,
        state,
        period,
        timeout,
        "press physical D-pad Right",
        |snapshot, command| snapshot.dpad_x.abs() > 0.5 || chassis_roll(command).abs() > 1.0e-6,
    )?;
    info!(
        "physical D-pad Right: raw_dpad_x={:+.1} cmd_roll={:+.3}",
        right.snapshot.dpad_x,
        chassis_roll(right.command)
    );
    wait_for_neutral(input, profile, state, period, timeout)?;

    let enabled_before = state.controls_enabled();
    let east = wait_for(
        input,
        profile,
        state,
        period,
        timeout,
        "press East/B once",
        |snapshot, _command| snapshot.east,
    )?;
    info!(
        "East/B: raw_east={} controls_enabled {} -> {} {}",
        east.snapshot.east,
        enabled_before,
        state.controls_enabled(),
        sample_signature(&east.snapshot, east.command)
    );
    wait_for_neutral(input, profile, state, period, timeout)?;

    info!(
        "guided check complete: final controls_enabled={} final_cmd_h={:+.3}",
        state.controls_enabled(),
        chassis_height(profile.command_with_state(&neutral.snapshot, state))
    );
    Ok(())
}

fn wait_for_neutral(
    input: &mut GamepadInput,
    profile: &zoo::GamepadCommandProfile,
    state: &mut zoo::GamepadCommandState,
    period: Duration,
    timeout: Duration,
) -> Result<Sample, Box<dyn std::error::Error>> {
    wait_for(
        input,
        profile,
        state,
        period,
        timeout,
        "release D-pad",
        |snapshot, _command| dpad_neutral(snapshot),
    )
}

fn wait_for<F>(
    input: &mut GamepadInput,
    profile: &zoo::GamepadCommandProfile,
    state: &mut zoo::GamepadCommandState,
    period: Duration,
    timeout: Duration,
    prompt: &str,
    mut predicate: F,
) -> Result<Sample, Box<dyn std::error::Error>>
where
    F: FnMut(&GamepadSnapshot, se3_command::Command) -> bool,
{
    info!("{prompt}");
    let started = Instant::now();
    loop {
        match input.poll() {
            Ok(snapshot) => {
                let command = profile.command_with_state(&snapshot, state);
                if predicate(&snapshot, command) {
                    return Ok(Sample { snapshot, command });
                }
            }
            Err(InputError::NoConnectedGamepad) => {
                if started.elapsed().as_millis() % 1_000 < period.as_millis().max(1) {
                    warn!("waiting for gamepad connection");
                }
            }
            Err(err) => warn!("xinput sample failed: {err}"),
        }
        if started.elapsed() >= timeout {
            return Err(std::io::Error::new(
                std::io::ErrorKind::TimedOut,
                format!("timed out while waiting to {prompt}"),
            )
            .into());
        }
        thread::sleep(period);
    }
}

fn report_height_step(label: &str, raw_dpad_y: f32, height_delta: f32) {
    let result = if height_delta > 1.0e-6 {
        "height increased"
    } else if height_delta < -1.0e-6 {
        "height decreased"
    } else {
        "height unchanged"
    };
    info!("{label}: raw_dpad_y={raw_dpad_y:+.1} cmd_h_delta={height_delta:+.3} ({result})");
    if label.ends_with("Up") && height_delta <= 0.0 {
        warn!("physical D-pad Up did not increase cmd_h with the current mapping");
    }
    if label.ends_with("Down") && height_delta >= 0.0 {
        warn!("physical D-pad Down did not decrease cmd_h with the current mapping");
    }
}

fn sample_signature(snapshot: &GamepadSnapshot, command: se3_command::Command) -> String {
    let policy = command
        .chassis
        .map(|chassis| chassis.to_policy_command())
        .unwrap_or([0.0; 8]);
    format!(
        "id={} name=`{}` raw_dpad=({:+.1},{:+.1}) east={} south={} left_y={:+.2} right_x={:+.2} cmd_vx={:+.3} cmd_yaw={:+.3} cmd_roll={:+.3} cmd_h={:+.3} cmd_jump={}",
        snapshot.id,
        snapshot.name,
        snapshot.dpad_x,
        snapshot.dpad_y,
        snapshot.east,
        snapshot.south,
        snapshot.left_stick_y,
        snapshot.right_stick_x,
        policy[0],
        policy[1],
        policy[3],
        policy[4],
        policy[5] > 0.5,
    )
}

fn dpad_neutral(snapshot: &GamepadSnapshot) -> bool {
    snapshot.dpad_x.abs() <= 0.5 && snapshot.dpad_y.abs() <= 0.5
}

fn chassis_height(command: se3_command::Command) -> f32 {
    command
        .chassis
        .map(|chassis| chassis.height_m)
        .unwrap_or(0.0)
}

fn chassis_roll(command: se3_command::Command) -> f32 {
    command
        .chassis
        .map(|chassis| chassis.roll_rad)
        .unwrap_or(0.0)
}

struct Sample {
    snapshot: GamepadSnapshot,
    command: se3_command::Command,
}
