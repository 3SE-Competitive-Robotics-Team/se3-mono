//! Typed robot profile registry for runtime defaults.

use std::path::PathBuf;

use locomotion_core::{PolicyObservationConfig, RobotConfig, policy_io::PolicyActionDecoderConfig};
use se3_command::{
    ChassisCommand, ChassisCommandLimits, Command, CommandSourceKind, GimbalCommand, JumpCommand,
};
use se3_input::{GamepadSnapshot, apply_deadzone};
use thiserror::Error;

#[derive(Debug, Clone)]
pub struct RobotProfile {
    pub id: String,
    pub kind: String,
    pub locomotion: LocomotionProfile,
    pub command: CommandProfile,
    pub sim: SimProfile,
    pub policies: Vec<PolicyProfile>,
}

impl RobotProfile {
    pub fn default_policy(&self) -> Result<&PolicyProfile, ZooError> {
        let default_policy_id = self.locomotion.default_policy_id.as_str();
        self.policy(default_policy_id)
            .map_err(|_| ZooError::DefaultPolicyMissing {
                robot_id: self.id.clone(),
                policy_id: default_policy_id.to_string(),
            })
    }

    pub fn policy(&self, id: &str) -> Result<&PolicyProfile, ZooError> {
        self.policies
            .iter()
            .find(|policy| policy.id == id)
            .ok_or_else(|| ZooError::UnknownPolicy {
                robot_id: self.id.clone(),
                policy_id: id.to_string(),
            })
    }
}

#[derive(Debug, Clone)]
pub struct LocomotionProfile {
    pub sim_socket_path: PathBuf,
    pub sim_client_socket_path: PathBuf,
    pub rate_hz: f64,
    pub state_timeout_s: f64,
    pub write_timeout_s: f64,
    pub default_policy_id: String,
    pub robot_cfg: RobotConfig,
}

#[derive(Debug, Clone)]
pub struct CommandProfile {
    pub default_source: CommandSourceKind,
    pub fixed: Command,
    pub gamepad: Option<GamepadCommandProfile>,
}

impl CommandProfile {
    pub fn fixed_chassis(height_m: f32) -> Self {
        Self {
            default_source: CommandSourceKind::Fixed,
            fixed: Command::chassis(ChassisCommand::idle(height_m)),
            gamepad: None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct GamepadCommandProfile {
    pub deadzone: f32,
    pub require_enable_button: bool,
    pub limits: ChassisCommandLimits,
    pub map: fn(&GamepadSnapshot, &GamepadCommandProfile) -> Command,
}

impl GamepadCommandProfile {
    pub fn command(&self, snapshot: &GamepadSnapshot) -> Command {
        (self.map)(snapshot, self)
    }
}

#[derive(Debug, Clone)]
pub struct SimProfile {
    pub model_path: PathBuf,
    pub socket_path: PathBuf,
    pub rate_hz: f64,
    pub leg_kp: f64,
    pub leg_kd: f64,
    pub wheel_kd: f64,
}

#[derive(Debug, Clone)]
pub struct PolicyProfile {
    pub id: String,
    pub checkpoint: Option<PathBuf>,
    pub ort_ep: String,
    pub observation_profile: PolicyObservationConfig,
    pub action_decoder_profile: PolicyActionDecoderConfig,
}

#[derive(Debug, Error)]
pub enum ZooError {
    #[error("unknown robot profile `{id}` (available: {available:?})")]
    UnknownRobot {
        id: String,
        available: Vec<&'static str>,
    },
    #[error("robot `{robot_id}` does not define default policy `{policy_id}`")]
    DefaultPolicyMissing { robot_id: String, policy_id: String },
    #[error("robot `{robot_id}` does not define policy `{policy_id}`")]
    UnknownPolicy { robot_id: String, policy_id: String },
}

struct RobotRegistryEntry {
    id: &'static str,
    build: fn() -> RobotProfile,
}

const ROBOT_REGISTRY: &[RobotRegistryEntry] = &[RobotRegistryEntry {
    id: "serial_leg_dev",
    build: serial_leg_dev,
}];

pub fn list_robots() -> Vec<&'static str> {
    ROBOT_REGISTRY.iter().map(|entry| entry.id).collect()
}

pub fn get_robot(id: &str) -> Result<RobotProfile, ZooError> {
    ROBOT_REGISTRY
        .iter()
        .find(|entry| entry.id == id)
        .map(|entry| (entry.build)())
        .ok_or_else(|| ZooError::UnknownRobot {
            id: id.to_string(),
            available: list_robots(),
        })
}

pub fn serial_leg_dev() -> RobotProfile {
    let robot_cfg = RobotConfig::default();
    let observation_profile = PolicyObservationConfig {
        expected_num_obs: Some(34),
        fourbar_surrogate: false,
        normalize_projected_gravity: true,
        phase_active_leg_observation: true,
        ..PolicyObservationConfig::default()
    };
    let default_policy_id = "recovery_default".to_string();

    RobotProfile {
        id: "serial_leg_dev".to_string(),
        kind: "serial_leg".to_string(),
        locomotion: LocomotionProfile {
            sim_socket_path: PathBuf::from("/tmp/se3_sim_loop.sock"),
            sim_client_socket_path: PathBuf::from("/tmp/se3_locomotion.sock"),
            rate_hz: 50.0,
            state_timeout_s: 0.10,
            write_timeout_s: 0.02,
            default_policy_id: default_policy_id.clone(),
            robot_cfg: robot_cfg.clone(),
        },
        command: CommandProfile {
            default_source: CommandSourceKind::Fixed,
            fixed: Command::chassis(ChassisCommand::idle(robot_cfg.default_base_height as f32)),
            gamepad: Some(GamepadCommandProfile {
                deadzone: 0.12,
                require_enable_button: true,
                limits: ChassisCommandLimits {
                    max_vx_mps: 0.8,
                    max_yaw_rate_rad_s: 1.5,
                    max_pitch_rad: 0.0,
                    max_roll_rad: 0.0,
                    min_height_m: 0.16,
                    max_height_m: 0.28,
                    max_jump_target_height_m: 0.35,
                },
                map: serial_leg_gamepad_command,
            }),
        },
        sim: SimProfile {
            model_path: PathBuf::from(
                "assets/robots/serial_leg/mjcf/serialleg_fourbar_surrogate_train.xml",
            ),
            socket_path: PathBuf::from("/tmp/se3_sim_loop.sock"),
            rate_hz: 500.0,
            leg_kp: 40.0,
            leg_kd: 2.0,
            wheel_kd: 0.5,
        },
        policies: vec![PolicyProfile {
            id: default_policy_id,
            checkpoint: Some(PathBuf::from("model/recovery/model_4999_recovery_gru.onnx")),
            ort_ep: "auto".to_string(),
            observation_profile,
            action_decoder_profile: PolicyActionDecoderConfig {
                robot_cfg,
                height_conditioned_action_default: true,
                active_rod_semantics: true,
                ..PolicyActionDecoderConfig::default()
            },
        }],
    }
}

fn serial_leg_gamepad_command(
    snapshot: &GamepadSnapshot,
    profile: &GamepadCommandProfile,
) -> Command {
    let default_height = RobotConfig::default().default_base_height as f32;
    let idle = ChassisCommand::idle(default_height);
    if profile.require_enable_button && !snapshot.right_bumper {
        return Command::chassis(idle);
    }

    let vx = apply_deadzone(snapshot.left_stick_y, profile.deadzone) * profile.limits.max_vx_mps;
    let yaw_rate = -apply_deadzone(snapshot.right_stick_x, profile.deadzone)
        * profile.limits.max_yaw_rate_rad_s;
    let height_delta = if snapshot.dpad_y.abs() > 0.5 {
        snapshot.dpad_y.signum() * 0.02
    } else {
        0.0
    };
    let height_m = (default_height + height_delta)
        .clamp(profile.limits.min_height_m, profile.limits.max_height_m);
    let jump = JumpCommand {
        enabled: snapshot.south,
        target_height_m: if snapshot.south {
            profile.limits.max_jump_target_height_m
        } else {
            0.0
        },
        phase: 0.0,
    };
    let chassis = ChassisCommand {
        vx_mps: vx,
        yaw_rate_rad_s: yaw_rate,
        pitch_rad: 0.0,
        roll_rad: 0.0,
        height_m,
        jump,
    }
    .validate(profile.limits)
    .unwrap_or(idle);

    Command {
        chassis: Some(chassis),
        gimbal: None::<GimbalCommand>,
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic, clippy::print_stdout)]
mod tests {
    use super::*;

    #[test]
    fn registry_lists_and_loads_default_robot() {
        let robot_ids = list_robots();
        assert_eq!(robot_ids, vec!["serial_leg_dev"]);

        let robot = get_robot("serial_leg_dev").unwrap();
        assert_eq!(robot.id, "serial_leg_dev");
        assert_eq!(robot.kind, "serial_leg");
        assert_eq!(
            robot.locomotion.sim_socket_path,
            PathBuf::from("/tmp/se3_sim_loop.sock")
        );
        assert_eq!(
            robot.locomotion.sim_client_socket_path,
            PathBuf::from("/tmp/se3_locomotion.sock")
        );
        assert_eq!(robot.sim.rate_hz, 500.0);
        assert_eq!(robot.command.default_source, CommandSourceKind::Fixed);
        assert!(robot.command.gamepad.is_some());
        assert!(robot.policy("recovery_default").is_ok());
    }

    #[test]
    fn serial_leg_gamepad_requires_right_bumper() {
        let robot = serial_leg_dev();
        let gamepad = robot.command.gamepad.as_ref().unwrap();
        let snapshot = test_gamepad_snapshot();

        let disabled = gamepad.command(&snapshot);
        assert_eq!(
            disabled.chassis.unwrap().to_policy_command(),
            ChassisCommand::idle(0.22).to_policy_command()
        );

        let enabled = gamepad.command(&GamepadSnapshot {
            right_bumper: true,
            left_stick_y: 1.0,
            right_stick_x: -1.0,
            ..snapshot
        });
        let chassis = enabled.chassis.unwrap();
        assert_eq!(chassis.vx_mps, gamepad.limits.max_vx_mps);
        assert_eq!(chassis.yaw_rate_rad_s, gamepad.limits.max_yaw_rate_rad_s);
    }

    #[test]
    fn serial_leg_gamepad_idle_is_same_with_enable_held() {
        let robot = serial_leg_dev();
        let gamepad = robot.command.gamepad.as_ref().unwrap();
        let snapshot = test_gamepad_snapshot();

        let disabled = gamepad.command(&snapshot);
        let enabled_idle = gamepad.command(&GamepadSnapshot {
            right_bumper: true,
            ..snapshot
        });

        assert_eq!(
            enabled_idle.chassis.unwrap().to_policy_command(),
            disabled.chassis.unwrap().to_policy_command()
        );
    }

    #[test]
    fn default_policy_exists() {
        let robot = serial_leg_dev();
        let policy = robot.default_policy().unwrap();

        assert_eq!(policy.id, "recovery_default");
        assert_eq!(
            policy.checkpoint.as_deref(),
            Some(std::path::Path::new(
                "model/recovery/model_4999_recovery_gru.onnx"
            ))
        );
        assert_eq!(policy.ort_ep, "auto");
        assert_eq!(policy.observation_profile.expected_num_obs, Some(34));
        assert!(
            policy
                .action_decoder_profile
                .height_conditioned_action_default
        );
        assert!(policy.action_decoder_profile.active_rod_semantics);
    }

    #[test]
    fn cloned_profiles_do_not_share_mutable_state() {
        let mut cloned = get_robot("serial_leg_dev").unwrap();
        cloned.locomotion.rate_hz = 60.0;
        cloned.locomotion.robot_cfg.leg_kp = 75.0;
        cloned.policies[0].ort_ep = "auto".to_string();
        cloned.policies[0].action_decoder_profile.robot_cfg.leg_kp = 12.0;

        let fresh = get_robot("serial_leg_dev").unwrap();
        let fresh_policy = fresh.default_policy().unwrap();

        assert_eq!(fresh.locomotion.rate_hz, 50.0);
        assert_eq!(
            fresh.locomotion.robot_cfg.leg_kp,
            RobotConfig::default().leg_kp
        );
        assert_eq!(fresh_policy.ort_ep, "auto");
        assert_eq!(
            fresh_policy.action_decoder_profile.robot_cfg.leg_kp,
            RobotConfig::default().leg_kp
        );
    }

    fn test_gamepad_snapshot() -> GamepadSnapshot {
        GamepadSnapshot {
            id: 0,
            name: "test".to_string(),
            connected: true,
            left_stick_x: 0.0,
            left_stick_y: 0.0,
            right_stick_x: 0.0,
            right_stick_y: 0.0,
            left_trigger: 0.0,
            right_trigger: 0.0,
            dpad_x: 0.0,
            dpad_y: 0.0,
            south: false,
            east: false,
            north: false,
            west: false,
            left_bumper: false,
            right_bumper: false,
            select: false,
            start: false,
            mode: false,
            left_thumb: false,
            right_thumb: false,
            sampled_at: std::time::Instant::now(),
        }
    }
}
