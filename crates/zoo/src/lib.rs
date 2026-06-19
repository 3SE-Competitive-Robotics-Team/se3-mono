//! Typed robot profile registry for runtime defaults.

use std::path::PathBuf;

use locomotion_core::{PolicyObservationConfig, RobotConfig, policy_io::PolicyActionDecoderConfig};
use thiserror::Error;

#[derive(Debug, Clone)]
pub struct RobotProfile {
    pub id: String,
    pub kind: String,
    pub locomotion: LocomotionProfile,
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
        assert!(robot.policy("recovery_default").is_ok());
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
}
