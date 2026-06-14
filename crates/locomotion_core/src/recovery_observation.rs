use crate::{ObservationConfig, PolicyObservationResult, RobotConfig, build_policy_observation};

use crate::protocol::PolicyStateFrame;

#[derive(Debug, Clone)]
pub struct RecoveryObservationBuilder {
    pub default_dof_pos: [f32; 6],
    pub command_scale: [f32; 5],
    pub command: [f32; 8],
}

impl Default for RecoveryObservationBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl RecoveryObservationBuilder {
    pub fn new() -> Self {
        let robot_cfg = RobotConfig::default();
        let obs_cfg = ObservationConfig::default();
        Self {
            default_dof_pos: robot_cfg.default_dof_pos.map(|v| v as f32),
            command_scale: obs_cfg.command_scale,
            command: [
                0.0,
                0.0,
                0.0,
                0.0,
                robot_cfg.default_base_height as f32,
                0.0,
                0.0,
                0.0,
            ],
        }
    }

    pub fn build(
        &self,
        state: &PolicyStateFrame,
        last_action: [f32; 6],
    ) -> Result<PolicyObservationResult, crate::policy_io::PolicyIoError> {
        let obs_cfg = ObservationConfig::default();
        build_policy_observation(
            state.base_ang_vel_body,
            state.projected_gravity,
            state.dof_pos(),
            state.dof_vel(),
            &self.command,
            last_action,
            self.default_dof_pos,
            Some(self.command_scale),
            Some(obs_cfg.num_obs),
            Some(obs_cfg.clip_value),
            false,
            true,
        )
    }
}

pub fn synthetic_recovery_state(seq: u32) -> PolicyStateFrame {
    let robot_cfg = RobotConfig::default();
    let dof_pos = robot_cfg.default_dof_pos.map(|v| v as f32);
    PolicyStateFrame {
        seq,
        tick_ms: seq.wrapping_mul((robot_cfg.control_dt() * 1000.0) as u32),
        target_seq: 0,
        target_age_ms: 0,
        target_valid: 0,
        rc_switch_r: 0,
        output_enabled: 0,
        base_ang_vel_body: [0.0, 0.0, 0.0],
        projected_gravity: [0.0, 0.0, -1.0],
        joint_pos: [dof_pos[0], dof_pos[1], dof_pos[2], dof_pos[3]],
        joint_vel: [0.0; 4],
        wheel_pos: [dof_pos[4], dof_pos[5]],
        wheel_vel: [0.0; 2],
        target_joint_pos: [0.0; 4],
        hip_torque: [0.0; 4],
        wheel_torque: [0.0; 2],
        wheel_motor_torque: [0.0; 2],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn synthetic_observation_matches_python_reference() {
        let builder = RecoveryObservationBuilder::new();
        let state = synthetic_recovery_state(7);
        let result = builder.build(&state, [0.0; 6]).unwrap();
        assert!(!result.had_nonfinite_input);
        assert_eq!(
            result.obs,
            [
                0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            ]
        );
    }
}
