//! Dry-run transport backend — inference against synthetic states, no hardware I/O.

use std::time::Instant;

use crate::policy_observation::synthetic_default_state;
use crate::policy_runtime::{
    ACTION_FLAG_COMMAND_INACTIVE, ACTION_FLAG_DRY_RUN, ACTION_FLAG_NONFINITE,
    LOCOMOTION_POLICY_RATE_HZ, LocomotionPolicyError, LocomotionPolicyRuntime, TelemetryTiming,
};

impl LocomotionPolicyRuntime {
    pub(crate) fn run_dry(&mut self) -> Result<(), LocomotionPolicyError> {
        let max_steps = if self.cfg.max_steps > 0 {
            self.cfg.max_steps
        } else {
            (LOCOMOTION_POLICY_RATE_HZ * 2.0) as usize
        };
        for step in 0..max_steps {
            let loop_started = Instant::now();
            let state = synthetic_default_state(step as u32);
            self.stats.state_frames += 1;
            self.sample_command();
            let (action, flags, obs, policy_inference_ms) = if self.last_command_sample.active {
                let (action, flags, obs, policy_inference_ms) = self.act_from_state(&state)?;
                self.decode_target(action)?;
                (
                    action,
                    flags | ACTION_FLAG_DRY_RUN,
                    obs,
                    Some(policy_inference_ms),
                )
            } else {
                self.reset_policy_memory(false, "command_inactive")?;
                let (obs, obs_flags) = self.build_observation(&state)?;
                if obs_flags & ACTION_FLAG_NONFINITE != 0 {
                    self.stats.nonfinite_frames += 1;
                }
                self.stats.last_target_joint_pos = state.joint_pos;
                self.stats.last_target_wheel_vel = [0.0, 0.0];
                self.action_seq = self.action_seq.wrapping_add(1);
                (
                    [0.0; 6],
                    obs_flags | ACTION_FLAG_DRY_RUN | ACTION_FLAG_COMMAND_INACTIVE,
                    obs,
                    None,
                )
            };
            self.record_action(&state, action, flags, policy_inference_ms);
            self.write_telemetry(
                &state,
                &obs,
                &action,
                flags,
                TelemetryTiming {
                    policy_inference_ms,
                    state_age_s: Some(0.0),
                    loop_started,
                    write_ms: None,
                },
            )?;
            self.maybe_print();
        }
        Ok(())
    }
}
