use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::fourbar::{output_to_policy_pos, output_to_policy_vel};
use crate::height_default::policy_default_from_height;
use crate::observation_config::ObservationConfig;
use crate::robot::{JointGroup, RobotConfig};

#[derive(Debug, Error, PartialEq)]
pub enum PolicyIoError {
    #[error("command must have 7 or 8 values, got {0}")]
    InvalidCommandLen(usize),
    #[error("observation shape mismatch: expected {expected}, got {got}")]
    ObservationShapeMismatch { expected: usize, got: usize },
    #[error("height-conditioned action default requires command_height")]
    MissingCommandHeight,
}

#[derive(Debug, Clone, PartialEq)]
pub struct DecodedPolicyAction {
    pub clipped_action: [f32; 6],
    pub leg_target: [f32; 4],
    pub wheel_vel_target: [f32; 2],
    pub policy_default: [f32; 4],
}

#[derive(Debug, Clone, PartialEq)]
pub struct PolicyObservationResult {
    pub obs: Vec<f32>,
    pub had_nonfinite_input: bool,
}

#[derive(Debug, Clone)]
pub struct PolicyActionDecoder {
    pub robot_cfg: RobotConfig,
    pub action_scale: [f32; 6],
    pub action_clip: Option<f32>,
    pub height_conditioned_action_default: bool,
    pub active_rod_semantics: bool,
    pub active_rod_target_lower: f32,
    pub active_rod_target_upper: f32,
    pub active_rod_angle_mid: f32,
    pub active_rod_angle_coeffs: [[f32; 2]; 2],
}

impl Default for PolicyActionDecoder {
    fn default() -> Self {
        Self::new(PolicyActionDecoderConfig::default())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PolicyActionDecoderConfig {
    pub robot_cfg: RobotConfig,
    pub action_scale: Option<[f32; 6]>,
    pub height_conditioned_action_default: bool,
    pub active_rod_semantics: bool,
    pub active_rod_target_lower_preload_margin: Option<f32>,
    pub active_rod_target_upper_preload_margin: f32,
}

impl Default for PolicyActionDecoderConfig {
    fn default() -> Self {
        Self {
            robot_cfg: RobotConfig::default(),
            action_scale: None,
            height_conditioned_action_default: false,
            active_rod_semantics: true,
            active_rod_target_lower_preload_margin: None,
            active_rod_target_upper_preload_margin: 0.0,
        }
    }
}

impl PolicyActionDecoder {
    pub fn new(config: PolicyActionDecoderConfig) -> Self {
        let action_scale = config
            .action_scale
            .unwrap_or_else(|| config.robot_cfg.action_scale.map(|v| v as f32));
        let action_clip = config.robot_cfg.action_clip.map(|v| v as f32);
        let [lower, upper] = config.robot_cfg.active_rod_angle_limits;
        let lower_margin = config
            .active_rod_target_lower_preload_margin
            .unwrap_or(config.robot_cfg.active_rod_lower_target_overdrive as f32);
        let active_rod_target_lower = lower as f32 - lower_margin;
        let active_rod_target_upper = upper as f32 + config.active_rod_target_upper_preload_margin;
        let active_rod_angle_mid = (0.5 * (lower + upper)) as f32;
        let active_rod_angle_coeffs = config
            .robot_cfg
            .active_rod_angle_coeffs
            .map(|row| row.map(|v| v as f32));
        Self {
            robot_cfg: config.robot_cfg,
            action_scale,
            action_clip,
            height_conditioned_action_default: config.height_conditioned_action_default,
            active_rod_semantics: config.active_rod_semantics,
            active_rod_target_lower,
            active_rod_target_upper,
            active_rod_angle_mid,
            active_rod_angle_coeffs,
        }
    }

    pub fn clip_action(&self, action: [f32; 6]) -> [f32; 6] {
        match self.action_clip {
            Some(clip) => action.map(|v| v.clamp(-clip, clip)),
            None => action,
        }
    }

    pub fn policy_default(
        &self,
        command_height: Option<f32>,
        fallback_default: Option<[f32; 4]>,
    ) -> Result<[f32; 4], PolicyIoError> {
        if self.height_conditioned_action_default {
            let height = command_height.ok_or(PolicyIoError::MissingCommandHeight)?;
            return Ok(
                policy_default_from_height(height as f64, Some(&self.robot_cfg)).map(|v| v as f32),
            );
        }
        if let Some(default) = fallback_default {
            return Ok(default);
        }
        Ok([
            self.robot_cfg.default_dof_pos[JointGroup::CTRL_LEGS[0]] as f32,
            self.robot_cfg.default_dof_pos[JointGroup::CTRL_LEGS[1]] as f32,
            self.robot_cfg.default_dof_pos[JointGroup::CTRL_LEGS[2]] as f32,
            self.robot_cfg.default_dof_pos[JointGroup::CTRL_LEGS[3]] as f32,
        ])
    }

    pub fn decode(
        &self,
        action: [f32; 6],
        command_height: Option<f32>,
        policy_default: Option<[f32; 4]>,
        fallback_default: Option<[f32; 4]>,
    ) -> Result<DecodedPolicyAction, PolicyIoError> {
        let clipped = self.clip_action(action);
        let default = match policy_default {
            Some(default) => default,
            None => self.policy_default(command_height, fallback_default)?,
        };
        let leg_target = self.leg_target([clipped[0], clipped[1], clipped[2], clipped[3]], default);
        let wheel_vel_target = [
            clipped[JointGroup::CTRL_WHEELS[0]] * self.action_scale[JointGroup::WHEEL_ACTUATORS[0]],
            clipped[JointGroup::CTRL_WHEELS[1]] * self.action_scale[JointGroup::WHEEL_ACTUATORS[1]],
        ];
        Ok(DecodedPolicyAction {
            clipped_action: clipped,
            leg_target,
            wheel_vel_target,
            policy_default: default,
        })
    }

    pub fn leg_target(&self, leg_action: [f32; 4], policy_default: [f32; 4]) -> [f32; 4] {
        if !self.active_rod_semantics {
            let mut out = [0.0; 4];
            for idx in 0..4 {
                out[idx] = policy_default[idx] + leg_action[idx] * self.action_scale[idx];
            }
            return out;
        }

        let mut target = [0.0; 4];
        for (side_idx, [front_idx, back_idx]) in
            [[0usize, 1usize], [2usize, 3usize]].into_iter().enumerate()
        {
            let [front_coef, back_coef] = self.active_rod_angle_coeffs[side_idx];
            let front_target =
                policy_default[front_idx] + leg_action[front_idx] * self.action_scale[front_idx];
            let active_default = if self.height_conditioned_action_default {
                front_coef * policy_default[front_idx] + back_coef * policy_default[back_idx]
            } else {
                self.active_rod_angle_mid
            };
            let active_raw = active_default + leg_action[back_idx] * self.action_scale[back_idx];
            let active_target =
                active_raw.clamp(self.active_rod_target_lower, self.active_rod_target_upper);
            target[front_idx] = front_target;
            target[back_idx] = (active_target - front_coef * front_target) / back_coef;
        }
        target
    }
}

/// Joint state passed to policy observation builder.
#[derive(Debug, Clone, Copy)]
pub struct JointState {
    pub pos: [f32; 6],
    pub vel: [f32; 6],
}

/// Optional config overrides for policy observation building.
#[derive(Debug, Clone, Copy)]
pub struct PolicyObservationConfig {
    pub command_scale: Option<[f32; 5]>,
    pub expected_num_obs: Option<usize>,
    pub clip_value: Option<f32>,
    pub fourbar_surrogate: bool,
    pub normalize_projected_gravity: bool,
    pub phase_active_leg_observation: bool,
}

impl Default for PolicyObservationConfig {
    fn default() -> Self {
        Self {
            command_scale: None,
            expected_num_obs: None,
            clip_value: None,
            fourbar_surrogate: false,
            normalize_projected_gravity: true,
            phase_active_leg_observation: true,
        }
    }
}

pub fn build_policy_observation(
    base_ang_vel_body: [f32; 3],
    projected_gravity: [f32; 3],
    joint: JointState,
    command: &[f32],
    action_obs: [f32; 6],
    default_dof_pos: [f32; 6],
    config: PolicyObservationConfig,
) -> Result<PolicyObservationResult, PolicyIoError> {
    let obs_cfg = ObservationConfig::default();
    let (base_ang_vel_body, mut had_nonfinite) = finite_array(base_ang_vel_body);
    let (projected_gravity, bad) =
        projected_gravity_clean(projected_gravity, config.normalize_projected_gravity);
    had_nonfinite |= bad;
    let (dof_pos, bad) = finite_array(joint.pos);
    had_nonfinite |= bad;
    let (dof_vel, bad) = finite_array(joint.vel);
    had_nonfinite |= bad;
    let (action_obs, bad) = finite_array(action_obs);
    had_nonfinite |= bad;

    if !(command.len() == 7 || command.len() == 8) {
        return Err(PolicyIoError::InvalidCommandLen(command.len()));
    }
    let mut command_arr = [0.0_f32; 8];
    for (idx, value) in command.iter().copied().enumerate() {
        if value.is_finite() {
            command_arr[idx] = value;
        } else {
            had_nonfinite = true;
        }
    }

    let scale = config.command_scale.unwrap_or(obs_cfg.command_scale);
    let mut obs = Vec::with_capacity(34);
    obs.extend(base_ang_vel_body.map(|v| v * obs_cfg.ang_vel_scale));
    obs.extend(projected_gravity);
    for idx in 0..5 {
        obs.push(command_arr[idx] * scale[idx]);
    }

    let mut leg_pos = [
        dof_pos[0] as f64,
        dof_pos[1] as f64,
        dof_pos[2] as f64,
        dof_pos[3] as f64,
    ];
    let mut default_leg_pos = [
        default_dof_pos[0] as f64,
        default_dof_pos[1] as f64,
        default_dof_pos[2] as f64,
        default_dof_pos[3] as f64,
    ];
    let mut leg_vel = [
        dof_vel[0] as f64,
        dof_vel[1] as f64,
        dof_vel[2] as f64,
        dof_vel[3] as f64,
    ];
    if config.fourbar_surrogate {
        leg_pos = output_to_policy_pos(leg_pos);
        default_leg_pos = output_to_policy_pos(default_leg_pos);
        leg_vel = output_to_policy_vel(
            [
                dof_pos[0] as f64,
                dof_pos[1] as f64,
                dof_pos[2] as f64,
                dof_pos[3] as f64,
            ],
            leg_vel,
        );
    }
    if config.phase_active_leg_observation {
        obs.extend(policy_leg_phase_active_obs(
            leg_pos,
            default_leg_pos,
            RobotConfig::default().active_rod_angle_coeffs,
        ));
    } else {
        for idx in 0..4 {
            obs.push((leg_pos[idx] - default_leg_pos[idx]) as f32);
        }
    }
    for vel in leg_vel {
        obs.push(vel as f32 * obs_cfg.leg_vel_scale);
    }
    obs.extend([0.0, 0.0]);
    obs.push(dof_vel[4] * obs_cfg.wheel_vel_scale);
    obs.push(dof_vel[5] * obs_cfg.wheel_vel_scale);
    obs.extend(action_obs);
    if command.len() >= 8 {
        obs.extend([command_arr[5], command_arr[6], command_arr[7]]);
    } else {
        obs.extend([command_arr[5], command_arr[6], 0.0]);
    }

    let expected = config.expected_num_obs.unwrap_or(obs_cfg.num_obs);
    if obs.len() != expected {
        return Err(PolicyIoError::ObservationShapeMismatch {
            expected,
            got: obs.len(),
        });
    }
    let limit = config.clip_value.unwrap_or(obs_cfg.clip_value);
    let mut out = Vec::with_capacity(expected);
    for value in obs {
        out.push(if value.is_nan() {
            0.0
        } else if value.is_infinite() && value.is_sign_positive() {
            limit
        } else if value.is_infinite() && value.is_sign_negative() {
            -limit
        } else {
            value.clamp(-limit, limit)
        });
    }
    Ok(PolicyObservationResult {
        obs: out,
        had_nonfinite_input: had_nonfinite,
    })
}

fn policy_leg_phase_active_obs(
    pos: [f64; 4],
    default: [f64; 4],
    coeffs: [[f64; 2]; 2],
) -> [f32; 6] {
    let front_delta = [pos[0] - default[0], pos[2] - default[2]];
    let active_delta = [
        coeffs[0][0] * pos[0] + coeffs[0][1] * pos[1]
            - (coeffs[0][0] * default[0] + coeffs[0][1] * default[1]),
        coeffs[1][0] * pos[2] + coeffs[1][1] * pos[3]
            - (coeffs[1][0] * default[2] + coeffs[1][1] * default[3]),
    ];
    [
        front_delta[0].sin() as f32,
        front_delta[0].cos() as f32,
        active_delta[0] as f32,
        front_delta[1].sin() as f32,
        front_delta[1].cos() as f32,
        active_delta[1] as f32,
    ]
}

fn finite_array<const N: usize>(values: [f32; N]) -> ([f32; N], bool) {
    let mut out = values;
    let mut bad = false;
    for value in &mut out {
        if !value.is_finite() {
            *value = 0.0;
            bad = true;
        }
    }
    (out, bad)
}

fn projected_gravity_clean(values: [f32; 3], normalize: bool) -> ([f32; 3], bool) {
    let (mut arr, mut bad) = finite_array(values);
    if !normalize {
        return (arr, bad);
    }
    let norm = (arr[0] * arr[0] + arr[1] * arr[1] + arr[2] * arr[2]).sqrt();
    if norm < 1.0e-6 {
        return ([0.0, 0.0, -1.0], true);
    }
    for value in &mut arr {
        *value = (*value / norm).clamp(-1.0, 1.0);
    }
    bad |= false;
    (arr, bad)
}
