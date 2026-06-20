use std::f64::consts::PI;

use serde::{Deserialize, Serialize};

use crate::action_delay::ActionDelayConfig;
use crate::motor::{DM8009P, M3508_C620_14};

const ACTIVE_ROD_UPPER: f64 = 1.5095352700498952;
const ACTIVE_ROD_ACTION_SCALE: f64 = 0.5 * ACTIVE_ROD_UPPER;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(usize)]
pub enum Joint {
    Lf0 = 0,
    Lb = 1,
    Rf0 = 2,
    Rb = 3,
    LWheel = 4,
    RWheel = 5,
}

impl Joint {
    pub fn mjcf_name(self) -> &'static str {
        match self {
            Self::Lf0 => "lf0_Joint",
            Self::Lb => "l_drive_bar_Joint",
            Self::Rf0 => "rf0_Joint",
            Self::Rb => "r_drive_bar_Joint",
            Self::LWheel => "l_wheel_Joint",
            Self::RWheel => "r_wheel_Joint",
        }
    }
}

pub struct JointGroup;

impl JointGroup {
    pub const LEGS: [usize; 4] = [0, 1, 2, 3];
    pub const WHEELS: [usize; 2] = [4, 5];
    pub const CTRL_LEGS: [usize; 4] = Self::LEGS;
    pub const CTRL_WHEELS: [usize; 2] = Self::WHEELS;
    pub const LEG_ACTUATORS: [usize; 4] = [0, 1, 2, 3];
    pub const WHEEL_ACTUATORS: [usize; 2] = [4, 5];
    pub const ALL: [usize; 6] = [0, 1, 2, 3, 4, 5];
    pub const POLICY_JOINT_NAMES: [&'static str; 6] = [
        "lf0_Joint",
        "l_drive_bar_Joint",
        "rf0_Joint",
        "r_drive_bar_Joint",
        "l_wheel_Joint",
        "r_wheel_Joint",
    ];
    pub const POLICY_LEG_NAMES: [&'static str; 4] = [
        "lf0_Joint",
        "l_drive_bar_Joint",
        "rf0_Joint",
        "r_drive_bar_Joint",
    ];
    pub const OPENCHAIN_LEG_NAMES: [&'static str; 4] =
        ["lf0_Joint", "lf1_Joint", "rf0_Joint", "rf1_Joint"];
    pub const WHEEL_NAMES: [&'static str; 2] = ["l_wheel_Joint", "r_wheel_Joint"];
    pub const OUTPUT_LEG_NAMES: [&'static str; 4] =
        ["lf0_Joint", "lf1_Joint", "rf0_Joint", "rf1_Joint"];
    pub const OUTPUT_KNEE_NAMES: [&'static str; 2] = ["lf1_Joint", "rf1_Joint"];
    pub const CLOSEDCHAIN_PASSIVE_JOINT_NAMES: [&'static str; 4] = [
        "lf1_Joint",
        "l_coupler_Joint",
        "rf1_Joint",
        "r_coupler_Joint",
    ];
    pub const POLICY_MOTOR_ACTUATOR_NAMES: [&'static str; 6] = [
        "lf0_Joint_motor",
        "l_drive_bar_Joint_motor",
        "rf0_Joint_motor",
        "r_drive_bar_Joint_motor",
        "l_wheel_Joint_motor",
        "r_wheel_Joint_motor",
    ];

    pub fn joint_names() -> [&'static str; 6] {
        Self::POLICY_JOINT_NAMES
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Termination {
    pub terminate_on_fall: bool,
    pub fail_tilt_deg: f64,
    pub fail_height_m: f64,
}

impl Default for Termination {
    fn default() -> Self {
        Self {
            terminate_on_fall: false,
            fail_tilt_deg: 80.0,
            fail_height_m: 0.12,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct RobotConfig {
    pub leg_kp: f64,
    pub leg_kd: f64,
    pub wheel_kd: f64,
    pub torque_limits: [f64; 6],
    pub default_dof_pos: [f64; 6],
    pub default_output_knee_pos: [f64; 2],
    pub default_coupler_pos: [f64; 2],
    pub active_rod_angle_limits: [f64; 2],
    pub active_rod_lower_target_overdrive: f64,
    pub active_rod_soft_limit_factor: f64,
    pub active_rod_angle_coeffs: [[f64; 2]; 2],
    pub default_base_height: f64,
    pub action_scale: [f64; 6],
    pub action_clip: Option<f64>,
    pub sim_dt: f64,
    pub action_delay: ActionDelayConfig,
    pub termination: Termination,
}

impl Default for RobotConfig {
    fn default() -> Self {
        Self {
            leg_kp: 60.0,
            leg_kd: 3.0,
            wheel_kd: 0.5,
            torque_limits: [
                DM8009P.stall_torque,
                DM8009P.stall_torque,
                DM8009P.stall_torque,
                DM8009P.stall_torque,
                M3508_C620_14.rated_torque,
                M3508_C620_14.rated_torque,
            ],
            default_dof_pos: [
                -0.275422946189,
                -1.592100148957,
                0.275422946189,
                1.592100148957,
                0.0,
                0.0,
            ],
            default_output_knee_pos: [-1.242259649307, 1.242259649307],
            default_coupler_pos: [1.401266340000, -1.401269410000],
            active_rod_angle_limits: [0.0, ACTIVE_ROD_UPPER],
            active_rod_lower_target_overdrive: 0.20,
            active_rod_soft_limit_factor: 1.0,
            active_rod_angle_coeffs: [[1.0, -1.0], [-1.0, 1.0]],
            default_base_height: 0.22,
            action_scale: [
                PI,
                ACTIVE_ROD_ACTION_SCALE,
                PI,
                ACTIVE_ROD_ACTION_SCALE,
                45.0,
                45.0,
            ],
            action_clip: Some(1.0),
            sim_dt: 0.002,
            action_delay: ActionDelayConfig::default(),
            termination: Termination::default(),
        }
    }
}

impl RobotConfig {
    pub fn default_active_rod_angles(&self) -> [f64; 2] {
        let [left_front, left_back] = self.active_rod_angle_coeffs[0];
        let [right_front, right_back] = self.active_rod_angle_coeffs[1];
        [
            left_front * self.default_dof_pos[Joint::Lf0 as usize]
                + left_back * self.default_dof_pos[Joint::Lb as usize],
            right_front * self.default_dof_pos[Joint::Rf0 as usize]
                + right_back * self.default_dof_pos[Joint::Rb as usize],
        ]
    }

    pub fn active_rod_soft_angle_limits(&self) -> [f64; 2] {
        let [lower, upper] = self.active_rod_angle_limits;
        let factor = self.active_rod_soft_limit_factor.clamp(0.0, 1.0);
        let center = 0.5 * (lower + upper);
        let half_range = 0.5 * (upper - lower) * factor;
        [center - half_range, center + half_range]
    }

    pub fn default_model_joint_pos(&self) -> Vec<(&'static str, f64)> {
        let mut values = Vec::with_capacity(10);
        for (name, value) in JointGroup::POLICY_JOINT_NAMES
            .iter()
            .copied()
            .zip(self.default_dof_pos)
        {
            values.push((name, value));
        }
        values.extend([
            ("lf1_Joint", self.default_output_knee_pos[0]),
            ("rf1_Joint", self.default_output_knee_pos[1]),
            ("l_coupler_Joint", self.default_coupler_pos[0]),
            ("r_coupler_Joint", self.default_coupler_pos[1]),
        ]);
        values
    }
}
