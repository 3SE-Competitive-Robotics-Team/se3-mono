//! Operator command types shared by control runtimes.

use thiserror::Error;

#[derive(Debug, Error, PartialEq)]
pub enum CommandError {
    #[error("command value `{field}` must be finite, got {value}")]
    NonFinite { field: &'static str, value: f32 },
    #[error("command limit `{field}` must be finite and non-negative, got {value}")]
    InvalidLimit { field: &'static str, value: f32 },
    #[error("chassis command height {value} is outside [{min}, {max}]")]
    HeightOutOfRange { value: f32, min: f32, max: f32 },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Command {
    pub chassis: Option<ChassisCommand>,
    pub gimbal: Option<GimbalCommand>,
}

impl Command {
    pub fn chassis(command: ChassisCommand) -> Self {
        Self {
            chassis: Some(command),
            gimbal: None,
        }
    }

    pub fn idle(default_height_m: f32) -> Self {
        Self::chassis(ChassisCommand::idle(default_height_m))
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChassisCommand {
    pub vx_mps: f32,
    pub yaw_rate_rad_s: f32,
    pub pitch_rad: f32,
    pub roll_rad: f32,
    pub height_m: f32,
    pub jump: JumpCommand,
}

impl ChassisCommand {
    pub fn idle(height_m: f32) -> Self {
        Self {
            vx_mps: 0.0,
            yaw_rate_rad_s: 0.0,
            pitch_rad: 0.0,
            roll_rad: 0.0,
            height_m,
            jump: JumpCommand::default(),
        }
    }

    pub fn to_policy_command(self) -> [f32; 8] {
        [
            self.vx_mps,
            self.yaw_rate_rad_s,
            self.pitch_rad,
            self.roll_rad,
            self.height_m,
            if self.jump.enabled { 1.0 } else { 0.0 },
            self.jump.target_height_m,
            self.jump.phase,
        ]
    }

    pub fn validate(self, limits: ChassisCommandLimits) -> Result<Self, CommandError> {
        for (field, value) in [
            ("vx_mps", self.vx_mps),
            ("yaw_rate_rad_s", self.yaw_rate_rad_s),
            ("pitch_rad", self.pitch_rad),
            ("roll_rad", self.roll_rad),
            ("height_m", self.height_m),
            ("jump_target_height_m", self.jump.target_height_m),
            ("jump_phase", self.jump.phase),
        ] {
            if !value.is_finite() {
                return Err(CommandError::NonFinite { field, value });
            }
        }
        limits.validate()?;
        if self.height_m < limits.min_height_m || self.height_m > limits.max_height_m {
            return Err(CommandError::HeightOutOfRange {
                value: self.height_m,
                min: limits.min_height_m,
                max: limits.max_height_m,
            });
        }
        Ok(Self {
            vx_mps: self.vx_mps.clamp(-limits.max_vx_mps, limits.max_vx_mps),
            yaw_rate_rad_s: self
                .yaw_rate_rad_s
                .clamp(-limits.max_yaw_rate_rad_s, limits.max_yaw_rate_rad_s),
            pitch_rad: self
                .pitch_rad
                .clamp(-limits.max_pitch_rad, limits.max_pitch_rad),
            roll_rad: self
                .roll_rad
                .clamp(-limits.max_roll_rad, limits.max_roll_rad),
            height_m: self
                .height_m
                .clamp(limits.min_height_m, limits.max_height_m),
            jump: JumpCommand {
                target_height_m: self
                    .jump
                    .target_height_m
                    .clamp(limits.min_height_m, limits.max_jump_target_height_m),
                phase: self.jump.phase.clamp(0.0, 1.0),
                ..self.jump
            },
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JumpCommand {
    pub enabled: bool,
    pub target_height_m: f32,
    pub phase: f32,
}

impl Default for JumpCommand {
    fn default() -> Self {
        Self {
            enabled: false,
            target_height_m: 0.0,
            phase: 0.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GimbalCommand {
    pub yaw_rad: f32,
    pub pitch_rad: f32,
    pub roll_rad: f32,
}

impl Default for GimbalCommand {
    fn default() -> Self {
        Self {
            yaw_rad: 0.0,
            pitch_rad: 0.0,
            roll_rad: 0.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChassisCommandLimits {
    pub max_vx_mps: f32,
    pub max_yaw_rate_rad_s: f32,
    pub max_pitch_rad: f32,
    pub max_roll_rad: f32,
    pub min_height_m: f32,
    pub max_height_m: f32,
    pub max_jump_target_height_m: f32,
}

impl ChassisCommandLimits {
    pub fn validate(self) -> Result<(), CommandError> {
        for (field, value) in [
            ("max_vx_mps", self.max_vx_mps),
            ("max_yaw_rate_rad_s", self.max_yaw_rate_rad_s),
            ("max_pitch_rad", self.max_pitch_rad),
            ("max_roll_rad", self.max_roll_rad),
            ("min_height_m", self.min_height_m),
            ("max_height_m", self.max_height_m),
            ("max_jump_target_height_m", self.max_jump_target_height_m),
        ] {
            if !value.is_finite() || value < 0.0 {
                return Err(CommandError::InvalidLimit { field, value });
            }
        }
        Ok(())
    }
}

impl Default for ChassisCommandLimits {
    fn default() -> Self {
        Self {
            max_vx_mps: 0.0,
            max_yaw_rate_rad_s: 0.0,
            max_pitch_rad: 0.0,
            max_roll_rad: 0.0,
            min_height_m: 0.0,
            max_height_m: 1.0,
            max_jump_target_height_m: 1.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandSourceKind {
    Fixed,
    XInput,
}

impl CommandSourceKind {
    pub fn parse(value: &str) -> Result<Self, CommandSourceKindParseError> {
        match value.trim().to_ascii_lowercase().as_str() {
            "fixed" => Ok(Self::Fixed),
            "xinput" | "gamepad" => Ok(Self::XInput),
            _ => Err(CommandSourceKindParseError(value.to_string())),
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Fixed => "fixed",
            Self::XInput => "xinput",
        }
    }
}

#[derive(Debug, Error, PartialEq, Eq)]
#[error("command source must be `fixed` or `xinput`, got `{0}`")]
pub struct CommandSourceKindParseError(String);

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic, clippy::print_stdout)]
mod tests {
    use super::*;

    #[test]
    fn chassis_command_encodes_policy_command() {
        let command = ChassisCommand {
            vx_mps: 0.3,
            yaw_rate_rad_s: -0.4,
            pitch_rad: 0.1,
            roll_rad: -0.2,
            height_m: 0.22,
            jump: JumpCommand {
                enabled: true,
                target_height_m: 0.35,
                phase: 0.75,
            },
        };
        assert_eq!(
            command.to_policy_command(),
            [0.3, -0.4, 0.1, -0.2, 0.22, 1.0, 0.35, 0.75]
        );
    }

    #[test]
    fn chassis_command_validation_clamps_limited_axes() {
        let limits = ChassisCommandLimits {
            max_vx_mps: 1.0,
            max_yaw_rate_rad_s: 2.0,
            max_pitch_rad: 0.3,
            max_roll_rad: 0.4,
            min_height_m: 0.16,
            max_height_m: 0.28,
            max_jump_target_height_m: 0.5,
        };
        let command = ChassisCommand {
            vx_mps: 2.0,
            yaw_rate_rad_s: -3.0,
            pitch_rad: 0.5,
            roll_rad: -0.6,
            height_m: 0.22,
            jump: JumpCommand {
                enabled: true,
                target_height_m: 0.7,
                phase: 1.5,
            },
        }
        .validate(limits)
        .unwrap();
        assert_eq!(command.vx_mps, 1.0);
        assert_eq!(command.yaw_rate_rad_s, -2.0);
        assert_eq!(command.pitch_rad, 0.3);
        assert_eq!(command.roll_rad, -0.4);
        assert_eq!(command.jump.target_height_m, 0.5);
        assert_eq!(command.jump.phase, 1.0);
    }

    #[test]
    fn command_source_kind_parses_aliases() {
        assert_eq!(
            CommandSourceKind::parse("fixed").unwrap(),
            CommandSourceKind::Fixed
        );
        assert_eq!(
            CommandSourceKind::parse("gamepad").unwrap(),
            CommandSourceKind::XInput
        );
        assert!(CommandSourceKind::parse("keyboard").is_err());
    }
}
