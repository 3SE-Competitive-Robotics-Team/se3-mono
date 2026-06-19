//! Command conversion for locomotion policy runtimes.

use se3_command::{ChassisCommand, Command};

use crate::RobotConfig;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocomotionCommand {
    pub chassis: ChassisCommand,
}

impl LocomotionCommand {
    pub fn idle(robot_cfg: &RobotConfig) -> Self {
        Self {
            chassis: ChassisCommand::idle(robot_cfg.default_base_height as f32),
        }
    }

    pub fn from_command(command: Command, robot_cfg: &RobotConfig) -> Self {
        Self {
            chassis: command
                .chassis
                .unwrap_or_else(|| ChassisCommand::idle(robot_cfg.default_base_height as f32)),
        }
    }

    pub fn to_policy_command(self) -> [f32; 8] {
        self.chassis.to_policy_command()
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic, clippy::print_stdout)]
mod tests {
    use super::*;

    #[test]
    fn idle_command_uses_robot_default_height() {
        let robot = RobotConfig::default();
        let command = LocomotionCommand::idle(&robot);
        assert_eq!(
            command.to_policy_command()[4],
            robot.default_base_height as f32
        );
    }
}
