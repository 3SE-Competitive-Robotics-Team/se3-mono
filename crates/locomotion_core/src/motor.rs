use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorSpec {
    pub name: &'static str,
    pub rated_voltage: f64,
    pub gear_ratio: f64,
    pub stall_torque: f64,
    pub no_load_speed: f64,
    pub rated_torque: f64,
    pub rated_current: f64,
    pub stall_current: f64,
    pub phase_resistance: f64,
}

impl MotorSpec {
    pub fn no_load_speed_rpm(&self) -> f64 {
        self.no_load_speed * 60.0 / (2.0 * PI)
    }

    pub fn rotor_kt(&self) -> f64 {
        self.stall_torque / (self.stall_current * self.gear_ratio)
    }

    pub fn rotor_ke(&self) -> f64 {
        self.rated_voltage / (self.no_load_speed * self.gear_ratio)
    }
}

const M3508_P19_NOMINAL_RATIO: f64 = 19.0;
const M3508_P19_NO_LOAD_RPM: f64 = 482.0;
const M3508_P19_RATED_TORQUE: f64 = 3.0;
const M3508_P19_RATED_TORQUE_SPEED_RPM: f64 = 469.0;
const M3508_WHEEL_GEAR_RATIO: f64 = 14.0;
const M3508_TORQUE_SCALE: f64 = M3508_WHEEL_GEAR_RATIO / M3508_P19_NOMINAL_RATIO;

pub const M3508_C620_14: MotorSpec = MotorSpec {
    name: "M3508-C620-14to1",
    rated_voltage: 24.0,
    gear_ratio: M3508_WHEEL_GEAR_RATIO,
    stall_torque: M3508_P19_RATED_TORQUE * M3508_TORQUE_SCALE
        / (1.0 - M3508_P19_RATED_TORQUE_SPEED_RPM / M3508_P19_NO_LOAD_RPM),
    no_load_speed: M3508_P19_NO_LOAD_RPM * M3508_P19_NOMINAL_RATIO / M3508_WHEEL_GEAR_RATIO
        * 2.0
        * PI
        / 60.0,
    rated_torque: M3508_P19_RATED_TORQUE * M3508_TORQUE_SCALE,
    rated_current: 20.0,
    stall_current: 20.0,
    phase_resistance: 0.194,
};

pub const M3508_HEXROLL: MotorSpec = M3508_C620_14;

pub const DM8009P: MotorSpec = MotorSpec {
    name: "DM-8009P-2EC",
    rated_voltage: 24.0,
    gear_ratio: 9.0,
    stall_torque: 40.0,
    no_load_speed: 160.0 * 2.0 * PI / 60.0,
    rated_torque: 20.0,
    rated_current: 20.0,
    stall_current: 50.0,
    phase_resistance: 0.145,
};
