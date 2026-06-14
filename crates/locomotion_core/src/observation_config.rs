#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObservationConfig {
    pub ang_vel_scale: f32,
    pub command_scale: [f32; 5],
    pub leg_vel_scale: f32,
    pub wheel_vel_scale: f32,
    pub clip_value: f32,
    pub num_obs: usize,
    pub num_actions: usize,
}

impl Default for ObservationConfig {
    fn default() -> Self {
        Self {
            ang_vel_scale: 0.25,
            command_scale: [2.0, 0.25, 5.0, 5.0, 5.0],
            leg_vel_scale: 0.25,
            wheel_vel_scale: 0.05,
            clip_value: 100.0,
            num_obs: 32,
            num_actions: 6,
        }
    }
}
