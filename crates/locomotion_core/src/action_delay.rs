use thiserror::Error;

#[derive(Debug, Error, PartialEq)]
pub enum ActionDelayError {
    #[error("sim_dt must be positive, got {0}")]
    NonPositiveSimDt(f64),
    #[error("min_delay_s must be <= max_delay_s, got {min} > {max}")]
    InvalidRange { min: f64, max: f64 },
    #[error(
        "delay_s must be inside [min_delay_s, max_delay_s] when randomize is enabled, got {delay} not in [{min}, {max}]"
    )]
    DelayOutsideRandomRange { delay: f64, min: f64, max: f64 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DelayResampleMode {
    Reset,
}

pub fn delay_seconds_to_steps(delay_s: f64, sim_dt: f64) -> Result<usize, ActionDelayError> {
    if sim_dt <= 0.0 {
        return Err(ActionDelayError::NonPositiveSimDt(sim_dt));
    }
    if delay_s <= 0.0 {
        return Ok(0);
    }
    Ok(((delay_s / sim_dt) + 0.5).floor().max(0.0) as usize)
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ActionDelayConfig {
    pub enabled: bool,
    pub delay_s: f64,
    pub randomize: bool,
    pub min_delay_s: f64,
    pub max_delay_s: f64,
    pub resample: DelayResampleMode,
}

impl Default for ActionDelayConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            delay_s: 0.005,
            randomize: true,
            min_delay_s: 0.004,
            max_delay_s: 0.006,
            resample: DelayResampleMode::Reset,
        }
    }
}

impl ActionDelayConfig {
    pub fn validate(&self) -> Result<(), ActionDelayError> {
        if self.min_delay_s > self.max_delay_s {
            return Err(ActionDelayError::InvalidRange {
                min: self.min_delay_s,
                max: self.max_delay_s,
            });
        }
        if self.enabled
            && self.randomize
            && !(self.min_delay_s..=self.max_delay_s).contains(&self.delay_s)
        {
            return Err(ActionDelayError::DelayOutsideRandomRange {
                delay: self.delay_s,
                min: self.min_delay_s,
                max: self.max_delay_s,
            });
        }
        Ok(())
    }

    pub fn nominal_steps(&self, sim_dt: f64) -> Result<usize, ActionDelayError> {
        if !self.enabled {
            return Ok(0);
        }
        delay_seconds_to_steps(self.delay_s, sim_dt)
    }

    pub fn step_bounds(&self, sim_dt: f64) -> Result<(usize, usize), ActionDelayError> {
        if !self.enabled {
            return Ok((0, 0));
        }
        if !self.randomize {
            let steps = self.nominal_steps(sim_dt)?;
            return Ok((steps, steps));
        }
        let mut min_steps = delay_seconds_to_steps(self.min_delay_s, sim_dt)?;
        let mut max_steps = delay_seconds_to_steps(self.max_delay_s, sim_dt)?;
        if min_steps > max_steps {
            std::mem::swap(&mut min_steps, &mut max_steps);
        }
        Ok((min_steps, max_steps))
    }

    pub fn actual_delay_s(&self, steps: usize, sim_dt: f64) -> f64 {
        let _ = self;
        steps as f64 * sim_dt
    }
}
