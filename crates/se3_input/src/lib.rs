//! Cross-platform operator input devices.

use std::time::Instant;

use gilrs::{Axis, Button, Event, EventType, GamepadId, Gilrs};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum InputError {
    #[error("failed to initialize gamepad input: {0}")]
    GamepadInit(Box<gilrs::Error>),
    #[error("no connected gamepad")]
    NoConnectedGamepad,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GamepadSelector {
    Auto,
    Index(usize),
}

impl GamepadSelector {
    pub fn parse(value: &str) -> Result<Self, InputSelectorParseError> {
        let value = value.trim();
        if value.is_empty() || value.eq_ignore_ascii_case("auto") {
            return Ok(Self::Auto);
        }
        let index = value
            .parse::<usize>()
            .map_err(|_| InputSelectorParseError(value.to_string()))?;
        Ok(Self::Index(index))
    }
}

#[derive(Debug, Error, PartialEq, Eq)]
#[error("gamepad selector must be `auto` or a numeric gamepad index, got `{0}`")]
pub struct InputSelectorParseError(String);

#[derive(Debug, Clone, PartialEq)]
pub struct GamepadSnapshot {
    pub id: usize,
    pub name: String,
    pub connected: bool,
    pub left_stick_x: f32,
    pub left_stick_y: f32,
    pub right_stick_x: f32,
    pub right_stick_y: f32,
    pub left_trigger: f32,
    pub right_trigger: f32,
    pub dpad_x: f32,
    pub dpad_y: f32,
    pub south: bool,
    pub east: bool,
    pub north: bool,
    pub west: bool,
    pub left_bumper: bool,
    pub right_bumper: bool,
    pub select: bool,
    pub start: bool,
    pub mode: bool,
    pub left_thumb: bool,
    pub right_thumb: bool,
    pub sampled_at: Instant,
}

#[derive(Debug)]
pub struct GamepadInput {
    gilrs: Gilrs,
    selector: GamepadSelector,
    active_id: Option<GamepadId>,
}

impl GamepadInput {
    pub fn new(selector: GamepadSelector) -> Result<Self, InputError> {
        let gilrs = Gilrs::new().map_err(|err| InputError::GamepadInit(Box::new(err)))?;
        Ok(Self {
            gilrs,
            selector,
            active_id: None,
        })
    }

    pub fn poll(&mut self) -> Result<GamepadSnapshot, InputError> {
        self.pump_events();
        let id = self
            .resolve_active_id()
            .ok_or(InputError::NoConnectedGamepad)?;
        let gamepad = self.gilrs.gamepad(id);
        Ok(GamepadSnapshot {
            id: usize::from(id),
            name: gamepad.name().to_string(),
            connected: gamepad.is_connected(),
            left_stick_x: axis(&gamepad, Axis::LeftStickX),
            left_stick_y: axis(&gamepad, Axis::LeftStickY),
            right_stick_x: axis(&gamepad, Axis::RightStickX),
            right_stick_y: axis(&gamepad, Axis::RightStickY),
            left_trigger: trigger(&gamepad, Axis::LeftZ, Button::LeftTrigger2),
            right_trigger: trigger(&gamepad, Axis::RightZ, Button::RightTrigger2),
            dpad_x: dpad_axis(&gamepad, Axis::DPadX, Button::DPadRight, Button::DPadLeft),
            dpad_y: dpad_axis(&gamepad, Axis::DPadY, Button::DPadUp, Button::DPadDown),
            south: gamepad.is_pressed(Button::South),
            east: gamepad.is_pressed(Button::East),
            north: gamepad.is_pressed(Button::North),
            west: gamepad.is_pressed(Button::West),
            left_bumper: gamepad.is_pressed(Button::LeftTrigger),
            right_bumper: gamepad.is_pressed(Button::RightTrigger),
            select: gamepad.is_pressed(Button::Select),
            start: gamepad.is_pressed(Button::Start),
            mode: gamepad.is_pressed(Button::Mode),
            left_thumb: gamepad.is_pressed(Button::LeftThumb),
            right_thumb: gamepad.is_pressed(Button::RightThumb),
            sampled_at: Instant::now(),
        })
    }

    pub fn connected_gamepads(&mut self) -> Vec<(usize, String)> {
        self.pump_events();
        self.gilrs
            .gamepads()
            .map(|(id, gamepad)| (usize::from(id), gamepad.name().to_string()))
            .collect()
    }

    fn pump_events(&mut self) {
        while let Some(Event { id, event, .. }) = self.gilrs.next_event() {
            self.update_active_id(id, event);
        }
    }

    fn resolve_active_id(&mut self) -> Option<GamepadId> {
        if self.active_id.is_some_and(|id| self.matches_selector(id)) {
            return self.active_id;
        }
        let id = self
            .gilrs
            .gamepads()
            .find(|(id, _)| self.matches_selector(*id))
            .map(|(id, _)| id)?;
        self.active_id = Some(id);
        Some(id)
    }

    fn matches_selector(&self, id: GamepadId) -> bool {
        match self.selector {
            GamepadSelector::Auto => true,
            GamepadSelector::Index(index) => usize::from(id) == index,
        }
    }

    fn update_active_id(&mut self, id: GamepadId, event: EventType) {
        match event {
            EventType::Connected => {
                if self.matches_selector(id) {
                    self.active_id = Some(id);
                }
            }
            EventType::Disconnected if self.active_id == Some(id) => {
                self.active_id = None;
            }
            _ if self.matches_selector(id) => {
                self.active_id = Some(id);
            }
            _ => {}
        }
    }
}

fn axis(gamepad: &gilrs::Gamepad<'_>, axis_name: Axis) -> f32 {
    gamepad
        .axis_data(axis_name)
        .map(|_| gamepad.value(axis_name))
        .unwrap_or(0.0)
        .clamp(-1.0, 1.0)
}

fn dpad_axis(
    gamepad: &gilrs::Gamepad<'_>,
    axis_name: Axis,
    positive_button: Button,
    negative_button: Button,
) -> f32 {
    dpad_axis_value(
        axis(gamepad, axis_name),
        gamepad.is_pressed(positive_button),
        gamepad.is_pressed(negative_button),
    )
}

fn dpad_axis_value(axis_value: f32, positive_pressed: bool, negative_pressed: bool) -> f32 {
    if axis_value.abs() > 1.0e-5 {
        return axis_value;
    }
    match (positive_pressed, negative_pressed) {
        (true, false) => 1.0,
        (false, true) => -1.0,
        _ => 0.0,
    }
}

fn trigger(gamepad: &gilrs::Gamepad<'_>, axis_name: Axis, button: Button) -> f32 {
    let axis_value = gamepad
        .axis_data(axis_name)
        .map(|_| gamepad.value(axis_name))
        .unwrap_or(0.0);
    if axis_value.abs() > 1.0e-5 {
        return ((axis_value + 1.0) * 0.5).clamp(0.0, 1.0);
    }
    if gamepad.is_pressed(button) { 1.0 } else { 0.0 }
}

pub fn apply_deadzone(value: f32, deadzone: f32) -> f32 {
    let deadzone = deadzone.clamp(0.0, 0.95);
    let magnitude = value.abs();
    if magnitude <= deadzone {
        return 0.0;
    }
    value.signum() * ((magnitude - deadzone) / (1.0 - deadzone)).clamp(0.0, 1.0)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic, clippy::print_stdout)]
mod tests {
    use super::*;

    #[test]
    fn selector_parses_auto_and_numeric_index() {
        assert_eq!(
            GamepadSelector::parse("auto").unwrap(),
            GamepadSelector::Auto
        );
        assert_eq!(GamepadSelector::parse("").unwrap(), GamepadSelector::Auto);
        assert_eq!(
            GamepadSelector::parse("2").unwrap(),
            GamepadSelector::Index(2)
        );
        assert!(GamepadSelector::parse("xbox").is_err());
    }

    #[test]
    fn deadzone_rescales_remaining_range() {
        assert_eq!(apply_deadzone(0.1, 0.2), 0.0);
        assert_eq!(apply_deadzone(-0.1, 0.2), 0.0);
        assert!((apply_deadzone(0.6, 0.2) - 0.5).abs() < 1.0e-6);
        assert!((apply_deadzone(-0.6, 0.2) + 0.5).abs() < 1.0e-6);
        assert_eq!(apply_deadzone(1.0, 0.2), 1.0);
    }

    #[test]
    fn dpad_axis_falls_back_to_buttons_when_axis_is_neutral() {
        assert_eq!(dpad_axis_value(0.0, true, false), 1.0);
        assert_eq!(dpad_axis_value(0.0, false, true), -1.0);
        assert_eq!(dpad_axis_value(0.0, true, true), 0.0);
        assert_eq!(dpad_axis_value(0.0, false, false), 0.0);
    }

    #[test]
    fn dpad_axis_prefers_non_neutral_axis_value() {
        assert_eq!(dpad_axis_value(1.0, false, true), 1.0);
        assert_eq!(dpad_axis_value(-1.0, true, false), -1.0);
    }
}
