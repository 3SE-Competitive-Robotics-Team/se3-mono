//! Locomotion policy runtime constants.
//!
//! These values are not configurable at runtime because they are
//! determined by the real-time constraints of the control loop or
//! the hardware telemetry protocol.

/// Default USB CDC port identifier ("auto" = first available STM32 device).
pub const DEFAULT_CDC_PORT: &str = "auto";

/// Delay before reconnecting after a CDC disconnect.
pub const CDC_RECONNECT_DELAY_S: f64 = 1.0;

/// Locomotion policy control frequency (Hz). Fixed at 50 Hz by the
/// motor controller CDC output rate and the policy's expected
/// inference cadence.
pub const LOCOMOTION_POLICY_RATE_HZ: f64 = 50.0;

/// Maximum age of the last received state frame (seconds) before the
/// runtime declares a timeout and resets policy memory.
/// 0.10 s = 5 periods at 50 Hz.
pub const STATE_TIMEOUT_S: f64 = 0.10;

/// Write timeout for serial / CDC output (seconds). One control period
/// at 50 Hz.
pub const WRITE_TIMEOUT_S: f64 = 0.02;

/// JSONL schema identifier embedded in every telemetry header line.
pub const TELEMETRY_SCHEMA: &str = "se3_locomotion_telemetry";

// --- Action flags (bitmask stored in telemetry) ---

/// Dry-run step — no hardware output.
pub const ACTION_FLAG_DRY_RUN: u32 = 1 << 0;
/// State frame timed out — policy memory was reset.
pub const ACTION_FLAG_TIMEOUT: u32 = 1 << 1;
/// Non-finite observation values detected.
pub const ACTION_FLAG_NONFINITE: u32 = 1 << 2;
/// Output was disabled; target held at last valid command.
pub const ACTION_FLAG_OUTPUT_DISABLED_HOLD: u32 = 1 << 3;
/// Command source is inactive (e.g. gamepad disconnected).
pub const ACTION_FLAG_COMMAND_INACTIVE: u32 = 1 << 4;
