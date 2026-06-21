//! CDC transport backend — USB serial to STM32 motor controller.

use std::thread;
use std::time::{Duration, Instant};

use log::{info, warn};
use serde_json::json;

use crate::cdc::{CdcError, CdcSerial, cdc_port_disappeared, resolve_cdc_port};
use crate::policy_runtime::{
    LocomotionPolicyError, LocomotionPolicyRuntime, StepOutput, TelemetryTiming,
    write_step_output_packets,
};
use crate::protocol::{MSG_POLICY_STATE, PolicyStateFrame, StreamParser, decode_policy_state};
use crate::runtime_constants::{CDC_RECONNECT_DELAY_S, LOCOMOTION_POLICY_RATE_HZ, WRITE_TIMEOUT_S};

impl LocomotionPolicyRuntime {
    pub(crate) fn run_cdc(&mut self) -> Result<(), LocomotionPolicyError> {
        let max_steps = self.cfg.max_steps;
        let period = Duration::from_secs_f64(1.0 / LOCOMOTION_POLICY_RATE_HZ);
        let period_s = period.as_secs_f64();
        self.sim_fourbar_surrogate = false;

        while max_steps == 0 || self.stats.steps < max_steps {
            self.reset_policy_memory(true, "cdc_connect")?;
            let mut parser = StreamParser::default();
            let mut latest_state: Option<PolicyStateFrame> = None;
            let mut latest_state_time = Instant::now();
            let mut next_tick = Instant::now();
            let port = resolve_cdc_port(&self.cfg.port);
            self.resolved_port = Some(port.clone());
            let loop_result = (|| -> Result<(), LocomotionPolicyError> {
                let mut serial = CdcSerial::new(&port, self.cfg.baudrate);
                serial.open()?;
                info!("USB CDC open: port={port} baudrate={}", self.cfg.baudrate);
                self.write_event(
                    "cdc_open",
                    json!({ "port": port, "reconnect_count": self.reconnect_count }),
                )?;
                while max_steps == 0 || self.stats.steps < max_steps {
                    let loop_started = Instant::now();
                    let now = Instant::now();
                    let wait_s = (next_tick.saturating_duration_since(now).as_secs_f64())
                        .min(period_s)
                        .max(0.0);
                    if serial.wait_readable(wait_s)? {
                        (latest_state, latest_state_time) = self.read_states(
                            &mut serial,
                            &mut parser,
                            latest_state,
                            latest_state_time,
                        )?;
                    }
                    if cdc_port_disappeared(&port) {
                        return Err(CdcError::Disconnected(std::io::Error::new(
                            std::io::ErrorKind::NotFound,
                            format!("USB CDC port disappeared: {port}"),
                        ))
                        .into());
                    }

                    let now = Instant::now();
                    if now < next_tick {
                        continue;
                    }
                    next_tick += period;
                    let Some(state) = latest_state.as_ref() else {
                        self.stats.steps += 1;
                        self.stats.timeout_frames += 1;
                        self.maybe_print();
                        continue;
                    };
                    let age_s = now
                        .saturating_duration_since(latest_state_time)
                        .as_secs_f64();
                    let StepOutput {
                        action,
                        flags,
                        obs,
                        policy_inference_ms,
                        packet,
                        command_packet,
                    } = self.step_from_state(state, age_s)?;
                    let write_started = Instant::now();
                    write_step_output_packets(
                        command_packet.as_deref(),
                        packet.as_deref(),
                        |packet| serial.write_all(packet, WRITE_TIMEOUT_S),
                    )?;
                    let write_ms = write_started.elapsed().as_secs_f64() * 1000.0;
                    self.record_action(state, action, flags, policy_inference_ms);
                    self.write_telemetry(
                        state,
                        &obs,
                        &action,
                        flags,
                        TelemetryTiming {
                            policy_inference_ms,
                            state_age_s: Some(age_s),
                            loop_started,
                            write_ms: Some(write_ms),
                        },
                    )?;
                    self.maybe_print();
                }
                Ok(())
            })();

            match loop_result {
                Ok(()) => return Ok(()),
                Err(LocomotionPolicyError::Cdc(
                    err @ (CdcError::Disconnected(_) | CdcError::WriteTimeout(_)),
                )) => {
                    let error_text = err.to_string();
                    self.reconnect_count += 1;
                    self.reset_policy_memory(true, "cdc_disconnected")?;
                    self.write_event(
                        "cdc_disconnected",
                        json!({
                            "port": port,
                            "reconnect_count": self.reconnect_count,
                            "error": error_text,
                        }),
                    )?;
                    warn!(
                        "USB CDC disconnected: port={port}; reconnecting in {CDC_RECONNECT_DELAY_S:.1}s"
                    );
                    thread::sleep(Duration::from_secs_f64(CDC_RECONNECT_DELAY_S));
                }
                Err(err) => return Err(err),
            }
        }
        Ok(())
    }

    fn read_states(
        &mut self,
        serial: &mut CdcSerial,
        parser: &mut StreamParser,
        mut latest_state: Option<PolicyStateFrame>,
        mut latest_state_time: Instant,
    ) -> Result<(Option<PolicyStateFrame>, Instant), LocomotionPolicyError> {
        loop {
            let data = serial.read_available()?;
            if data.is_empty() {
                return Ok((latest_state, latest_state_time));
            }
            for message in parser.feed(&data) {
                if message.msg_type != MSG_POLICY_STATE {
                    continue;
                }
                let state = decode_policy_state(&message)?;
                latest_state_time = Instant::now();
                self.stats.state_frames += 1;
                self.stats.last_state_seq = state.seq;
                latest_state = Some(state);
            }
        }
    }
}
