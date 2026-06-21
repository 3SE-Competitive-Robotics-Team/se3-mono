//! Sim transport backend — Unix datagram socket to MuJoCo sim_loop.

use std::os::unix::net::UnixDatagram;
use std::time::{Duration, Instant};

use serde_json::json;

use crate::policy_observation::synthetic_default_state;
use crate::policy_runtime::{
    LocomotionPolicyError, LocomotionPolicyRuntime, SocketPathGuard, StepOutput, TelemetryTiming,
    sim_output_state_to_policy_state, unlink_socket_path_if_present, write_step_output_packets,
};
use crate::protocol::{MSG_POLICY_STATE, PolicyStateFrame, StreamParser, decode_policy_state};
use crate::runtime_constants::{LOCOMOTION_POLICY_RATE_HZ, STATE_TIMEOUT_S};

impl LocomotionPolicyRuntime {
    pub(crate) fn run_sim(&mut self) -> Result<(), LocomotionPolicyError> {
        let max_steps = self.cfg.max_steps;
        let period = Duration::from_secs_f64(1.0 / LOCOMOTION_POLICY_RATE_HZ);
        let period_s = period.as_secs_f64();
        self.sim_fourbar_surrogate = true;
        let state_timeout = Duration::from_secs_f64(STATE_TIMEOUT_S);
        self.reset_policy_memory(true, "sim_connect")?;
        unlink_socket_path_if_present(&self.cfg.sim_client_socket_path)?;
        let client_socket_guard = SocketPathGuard::new(self.cfg.sim_client_socket_path.clone());
        let socket = match UnixDatagram::bind(&self.cfg.sim_client_socket_path) {
            Ok(socket) => socket,
            Err(err) => {
                return Err(LocomotionPolicyError::SimSocketBind {
                    client_socket_path: self.cfg.sim_client_socket_path.clone(),
                    source: err,
                });
            }
        };
        if let Err(err) = socket.connect(&self.cfg.sim_socket_path) {
            return Err(LocomotionPolicyError::SimSocketConnect {
                sim_socket_path: self.cfg.sim_socket_path.clone(),
                client_socket_path: self.cfg.sim_client_socket_path.clone(),
                source: err,
            });
        }
        socket.set_read_timeout(Some(state_timeout))?;
        self.resolved_port = Some(self.cfg.sim_socket_path.to_string_lossy().into_owned());
        self.write_event(
            "sim_open",
            json!({
                "sim_socket_path": self.cfg.sim_socket_path,
                "sim_client_socket_path": self.cfg.sim_client_socket_path,
            }),
        )?;
        let handshake_state = synthetic_default_state(0);
        let handshake = self.make_hold_target_packet(&handshake_state)?;
        socket.send(&handshake)?;
        let mut parser = StreamParser::default();
        let mut buf = [0_u8; 4096];
        let mut latest_state: Option<PolicyStateFrame> = None;
        let mut latest_state_time = Instant::now();
        let mut next_tick = Instant::now() + period;
        while max_steps == 0 || self.stats.steps < max_steps {
            let loop_started = Instant::now();
            let now = Instant::now();
            if now >= next_tick {
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
                    |packet| socket.send(packet).map(|_| ()),
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
                continue;
            }
            let wait_s = (next_tick.saturating_duration_since(now).as_secs_f64())
                .min(period_s)
                .max(0.0);
            socket.set_read_timeout(Some(Duration::from_secs_f64(wait_s)))?;
            let n = match socket.recv(&mut buf) {
                Ok(n) => n,
                Err(err)
                    if matches!(
                        err.kind(),
                        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                    ) =>
                {
                    0
                }
                Err(err) => return Err(err.into()),
            };
            if n > 0 {
                (latest_state, latest_state_time) =
                    self.read_sim_states(&mut parser, &buf[..n], latest_state, latest_state_time)?;
            }
        }
        client_socket_guard.cleanup()?;
        Ok(())
    }

    fn read_sim_states(
        &mut self,
        parser: &mut StreamParser,
        data: &[u8],
        mut latest_state: Option<PolicyStateFrame>,
        mut latest_state_time: Instant,
    ) -> Result<(Option<PolicyStateFrame>, Instant), LocomotionPolicyError> {
        for message in parser.feed(data) {
            if message.msg_type != MSG_POLICY_STATE {
                continue;
            }
            let mut state = decode_policy_state(&message)?;
            if self.sim_fourbar_surrogate {
                state = sim_output_state_to_policy_state(state);
            }
            latest_state_time = Instant::now();
            self.stats.state_frames += 1;
            self.stats.last_state_seq = state.seq;
            latest_state = Some(state);
        }
        Ok((latest_state, latest_state_time))
    }
}
