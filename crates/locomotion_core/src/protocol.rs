use thiserror::Error;

pub const SOF: [u8; 2] = [0xA5, 0x5A];
pub const VERSION: u8 = 1;
pub const MAX_PAYLOAD_SIZE: usize = 160;

pub const MSG_STATE: u8 = 0x01;
pub const MSG_TARGET: u8 = 0x02;
pub const MSG_LATENCY: u8 = 0x03;

pub const MSG_POLICY_STATE: u8 = MSG_STATE;
pub const MSG_POLICY_ACTION: u8 = MSG_TARGET;

const HEADER_SIZE: usize = 8;
const CRC_SIZE: usize = 2;
const POLICY_STATE_V1_SIZE: usize = 92;
const POLICY_STATE_V2_SIZE: usize = 124;
const POLICY_STATE_SIZE: usize = 140;
const POLICY_TARGET_SIZE: usize = 28;
const POLICY_LATENCY_SIZE: usize = 12;

#[derive(Debug, Error, PartialEq)]
pub enum ProtocolError {
    #[error("state payload size mismatch: {0}")]
    StatePayloadSize(usize),
    #[error("target payload size mismatch: {0}")]
    TargetPayloadSize(usize),
    #[error("latency payload size mismatch: {0}")]
    LatencyPayloadSize(usize),
    #[error("unexpected message type {actual}, expected {expected}")]
    UnexpectedMessageType { actual: u8, expected: u8 },
    #[error("payload too large: {actual} > {max}")]
    PayloadTooLarge { actual: usize, max: usize },
    #[error("{name} contains non-finite values")]
    NonFinite { name: &'static str },
}

#[derive(Debug, Clone, PartialEq)]
pub struct PolicyStateFrame {
    pub seq: u32,
    pub tick_ms: u32,
    pub target_seq: u32,
    pub target_age_ms: u16,
    pub target_valid: u8,
    pub rc_switch_r: u8,
    pub output_enabled: u8,
    pub base_ang_vel_body: [f32; 3],
    pub projected_gravity: [f32; 3],
    pub joint_pos: [f32; 4],
    pub joint_vel: [f32; 4],
    pub wheel_pos: [f32; 2],
    pub wheel_vel: [f32; 2],
    pub target_joint_pos: [f32; 4],
    pub hip_torque: [f32; 4],
    pub wheel_torque: [f32; 2],
    pub wheel_motor_torque: [f32; 2],
}

impl PolicyStateFrame {
    pub fn timestamp_us(&self) -> u32 {
        self.tick_ms.wrapping_mul(1000)
    }

    pub fn dof_pos(&self) -> [f32; 6] {
        [
            self.joint_pos[0],
            self.joint_pos[1],
            self.joint_pos[2],
            self.joint_pos[3],
            self.wheel_pos[0],
            self.wheel_pos[1],
        ]
    }

    pub fn dof_vel(&self) -> [f32; 6] {
        [
            self.joint_vel[0],
            self.joint_vel[1],
            self.joint_vel[2],
            self.joint_vel[3],
            self.wheel_vel[0],
            self.wheel_vel[1],
        ]
    }

    pub fn pack_payload(&self) -> Result<Vec<u8>, ProtocolError> {
        finite_floats(&self.base_ang_vel_body, "base_ang_vel_body")?;
        finite_floats(&self.projected_gravity, "projected_gravity")?;
        finite_floats(&self.joint_pos, "joint_pos")?;
        finite_floats(&self.joint_vel, "joint_vel")?;
        finite_floats(&self.wheel_pos, "wheel_pos")?;
        finite_floats(&self.wheel_vel, "wheel_vel")?;
        finite_floats(&self.target_joint_pos, "target_joint_pos")?;
        finite_floats(&self.hip_torque, "hip_torque")?;
        finite_floats(&self.wheel_torque, "wheel_torque")?;
        finite_floats(&self.wheel_motor_torque, "wheel_motor_torque")?;

        let mut out = Vec::with_capacity(POLICY_STATE_SIZE);
        write_u32(&mut out, self.tick_ms);
        write_u32(&mut out, self.seq);
        write_u32(&mut out, self.target_seq);
        write_u16(&mut out, self.target_age_ms);
        out.extend([
            self.target_valid,
            self.rc_switch_r,
            self.output_enabled,
            0,
            0,
            0,
        ]);
        write_f32s(&mut out, &self.base_ang_vel_body);
        write_f32s(&mut out, &self.projected_gravity);
        write_f32s(&mut out, &self.joint_pos);
        write_f32s(&mut out, &self.joint_vel);
        write_f32s(&mut out, &self.wheel_pos);
        write_f32s(&mut out, &self.wheel_vel);
        write_f32s(&mut out, &self.target_joint_pos);
        write_f32s(&mut out, &self.hip_torque);
        write_f32s(&mut out, &self.wheel_torque);
        write_f32s(&mut out, &self.wheel_motor_torque);
        debug_assert_eq!(out.len(), POLICY_STATE_SIZE);
        Ok(out)
    }

    pub fn from_payload(payload: &[u8]) -> Result<Self, ProtocolError> {
        match payload.len() {
            POLICY_STATE_V1_SIZE => Self::from_payload_v1(payload),
            POLICY_STATE_V2_SIZE => Self::from_payload_v2(payload),
            POLICY_STATE_SIZE => Self::from_payload_v3(payload),
            len => Err(ProtocolError::StatePayloadSize(len)),
        }
    }

    fn from_payload_common(payload: &[u8]) -> PayloadReader<'_> {
        PayloadReader::new(payload)
    }

    fn from_payload_v1(payload: &[u8]) -> Result<Self, ProtocolError> {
        let mut r = Self::from_payload_common(payload);
        Ok(Self {
            tick_ms: r.u32(),
            seq: r.u32(),
            target_seq: r.u32(),
            target_age_ms: r.u16(),
            target_valid: r.u8(),
            rc_switch_r: r.u8(),
            output_enabled: r.u8(),
            base_ang_vel_body: r.f32x3_after_padding(3),
            projected_gravity: r.f32x3(),
            joint_pos: r.f32x4(),
            joint_vel: r.f32x4(),
            wheel_pos: r.f32x2(),
            wheel_vel: r.f32x2(),
            target_joint_pos: [0.0; 4],
            hip_torque: [0.0; 4],
            wheel_torque: [0.0; 2],
            wheel_motor_torque: [0.0; 2],
        })
    }

    fn from_payload_v2(payload: &[u8]) -> Result<Self, ProtocolError> {
        let mut r = Self::from_payload_common(payload);
        Ok(Self {
            tick_ms: r.u32(),
            seq: r.u32(),
            target_seq: r.u32(),
            target_age_ms: r.u16(),
            target_valid: r.u8(),
            rc_switch_r: r.u8(),
            output_enabled: r.u8(),
            base_ang_vel_body: r.f32x3_after_padding(3),
            projected_gravity: r.f32x3(),
            joint_pos: r.f32x4(),
            joint_vel: r.f32x4(),
            wheel_pos: r.f32x2(),
            wheel_vel: r.f32x2(),
            target_joint_pos: r.f32x4(),
            hip_torque: r.f32x4(),
            wheel_torque: [0.0; 2],
            wheel_motor_torque: [0.0; 2],
        })
    }

    fn from_payload_v3(payload: &[u8]) -> Result<Self, ProtocolError> {
        let mut r = Self::from_payload_common(payload);
        Ok(Self {
            tick_ms: r.u32(),
            seq: r.u32(),
            target_seq: r.u32(),
            target_age_ms: r.u16(),
            target_valid: r.u8(),
            rc_switch_r: r.u8(),
            output_enabled: r.u8(),
            base_ang_vel_body: r.f32x3_after_padding(3),
            projected_gravity: r.f32x3(),
            joint_pos: r.f32x4(),
            joint_vel: r.f32x4(),
            wheel_pos: r.f32x2(),
            wheel_vel: r.f32x2(),
            target_joint_pos: r.f32x4(),
            hip_torque: r.f32x4(),
            wheel_torque: r.f32x2(),
            wheel_motor_torque: r.f32x2(),
        })
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PolicyTargetFrame {
    pub seq: u32,
    pub joint_pos: [f32; 4],
    pub wheel_vel: [f32; 2],
}

impl PolicyTargetFrame {
    pub fn pack_payload(&self) -> Result<Vec<u8>, ProtocolError> {
        finite_floats(&self.joint_pos, "joint_pos")?;
        finite_floats(&self.wheel_vel, "wheel_vel")?;
        let mut out = Vec::with_capacity(POLICY_TARGET_SIZE);
        write_u32(&mut out, self.seq);
        write_f32s(&mut out, &self.joint_pos);
        write_f32s(&mut out, &self.wheel_vel);
        Ok(out)
    }

    pub fn from_payload(payload: &[u8]) -> Result<Self, ProtocolError> {
        if payload.len() != POLICY_TARGET_SIZE {
            return Err(ProtocolError::TargetPayloadSize(payload.len()));
        }
        let mut r = PayloadReader::new(payload);
        Ok(Self {
            seq: r.u32(),
            joint_pos: r.f32x4(),
            wheel_vel: r.f32x2(),
        })
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PolicyLatencyFrame {
    pub policy_seq: u32,
    pub rx_to_output_us: u32,
    pub output_enabled: u8,
}

impl PolicyLatencyFrame {
    pub fn from_payload(payload: &[u8]) -> Result<Self, ProtocolError> {
        if payload.len() != POLICY_LATENCY_SIZE {
            return Err(ProtocolError::LatencyPayloadSize(payload.len()));
        }
        let mut r = PayloadReader::new(payload);
        Ok(Self {
            policy_seq: r.u32(),
            rx_to_output_us: r.u32(),
            output_enabled: r.u8(),
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParsedMessage {
    pub msg_type: u8,
    pub frame_seq: u16,
    pub payload: Vec<u8>,
}

#[derive(Debug, Clone)]
pub struct StreamParser {
    max_payload_size: usize,
    buffer: Vec<u8>,
}

impl Default for StreamParser {
    fn default() -> Self {
        Self::new(MAX_PAYLOAD_SIZE)
    }
}

impl StreamParser {
    pub fn new(max_payload_size: usize) -> Self {
        Self {
            max_payload_size,
            buffer: Vec::new(),
        }
    }

    pub fn feed(&mut self, data: &[u8]) -> Vec<ParsedMessage> {
        if !data.is_empty() {
            self.buffer.extend_from_slice(data);
        }
        let mut messages = Vec::new();

        loop {
            let Some(start) = find_sof(&self.buffer) else {
                self.drop_noise();
                break;
            };
            if start > 0 {
                self.buffer.drain(..start);
            }
            if self.buffer.len() < HEADER_SIZE {
                break;
            }

            let sof0 = self.buffer[0];
            let sof1 = self.buffer[1];
            let msg_type = self.buffer[2];
            let version = self.buffer[3];
            let payload_len = u16::from_le_bytes([self.buffer[4], self.buffer[5]]) as usize;
            let frame_seq = u16::from_le_bytes([self.buffer[6], self.buffer[7]]);
            if [sof0, sof1] != SOF {
                self.buffer.drain(..1);
                continue;
            }
            if version != VERSION || payload_len > self.max_payload_size {
                self.buffer.drain(..1);
                continue;
            }
            let frame_len = HEADER_SIZE + payload_len + CRC_SIZE;
            if self.buffer.len() < frame_len {
                break;
            }
            let expected_crc = u16::from_le_bytes([
                self.buffer[HEADER_SIZE + payload_len],
                self.buffer[HEADER_SIZE + payload_len + 1],
            ]);
            let actual_crc = crc16(&self.buffer[..HEADER_SIZE + payload_len]);
            if actual_crc != expected_crc {
                self.buffer.drain(..1);
                continue;
            }
            let payload = self.buffer[HEADER_SIZE..HEADER_SIZE + payload_len].to_vec();
            messages.push(ParsedMessage {
                msg_type,
                frame_seq,
                payload,
            });
            self.buffer.drain(..frame_len);
        }

        messages
    }

    fn drop_noise(&mut self) {
        if self.buffer.len() > SOF.len() - 1 {
            let keep = SOF.len() - 1;
            let drop_len = self.buffer.len() - keep;
            self.buffer.drain(..drop_len);
        }
    }
}

pub fn pack_message(
    msg_type: u8,
    payload: &[u8],
    frame_seq: u16,
) -> Result<Vec<u8>, ProtocolError> {
    if payload.len() > MAX_PAYLOAD_SIZE {
        return Err(ProtocolError::PayloadTooLarge {
            actual: payload.len(),
            max: MAX_PAYLOAD_SIZE,
        });
    }
    let mut out = Vec::with_capacity(HEADER_SIZE + payload.len() + CRC_SIZE);
    out.extend(SOF);
    out.push(msg_type);
    out.push(VERSION);
    write_u16(&mut out, payload.len() as u16);
    write_u16(&mut out, frame_seq);
    out.extend_from_slice(payload);
    let crc = crc16(&out);
    write_u16(&mut out, crc);
    Ok(out)
}

pub fn pack_policy_state(frame: &PolicyStateFrame) -> Result<Vec<u8>, ProtocolError> {
    pack_message(MSG_STATE, &frame.pack_payload()?, frame.seq as u16)
}

pub fn pack_policy_target(frame: &PolicyTargetFrame) -> Result<Vec<u8>, ProtocolError> {
    pack_message(MSG_TARGET, &frame.pack_payload()?, frame.seq as u16)
}

pub fn pack_policy_action(frame: &PolicyTargetFrame) -> Result<Vec<u8>, ProtocolError> {
    pack_policy_target(frame)
}

pub fn decode_policy_state(message: &ParsedMessage) -> Result<PolicyStateFrame, ProtocolError> {
    if message.msg_type != MSG_STATE {
        return Err(ProtocolError::UnexpectedMessageType {
            actual: message.msg_type,
            expected: MSG_STATE,
        });
    }
    PolicyStateFrame::from_payload(&message.payload)
}

pub fn decode_policy_target(message: &ParsedMessage) -> Result<PolicyTargetFrame, ProtocolError> {
    if message.msg_type != MSG_TARGET {
        return Err(ProtocolError::UnexpectedMessageType {
            actual: message.msg_type,
            expected: MSG_TARGET,
        });
    }
    PolicyTargetFrame::from_payload(&message.payload)
}

pub fn decode_policy_action(message: &ParsedMessage) -> Result<PolicyTargetFrame, ProtocolError> {
    decode_policy_target(message)
}

pub fn decode_policy_latency(message: &ParsedMessage) -> Result<PolicyLatencyFrame, ProtocolError> {
    if message.msg_type != MSG_LATENCY {
        return Err(ProtocolError::UnexpectedMessageType {
            actual: message.msg_type,
            expected: MSG_LATENCY,
        });
    }
    PolicyLatencyFrame::from_payload(&message.payload)
}

pub fn finite_floats<const N: usize>(
    values: &[f32; N],
    name: &'static str,
) -> Result<[f32; N], ProtocolError> {
    if values.iter().any(|value| !value.is_finite()) {
        return Err(ProtocolError::NonFinite { name });
    }
    Ok(*values)
}

pub fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0xFFFF_u16;
    for &byte in data {
        crc ^= byte as u16;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0x8408;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

fn find_sof(buffer: &[u8]) -> Option<usize> {
    buffer.windows(2).position(|pair| pair == SOF)
}

fn write_u16(out: &mut Vec<u8>, value: u16) {
    out.extend(value.to_le_bytes());
}

fn write_u32(out: &mut Vec<u8>, value: u32) {
    out.extend(value.to_le_bytes());
}

fn write_f32s(out: &mut Vec<u8>, values: &[f32]) {
    for value in values {
        out.extend(value.to_le_bytes());
    }
}

struct PayloadReader<'a> {
    payload: &'a [u8],
    offset: usize,
}

impl<'a> PayloadReader<'a> {
    fn new(payload: &'a [u8]) -> Self {
        Self { payload, offset: 0 }
    }

    fn u8(&mut self) -> u8 {
        let value = self.payload[self.offset];
        self.offset += 1;
        value
    }

    fn u16(&mut self) -> u16 {
        let value = u16::from_le_bytes(
            self.payload[self.offset..self.offset + 2]
                .try_into()
                .expect("valid u16 payload slice"),
        );
        self.offset += 2;
        value
    }

    fn u32(&mut self) -> u32 {
        let value = u32::from_le_bytes(
            self.payload[self.offset..self.offset + 4]
                .try_into()
                .expect("valid u32 payload slice"),
        );
        self.offset += 4;
        value
    }

    fn f32(&mut self) -> f32 {
        let value = f32::from_le_bytes(
            self.payload[self.offset..self.offset + 4]
                .try_into()
                .expect("valid u32 payload slice"),
        );
        self.offset += 4;
        value
    }

    fn skip(&mut self, n: usize) {
        self.offset += n;
    }

    fn f32x2(&mut self) -> [f32; 2] {
        [self.f32(), self.f32()]
    }

    fn f32x3(&mut self) -> [f32; 3] {
        [self.f32(), self.f32(), self.f32()]
    }

    fn f32x3_after_padding(&mut self, padding: usize) -> [f32; 3] {
        self.skip(padding);
        self.f32x3()
    }

    fn f32x4(&mut self) -> [f32; 4] {
        [self.f32(), self.f32(), self.f32(), self.f32()]
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic, clippy::print_stdout)]
mod tests {
    use super::*;

    #[test]
    fn crc_matches_python_reference() {
        assert_eq!(crc16(b"123456789"), 28561);
    }

    #[test]
    fn target_packet_matches_python_reference() {
        let packet = pack_policy_target(&PolicyTargetFrame {
            seq: 42,
            joint_pos: [1.0, 2.0, 3.0, 4.0],
            wheel_vel: [5.0, 6.0],
        })
        .unwrap();
        assert_eq!(
            hex_lower(&packet),
            "a55a02011c002a002a0000000000803f0000004000004040000080400000a0400000c0408291"
        );
    }

    #[test]
    fn stream_parser_handles_noise_and_split_frames() {
        let packet = pack_policy_target(&PolicyTargetFrame {
            seq: 7,
            joint_pos: [0.1, 0.2, 0.3, 0.4],
            wheel_vel: [0.5, 0.6],
        })
        .unwrap();
        let mut parser = StreamParser::default();
        assert!(parser.feed(&[0, 1, 2, packet[0]]).is_empty());
        let messages = parser.feed(&packet[1..]);
        assert_eq!(messages.len(), 1);
        let target = decode_policy_target(&messages[0]).unwrap();
        assert_eq!(target.seq, 7);
    }

    fn hex_lower(bytes: &[u8]) -> String {
        const LUT: &[u8; 16] = b"0123456789abcdef";
        let mut out = String::with_capacity(bytes.len() * 2);
        for byte in bytes {
            out.push(LUT[(byte >> 4) as usize] as char);
            out.push(LUT[(byte & 0x0f) as usize] as char);
        }
        out
    }
}
