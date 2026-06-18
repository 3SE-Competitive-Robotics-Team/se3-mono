from __future__ import annotations

import struct
from dataclasses import dataclass

SOF = b"\xa5\x5a"
VERSION = 1
MSG_STATE = 0x01
MSG_TARGET = 0x02
POLICY_STATE_PAYLOAD_SIZE = struct.calcsize("<IIIH6B3f3f4f4f2f2f4f4f2f2f")
POLICY_TARGET_PAYLOAD_SIZE = struct.calcsize("<I4f2f")


@dataclass(slots=True)
class PolicyTargetFrame:
    seq: int
    joint_pos: tuple[float, float, float, float]
    wheel_vel: tuple[float, float]


@dataclass(slots=True)
class PolicyStateFrame:
    seq: int
    tick_ms: int
    target_seq: int
    target_age_ms: int
    target_valid: int
    rc_switch_r: int
    output_enabled: int
    base_ang_vel_body: tuple[float, float, float]
    projected_gravity: tuple[float, float, float]
    joint_pos: tuple[float, float, float, float]
    joint_vel: tuple[float, float, float, float]
    wheel_pos: tuple[float, float]
    wheel_vel: tuple[float, float]
    target_joint_pos: tuple[float, float, float, float]
    hip_torque: tuple[float, float, float, float]
    wheel_torque: tuple[float, float]
    wheel_motor_torque: tuple[float, float]


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


def _pack_message(msg_type: int, seq: int, payload: bytes) -> bytes:
    header = struct.pack("<2sBBHH", SOF, msg_type, VERSION, len(payload), seq & 0xFFFF)
    packet = header + payload
    return packet + struct.pack("<H", crc16(packet))


def _unpack_message(
    packet: bytes,
    expected_msg_type: int,
    expected_payload_size: int,
    *,
    seq_offset: int,
) -> bytes:
    if len(packet) < 10:
        raise ValueError("packet too short")
    if packet[:2] != SOF:
        raise ValueError("invalid sof")
    msg_type, version, payload_len, frame_seq = struct.unpack("<BBHH", packet[2:8])
    if msg_type != expected_msg_type:
        raise ValueError(f"unexpected message type {msg_type}")
    if version != VERSION:
        raise ValueError(f"unexpected version {version}")
    frame_len = 8 + payload_len + 2
    if len(packet) != frame_len:
        raise ValueError("packet length mismatch")
    if payload_len != expected_payload_size:
        raise ValueError(f"unexpected payload length {payload_len}")
    expected_crc = struct.unpack("<H", packet[-2:])[0]
    actual_crc = crc16(packet[:-2])
    if expected_crc != actual_crc:
        raise ValueError("crc mismatch")
    payload = packet[8:-2]
    payload_seq = struct.unpack("<I", payload[seq_offset : seq_offset + 4])[0]
    if payload_seq & 0xFFFF != frame_seq:
        raise ValueError("frame seq mismatch")
    return payload


def pack_policy_target(frame: PolicyTargetFrame) -> bytes:
    payload = struct.pack("<I4f2f", frame.seq, *frame.joint_pos, *frame.wheel_vel)
    return _pack_message(MSG_TARGET, frame.seq, payload)


def unpack_policy_target(packet: bytes) -> PolicyTargetFrame:
    payload = _unpack_message(packet, MSG_TARGET, POLICY_TARGET_PAYLOAD_SIZE, seq_offset=0)
    try:
        seq, *values = struct.unpack("<I4f2f", payload)
    except struct.error as exc:
        raise ValueError("invalid payload encoding") from exc
    return PolicyTargetFrame(
        seq=seq,
        joint_pos=(values[0], values[1], values[2], values[3]),
        wheel_vel=(values[4], values[5]),
    )


def pack_policy_state(frame: PolicyStateFrame) -> bytes:
    payload = struct.pack(
        "<IIIH6B3f3f4f4f2f2f4f4f2f2f",
        frame.tick_ms,
        frame.seq,
        frame.target_seq,
        frame.target_age_ms,
        frame.target_valid,
        frame.rc_switch_r,
        frame.output_enabled,
        0,
        0,
        0,
        *frame.base_ang_vel_body,
        *frame.projected_gravity,
        *frame.joint_pos,
        *frame.joint_vel,
        *frame.wheel_pos,
        *frame.wheel_vel,
        *frame.target_joint_pos,
        *frame.hip_torque,
        *frame.wheel_torque,
        *frame.wheel_motor_torque,
    )
    return _pack_message(MSG_STATE, frame.seq, payload)


def unpack_policy_state(packet: bytes) -> PolicyStateFrame:
    payload = _unpack_message(packet, MSG_STATE, POLICY_STATE_PAYLOAD_SIZE, seq_offset=4)
    try:
        values = struct.unpack("<IIIH6B3f3f4f4f2f2f4f4f2f2f", payload)
    except struct.error as exc:
        raise ValueError("invalid payload encoding") from exc
    (
        tick_ms,
        seq,
        target_seq,
        target_age_ms,
        target_valid,
        rc_switch_r,
        output_enabled,
        _pad0,
        _pad1,
        _pad2,
        *floats,
    ) = values
    return PolicyStateFrame(
        seq=seq,
        tick_ms=tick_ms,
        target_seq=target_seq,
        target_age_ms=target_age_ms,
        target_valid=target_valid,
        rc_switch_r=rc_switch_r,
        output_enabled=output_enabled,
        base_ang_vel_body=(floats[0], floats[1], floats[2]),
        projected_gravity=(floats[3], floats[4], floats[5]),
        joint_pos=(floats[6], floats[7], floats[8], floats[9]),
        joint_vel=(floats[10], floats[11], floats[12], floats[13]),
        wheel_pos=(floats[14], floats[15]),
        wheel_vel=(floats[16], floats[17]),
        target_joint_pos=(floats[18], floats[19], floats[20], floats[21]),
        hip_torque=(floats[22], floats[23], floats[24], floats[25]),
        wheel_torque=(floats[26], floats[27]),
        wheel_motor_torque=(floats[28], floats[29]),
    )
