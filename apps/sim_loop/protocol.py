from __future__ import annotations

import struct
from dataclasses import dataclass

SOF = b"\xA5\x5A"
VERSION = 1
MSG_TARGET = 0x02
POLICY_TARGET_PAYLOAD_SIZE = struct.calcsize("<I4f2f")


@dataclass(slots=True)
class PolicyTargetFrame:
    seq: int
    joint_pos: tuple[float, float, float, float]
    wheel_vel: tuple[float, float]


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


def pack_policy_target(frame: PolicyTargetFrame) -> bytes:
    payload = struct.pack("<I4f2f", frame.seq, *frame.joint_pos, *frame.wheel_vel)
    header = struct.pack("<2sBBHH", SOF, MSG_TARGET, VERSION, len(payload), frame.seq & 0xFFFF)
    packet = header + payload
    return packet + struct.pack("<H", crc16(packet))


def unpack_policy_target(packet: bytes) -> PolicyTargetFrame:
    if len(packet) < 10:
        raise ValueError("packet too short")
    if packet[:2] != SOF:
        raise ValueError("invalid sof")
    msg_type, version, payload_len, frame_seq = struct.unpack("<BBHH", packet[2:8])
    if msg_type != MSG_TARGET:
        raise ValueError(f"unexpected message type {msg_type}")
    if version != VERSION:
        raise ValueError(f"unexpected version {version}")
    frame_len = 8 + payload_len + 2
    if len(packet) != frame_len:
        raise ValueError("packet length mismatch")
    if payload_len != POLICY_TARGET_PAYLOAD_SIZE:
        raise ValueError(f"unexpected payload length {payload_len}")
    expected_crc = struct.unpack("<H", packet[-2:])[0]
    actual_crc = crc16(packet[:-2])
    if expected_crc != actual_crc:
        raise ValueError("crc mismatch")
    try:
        seq, *values = struct.unpack("<I4f2f", packet[8:-2])
    except struct.error as exc:
        raise ValueError("invalid payload encoding") from exc
    if seq & 0xFFFF != frame_seq:
        raise ValueError("frame seq mismatch")
    return PolicyTargetFrame(
        seq=seq,
        joint_pos=(values[0], values[1], values[2], values[3]),
        wheel_vel=(values[4], values[5]),
    )
