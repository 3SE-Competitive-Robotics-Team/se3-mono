import struct

from sim_loop.protocol import (
    MSG_STATE,
    MSG_TARGET,
    POLICY_STATE_PAYLOAD_SIZE,
    SOF,
    VERSION,
    PolicyStateFrame,
    PolicyTargetFrame,
    crc16,
    pack_policy_state,
    pack_policy_target,
    unpack_policy_state,
    unpack_policy_target,
)


def test_pack_policy_target_matches_rust_reference() -> None:
    packet = pack_policy_target(PolicyTargetFrame(42, (1.0, 2.0, 3.0, 4.0), (5.0, 6.0)))
    assert (
        packet.hex()
        == "a55a02011c002a002a0000000000803f0000004000004040000080400000a0400000c0408291"
    )
    assert unpack_policy_target(packet) == PolicyTargetFrame(42, (1.0, 2.0, 3.0, 4.0), (5.0, 6.0))


def test_crc16_matches_reference() -> None:
    assert crc16(b"123456789") == 28561


def test_pack_policy_state_round_trip() -> None:
    frame = PolicyStateFrame(
        seq=42,
        tick_ms=840,
        target_seq=41,
        target_age_ms=20,
        target_valid=1,
        rc_switch_r=1,
        output_enabled=1,
        base_ang_vel_body=(0.125, 0.25, 0.5),
        projected_gravity=(0.0, 0.0, -1.0),
        joint_pos=(1.0, 2.0, 3.0, 4.0),
        joint_vel=(5.0, 6.0, 7.0, 8.0),
        wheel_pos=(9.0, 10.0),
        wheel_vel=(11.0, 12.0),
        target_joint_pos=(13.0, 14.0, 15.0, 16.0),
        hip_torque=(17.0, 18.0, 19.0, 20.0),
        wheel_torque=(21.0, 22.0),
        wheel_motor_torque=(23.0, 24.0),
    )
    packet = pack_policy_state(frame)
    assert packet[:4] == b"\xa5\x5a\x01\x01"
    assert len(packet) == 8 + POLICY_STATE_PAYLOAD_SIZE + 2
    assert unpack_policy_state(packet) == frame


def test_unpack_rejects_bad_crc() -> None:
    packet = bytearray(pack_policy_target(PolicyTargetFrame(7, (0.1, 0.2, 0.3, 0.4), (0.5, 0.6))))
    packet[-1] ^= 0xFF
    try:
        unpack_policy_target(bytes(packet))
    except ValueError as exc:
        assert "crc" in str(exc)
    else:
        raise AssertionError("bad crc should be rejected")


def test_unpack_rejects_wrong_message_type() -> None:
    packet = bytearray(pack_policy_target(PolicyTargetFrame(7, (0.1, 0.2, 0.3, 0.4), (0.5, 0.6))))
    packet[2] = 0x01
    crc = crc16(bytes(packet[:-2]))
    packet[-2:] = crc.to_bytes(2, "little")
    try:
        unpack_policy_target(bytes(packet))
    except ValueError as exc:
        assert "message type" in str(exc)
    else:
        raise AssertionError("wrong message type should be rejected")


def test_unpack_rejects_wrong_payload_length_as_value_error() -> None:
    payload = b"\x00" * 4
    header = struct.pack("<2sBBHH", SOF, MSG_TARGET, VERSION, len(payload), 0)
    packet = header + payload
    packet += crc16(packet).to_bytes(2, "little")
    try:
        unpack_policy_target(packet)
    except ValueError as exc:
        assert "payload length" in str(exc)
    else:
        raise AssertionError("wrong payload length should be rejected")


def test_unpack_state_rejects_wrong_payload_length_as_value_error() -> None:
    payload = b"\x00" * 4
    header = struct.pack("<2sBBHH", SOF, MSG_STATE, VERSION, len(payload), 0)
    packet = header + payload
    packet += crc16(packet).to_bytes(2, "little")
    try:
        unpack_policy_state(packet)
    except ValueError as exc:
        assert "payload length" in str(exc)
    else:
        raise AssertionError("wrong state payload length should be rejected")
