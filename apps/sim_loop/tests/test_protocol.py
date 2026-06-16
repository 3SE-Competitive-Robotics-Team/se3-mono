import struct

from sim_loop.protocol import (
    MSG_TARGET,
    SOF,
    VERSION,
    PolicyTargetFrame,
    crc16,
    pack_policy_target,
    unpack_policy_target,
)


def test_pack_policy_target_matches_rust_reference() -> None:
    packet = pack_policy_target(
        PolicyTargetFrame(42, (1.0, 2.0, 3.0, 4.0), (5.0, 6.0))
    )
    assert packet.hex() == "a55a02011c002a002a0000000000803f0000004000004040000080400000a0400000c0408291"
    assert unpack_policy_target(packet) == PolicyTargetFrame(
        42, (1.0, 2.0, 3.0, 4.0), (5.0, 6.0)
    )


def test_crc16_matches_reference() -> None:
    assert crc16(b"123456789") == 28561


def test_unpack_rejects_bad_crc() -> None:
    packet = bytearray(
        pack_policy_target(PolicyTargetFrame(7, (0.1, 0.2, 0.3, 0.4), (0.5, 0.6)))
    )
    packet[-1] ^= 0xFF
    try:
        unpack_policy_target(bytes(packet))
    except ValueError as exc:
        assert "crc" in str(exc)
    else:
        raise AssertionError("bad crc should be rejected")


def test_unpack_rejects_wrong_message_type() -> None:
    packet = bytearray(
        pack_policy_target(PolicyTargetFrame(7, (0.1, 0.2, 0.3, 0.4), (0.5, 0.6)))
    )
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
