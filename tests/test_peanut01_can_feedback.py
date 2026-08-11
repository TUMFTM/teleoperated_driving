import json
import pathlib
import runpy

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
CAN_FEEDBACK = ROOT / "work/peanut01_can_feedback.py"
MODULE = runpy.run_path(str(CAN_FEEDBACK))

CanFeedbackTracker = MODULE["CanFeedbackTracker"]
mcu_checksum = MODULE["mcu_checksum"]
eps_xor = MODULE["eps_xor"]

NOW_NS = 1_000_000_000
VALID_NEUTRAL_COMMAND = "29000000000000D6"
VALID_FRAMES = {
    0x1A2: "891040050005011B",
    0x1A3: "01F4000A0064009C",
    0x1A4: "00000000000000FF",
    0x401: "20800004005500F1",
}


def raw_frame(can_id, payload_hex, **changes):
    value = {
        "stamp_ns": 123,
        "interface": "can_mingnuo",
        "id": can_id,
        "id_hex": f"{can_id:X}",
        "is_extended": False,
        "is_rtr": False,
        "is_error": False,
        "dlc": 8,
        "data_hex": payload_hex,
    }
    value.update(changes)
    return json.dumps(value, separators=(",", ":"))


def valid_payload(can_id, first_seven):
    data = bytearray(first_seven)
    data.append(eps_xor(data) if can_id == 0x401 else mcu_checksum(data))
    return data.hex().upper()


def populated_tracker(overrides=None):
    tracker = CanFeedbackTracker(timeout_ns=300_000_000)
    frames = dict(VALID_FRAMES)
    frames.update(overrides or {})
    for can_id, payload in frames.items():
        assert tracker.update_json(raw_frame(can_id, payload), NOW_NS)
    return tracker


def test_decodes_captured_mcu_and_eps_frames():
    tracker = CanFeedbackTracker(timeout_ns=300_000_000)

    for can_id, payload in VALID_FRAMES.items():
        assert tracker.update_json(raw_frame(can_id, payload), NOW_NS)

    status = tracker.snapshot(NOW_NS)
    assert (status.mcu.power_up, status.mcu.enabled) == (True, False)
    assert (status.mcu.direction, status.mcu.gear, status.mcu.brake_locked) == (
        0,
        2,
        True,
    )
    assert status.mcu.motor_angle_deg == pytest.approx(0x1040 * 0.0054932478828107)
    assert status.mcu.motor_temperature_c == pytest.approx(28.1)
    assert (status.mcu.voltage_v, status.mcu.current_a, status.mcu.motor_rpm) == (
        50.0,
        1.0,
        100,
    )
    assert status.mcu.error_codes == (0, 0, 0, 0, 0)
    assert status.mcu.error_count == 0
    assert not status.mcu.manual_override
    assert (status.eps.mode, status.eps.init_status) == (0x20, 0x55)
    assert (status.eps.error_1, status.eps.error_2) == (0, 0)
    assert status.eps.angle_deg == pytest.approx(-921.6)


def test_decodes_captured_neutral_stop_command():
    tracker = CanFeedbackTracker(timeout_ns=300_000_000)

    assert tracker.update_json(
        raw_frame(0x1A1, VALID_NEUTRAL_COMMAND), NOW_NS
    )

    status = tracker.snapshot(NOW_NS)
    assert not status.command.enabled
    assert status.command.mode == 1
    assert status.command.gear == 2
    assert status.command.brake_mode == 1
    assert status.command.motor_rpm == 0
    assert status.command.stamp_ns == NOW_NS
    assert status.software_neutral
    assert tracker.ages_ns(NOW_NS + 5)["mcu_command"] == 5


@pytest.mark.parametrize(
    "first_seven",
    (
        bytes.fromhex("A9000000000000"),
        bytes.fromhex("09000000000000"),
        bytes.fromhex("25000000000000"),
        bytes.fromhex("28000000000000"),
        bytes.fromhex("29000001000000"),
    ),
)
def test_software_neutral_rejects_non_neutral_command_semantics(first_seven):
    tracker = CanFeedbackTracker(timeout_ns=300_000_000)
    payload = valid_payload(0x1A1, first_seven)

    assert tracker.update_json(raw_frame(0x1A1, payload), NOW_NS)

    assert not tracker.snapshot(NOW_NS).software_neutral


def test_software_neutral_requires_fresh_command():
    tracker = CanFeedbackTracker(timeout_ns=300_000_000)
    assert tracker.update_json(
        raw_frame(0x1A1, VALID_NEUTRAL_COMMAND), NOW_NS
    )

    assert tracker.snapshot(NOW_NS + 300_000_000).software_neutral
    assert not tracker.snapshot(NOW_NS + 300_000_001).software_neutral


def test_rejected_command_checksum_does_not_refresh_timestamp():
    tracker = CanFeedbackTracker(timeout_ns=300_000_000)
    assert tracker.update_json(
        raw_frame(0x1A1, VALID_NEUTRAL_COMMAND), NOW_NS
    )

    assert not tracker.update_json(
        raw_frame(0x1A1, "29000000000000D7"), NOW_NS + 1
    )

    status = tracker.snapshot(NOW_NS + 1)
    assert status.command.stamp_ns == NOW_NS
    assert status.software_neutral


@pytest.mark.parametrize(
    "payload",
    (
        "not-json",
        "[]",
        json.dumps({}),
        raw_frame(0x1A2, VALID_FRAMES[0x1A2], id="418"),
        raw_frame(0x1A2, VALID_FRAMES[0x1A2], id_hex=418),
        raw_frame(0x1A2, VALID_FRAMES[0x1A2], is_extended=0),
        raw_frame(0x1A2, VALID_FRAMES[0x1A2], is_rtr=0),
        raw_frame(0x1A2, VALID_FRAMES[0x1A2], is_error=0),
        raw_frame(0x1A2, VALID_FRAMES[0x1A2], dlc="8"),
        raw_frame(0x1A2, VALID_FRAMES[0x1A2], data_hex=123),
        raw_frame(0x1A2, VALID_FRAMES[0x1A2], extra=True),
    ),
)
def test_rejects_malformed_json_envelopes(payload):
    tracker = CanFeedbackTracker(timeout_ns=300_000_000)

    assert not tracker.update_json(payload, NOW_NS)
    assert tracker.snapshot(NOW_NS).mcu.stat1_stamp_ns == 0


@pytest.mark.parametrize(
    "changes",
    (
        {"dlc": 7},
        {"id_hex": "1A3"},
        {"is_extended": True},
        {"is_rtr": True},
        {"is_error": True},
        {"data_hex": "89104005000501"},
        {"data_hex": "891040050005011"},
        {"data_hex": "89104005000501ZZ"},
        {"data_hex": "891040050005011A"},
    ),
)
def test_rejected_mcu_frames_do_not_refresh_timestamp(changes):
    tracker = CanFeedbackTracker(timeout_ns=300_000_000)

    assert tracker.update_json(raw_frame(0x1A2, VALID_FRAMES[0x1A2]), NOW_NS)
    assert not tracker.update_json(
        raw_frame(0x1A2, VALID_FRAMES[0x1A2], **changes), NOW_NS + 1
    )

    assert tracker.snapshot(NOW_NS + 1).mcu.stat1_stamp_ns == NOW_NS


def test_rejected_eps_xor_does_not_refresh_timestamp():
    tracker = CanFeedbackTracker(timeout_ns=300_000_000)
    assert tracker.update_json(raw_frame(0x401, VALID_FRAMES[0x401]), NOW_NS)

    assert not tracker.update_json(
        raw_frame(0x401, "20800004005500F0"), NOW_NS + 1
    )

    assert tracker.snapshot(NOW_NS + 1).eps.status_stamp_ns == NOW_NS


def test_unrecognized_frame_is_ignored_without_mutation():
    tracker = CanFeedbackTracker(timeout_ns=300_000_000)

    assert not tracker.update_json(raw_frame(0x123, "00000000000000FF"), NOW_NS)

    status = tracker.snapshot(NOW_NS)
    assert status.mcu.stat1_stamp_ns == 0
    assert status.eps.status_stamp_ns == 0


def test_approvals_require_all_independently_fresh_frames():
    tracker = populated_tracker()

    approved = tracker.snapshot(
        NOW_NS + 300_000_000, emergency_fresh=True, emergency=False
    )
    assert approved.emergency_released
    assert approved.lateral_approved
    assert approved.longitudinal_approved

    stale = tracker.snapshot(
        NOW_NS + 300_000_001, emergency_fresh=True, emergency=False
    )
    assert not stale.lateral_approved
    assert not stale.longitudinal_approved


def test_mcu_enable_brake_direction_and_gear_are_not_prearm_approval_inputs():
    status = populated_tracker().snapshot(
        NOW_NS, emergency_fresh=True, emergency=False
    )

    assert not status.mcu.enabled
    assert status.mcu.brake_locked
    assert status.mcu.gear == 2
    assert status.longitudinal_approved


@pytest.mark.parametrize("missing_id", (0x1A2, 0x1A3, 0x1A4))
def test_longitudinal_approval_requires_every_mcu_status_group(missing_id):
    tracker = CanFeedbackTracker(timeout_ns=300_000_000)
    for can_id, payload in VALID_FRAMES.items():
        if can_id != missing_id:
            assert tracker.update_json(raw_frame(can_id, payload), NOW_NS)

    status = tracker.snapshot(NOW_NS, emergency_fresh=True, emergency=False)

    assert not status.longitudinal_approved


@pytest.mark.parametrize(
    "overrides",
    (
        {0x1A2: valid_payload(0x1A2, bytes.fromhex("09104005000501"))},
        {0x1A4: valid_payload(0x1A4, bytes.fromhex("01000000000000"))},
        {0x1A4: valid_payload(0x1A4, bytes.fromhex("00000000000100"))},
    ),
)
def test_longitudinal_approval_rejects_power_fault_and_manual_override(overrides):
    status = populated_tracker(overrides).snapshot(
        NOW_NS, emergency_fresh=True, emergency=False
    )

    assert not status.longitudinal_approved


@pytest.mark.parametrize(
    "eps_first_seven",
    (
        bytes.fromhex("10800004005500"),
        bytes.fromhex("20800104005500"),
        bytes.fromhex("20800004000000"),
        bytes.fromhex("20800004005501"),
    ),
)
def test_lateral_approval_rejects_mode_initialization_and_errors(eps_first_seven):
    eps_payload = valid_payload(0x401, eps_first_seven)
    status = populated_tracker({0x401: eps_payload}).snapshot(
        NOW_NS, emergency_fresh=True, emergency=False
    )

    assert not status.lateral_approved


def test_lateral_approval_accepts_alternate_deployed_eps_states():
    eps_payload = valid_payload(0x401, bytes.fromhex("2380000400EE00"))
    status = populated_tracker({0x401: eps_payload}).snapshot(
        NOW_NS, emergency_fresh=True, emergency=False
    )

    assert status.lateral_approved


@pytest.mark.parametrize(
    ("emergency_fresh", "emergency"),
    ((False, False), (True, True)),
)
def test_emergency_release_requires_fresh_false_signal(emergency_fresh, emergency):
    status = populated_tracker().snapshot(
        NOW_NS, emergency_fresh=emergency_fresh, emergency=emergency
    )

    assert not status.emergency_released


def test_reports_independent_feedback_ages():
    tracker = populated_tracker()

    assert tracker.ages_ns(NOW_NS + 5) == {
        "mcu_command": None,
        "mcu_stat1": 5,
        "mcu_stat2": 5,
        "mcu_error": 5,
        "eps_status1": 5,
    }

    empty = CanFeedbackTracker(timeout_ns=300_000_000)
    assert empty.ages_ns(NOW_NS) == {
        "mcu_command": None,
        "mcu_stat1": None,
        "mcu_stat2": None,
        "mcu_error": None,
        "eps_status1": None,
    }
