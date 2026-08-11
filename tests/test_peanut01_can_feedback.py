import json
import pathlib
import runpy

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
CAN_FEEDBACK = ROOT / "work/peanut01_can_feedback.py"
MODULE = runpy.run_path(str(CAN_FEEDBACK))

CanFeedbackTracker = MODULE["CanFeedbackTracker"]

NOW_NS = 1_000_000_000
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
