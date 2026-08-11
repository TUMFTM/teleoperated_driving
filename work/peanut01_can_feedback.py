import dataclasses
import json


RECOGNIZED_CAN_IDS = (0x1A1, 0x1A2, 0x1A3, 0x1A4, 0x401)
ENVELOPE_FIELDS = {
    "stamp_ns",
    "interface",
    "id",
    "id_hex",
    "is_extended",
    "is_rtr",
    "is_error",
    "dlc",
    "data_hex",
}


@dataclasses.dataclass(frozen=True)
class McuCommand:
    enabled: bool = False
    mode: int = 0
    gear: int = 0
    brake_mode: int = 0
    motor_rpm: int = 0
    stamp_ns: int = 0


@dataclasses.dataclass(frozen=True)
class McuStatus:
    power_up: bool = False
    enabled: bool = False
    direction: int = 0
    gear: int = 0
    brake_locked: bool = True
    motor_angle_deg: float = 0.0
    motor_temperature_c: float = 0.0
    voltage_v: float = 0.0
    current_a: float = 0.0
    motor_rpm: int = 0
    error_codes: tuple = ()
    error_count: int = 0
    manual_override: bool = False
    stat1_stamp_ns: int = 0
    stat2_stamp_ns: int = 0
    error_stamp_ns: int = 0


@dataclasses.dataclass(frozen=True)
class EpsStatus:
    mode: int = 0
    init_status: int = 0
    error_1: int = 0
    error_2: int = 0
    angle_deg: float = 0.0
    status_stamp_ns: int = 0


@dataclasses.dataclass(frozen=True)
class FeedbackSnapshot:
    mcu: McuStatus
    eps: EpsStatus
    command: McuCommand
    software_neutral: bool = False
    emergency_released: bool = False
    lateral_approved: bool = False
    longitudinal_approved: bool = False
    last_reject_reason: str = ""


def mcu_checksum(data):
    return (sum(data[:7]) & 0xFF) ^ 0xFF


def eps_xor(data):
    value = 0
    for byte in data[:7]:
        value ^= byte
    return value


class CanFeedbackTracker:
    def __init__(self, timeout_ns):
        self.timeout_ns = timeout_ns
        self._command = McuCommand()
        self._mcu = McuStatus()
        self._eps = EpsStatus()
        self._last_reject_reason = ""

    def _fresh(self, now_ns, stamp_ns):
        age_ns = now_ns - stamp_ns
        return stamp_ns > 0 and 0 <= age_ns <= self.timeout_ns

    def ages_ns(self, now_ns):
        stamps = {
            "mcu_command": self._command.stamp_ns,
            "mcu_stat1": self._mcu.stat1_stamp_ns,
            "mcu_stat2": self._mcu.stat2_stamp_ns,
            "mcu_error": self._mcu.error_stamp_ns,
            "eps_status1": self._eps.status_stamp_ns,
        }
        return {
            name: (now_ns - stamp if stamp > 0 and now_ns >= stamp else None)
            for name, stamp in stamps.items()
        }

    def _reject(self, reason):
        self._last_reject_reason = reason
        return False

    def _parse_envelope(self, payload):
        try:
            value = json.loads(payload)
        except (TypeError, ValueError, json.JSONDecodeError):
            return None, "malformed JSON"
        if type(value) is not dict or set(value) != ENVELOPE_FIELDS:
            return None, "invalid JSON fields"
        expected_types = {
            "stamp_ns": int,
            "interface": str,
            "id": int,
            "id_hex": str,
            "is_extended": bool,
            "is_rtr": bool,
            "is_error": bool,
            "dlc": int,
            "data_hex": str,
        }
        if any(type(value[name]) is not kind for name, kind in expected_types.items()):
            return None, "invalid JSON field type"
        try:
            id_from_hex = int(value["id_hex"], 16)
        except ValueError:
            return None, "invalid CAN ID hex"
        if id_from_hex != value["id"]:
            return None, "inconsistent CAN ID"
        if value["id"] not in RECOGNIZED_CAN_IDS:
            return None, "unrecognized CAN ID"
        if value["is_extended"] or value["is_rtr"] or value["is_error"]:
            return None, "unsupported CAN frame flags"
        if value["dlc"] != 8 or len(value["data_hex"]) != 16:
            return None, "feedback DLC must be 8"
        try:
            data = bytes.fromhex(value["data_hex"])
        except ValueError:
            return None, "invalid CAN payload hex"
        if len(data) != 8:
            return None, "feedback payload must be 8 bytes"
        return (value["id"], data), ""

    def update_json(self, payload, receipt_ns):
        parsed, reason = self._parse_envelope(payload)
        if parsed is None:
            return self._reject(reason)
        can_id, data = parsed
        if can_id in (0x1A1, 0x1A2, 0x1A3, 0x1A4) and mcu_checksum(data) != data[7]:
            return self._reject("MCU checksum mismatch")
        if can_id == 0x401 and eps_xor(data) != data[7]:
            return self._reject("EPS XOR mismatch")

        if can_id == 0x1A1:
            self._command = McuCommand(
                enabled=bool(data[0] & 0x80),
                mode=(data[0] >> 5) & 0x03,
                gear=(data[0] >> 2) & 0x03,
                brake_mode=data[0] & 0x03,
                motor_rpm=int.from_bytes(data[3:5], "big"),
                stamp_ns=receipt_ns,
            )
        elif can_id == 0x1A2:
            self._mcu = dataclasses.replace(
                self._mcu,
                power_up=bool(data[0] & 0x80),
                enabled=bool(data[0] & 0x40),
                direction=(data[0] >> 4) & 0x03,
                gear=(data[0] >> 2) & 0x03,
                brake_locked=bool(data[0] & 0x01),
                motor_angle_deg=int.from_bytes(data[1:3], "big")
                * 0.0054932478828107,
                motor_temperature_c=int.from_bytes(data[5:7], "big") * 0.1
                - 100.0,
                stat1_stamp_ns=receipt_ns,
            )
        elif can_id == 0x1A3:
            self._mcu = dataclasses.replace(
                self._mcu,
                voltage_v=int.from_bytes(data[0:2], "big") * 0.1,
                current_a=int.from_bytes(data[2:4], "big") * 0.1,
                motor_rpm=int.from_bytes(data[4:6], "big"),
                stat2_stamp_ns=receipt_ns,
            )
        elif can_id == 0x1A4:
            self._mcu = dataclasses.replace(
                self._mcu,
                error_codes=tuple(data[0:5]),
                manual_override=data[5] == 1,
                error_count=data[6],
                error_stamp_ns=receipt_ns,
            )
        else:
            self._eps = dataclasses.replace(
                self._eps,
                mode=data[0],
                error_1=data[2],
                angle_deg=int.from_bytes(data[3:5], "big") * 0.1 - 1024.0,
                init_status=data[5],
                error_2=data[6],
                status_stamp_ns=receipt_ns,
            )
        self._last_reject_reason = ""
        return True

    def snapshot(self, now_ns, emergency_fresh=False, emergency=True):
        command_fresh = self._fresh(now_ns, self._command.stamp_ns)
        mcu_stat1_fresh = self._fresh(now_ns, self._mcu.stat1_stamp_ns)
        mcu_stat2_fresh = self._fresh(now_ns, self._mcu.stat2_stamp_ns)
        mcu_error_fresh = self._fresh(now_ns, self._mcu.error_stamp_ns)
        eps_fresh = self._fresh(now_ns, self._eps.status_stamp_ns)
        emergency_released = emergency_fresh and not emergency
        lateral_approved = (
            eps_fresh
            and self._eps.mode in (0x20, 0x23)
            and self._eps.init_status in (0x55, 0xEE)
            and self._eps.error_1 == 0
            and self._eps.error_2 == 0
        )
        longitudinal_approved = (
            mcu_stat1_fresh
            and mcu_stat2_fresh
            and mcu_error_fresh
            and self._mcu.power_up
            and not any(self._mcu.error_codes)
            and not self._mcu.manual_override
        )
        software_neutral = command_fresh and (
            not self._command.enabled
            and self._command.mode == 1
            and self._command.gear == 2
            and self._command.brake_mode == 1
            and self._command.motor_rpm == 0
        )
        return FeedbackSnapshot(
            mcu=self._mcu,
            eps=self._eps,
            command=self._command,
            software_neutral=software_neutral,
            emergency_released=emergency_released,
            lateral_approved=lateral_approved,
            longitudinal_approved=longitudinal_approved,
            last_reject_reason=self._last_reject_reason,
        )
