import dataclasses
import enum
import math


class State(enum.Enum):
    DISABLED = "DISABLED"
    ARMING = "ARMING"
    ACTIVE = "ACTIVE"
    FAULT = "FAULT"


@dataclasses.dataclass(frozen=True)
class Parameters:
    command_timeout_ns: int = 300_000_000
    feedback_timeout_ns: int = 300_000_000
    arming_duration_ns: int = 1_000_000_000
    execution_confirmation_timeout_ns: int = 500_000_000
    stopped_velocity_mps: float = 0.02


@dataclasses.dataclass(frozen=True)
class InputSnapshot:
    now_ns: int
    primary_stamp_ns: int
    secondary_stamp_ns: int
    status_stamp_ns: int
    feedback_stamp_ns: int
    tod_teleoperation: bool
    requested_velocity_mps: float
    tod_gear: int
    vehicle_velocity_mps: float
    vehicle_autonomous: bool
    emergency: bool
    emergency_released: bool
    local_override: bool
    values_valid: bool
    lateral_approved: bool
    longitudinal_approved: bool
    mcu_power_up: bool
    mcu_enabled: bool
    mcu_direction: int
    mcu_gear: int
    mcu_brake_locked: bool
    software_neutral: bool


@dataclasses.dataclass(frozen=True)
class Decision:
    state: State
    publish_commands: bool = False
    publish_stop: bool = False
    request_autonomous: bool = False
    request_manual: bool = False
    reason: str = ""


def _fresh(now_ns, stamp_ns, timeout_ns):
    age_ns = now_ns - stamp_ns
    return 0 <= age_ns <= timeout_ns


def _common_fault(snapshot, params, require_autonomous):
    command_stamps = (
        snapshot.primary_stamp_ns,
        snapshot.secondary_stamp_ns,
        snapshot.status_stamp_ns,
    )
    if not all(
        _fresh(snapshot.now_ns, stamp, params.command_timeout_ns)
        for stamp in command_stamps
    ):
        return "stale TOD command or status"
    if not _fresh(
        snapshot.now_ns, snapshot.feedback_stamp_ns, params.feedback_timeout_ns
    ):
        return "stale vehicle feedback"
    if not snapshot.tod_teleoperation:
        return "TOD is not in teleoperation"
    if not snapshot.values_valid or not all(
        math.isfinite(value)
        for value in (
            snapshot.requested_velocity_mps,
            snapshot.vehicle_velocity_mps,
        )
    ):
        return "invalid command or feedback value"
    if snapshot.emergency:
        return "vehicle emergency stop is active"
    if not snapshot.emergency_released:
        return "vehicle emergency stop release is unavailable"
    if snapshot.local_override:
        return "vehicle-local F710 override is active"
    if not snapshot.lateral_approved:
        return "lateral actuation is not approved"
    if not snapshot.longitudinal_approved:
        return "longitudinal actuation is not approved"
    if require_autonomous and not snapshot.vehicle_autonomous:
        return "vehicle left autonomous control mode"
    return ""


def input_ready(snapshot, params):
    if _common_fault(snapshot, params, require_autonomous=False):
        return False
    return (
        abs(snapshot.vehicle_velocity_mps) <= params.stopped_velocity_mps
        and snapshot.tod_gear == 2
        and abs(snapshot.requested_velocity_mps) <= params.stopped_velocity_mps
        and snapshot.mcu_power_up
        and not snapshot.mcu_enabled
        and snapshot.mcu_brake_locked
        and snapshot.software_neutral
    )


class Supervisor:
    def __init__(self, params, configured_enable=False):
        del configured_enable
        self.params = params
        self.state = State.DISABLED
        self._arming_started_ns = None
        self._mode_request_pending = False
        self._execution_expected = None
        self._execution_started_ns = None

    def _decision(self, **changes):
        return Decision(state=self.state, **changes)

    def request_enable(self, enabled):
        if enabled:
            if self.state is State.DISABLED:
                self.state = State.ARMING
                self._arming_started_ns = None
                self._mode_request_pending = False
                self._execution_expected = None
                self._execution_started_ns = None
                return self._decision(reason="waiting for arming conditions")
            return self._decision(reason="enable request does not change state")

        previous = self.state
        self.state = State.DISABLED
        self._arming_started_ns = None
        self._mode_request_pending = False
        self._execution_expected = None
        self._execution_started_ns = None
        needs_shutdown = previous in (State.ACTIVE, State.FAULT)
        return self._decision(
            publish_stop=needs_shutdown,
            request_manual=needs_shutdown,
            reason="disabled by operator",
        )

    def step(self, snapshot):
        if self.state is State.DISABLED:
            return self._decision(reason="actuation disabled")
        if self.state is State.FAULT:
            return self._decision(
                publish_stop=True,
                reason="fault latched; disable actuation before rearming",
            )
        if self.state is State.ARMING:
            if not input_ready(snapshot, self.params):
                self._arming_started_ns = None
                self._mode_request_pending = False
                return self._decision(reason="arming conditions not satisfied")
            if self._arming_started_ns is None:
                self._arming_started_ns = snapshot.now_ns
            elapsed_ns = snapshot.now_ns - self._arming_started_ns
            if (
                elapsed_ns >= self.params.arming_duration_ns
                and not self._mode_request_pending
            ):
                self._mode_request_pending = True
                return self._decision(
                    request_autonomous=True,
                    reason="arming conditions stable",
                )
            return self._decision(reason="arming conditions stabilizing")

        fault = _common_fault(snapshot, self.params, require_autonomous=True)
        if fault:
            self.state = State.FAULT
            return self._decision(publish_stop=True, reason=fault)
        expected = self._expected_execution(snapshot)
        if expected != self._execution_expected:
            self._execution_expected = expected
            self._execution_started_ns = snapshot.now_ns
        if not self._execution_matches(snapshot):
            elapsed_ns = snapshot.now_ns - self._execution_started_ns
            if elapsed_ns > self.params.execution_confirmation_timeout_ns:
                self.state = State.FAULT
                return self._decision(
                    publish_stop=True,
                    reason=f"execution feedback mismatch: {expected}",
                )
        return self._decision(publish_commands=True, reason="actuation active")

    def _expected_execution(self, snapshot):
        moving = (
            abs(snapshot.requested_velocity_mps) > self.params.stopped_velocity_mps
        )
        if moving and snapshot.tod_gear == 3:
            return "drive"
        if moving and snapshot.tod_gear == 1:
            return "reverse"
        return "stopped_neutral" if snapshot.tod_gear == 2 else "stopped"

    def _execution_matches(self, snapshot):
        if self._execution_expected == "drive":
            return (
                snapshot.mcu_enabled
                and not snapshot.mcu_brake_locked
                and snapshot.mcu_direction == 2
                and snapshot.mcu_gear == 1
            )
        if self._execution_expected == "reverse":
            return (
                snapshot.mcu_enabled
                and not snapshot.mcu_brake_locked
                and snapshot.mcu_direction == 1
                and snapshot.mcu_gear == 2
            )
        matches = not snapshot.mcu_enabled and snapshot.mcu_brake_locked
        if self._execution_expected == "stopped_neutral":
            matches = matches and snapshot.software_neutral
        return matches

    @property
    def execution_expected(self):
        return self._execution_expected or "none"

    def execution_remaining_ns(self, now_ns):
        if self._execution_started_ns is None:
            return None
        return max(
            0,
            self.params.execution_confirmation_timeout_ns
            - (now_ns - self._execution_started_ns),
        )

    def on_mode_response(self, success):
        if self.state is not State.ARMING or not self._mode_request_pending:
            return self._decision(reason="ignored unexpected mode response")
        self._mode_request_pending = False
        if success:
            self.state = State.ACTIVE
            self._execution_expected = None
            self._execution_started_ns = None
            return self._decision(reason="autonomous mode accepted")
        self.state = State.FAULT
        return self._decision(
            publish_stop=True,
            reason="autonomous mode request rejected",
        )
