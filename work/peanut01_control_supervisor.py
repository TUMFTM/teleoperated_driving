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
    local_override: bool
    values_valid: bool


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
    if snapshot.local_override:
        return "vehicle-local F710 override is active"
    if require_autonomous and not snapshot.vehicle_autonomous:
        return "vehicle left autonomous control mode"
    return ""


def input_ready(snapshot, params):
    if _common_fault(snapshot, params, require_autonomous=False):
        return False
    return (
        abs(snapshot.vehicle_velocity_mps) <= params.stopped_velocity_mps
        and snapshot.tod_gear in (0, 2)
        and abs(snapshot.requested_velocity_mps) <= params.stopped_velocity_mps
    )


class Supervisor:
    def __init__(self, params, configured_enable=False):
        del configured_enable
        self.params = params
        self.state = State.DISABLED
        self._arming_started_ns = None
        self._mode_request_pending = False

    def _decision(self, **changes):
        return Decision(state=self.state, **changes)

    def request_enable(self, enabled):
        if enabled:
            if self.state is State.DISABLED:
                self.state = State.ARMING
                self._arming_started_ns = None
                self._mode_request_pending = False
                return self._decision(reason="waiting for arming conditions")
            return self._decision(reason="enable request does not change state")

        previous = self.state
        self.state = State.DISABLED
        self._arming_started_ns = None
        self._mode_request_pending = False
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
        return self._decision(publish_commands=True, reason="actuation active")

    def on_mode_response(self, success):
        if self.state is not State.ARMING or not self._mode_request_pending:
            return self._decision(reason="ignored unexpected mode response")
        self._mode_request_pending = False
        if success:
            self.state = State.ACTIVE
            return self._decision(reason="autonomous mode accepted")
        self.state = State.FAULT
        return self._decision(
            publish_stop=True,
            reason="autonomous mode request rejected",
        )
