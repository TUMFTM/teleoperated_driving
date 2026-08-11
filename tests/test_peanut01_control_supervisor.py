import dataclasses
import pathlib
import runpy


ROOT = pathlib.Path(__file__).resolve().parents[1]
SUPERVISOR = ROOT / "work/peanut01_control_supervisor.py"
MODULE = runpy.run_path(str(SUPERVISOR))

Decision = MODULE["Decision"]
InputSnapshot = MODULE["InputSnapshot"]
Parameters = MODULE["Parameters"]
State = MODULE["State"]
Supervisor = MODULE["Supervisor"]


def ready_snapshot(now_ns=1_000_000_000):
    return InputSnapshot(
        now_ns=now_ns,
        primary_stamp_ns=now_ns,
        secondary_stamp_ns=now_ns,
        status_stamp_ns=now_ns,
        feedback_stamp_ns=now_ns,
        tod_teleoperation=True,
        requested_velocity_mps=0.0,
        tod_gear=2,
        vehicle_velocity_mps=0.0,
        vehicle_autonomous=False,
        emergency=False,
        emergency_released=True,
        local_override=False,
        values_valid=True,
        lateral_approved=True,
        longitudinal_approved=True,
        mcu_power_up=True,
        mcu_enabled=False,
        mcu_direction=0,
        mcu_gear=0,
        mcu_brake_locked=True,
    )


def make_active_supervisor():
    supervisor = Supervisor(Parameters())
    supervisor.request_enable(True)
    supervisor.step(ready_snapshot())
    decision = supervisor.step(ready_snapshot(2_000_000_000))
    assert decision.request_autonomous
    decision = supervisor.on_mode_response(True)
    assert decision.state is State.ACTIVE
    return supervisor


def make_faulted_supervisor():
    supervisor = make_active_supervisor()
    snapshot = dataclasses.replace(ready_snapshot(2_100_000_000), emergency=True)
    assert supervisor.step(snapshot).state is State.FAULT
    return supervisor


def test_starts_disabled_even_when_configuration_requested_true():
    supervisor = Supervisor(Parameters(), configured_enable=True)

    assert supervisor.state is State.DISABLED


def test_requires_one_second_of_continuous_readiness():
    supervisor = Supervisor(Parameters())

    assert supervisor.request_enable(True).state is State.ARMING
    assert not supervisor.step(ready_snapshot()).request_autonomous
    almost_ready = supervisor.step(ready_snapshot(1_999_999_999))
    assert almost_ready.state is State.ARMING
    assert not almost_ready.request_autonomous
    ready = supervisor.step(ready_snapshot(2_000_000_000))
    assert ready.request_autonomous
    assert not supervisor.step(ready_snapshot(2_100_000_000)).request_autonomous
    assert supervisor.on_mode_response(True).state is State.ACTIVE


def test_arming_readiness_interruption_restarts_timer():
    supervisor = Supervisor(Parameters())
    supervisor.request_enable(True)
    supervisor.step(ready_snapshot())

    moving = dataclasses.replace(
        ready_snapshot(1_500_000_000), vehicle_velocity_mps=0.03
    )
    assert not supervisor.step(moving).request_autonomous
    assert not supervisor.step(ready_snapshot(2_000_000_000)).request_autonomous
    assert supervisor.step(ready_snapshot(3_000_000_000)).request_autonomous


def test_stale_command_feedback_emergency_and_local_override_latch_fault():
    mutations = {
        "stale command": {"primary_stamp_ns": 1_700_000_000},
        "stale feedback": {"feedback_stamp_ns": 1_700_000_000},
        "emergency": {"emergency": True},
        "override": {"local_override": True},
        "invalid values": {"values_valid": False},
        "mode exit": {"vehicle_autonomous": False},
    }
    for name, changes in mutations.items():
        supervisor = make_active_supervisor()
        base = ready_snapshot(2_100_000_000)
        if name != "mode exit":
            base = dataclasses.replace(base, vehicle_autonomous=True)
        snapshot = dataclasses.replace(base, **changes)

        decision = supervisor.step(snapshot)

        assert decision.state is State.FAULT, name
        assert decision.publish_stop, name


def test_fault_requires_disable_before_rearming():
    supervisor = make_faulted_supervisor()

    assert supervisor.request_enable(True).state is State.FAULT
    disabled = supervisor.request_enable(False)
    assert disabled.state is State.DISABLED
    assert disabled.publish_stop
    assert disabled.request_manual
    assert supervisor.request_enable(True).state is State.ARMING


def test_rejected_mode_response_latches_fault():
    supervisor = Supervisor(Parameters())
    supervisor.request_enable(True)
    supervisor.step(ready_snapshot())
    supervisor.step(ready_snapshot(2_000_000_000))

    decision = supervisor.on_mode_response(False)

    assert decision.state is State.FAULT
    assert decision.publish_stop
    assert decision.reason == "autonomous mode request rejected"


def test_disabling_active_requests_stop_and_manual_mode():
    supervisor = make_active_supervisor()

    decision = supervisor.request_enable(False)

    assert decision == Decision(
        state=State.DISABLED,
        publish_commands=False,
        publish_stop=True,
        request_autonomous=False,
        request_manual=True,
        reason="disabled by operator",
    )


def test_arming_requires_neutral_zero_motion_safety_approvals_and_safe_mcu_state():
    mutations = {
        "not neutral": {"tod_gear": 3},
        "requested motion": {"requested_velocity_mps": 0.05},
        "vehicle moving": {"vehicle_velocity_mps": 0.03},
        "emergency not released": {"emergency_released": False},
        "lateral not approved": {"lateral_approved": False},
        "longitudinal not approved": {"longitudinal_approved": False},
        "MCU power down": {"mcu_power_up": False},
        "MCU enabled": {"mcu_enabled": True},
        "brake unlocked": {"mcu_brake_locked": False},
        "MCU not neutral": {"mcu_gear": 1},
        "F710 override": {"local_override": True},
    }
    for name, changes in mutations.items():
        supervisor = Supervisor(Parameters())
        supervisor.request_enable(True)

        decision = supervisor.step(dataclasses.replace(ready_snapshot(), **changes))

        assert decision.state is State.ARMING, name
        assert not decision.request_autonomous, name
        assert not supervisor.step(ready_snapshot(2_000_000_000)).request_autonomous
        assert supervisor.step(ready_snapshot(3_000_000_000)).request_autonomous


def active_snapshot(now_ns=2_100_000_000, **changes):
    base = dataclasses.replace(ready_snapshot(now_ns), vehicle_autonomous=True)
    return dataclasses.replace(base, **changes)


def test_drive_execution_can_confirm_within_500_ms():
    supervisor = make_active_supervisor()
    requested = active_snapshot(requested_velocity_mps=0.05, tod_gear=3)

    pending = supervisor.step(requested)
    confirmed = supervisor.step(
        dataclasses.replace(
            requested,
            now_ns=requested.now_ns + 400_000_000,
            primary_stamp_ns=requested.now_ns + 400_000_000,
            secondary_stamp_ns=requested.now_ns + 400_000_000,
            status_stamp_ns=requested.now_ns + 400_000_000,
            feedback_stamp_ns=requested.now_ns + 400_000_000,
            mcu_enabled=True,
            mcu_direction=1,
            mcu_gear=1,
            mcu_brake_locked=False,
        )
    )

    assert pending.state is State.ACTIVE
    assert pending.publish_commands
    assert confirmed.state is State.ACTIVE
    assert confirmed.publish_commands


def test_reverse_execution_requires_reverse_direction_and_gear():
    supervisor = make_active_supervisor()
    requested = active_snapshot(requested_velocity_mps=-0.05, tod_gear=1)
    supervisor.step(requested)

    confirmed = supervisor.step(
        dataclasses.replace(
            requested,
            now_ns=requested.now_ns + 400_000_000,
            primary_stamp_ns=requested.now_ns + 400_000_000,
            secondary_stamp_ns=requested.now_ns + 400_000_000,
            status_stamp_ns=requested.now_ns + 400_000_000,
            feedback_stamp_ns=requested.now_ns + 400_000_000,
            mcu_enabled=True,
            mcu_direction=2,
            mcu_gear=2,
            mcu_brake_locked=False,
        )
    )

    assert confirmed.state is State.ACTIVE
    assert confirmed.publish_commands


def fresh_late_snapshot(snapshot, elapsed_ns=500_000_001, **changes):
    now_ns = snapshot.now_ns + elapsed_ns
    return dataclasses.replace(
        snapshot,
        now_ns=now_ns,
        primary_stamp_ns=now_ns,
        secondary_stamp_ns=now_ns,
        status_stamp_ns=now_ns,
        feedback_stamp_ns=now_ns,
        **changes,
    )


def test_drive_execution_mismatches_fault_after_500_ms():
    mismatches = {
        "enable": {},
        "brake": {"mcu_enabled": True},
        "direction": {"mcu_enabled": True, "mcu_brake_locked": False},
        "gear": {
            "mcu_enabled": True,
            "mcu_brake_locked": False,
            "mcu_direction": 1,
        },
    }
    for name, changes in mismatches.items():
        supervisor = make_active_supervisor()
        requested = active_snapshot(requested_velocity_mps=0.05, tod_gear=3)
        supervisor.step(requested)

        decision = supervisor.step(fresh_late_snapshot(requested, **changes))

        assert decision.state is State.FAULT, name
        assert decision.publish_stop, name
        assert "execution feedback mismatch" in decision.reason, name


def test_neutral_and_zero_speed_execution_require_disabled_locked_state():
    supervisor = make_active_supervisor()
    drive = active_snapshot(
        requested_velocity_mps=0.05,
        tod_gear=3,
        mcu_enabled=True,
        mcu_direction=1,
        mcu_gear=1,
        mcu_brake_locked=False,
    )
    assert supervisor.step(drive).state is State.ACTIVE
    neutral = dataclasses.replace(
        drive, now_ns=drive.now_ns + 100_000_000, requested_velocity_mps=0.0, tod_gear=2
    )
    assert supervisor.step(neutral).state is State.ACTIVE

    confirmed = fresh_late_snapshot(
        neutral,
        elapsed_ns=400_000_000,
        mcu_enabled=False,
        mcu_direction=0,
        mcu_gear=0,
        mcu_brake_locked=True,
    )

    assert supervisor.step(confirmed).state is State.ACTIVE


def test_neutral_execution_mismatch_faults_after_timeout():
    supervisor = make_active_supervisor()
    neutral = active_snapshot(mcu_enabled=True, mcu_gear=1, mcu_brake_locked=False)
    supervisor.step(neutral)

    decision = supervisor.step(fresh_late_snapshot(neutral))

    assert decision.state is State.FAULT
    assert decision.publish_stop


def test_approval_loss_and_f710_override_fault_immediately_without_auto_recovery():
    mutations = (
        {"lateral_approved": False},
        {"longitudinal_approved": False},
        {"emergency_released": False},
        {"local_override": True},
    )
    for changes in mutations:
        supervisor = make_active_supervisor()

        decision = supervisor.step(active_snapshot(**changes))

        assert decision.state is State.FAULT
        assert decision.publish_stop
        released = supervisor.step(active_snapshot(now_ns=2_200_000_000))
        assert released.state is State.FAULT
        assert not released.publish_commands
