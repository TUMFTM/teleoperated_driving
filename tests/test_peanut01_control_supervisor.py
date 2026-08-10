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
        tod_gear=0,
        vehicle_velocity_mps=0.0,
        vehicle_autonomous=False,
        emergency=False,
        local_override=False,
        values_valid=True,
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
