#!/usr/bin/env python3

import os
import threading
import time

import rclpy
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import (
    ControlModeReport,
    GearCommand,
    HazardLightsCommand,
    SteeringReport,
    TurnIndicatorsCommand,
    VelocityReport,
)
from autoware_vehicle_msgs.srv import ControlModeCommand
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from peanut01_control_mapping import ConvertedCommand, convert_command
from peanut01_control_supervisor import InputSnapshot, Parameters, State, Supervisor
from rcl_interfaces.msg import SetParametersResult
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from tod_status_msgs.msg import Status
from tod_vehicle_msgs.msg import PrimaryControlCmd, SecondaryControlCmd


PRIMARY_TOPIC = "/vehicle/safety/output/primary_control_cmd"
SECONDARY_TOPIC = "/vehicle/safety/output/secondary_control_cmd"
STATUS_TOPIC = "/vehicle/statemachine/output/vehicle_status"

VELOCITY_TOPIC = "/vehicle/status/velocity_status"
STEERING_TOPIC = "/vehicle/status/steering_status"
CONTROL_MODE_TOPIC = "/vehicle/status/control_mode"
EMERGENCY_TOPIC = "/minguo/emergency_stop"
OVERRIDE_TOPIC = "/minguo/teleop_override"

DEBUG_CONTROL_TOPIC = "/debug/tod_peanut01/autoware_control_cmd"
DEBUG_GEAR_TOPIC = "/debug/tod_peanut01/autoware_gear_cmd"
DEBUG_TURN_TOPIC = "/debug/tod_peanut01/autoware_turn_indicators_cmd"
DEBUG_HAZARD_TOPIC = "/debug/tod_peanut01/autoware_hazard_lights_cmd"
DIAGNOSTICS_TOPIC = "/debug/tod_peanut01/bridge_diagnostics"

REAL_CONTROL_TOPIC = "/control/command/control_cmd"
REAL_GEAR_TOPIC = "/control/command/gear_cmd"
REAL_TURN_TOPIC = "/control/command/turn_indicators_cmd"
REAL_HAZARD_TOPIC = "/control/command/hazard_lights_cmd"
CONTROL_MODE_SERVICE = "/control/control_mode_request"


def receipt_time_ns():
    return time.monotonic_ns()


class SharedInputs:
    def __init__(self):
        self.lock = threading.Lock()
        self.primary = None
        self.secondary = None
        self.status = None
        self.primary_stamp_ns = 0
        self.secondary_stamp_ns = 0
        self.status_stamp_ns = 0
        self.vehicle_velocity_mps = float("nan")
        self.steering_tire_angle_rad = float("nan")
        self.control_mode = ControlModeReport.NOT_READY
        self.emergency = True
        self.local_override = True
        self.feedback_stamps = {
            "velocity": 0,
            "steering": 0,
            "control_mode": 0,
            "emergency": 0,
            "override": 0,
        }
        self.requested_enable = False

    def feedback_stamp_ns(self):
        return min(self.feedback_stamps.values())


class Peanut01ControlBridge:
    def __init__(self, source_node, target_node, shared, config):
        self.source_node = source_node
        self.target_node = target_node
        self.shared = shared
        self.steering_ratio = config["steering_ratio"]
        self.supervisor = Supervisor(
            Parameters(
                command_timeout_ns=config["command_timeout_ms"] * 1_000_000,
                feedback_timeout_ns=config["feedback_timeout_ms"] * 1_000_000,
                arming_duration_ns=config["arming_duration_ms"] * 1_000_000,
                stopped_velocity_mps=config["stopped_velocity_mps"],
            ),
            configured_enable=config["configured_enable"],
        )
        self._applied_enable = False
        self._real_publishers = None
        self._mode_future = None
        self._deactivation_stop_cycles = 0
        self._hold_current_steering_once = False
        self._last_reason = "actuation disabled"

        self.debug_publishers = {
            "control": target_node.create_publisher(Control, DEBUG_CONTROL_TOPIC, 10),
            "gear": target_node.create_publisher(GearCommand, DEBUG_GEAR_TOPIC, 10),
            "turn": target_node.create_publisher(
                TurnIndicatorsCommand, DEBUG_TURN_TOPIC, 10
            ),
            "hazard": target_node.create_publisher(
                HazardLightsCommand, DEBUG_HAZARD_TOPIC, 10
            ),
        }
        self.diagnostics_publisher = target_node.create_publisher(
            DiagnosticArray, DIAGNOSTICS_TOPIC, 10
        )
        self.control_mode_client = target_node.create_client(
            ControlModeCommand, CONTROL_MODE_SERVICE
        )
        self.subscriptions = []
        self._create_subscriptions()
        self.timer = target_node.create_timer(
            1.0 / config["publish_rate_hz"], self.on_timer
        )

    def _create_subscriptions(self):
        self.subscriptions.extend(
            (
                self.source_node.create_subscription(
                    PrimaryControlCmd, PRIMARY_TOPIC, self.on_primary, 10
                ),
                self.source_node.create_subscription(
                    SecondaryControlCmd, SECONDARY_TOPIC, self.on_secondary, 10
                ),
                self.source_node.create_subscription(
                    Status, STATUS_TOPIC, self.on_status, 10
                ),
                self.target_node.create_subscription(
                    VelocityReport, VELOCITY_TOPIC, self.on_velocity, 10
                ),
                self.target_node.create_subscription(
                    SteeringReport, STEERING_TOPIC, self.on_steering, 10
                ),
                self.target_node.create_subscription(
                    ControlModeReport,
                    CONTROL_MODE_TOPIC,
                    self.on_control_mode,
                    10,
                ),
                self.target_node.create_subscription(
                    Bool, EMERGENCY_TOPIC, self.on_emergency, 10
                ),
                self.target_node.create_subscription(
                    Bool, OVERRIDE_TOPIC, self.on_override, 10
                ),
            )
        )

    def on_primary(self, message):
        with self.shared.lock:
            self.shared.primary = message
            self.shared.primary_stamp_ns = receipt_time_ns()

    def on_secondary(self, message):
        with self.shared.lock:
            self.shared.secondary = message
            self.shared.secondary_stamp_ns = receipt_time_ns()

    def on_status(self, message):
        with self.shared.lock:
            self.shared.status = message
            self.shared.status_stamp_ns = receipt_time_ns()

    def on_velocity(self, message):
        with self.shared.lock:
            self.shared.vehicle_velocity_mps = message.longitudinal_velocity
            self.shared.feedback_stamps["velocity"] = receipt_time_ns()

    def on_steering(self, message):
        with self.shared.lock:
            self.shared.steering_tire_angle_rad = message.steering_tire_angle
            self.shared.feedback_stamps["steering"] = receipt_time_ns()

    def on_control_mode(self, message):
        with self.shared.lock:
            self.shared.control_mode = message.mode
            self.shared.feedback_stamps["control_mode"] = receipt_time_ns()

    def on_emergency(self, message):
        with self.shared.lock:
            self.shared.emergency = message.data
            self.shared.feedback_stamps["emergency"] = receipt_time_ns()

    def on_override(self, message):
        with self.shared.lock:
            self.shared.local_override = message.data
            self.shared.feedback_stamps["override"] = receipt_time_ns()

    def create_real_publishers(self):
        if self._real_publishers is not None:
            return
        self._real_publishers = {
            "control": self.target_node.create_publisher(Control, REAL_CONTROL_TOPIC, 10),
            "gear": self.target_node.create_publisher(GearCommand, REAL_GEAR_TOPIC, 10),
            "turn": self.target_node.create_publisher(
                TurnIndicatorsCommand, REAL_TURN_TOPIC, 10
            ),
            "hazard": self.target_node.create_publisher(
                HazardLightsCommand, REAL_HAZARD_TOPIC, 10
            ),
        }
        self.target_node.get_logger().warn("Real Autoware command publishers created")

    def destroy_real_publishers(self):
        if self._real_publishers is None:
            return
        for publisher in self._real_publishers.values():
            self.target_node.destroy_publisher(publisher)
        self._real_publishers = None
        self.target_node.get_logger().info("Real Autoware command publishers destroyed")

    def _copy_inputs(self, now_ns):
        with self.shared.lock:
            primary = self.shared.primary
            secondary = self.shared.secondary
            status = self.shared.status
            steering = self.shared.steering_tire_angle_rad
            snapshot_values = {
                "now_ns": now_ns,
                "primary_stamp_ns": self.shared.primary_stamp_ns,
                "secondary_stamp_ns": self.shared.secondary_stamp_ns,
                "status_stamp_ns": self.shared.status_stamp_ns,
                "feedback_stamp_ns": self.shared.feedback_stamp_ns(),
                "tod_teleoperation": bool(
                    status is not None
                    and status.tod_status == Status.TOD_STATUS_TELEOPERATION
                ),
                "requested_velocity_mps": (
                    primary.velocity if primary is not None else float("nan")
                ),
                "tod_gear": (
                    secondary.gear_position if secondary is not None else -1
                ),
                "vehicle_velocity_mps": self.shared.vehicle_velocity_mps,
                "vehicle_autonomous": (
                    self.shared.control_mode == ControlModeReport.AUTONOMOUS
                ),
                "emergency": self.shared.emergency,
                "local_override": self.shared.local_override,
            }
            requested_enable = self.shared.requested_enable
            unsupported = (
                ()
                if secondary is None
                else tuple(
                    name
                    for name in ("honk", "wiper", "head_light", "flash_light")
                    if getattr(secondary, name) != 0
                )
            )

        converted = None
        try:
            if primary is not None and secondary is not None:
                converted = convert_command(
                    primary.velocity,
                    primary.steering_wheel_angle,
                    secondary.gear_position,
                    secondary.indicator,
                    self.steering_ratio,
                )
        except (TypeError, ValueError):
            converted = None
        snapshot_values["values_valid"] = converted is not None
        return (
            InputSnapshot(**snapshot_values),
            converted,
            steering,
            requested_enable,
            unsupported,
        )

    def _control_message(self, velocity, steering_tire_angle):
        message = Control()
        stamp = self.target_node.get_clock().now().to_msg()
        message.stamp = stamp
        message.lateral.stamp = stamp
        message.lateral.steering_tire_angle = steering_tire_angle
        message.lateral.is_defined_steering_tire_rotation_rate = False
        message.longitudinal.stamp = stamp
        message.longitudinal.velocity = velocity
        message.longitudinal.is_defined_acceleration = False
        message.longitudinal.is_defined_jerk = False
        return message

    def _command_messages(self, converted, steering_override=None):
        steering = converted.steering_tire_angle_rad
        if steering_override is not None:
            steering = steering_override
        control = self._control_message(converted.velocity_mps, steering)
        stamp = control.stamp
        gear = GearCommand(stamp=stamp, command=converted.gear)
        turn = TurnIndicatorsCommand(stamp=stamp, command=converted.turn)
        hazard = HazardLightsCommand(stamp=stamp, command=converted.hazard)
        return {"control": control, "gear": gear, "turn": turn, "hazard": hazard}

    @staticmethod
    def _publish_messages(publishers, messages):
        for name, publisher in publishers.items():
            publisher.publish(messages[name])

    def _publish_debug(self, converted):
        if converted is not None:
            self._publish_messages(
                self.debug_publishers, self._command_messages(converted)
            )

    def _publish_real(self, converted, steering):
        if self._real_publishers is None:
            return
        steering_override = steering if self._hold_current_steering_once else None
        self._publish_messages(
            self._real_publishers,
            self._command_messages(converted, steering_override=steering_override),
        )
        self._hold_current_steering_once = False

    def _publish_real_stop(self, steering):
        if self._real_publishers is None:
            return
        safe_steering = steering
        if not isinstance(safe_steering, (float, int)) or not (
            float("-inf") < safe_steering < float("inf")
        ):
            safe_steering = 0.0
        self._real_publishers["control"].publish(
            self._control_message(0.0, safe_steering)
        )

    def _request_mode(self, mode, callback):
        if not self.control_mode_client.service_is_ready():
            callback(False)
            return
        request = ControlModeCommand.Request()
        request.stamp = self.target_node.get_clock().now().to_msg()
        request.mode = mode
        self._mode_future = self.control_mode_client.call_async(request)

        def on_response(future):
            try:
                callback(bool(future.result().success))
            except Exception as error:
                self.target_node.get_logger().error(
                    f"Control-mode request failed: {error}"
                )
                callback(False)

        self._mode_future.add_done_callback(on_response)

    def _request_autonomous(self):
        def on_response(success):
            decision = self.supervisor.on_mode_response(success)
            self._last_reason = decision.reason
            if decision.state is State.ACTIVE:
                self.create_real_publishers()
                self._hold_current_steering_once = True

        self._request_mode(ControlModeCommand.Request.AUTONOMOUS, on_response)

    def _request_manual_and_destroy(self):
        def on_response(success):
            if not success:
                self.target_node.get_logger().error("MANUAL mode request rejected")
            self.destroy_real_publishers()

        self._request_mode(ControlModeCommand.Request.MANUAL, on_response)

    def _start_deactivation(self):
        self._deactivation_stop_cycles = 3

    def _publish_diagnostics(self, reason, unsupported):
        message = DiagnosticArray()
        message.header.stamp = self.target_node.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "tod_peanut01_control_bridge"
        status.hardware_id = "peanut01"
        status.message = reason
        if self.supervisor.state is State.FAULT:
            status.level = DiagnosticStatus.ERROR
        elif unsupported:
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.OK
        status.values = [
            KeyValue(key="state", value=self.supervisor.state.value),
            KeyValue(key="enable_actuation", value=str(self._applied_enable)),
            KeyValue(key="unsupported_secondary", value=",".join(unsupported)),
        ]
        message.status = [status]
        self.diagnostics_publisher.publish(message)

    def on_timer(self):
        now_ns = receipt_time_ns()
        snapshot, converted, steering, requested_enable, unsupported = (
            self._copy_inputs(now_ns)
        )
        self._publish_debug(converted)

        if requested_enable != self._applied_enable:
            decision = self.supervisor.request_enable(requested_enable)
            self._applied_enable = requested_enable
            self._last_reason = decision.reason
            if decision.request_manual:
                self._start_deactivation()

        if self._deactivation_stop_cycles > 0:
            self._publish_real_stop(steering)
            self._deactivation_stop_cycles -= 1
            if self._deactivation_stop_cycles == 0:
                self._request_manual_and_destroy()
            self._publish_diagnostics(self._last_reason, unsupported)
            return

        decision = self.supervisor.step(snapshot)
        self._last_reason = decision.reason
        if decision.request_autonomous:
            self._request_autonomous()
        if decision.publish_commands and converted is not None:
            self._publish_real(converted, steering)
        if decision.publish_stop:
            self._publish_real_stop(steering)
        self._publish_diagnostics(self._last_reason, unsupported)


def declare_config(source_node):
    configured_enable = bool(
        source_node.declare_parameter("enable_actuation", False).value
    )
    config = {
        "configured_enable": configured_enable,
        "source_domain_id": int(
            source_node.declare_parameter("source_domain_id", 7).value
        ),
        "target_domain_id": int(
            source_node.declare_parameter("target_domain_id", 0).value
        ),
        "command_timeout_ms": int(
            source_node.declare_parameter("command_timeout_ms", 300).value
        ),
        "feedback_timeout_ms": int(
            source_node.declare_parameter("feedback_timeout_ms", 300).value
        ),
        "arming_duration_ms": int(
            source_node.declare_parameter("arming_duration_ms", 1000).value
        ),
        "stopped_velocity_mps": float(
            source_node.declare_parameter("stopped_velocity_mps", 0.02).value
        ),
        "steering_ratio": float(
            source_node.declare_parameter("steering_ratio", 16.0).value
        ),
        "publish_rate_hz": float(
            source_node.declare_parameter("publish_rate_hz", 20.0).value
        ),
    }
    if configured_enable:
        source_node.get_logger().error(
            "Ignoring enable_actuation=true at startup; explicit runtime arming required"
        )
        source_node.set_parameters([Parameter("enable_actuation", value=False)])
    return config


def install_parameter_callback(source_node, shared):
    def on_parameters(parameters):
        for parameter in parameters:
            if parameter.name != "enable_actuation":
                return SetParametersResult(
                    successful=False,
                    reason="restart required to change bridge safety parameters",
                )
            if parameter.type_ is not Parameter.Type.BOOL:
                return SetParametersResult(
                    successful=False, reason="enable_actuation must be boolean"
                )
        with shared.lock:
            for parameter in parameters:
                shared.requested_enable = parameter.value
        return SetParametersResult(successful=True)

    return source_node.add_on_set_parameters_callback(on_parameters)


def main():
    source_domain_id = int(os.getenv("ROS_DOMAIN_ID", "7"))
    target_domain_id = int(os.getenv("TOD_CONTROL_TARGET_DOMAIN_ID", "0"))
    source_context = Context()
    target_context = Context()
    rclpy.init(context=source_context, domain_id=source_domain_id)
    rclpy.init(context=target_context, domain_id=target_domain_id)

    source_node = Node(
        "ControlBridge",
        namespace="/vehicle/interface/peanut01",
        context=source_context,
    )
    config = declare_config(source_node)
    if config["source_domain_id"] != source_domain_id:
        source_node.get_logger().warn(
            "source_domain_id parameter differs from process ROS_DOMAIN_ID"
        )
    if config["target_domain_id"] != target_domain_id:
        source_node.get_logger().warn(
            "target_domain_id parameter differs from TOD_CONTROL_TARGET_DOMAIN_ID"
        )
    target_node = Node("peanut01_control_target", context=target_context)
    shared = SharedInputs()
    parameter_callback = install_parameter_callback(source_node, shared)
    bridge = Peanut01ControlBridge(source_node, target_node, shared, config)

    source_executor = SingleThreadedExecutor(context=source_context)
    target_executor = SingleThreadedExecutor(context=target_context)
    source_executor.add_node(source_node)
    target_executor.add_node(target_node)
    target_thread = threading.Thread(target=target_executor.spin, daemon=True)
    target_thread.start()

    source_node.get_logger().info(
        f"Peanut01 control bridge running disabled: domain {source_domain_id} -> "
        f"domain {target_domain_id}"
    )
    try:
        source_executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        del parameter_callback
        bridge.destroy_real_publishers()
        source_executor.shutdown()
        target_executor.shutdown()
        target_thread.join(timeout=2.0)
        source_node.destroy_node()
        target_node.destroy_node()
        rclpy.shutdown(context=source_context)
        rclpy.shutdown(context=target_context)


if __name__ == "__main__":
    main()
