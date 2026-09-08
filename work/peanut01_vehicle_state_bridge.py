#!/usr/bin/env python3

import os
import threading

import rclpy
from autoware_vehicle_msgs.msg import (
    GearReport,
    HazardLightsReport,
    SteeringReport,
    TurnIndicatorsReport,
    VelocityReport,
)
from peanut01_vehicle_state_mapping import map_gear, map_indicator
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tod_vehicle_msgs.msg import PrimaryVehicleState, SecondaryVehicleState


PRIMARY_TOPIC = "/vehicle/interface/actuation/from_actuation/primary_vehicle_state"
SECONDARY_TOPIC = "/vehicle/interface/actuation/from_actuation/secondary_vehicle_state"


def main():
    sensor_domain_id = int(os.getenv("TOD_VEHICLE_STATE_SENSOR_DOMAIN_ID", "0"))
    tod_domain_id = int(os.getenv("ROS_DOMAIN_ID", "7"))
    steering_ratio = float(os.getenv("PEANUT01_STEERING_RATIO", "1.0"))

    sensor_context = Context()
    tod_context = Context()
    rclpy.init(context=sensor_context, domain_id=sensor_domain_id)
    rclpy.init(context=tod_context, domain_id=tod_domain_id)

    sensor_node = Node("peanut01_vehicle_state_source", context=sensor_context)
    tod_node = Node("peanut01_vehicle_state_destination", context=tod_context)
    primary_publisher = tod_node.create_publisher(PrimaryVehicleState, PRIMARY_TOPIC, 10)
    secondary_publisher = tod_node.create_publisher(
        SecondaryVehicleState, SECONDARY_TOPIC, 10
    )

    state = {
        "velocity": 0.0,
        "steering_tire_angle": 0.0,
        "gear": 0,
        "turn": 1,
        "hazard": 1,
    }

    def publish_primary():
        message = PrimaryVehicleState()
        message.header.stamp = tod_node.get_clock().now().to_msg()
        message.steering_tire_angle = state["steering_tire_angle"]
        message.steering_wheel_angle = state["steering_tire_angle"] * steering_ratio
        message.velocity = state["velocity"]
        message.acceleration = 0.0
        primary_publisher.publish(message)

    def publish_secondary():
        message = SecondaryVehicleState()
        message.header.stamp = tod_node.get_clock().now().to_msg()
        message.gear_position = map_gear(state["gear"])
        message.indicator = map_indicator(state["turn"], state["hazard"])
        secondary_publisher.publish(message)

    def on_velocity(message):
        state["velocity"] = message.longitudinal_velocity
        publish_primary()

    def on_steering(message):
        state["steering_tire_angle"] = message.steering_tire_angle
        publish_primary()

    def on_gear(message):
        state["gear"] = message.report
        publish_secondary()

    def on_turn_indicator(message):
        state["turn"] = message.report
        publish_secondary()

    def on_hazard_lights(message):
        state["hazard"] = message.report
        publish_secondary()

    subscriptions = (
        (VelocityReport, "/vehicle/status/velocity_status", on_velocity),
        (SteeringReport, "/vehicle/status/steering_status", on_steering),
        (GearReport, "/vehicle/status/gear_status", on_gear),
        (
            TurnIndicatorsReport,
            "/vehicle/status/turn_indicators_status",
            on_turn_indicator,
        ),
        (
            HazardLightsReport,
            "/vehicle/status/hazard_lights_status",
            on_hazard_lights,
        ),
    )
    for message_type, topic, callback in subscriptions:
        sensor_node.create_subscription(message_type, topic, callback, 10)

    sensor_executor = SingleThreadedExecutor(context=sensor_context)
    tod_executor = SingleThreadedExecutor(context=tod_context)
    sensor_executor.add_node(sensor_node)
    tod_executor.add_node(tod_node)
    tod_thread = threading.Thread(target=tod_executor.spin, daemon=True)
    tod_thread.start()

    sensor_node.get_logger().info(
        f"Bridging vehicle state from domain {sensor_domain_id} "
        f"to domain {tod_domain_id}"
    )
    try:
        sensor_executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        sensor_executor.shutdown()
        tod_executor.shutdown()
        sensor_node.destroy_node()
        tod_node.destroy_node()
        rclpy.shutdown(context=sensor_context)
        rclpy.shutdown(context=tod_context)


if __name__ == "__main__":
    main()
