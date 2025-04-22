/**
 * @file static_transform_publisher_node.cpp
 * @author Feiler
 * @copyright TUMFTM 2020
 */

#include <rclcpp/rclcpp.hpp>
#include <tod_core/param_set/TransformParameters.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <memory>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("StaticTransformPublisher");
    node->declare_parameter<std::string>("config_path", "");
    std::string config_path;
    if(! node->get_parameter("config_path", config_path)) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Failed to retrieve 'config_path' parameter. Ensure it is set in the launch file.");
    }
    auto _transformParams(std::make_unique<tod_core::param_set::Transform>(node.get(), config_path + "/vehicle_config/"));
    _transformParams->load_parameters();
    static tf2_ros::StaticTransformBroadcaster tf_broadcaster(node);
    rclcpp::Rate r{20};
    while (rclcpp::ok()) {
        r.sleep();
        rclcpp::spin_some(node);
        _transformParams->updateStamp();
        tf_broadcaster.sendTransform(_transformParams->get_transforms());
    }
    return 0;
}
