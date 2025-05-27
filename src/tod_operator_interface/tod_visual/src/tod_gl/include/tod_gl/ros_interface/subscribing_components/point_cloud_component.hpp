/**
 * @file point_cloud_component.hpp
 * @brief Decompressed PointCloud holds the data of the decoded pointclouds from @ref tod_lidar_compression
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include "rclcpp/rclcpp.hpp"

#include "tod_gl/ros_interface/subscribing_component_base.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace tod_gl {
  
class PointCloudComponent : public SubscribingComponent<sensor_msgs::msg::PointCloud2> {

  public:
    explicit PointCloudComponent(std::shared_ptr<rclcpp::Node> sub_node)
      : SubscribingComponent(sub_node, "input/pointcloud"),
      point_cloud_()
    {}

    pcl::PointCloud<pcl::PointXYZ>::Ptr get_point_cloud(); 

  private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;
    void cb_message(const sensor_msgs::msg::PointCloud2::SharedPtr msg) override;
};

}  // namespace tod_gl