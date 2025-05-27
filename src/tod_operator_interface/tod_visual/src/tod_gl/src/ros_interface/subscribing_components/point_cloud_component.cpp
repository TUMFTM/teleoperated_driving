/**
 * @file point_cloud_component.cpp
 * @brief  Decompressed PointCloud holds the data of the decoded pointclouds from @ref tod_lidar_compression
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/ros_interface/subscribing_components/point_cloud_component.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace tod_gl {

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudComponent::get_point_cloud(){
        return point_cloud_ ? point_cloud_ : std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

void PointCloudComponent::cb_message(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    point_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *point_cloud_);
}

}  // namespace tod_gl