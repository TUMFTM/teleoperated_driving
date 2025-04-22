/**
 * @file point_cloud_decoder.cpp
 * @author Niklas Krauss
 * @brief PointCloudDecoderNode who manages the interaction betwee Draco and ROS2 
 * @version 1.0
 *
 * @copyright TUMFTM 2024
 */

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "tod_vehicle_msgs/msg/compressed_point_cloud.hpp"
#include "tod_lidar/operator/point_cloud_decoder.hpp"


namespace tod_lidar {
namespace tod_point_cloud_compression {


/**
 * @brief PointCloudDecoder Receiver who decodes and republishes the compressed pointcloud
 * @ingroup tod_lidar_compression
 */
class PointCloudDecoderNode : public rclcpp::Node
{
public:
    PointCloudDecoderNode() : Node("pointcloud_decoder" + std::to_string(std::time(nullptr)))
    {
        this->declare_parameter("vehicleID", "edgar");
        this->declare_parameter("target_frame", "base_link");
    }

    void init() {
        subscription_ = this->create_subscription<tod_vehicle_msgs::msg::CompressedPointCloud>(
            "input/pointcloud_compressed", 1, [this](const tod_vehicle_msgs::msg::CompressedPointCloud& msg){this->callback_compressed_msg(msg);});
        
        decoder_ = std::make_unique<DracoDecoder>();
        DracoDecoder::DecoderParams params;

        // DECODER SPEED?
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        decoder_->set_parameters(params);

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/pointcloud_decompressed", 1);
    }

private:

    /**
    * @brief  Subscribes to Pointcloud from receiver and decompresses and republishes the pointcloud 
    */
    void callback_compressed_msg(const tod_vehicle_msgs::msg::CompressedPointCloud& msg)
    {
        
        auto result = decoder_->decode(msg);
        RCLCPP_INFO_ONCE(this->get_logger(), "callback_compressed_msg ");

        if(result) 
        {
            sensor_msgs::msg::PointCloud2 decoded_cloud = result.value();

            std::string target_frame = this->get_parameter("target_frame").as_string();
            auto trans_res = transform_point_cloud( decoded_cloud, target_frame);


            publisher_->publish(decoded_cloud);
            RCLCPP_INFO_ONCE(this->get_logger(), "published messages");

        }
        else {
            std::string error_message = result.error();
            RCLCPP_INFO_ONCE(this->get_logger(), "Failed: %s", error_message.c_str());
        }

    }

    /**
    * @brief  Transforms Decompressed Pointcloud to Targetframe e.g. base_link 
    */
    bool transform_point_cloud(sensor_msgs::msg::PointCloud2& cloud, const std::string& target_frame)
    {
        if (cloud.header.frame_id == target_frame) {
            return true;  // No transform needed
        }

        try {
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer_->lookupTransform(target_frame, cloud.header.frame_id,
                                            tf2::TimePointZero);
            
            sensor_msgs::msg::PointCloud2 cloud_out;
            tf2::doTransform(cloud, cloud_out, transform_stamped);
            cloud = std::move(cloud_out);
            return true;
        }
        catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
            return false;
        }
    }
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<tod_vehicle_msgs::msg::CompressedPointCloud>::SharedPtr subscription_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::unique_ptr<DracoDecoder> decoder_;
    
    const std::string transport_{"draco"};
};


} // namespace tod_lidar_compression
} // namespace tod_lidar

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tod_lidar::tod_point_cloud_compression::PointCloudDecoderNode>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

