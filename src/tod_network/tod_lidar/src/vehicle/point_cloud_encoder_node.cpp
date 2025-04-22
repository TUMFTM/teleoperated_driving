/**
 * @file point_cloud_encoder_node.cpp
 * @brief PointCloudEncoderNode who manages the interaction betwee Draco and ROS2 based on PointCloudTransport - used to compress larger pointlcouds for them t the operator
 * @version 1.0
 * @copyright TUMFTM 2024
 */

#include <chrono>
#include <deque>


#include "tod_lidar/vehicle/point_cloud_encoder.hpp"
#include "tod_core/param_set/LidarParameters.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/approximate_voxel_grid.h"
#include "pcl/filters/uniform_sampling.h"
#include "pcl/filters/crop_box.h"

#include "ament_index_cpp/get_package_share_directory.hpp"



namespace tod_lidar {
/**
 * @ingroup tod_perception
 * @note maybe change to tod_lidar not sure
 * @brief Components for Lidar processing, compression and transmission
 */


namespace tod_point_cloud_compression {
/**
 * @brief @defgroup tod_lidar_compression TOD Lidar Compression
 * @ingroup tod_lidar Logical Grouping for  LiDAR Processing in the TOD Stack
 * @brief Components for Lidar compression and transmission using draco and PCL
 */



/**
 * @brief PointCloud Sender who subscribes on existing pointcloud and does all the processing and compression using PCL, Pruning and Draco
 * The compressed pointcloud is send out to the sender for transmission over network.
 * @ingroup tod_lidar_compression
 */
class PointCloudEncoderNode : public rclcpp::Node
{
public:
    PointCloudEncoderNode() : Node("pointcloud_sender_" + std::to_string(std::time(nullptr)))
    {
        this->declare_parameter("enable_logging", false);
        this->declare_parameter("target_points", 100001);
        this->declare_parameter("vehicleID", "edgar");
        this->declare_parameter("smooth_pointclouds", false);
        this->declare_parameter("history_size", 3);
        this->declare_parameter("exponential_decay_rate", 0.9); 
        this->declare_parameter("voxel_leaf_size", 0.45); 
        this->declare_parameter("grid_size", 0.05); 
        this->declare_parameter("crop_box_min_x", -15.0);
        this->declare_parameter("crop_box_min_y", -20.0);
        this->declare_parameter("crop_box_min_z", 0.3);
        this->declare_parameter("crop_box_max_x", 20.0);
        this->declare_parameter("crop_box_max_y", 20.0);
        this->declare_parameter("crop_box_max_z", 2.2);

        this->declare_parameter("draco_encode_method", 2);
        this->declare_parameter("draco_encode_speed", 1);
        this->declare_parameter("draco_decode_speed", 1);
        this->declare_parameter("draco_quatization_position", 7);

        history_size_ = this->get_parameter("history_size").as_int();
        exponential_decay_rate_ = this->get_parameter("exponential_decay_rate").as_double();
        grid_size_ = this->get_parameter("grid_size").as_double();
        voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
        crop_box_min_x_ = this->get_parameter("crop_box_min_x").as_double();
        crop_box_min_y_ = this->get_parameter("crop_box_min_y").as_double();
        crop_box_min_z_ = this->get_parameter("crop_box_min_z").as_double();
        crop_box_max_x_ = this->get_parameter("crop_box_max_x").as_double();
        crop_box_max_y_ = this->get_parameter("crop_box_max_y").as_double();
        crop_box_max_z_ = this->get_parameter("crop_box_max_z").as_double();

        draco_decode_speed_ = this->get_parameter("draco_encode_speed").as_int();
        draco_encode_speed_ = this->get_parameter("draco_decode_speed").as_int();
        draco_quatization_position_ = this->get_parameter("draco_quatization_position").as_int();
        draco_encode_method_ = this->get_parameter("draco_encode_method").as_int();

        this->declare_parameter("config_path", "");
        
        std::string desiredPath = this->get_parameter("config_path").as_string() + "/vehicle_config/";
        lidarParamHandler_ = std::make_unique<tod_core::param_set::Lidar>(this, desiredPath);
        if (!lidarParamHandler_->load_parameters()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load lidar parameters");
            return;
        }
    }

    void init()
    {
        const auto& lidar = lidarParamHandler_->get_sensors();

        auto topic_name =  lidarParamHandler_->get_lidar_topics_namespace() + lidarParamHandler_->get_pointcloud_name();
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos.best_effort();
        encoder_ = std::make_unique<DracoEncoder>();
        auto draco_params = DracoEncoder::EncoderParams();
        
        draco_params.encode_speed = draco_encode_speed_;
        draco_params.decode_speed = draco_decode_speed_;
        draco_params.quantization_POSITION = draco_quatization_position_;
        draco_params.encode_method = draco_encode_method_;
        encoder_->set_parameters(draco_params);

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name, qos, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg){this->cloud_callback(msg);} 
            );
        pub_ = this->create_publisher<tod_vehicle_msgs::msg::CompressedPointCloud>("output/pointcloud_compressed", 1);
    }

private:
/**
 * @brief  Subscribing function for the vehicle's lidar topic - contains processing, compression and republishing of said point cloud 
 * @details first performs \ref efficient_filter_point_cloud and if output cloud > target_points then downsample_point_cloud. If smooth_pointclouds enabled performs  \ref applyExponentialDecaySmoothing
 */
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        bool enable_logging = this->get_parameter("enable_logging").as_bool();
        int target_points = this->get_parameter("target_points").as_int();

        auto start_time = std::chrono::high_resolution_clock::now();

        sensor_msgs::msg::PointCloud2::SharedPtr filtered_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        efficient_filter_point_cloud(msg, filtered_cloud, target_points, enable_logging);
        
        if (filtered_cloud->height * filtered_cloud->width > this->get_parameter("target_points").as_int() ) {
            downsample_point_cloud(filtered_cloud,target_points);
        }

        sensor_msgs::msg::PointCloud2::SharedPtr smoothed_pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        if (this->get_parameter("smooth_pointclouds").as_bool()){
            add_to_history(filtered_cloud);
            apply_exponential_decay_smoothing(smoothed_pointcloud);
        }

        if (filtered_cloud->width * filtered_cloud->height == 0) {
            RCLCPP_WARN(this->get_logger(), "Averaged point cloud is empty. Skipping encoding.");
            return;
        }

        tl::expected<tod_vehicle_msgs::msg::CompressedPointCloud, std::string> result;
        if (this->get_parameter("smooth_pointclouds").as_bool()){
            result = encoder_->encode(*smoothed_pointcloud);
        } else {
            result = encoder_->encode(*filtered_cloud);
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if (enable_logging) {
            RCLCPP_INFO(this->get_logger(), "Lidar Preprocessing and Encoding took %ld ms", 
                        duration.count());
        }

        if (result) {
            tod_vehicle_msgs::msg::CompressedPointCloud compressed_msg = result.value();
            compressed_msg.header = filtered_cloud->header;
            //log_compression_stats(filtered_cloud, compressed_msg);
            pub_->publish(compressed_msg);  
        } else {
            std::string error_message = result.error();
            RCLCPP_ERROR(this->get_logger(), "Encoding failed: %s", error_message.c_str());
        }
    }


    void log_compression_stats(const sensor_msgs::msg::PointCloud2& raw_cloud, 
                            const tod_vehicle_msgs::msg::CompressedPointCloud& compressed_msg)
    {
        static bool logged = false;
        if (logged) return;

        // Convert bytes to bits and calculate kilobits per second
        double raw_size_kbps = (raw_cloud.data.size() * 8.0 * 10) / 1000.0; // *10 for 10Hz, /1000 for kilo
        double compressed_size_kbps = (compressed_msg.compressed_data.size() * 8.0 * 10) / 1000.0;
        
        double compression_ratio = (raw_size_kbps != 0) ? (compressed_size_kbps / raw_size_kbps) : 0;

        RCLCPP_INFO(this->get_logger(),
                    "Compression stats (extrapolated to 1 second at 10 Hz):\n"
                    "  Raw data rate: %.2f kbps\n"
                    "  Compressed data rate: %.2f kbps\n"
                    "  Compression ratio: %.2f%%\n"
                    "  Number of points per message: %u\n"
                    "  Estimated points per second: %u",
                    raw_size_kbps, 
                    compressed_size_kbps,
                    compression_ratio * 100,
                    raw_cloud.width * raw_cloud.height,
                    (raw_cloud.width * raw_cloud.height) * 10); // *10 for 10Hz

        logged = false;
    }
 
    /**
    * @brief  Filtering function for incomming pointcloud using PCL, CropBox and Approx. Voxelization
    */
    void efficient_filter_point_cloud(sensor_msgs::msg::PointCloud2::SharedPtr input_cloud, 
                                   sensor_msgs::msg::PointCloud2::SharedPtr output_cloud, 
                                   int target_points,
                                   bool enable_logging)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input_cloud, *cloud);

        if (enable_logging) {
            RCLCPP_INFO(this->get_logger(), "Initial cloud size: %zu", cloud->size());
        }



        pcl::CropBox<pcl::PointXYZ> crop_box;
        crop_box.setMin(Eigen::Vector4f(crop_box_min_x_, crop_box_min_y_, crop_box_min_z_, 1.0));
        crop_box.setMax(Eigen::Vector4f(crop_box_max_x_, crop_box_max_y_, crop_box_max_z_, 1.0));
        crop_box.setInputCloud(cloud);
        crop_box.filter(*cloud);  // In-place filtering
        
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> approx_voxel_grid;
        approx_voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        approx_voxel_grid.setInputCloud(cloud);
        approx_voxel_grid.filter(*cloud); 

         if (cloud->width * cloud->height == 0) {
            RCLCPP_WARN(this->get_logger(), "point cloud is empty. Skipping encoding.");
            return;
        }


        pcl::toROSMsg(*cloud, *output_cloud);
        output_cloud->header = input_cloud->header;
    }


    /*
    * @brief perform uniform downsampling of the input pointcloud
    */
    void downsample_point_cloud(sensor_msgs::msg::PointCloud2::SharedPtr cloud, int target_points) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud, *pcl_cloud);

        pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
        uniform_sampling.setInputCloud(pcl_cloud);
        size_t step = cloud->height * cloud->width / target_points;
        uniform_sampling.setRadiusSearch(step);
        uniform_sampling.filter(*pcl_cloud);

        pcl::toROSMsg(*pcl_cloud, *cloud);
    }

    void add_to_history(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
    {
        cloud_history_.push_front(cloud);
        if (cloud_history_.size() > history_size_)
        {
            cloud_history_.pop_back();
        }
    }



    /*
    * @brief expoential smoothing of the pointclouds based on history size of the incoming pointcloud. Only use when raw sensor data is noisy
    */
    void apply_exponential_decay_smoothing(const sensor_msgs::msg::PointCloud2::SharedPtr output_cloud)
    {
        if (cloud_history_.empty())
        {
            return;
        }

        std::unordered_map<std::string, pcl::PointXYZ> grid;
        std::unordered_map<std::string, float> total_weights;

        float current_weight = 1.0;
        for (size_t i = 0; i < cloud_history_.size(); ++i)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*cloud_history_[i], *pcl_cloud);

            for (const auto& point : pcl_cloud->points)
            {
                int x = std::round(point.x / grid_size_);
                int y = std::round(point.y / grid_size_);
                int z = std::round(point.z / grid_size_);
                std::string key = std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);

                if (grid.find(key) == grid.end()) {
                    grid[key] = pcl::PointXYZ(0, 0, 0);
                    total_weights[key] = 0.0;
                }

                grid[key].x += point.x * current_weight;
                grid[key].y += point.y * current_weight;
                grid[key].z += point.z * current_weight;
                total_weights[key] += current_weight;
            }

            current_weight *= exponential_decay_rate_;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& cell : grid)
        {
            pcl::PointXYZ averaged_point = cell.second;
            float total_weight = total_weights[cell.first];
            averaged_point.x /= total_weight;
            averaged_point.y /= total_weight;
            averaged_point.z /= total_weight;
            smoothed_cloud->points.push_back(averaged_point);
        }

        smoothed_cloud->width = smoothed_cloud->points.size();
        smoothed_cloud->height = 1;
        smoothed_cloud->is_dense = false;

        pcl::toROSMsg(*smoothed_cloud, *output_cloud);
        output_cloud->header = cloud_history_.front()->header;

    }



    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<tod_vehicle_msgs::msg::CompressedPointCloud>::SharedPtr pub_;
    

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubDebug_;

    size_t history_size_;
    float exponential_decay_rate_;
    double crop_box_min_x_;
    double crop_box_min_y_;
    double crop_box_min_z_;
    double crop_box_max_x_;
    double crop_box_max_y_;
    double crop_box_max_z_;
    double voxel_leaf_size_;
    double grid_size_;
    int draco_decode_speed_;
    int draco_encode_speed_;
    int draco_quatization_position_;
    int draco_encode_method_;

    std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> cloud_history_;

    std::unique_ptr<DracoEncoder> encoder_;
    std::unique_ptr<tod_core::param_set::Lidar> lidarParamHandler_;
    std::string _vehicleID{"edgar"};
    bool logged = false;
};

}  // namespace tod_lidar_compression
} // namespace tod_lidar

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tod_lidar::tod_point_cloud_compression::PointCloudEncoderNode>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

