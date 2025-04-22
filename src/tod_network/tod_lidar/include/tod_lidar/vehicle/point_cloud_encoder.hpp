/**
 * @file PointCloudEncoder.hpp
 * @author Niklas Krauss
 * @brief Draco ROS Encoder Interface 
 * @version 1.0
 *
 * @copyright TUMFTM 2024
 * Used as source \url{https://github.com/ros-perception/point_cloud_transport_plugins/tree/rolling/draco_point_cloud_transport}
 */



#pragma once
#include <draco/compression/encode.h>
#include <draco/compression/expert_encode.h>
#include <draco/point_cloud/point_cloud.h>
#include <draco/point_cloud/point_cloud_builder.h>

#include <rcpputils/tl_expected/expected.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tod_vehicle_msgs/msg/compressed_point_cloud.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace tod_lidar {
namespace tod_point_cloud_compression {

/**
 * @brief Draco Encoder API simplified from PointCloudTransport.
 * 
 * This class provides an interface to the Draco compression algorithm for point cloud data.
 * It includes functionality to encode point clouds, set encoding parameters, and handle 
 * metadata copying between sensor_msgs and the compressed point cloud format.
 * 
 * @ingroup tod_lidar_compression
 */
class DracoEncoder {
public:
  /**
   * @brief Structure to define the parameters for Draco encoding.
   * 
   * This structure contains parameters that control various aspects of the Draco encoder,
   * including speed and quantization options for the encoded point cloud.
   */
  struct EncoderParams {
    int encode_speed = 10;
    int decode_speed = 10;
    int encode_method = 2;
    bool deduplicate = false;
    bool force_quantization = true;
    int quantization_POSITION = 7;
    int quantization_GENERIC = 0; // Intensity
  };

  DracoEncoder() = default;
  /**
   * @brief Set encoding parameters for the Draco encoder.
   * 
   * This method allows users to specify encoding parameters such as speed, method, and quantization.
   * 
   * @param params The encoding parameters to set.
   */
  void set_parameters(const EncoderParams &params);

    /**
   * @brief Encode a PointCloud2 message into a compressed point cloud.
   * 
   * This method takes a `sensor_msgs::msg::PointCloud2` message and encodes it into a compressed
   * `tod_vehicle_msgs::msg::CompressedPointCloud` using the Draco algorithm.
   * 
   * @param input The PointCloud2 message to encode.
   * @return A `tl::expected` result containing the compressed point cloud or an error message.
   */
  tl::expected<tod_vehicle_msgs::msg::CompressedPointCloud, std::string>
  encode(const sensor_msgs::msg::PointCloud2& input);

private:
  EncoderParams params_;
  static std::unordered_map<std::string, draco::GeometryAttribute::Type>
      attributeTypes;

  void copy_cloud_metadata(tod_vehicle_msgs::msg::CompressedPointCloud &target,
                         const sensor_msgs::msg::PointCloud2 &source);
  /**
   * @brief Convert a PointCloud2 message to a Draco PointCloud.
   * 
   * This method converts a `sensor_msgs::msg::PointCloud2` message into a Draco PointCloud object
   * that can be encoded using the Draco compression algorithm.
   * 
   * @param pc2 The PointCloud2 message to convert.
   * @return A `tl::expected` result containing a Draco PointCloud or an error message.
   */
  tl::expected<std::unique_ptr<draco::PointCloud>, std::string>
  convert_PC2_to_draco(const sensor_msgs::msg::PointCloud2 &pc2);

  /**
   * @brief Get the attribute type for a specific point cloud field.
   * 
   * This method determines the type of attribute (e.g., POSITION, INTENSITY) based on the field name.
   * 
   * @param fieldName The name of the field (e.g., "x", "y", "intensity").
   * @return The corresponding Draco attribute type.
   */
  draco::GeometryAttribute::Type get_attribute_type(const std::string &fieldName);

 /**
   * @brief Get the data type for a specific sensor_msgs field datatype.
   * 
   * This method converts a sensor_msgs data type (e.g., INT8, UINT8) to the corresponding Draco data type.
   * 
   * @param datatype The sensor_msgs field datatype (e.g., `sensor_msgs::msg::PointField::INT8`).
   * @return The corresponding Draco data type.
   */
  draco::DataType get_data_type(uint8_t datatype);

  int get_quantization_bits(draco::GeometryAttribute::Type type);

public:
  static void register_position_field(const std::string &field);
};


} // namespace tod_point_cloud_compression
} // namespace tod_lidar