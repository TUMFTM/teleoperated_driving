/**
 * @file point_cloud_decoder.hpp
 * @author Niklas Krauss
 * @brief Draco ROS Decoder Interface 
 * @version 1.0
 *
 * @copyright TUMFTM 2024
 * Used as source \url{https://github.com/ros-perception/point_cloud_transport_plugins/tree/rolling/draco_point_cloud_transport}
 */


#pragma once

#include <draco/point_cloud/point_cloud.h>
#include <draco/compression/decode.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tod_vehicle_msgs/msg/compressed_point_cloud.hpp>
#include <rcpputils/tl_expected/expected.hpp>

#include <string>
#include <vector>
#include <memory>

namespace tod_lidar {
namespace tod_point_cloud_compression {

/**
 * @brief Draco Decoder API simplifed from PointCloudTransport -
 * @ingroup tod_lidar_compression
 */
class DracoDecoder
{
public:
 /**
   * @brief Structure to define the parameters for Draco decoding.
   * 
   * This structure contains parameters that control various aspects of the Draco decoder,
   * including options to skip dequantization of specific attributes (e.g., POSITION, GENERIC).
   */
  struct DecoderParams
  {
      bool SkipDequantizationPOSITION = false;
      bool SkipDequantizationGENERIC = false;
  };

  DracoDecoder() = default;

 /**
   * @brief Set decoding parameters for the Draco decoder.
   * 
   * This method allows users to specify decoding parameters such as whether to skip dequantization
   * for position or generic attributes.
   * 
   * @param params The decoding parameters to set.
   */
  void set_parameters(const DecoderParams& params);

  /**
   * @brief Decode a CompressedPointCloud message into a PointCloud2 message.
   * 
   * This method takes a `tod_vehicle_msgs::msg::CompressedPointCloud` message and decodes it into a
   * `sensor_msgs::msg::PointCloud2` message using the Draco algorithm.
   * 
   * @param input The CompressedPointCloud message to decode.
   * @return A `tl::expected` result containing the decoded PointCloud2 message or an error message.
   */
  tl::expected<sensor_msgs::msg::PointCloud2, std::string>
  decode(const tod_vehicle_msgs::msg::CompressedPointCloud& input);


private:
  DecoderParams params_;

  void configure_decoder(draco::Decoder& decoder);

 /**
   * @brief Convert a Draco PointCloud to a PointCloud2 message.
   * 
   * This method converts a Draco PointCloud to a `sensor_msgs::msg::PointCloud2` message and
   * copies relevant metadata from the compressed point cloud message.
   * 
   * @param pc The Draco PointCloud to convert.
   * @param compressed_PC2 The source CompressedPointCloud message.
   * @param PC2 The target PointCloud2 message to store the decoded data.
   * @return A `tl::expected` result indicating whether the conversion was successful or not.
   */
  tl::expected<bool, std::string> convertDracoToPC2(
    const draco::PointCloud& pc,
    const tod_vehicle_msgs::msg::CompressedPointCloud& compressed_PC2,
    sensor_msgs::msg::PointCloud2& PC2);


  void copy_cloud_metadata(sensor_msgs::msg::PointCloud2& target,
                         const tod_vehicle_msgs::msg::CompressedPointCloud& source);
};

} // namespace tod_point_cloud_compression
} // namespace tod_lidar