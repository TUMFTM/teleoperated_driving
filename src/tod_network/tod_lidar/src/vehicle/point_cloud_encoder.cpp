/**
 * @file point_cloud_encoder.cpp
 * @author Niklas Krauss
 * @brief Draco ROS Encoder Interface 
 * @version 1.0
 *
 * @copyright TUMFTM 2024
 * Used as source \url{https://github.com/ros-perception/point_cloud_transport_plugins/tree/rolling/draco_point_cloud_transport}
 */

#include "tod_lidar/vehicle/point_cloud_encoder.hpp"

namespace tod_lidar {
namespace tod_point_cloud_compression {
  /**
  * @brief Add Parameter to Draco Encoder
  */
  void DracoEncoder::set_parameters(const EncoderParams& params)
  {
    params_ = params;
    if (params_.encode_method < 0 || params_.encode_method > 2) {
      printf("Invalid encode_method (%d). Setting to Auto (0).", params_.encode_method);
      params_.encode_method = 0;
    }
    
    // Validate encode_speed and decode_speed
    params_.encode_speed = std::clamp(params_.encode_speed, 0, 10);
    params_.decode_speed = std::clamp(params_.decode_speed, 0, 10);

  }

  tl::expected<tod_vehicle_msgs::msg::CompressedPointCloud, std::string>
  DracoEncoder::encode(const sensor_msgs::msg::PointCloud2 &input)
  {
    tod_vehicle_msgs::msg::CompressedPointCloud compressed_msg;

    copy_cloud_metadata(compressed_msg, input);

    auto pc_result = convert_PC2_to_draco(input);
    if (!pc_result)
    {
        std::cout << "Failed to convert PointCloud2 to Draco format: " << pc_result.error() << std::endl;
      return tl::make_unexpected(pc_result.error());
    }
    const auto& pc = pc_result.value();


    if (params_.deduplicate)
    {
      compressed_msg.height = 1;
      compressed_msg.width = pc->num_points();
      compressed_msg.row_step = compressed_msg.width * compressed_msg.point_step;
      compressed_msg.is_dense = true;
    }

    draco::EncoderBuffer encode_buffer;
    
    draco::Encoder encoder;

    encoder.SetSpeedOptions(params_.encode_speed, params_.decode_speed);

    if (params_.encode_method == 0 ) {}
    else if (params_.encode_method == 1 || params_.force_quantization) {
        encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, params_.quantization_POSITION);
        encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, params_.quantization_GENERIC);
      
      encoder.SetEncodingMethod(draco::POINT_CLOUD_KD_TREE_ENCODING);


    } else if (params_.encode_method == 2) {

        encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, params_.quantization_POSITION);
        encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, params_.quantization_GENERIC);
      
      encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);
    }



    draco::Status status = encoder.EncodePointCloudToBuffer(*pc, &encode_buffer);
    if (!status.ok())
    {
        std::cout << "Draco encoding failed with status code: " << status.code() << std::endl;
        std::cout << "Error message: " << status.error_msg_string() << std::endl;
      return tl::make_unexpected("Draco encoder error: " + status.error_msg_string());
    }
  


    compressed_msg.compressed_data.resize(encode_buffer.size());
    std::memcpy(compressed_msg.compressed_data.data(), encode_buffer.data(), encode_buffer.size());
    compressed_msg.format = "draco";
    return compressed_msg;
  }


  void DracoEncoder::copy_cloud_metadata(tod_vehicle_msgs::msg::CompressedPointCloud& target,
                         const sensor_msgs::msg::PointCloud2& source)
  {
    target.header = source.header;
    target.height = source.height;
    target.width = source.width;
    target.fields = source.fields;
    target.is_bigendian = source.is_bigendian;
    target.point_step = source.point_step;
    target.row_step = source.row_step;
    target.is_dense = source.is_dense;
  }

  tl::expected<std::unique_ptr<draco::PointCloud>, std::string>
  DracoEncoder::convert_PC2_to_draco(const sensor_msgs::msg::PointCloud2& PC2)
  {
      draco::PointCloudBuilder builder;
      uint64_t number_of_points = PC2.height * PC2.width;
      builder.Start(static_cast<int>(number_of_points));

      // Find x, y, z fields
      const sensor_msgs::msg::PointField* x_field = nullptr;
      const sensor_msgs::msg::PointField* y_field = nullptr;
      const sensor_msgs::msg::PointField* z_field = nullptr;

      for (const auto& field : PC2.fields) {
          if (field.name == "x") x_field = &field;
          else if (field.name == "y") y_field = &field;
          else if (field.name == "z") z_field = &field;
      }

      if (!x_field || !y_field || !z_field) {
          return tl::make_unexpected("Missing x, y, or z field in PointCloud2");
      }

      // Add position attribute
      int pos_att_id = builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);

      // Set position values
      std::vector<float> positions(number_of_points * 3);
      for (size_t i = 0; i < number_of_points; ++i) {
          const uint8_t* point_data = &PC2.data[i * PC2.point_step];
          positions[i*3] = *reinterpret_cast<const float*>(point_data + x_field->offset);
          positions[i*3+1] = *reinterpret_cast<const float*>(point_data + y_field->offset);
          positions[i*3+2] = *reinterpret_cast<const float*>(point_data + z_field->offset);
      }

      builder.SetAttributeValuesForAllPoints(pos_att_id, positions.data(),0 );

      std::unique_ptr<draco::PointCloud> pc = builder.Finalize(false);  // Set to false to disable deduplication
      if (pc == nullptr) {
          return tl::make_unexpected("Conversion to Draco::PointCloud failed");
      }
      return pc;
  }

  draco::GeometryAttribute::Type DracoEncoder::get_attribute_type(const std::string& fieldName)
  {
    if (fieldName == "x" || fieldName == "y" || fieldName == "z") {
      return draco::GeometryAttribute::POSITION;
    } else if (fieldName == "intensity") {
      return draco::GeometryAttribute::GENERIC;
    }
    return draco::GeometryAttribute::GENERIC;
  }

  draco::DataType DracoEncoder::get_data_type(uint8_t datatype)
  {
    switch (datatype)
    {
      case sensor_msgs::msg::PointField::INT8: return draco::DT_INT8;
      case sensor_msgs::msg::PointField::UINT8: return draco::DT_UINT8;
      case sensor_msgs::msg::PointField::INT16: return draco::DT_INT16;
      case sensor_msgs::msg::PointField::UINT16: return draco::DT_UINT16;
      case sensor_msgs::msg::PointField::INT32: return draco::DT_INT32;
      case sensor_msgs::msg::PointField::UINT32: return draco::DT_UINT32;
      case sensor_msgs::msg::PointField::FLOAT32: return draco::DT_FLOAT32;
      case sensor_msgs::msg::PointField::FLOAT64: return draco::DT_FLOAT64;
      default: throw std::runtime_error("Unsupported data type");
    }
  }

  int DracoEncoder::get_quantization_bits(draco::GeometryAttribute::Type type)
  {
    switch (type)
    {
      case draco::GeometryAttribute::POSITION: return params_.quantization_POSITION;
      default: return params_.quantization_GENERIC;
    }
  }



std::unordered_map<std::string, draco::GeometryAttribute::Type> DracoEncoder::attributeTypes = {
  {"x", draco::GeometryAttribute::Type::POSITION},
  {"y", draco::GeometryAttribute::Type::POSITION},
  {"z", draco::GeometryAttribute::Type::POSITION},
  {"intensity", draco::GeometryAttribute::Type::GENERIC},
};


const sensor_msgs::msg::PointField* find_field(const sensor_msgs::msg::PointCloud2& pc2, const std::string& fieldName) {
  for (const auto& field : pc2.fields) {
    if (field.name == fieldName) {
      return &field;
    }
  }
  return nullptr;
}

} // namespace tod_point_cloud_compression
} // namespace tod_lidar