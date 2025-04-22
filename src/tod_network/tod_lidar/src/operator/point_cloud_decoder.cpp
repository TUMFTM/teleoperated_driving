/**
 * @file point_cloud_decoder.cpp
 * @author Niklas Krauss
 * @brief Draco ROS Decoder Interface 
 * @version 1.0
 *
 * @copyright TUMFTM 2024
 * Used as source \url{https://github.com/ros-perception/point_cloud_transport_plugins/tree/rolling/draco_point_cloud_transport}
 */


#include "tod_lidar/operator/point_cloud_decoder.hpp"

namespace tod_lidar {
namespace tod_point_cloud_compression {
void DracoDecoder::set_parameters(const DecoderParams& params)
{
params_ = params;
}

tl::expected<sensor_msgs::msg::PointCloud2, std::string>
DracoDecoder::decode(const tod_vehicle_msgs::msg::CompressedPointCloud& input)
{
    if (input.compressed_data.empty()) {
        return tl::make_unexpected("Received compressed Draco message with zero length.");
    }

    draco::DecoderBuffer decode_buffer;
    decode_buffer.Init(reinterpret_cast<const char*>(input.compressed_data.data()),
                       input.compressed_data.size());

    draco::Decoder decoder;
    
    if (params_.SkipDequantizationPOSITION) {
        decoder.SetSkipAttributeTransform(draco::GeometryAttribute::POSITION);
    }
    if (params_.SkipDequantizationGENERIC) {
        decoder.SetSkipAttributeTransform(draco::GeometryAttribute::GENERIC);
    }

    auto pc_status = decoder.DecodePointCloudFromBuffer(&decode_buffer);
    if (!pc_status.ok()) {
        return tl::make_unexpected("Draco decoder error: " + pc_status.status().error_msg_string());
    }

    const auto& draco_pc = pc_status.value();

    // Convert Draco PointCloud to PointCloud2
    sensor_msgs::msg::PointCloud2 output;
    output.header = input.header;
    output.height = 1;
    output.width = draco_pc->num_points();
    output.is_dense = true;

    output.fields.resize(3);
    output.fields[0].name = "x";
    output.fields[1].name = "y";
    output.fields[2].name = "z";
    for (int i = 0; i < 3; ++i) {
        output.fields[i].offset = i * sizeof(float);
        output.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output.fields[i].count = 1;
    }

    output.point_step = 3 * sizeof(float);
    output.row_step = output.point_step * output.width;
    output.data.resize(output.row_step);

    const auto* pos_attr = draco_pc->GetNamedAttribute(draco::GeometryAttribute::POSITION);
    if (pos_attr == nullptr) {
        return tl::make_unexpected("Draco point cloud does not contain position attribute");
    }

    for (draco::PointIndex i(0); i < draco_pc->num_points(); ++i) {
        float pos[3];
        pos_attr->GetValue(pos_attr->mapped_index(i), &pos);
        memcpy(&output.data[i.value() * output.point_step], pos, sizeof(float) * 3);
    }

    return output;
}


void DracoDecoder::configure_decoder(draco::Decoder& decoder)
{
if (params_.SkipDequantizationPOSITION)
    decoder.SetSkipAttributeTransform(draco::GeometryAttribute::POSITION);
if (params_.SkipDequantizationGENERIC)
    decoder.SetSkipAttributeTransform(draco::GeometryAttribute::GENERIC);
}


void DracoDecoder::copy_cloud_metadata(sensor_msgs::msg::PointCloud2& target,
                        const tod_vehicle_msgs::msg::CompressedPointCloud& source)
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


} // namespace tod_point_cloud_compression
} // namespace tod_lidar