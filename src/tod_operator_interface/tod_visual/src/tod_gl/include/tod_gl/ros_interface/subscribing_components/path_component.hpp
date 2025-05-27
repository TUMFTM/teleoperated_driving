/**
 * @file path_component.hpp
 * @brief Path component that manages the subscription and the data for path-like topics that are used to render the trajectories during the trajectory guidance control mode
 * @copyright TUMFTM 2024
 **/

#pragma once


#include "tod_gl/ros_interface/subscribing_component_base.hpp"

#include <rclcpp/rclcpp.hpp>


namespace tod_gl {
template<typename PathMsg>
class PathComponent : public SubscribingComponent<PathMsg> 
{
public:
    explicit  PathComponent(std::shared_ptr<rclcpp::Node> sub_node, const std::string& topic_name ) 
        : SubscribingComponent<PathMsg>(sub_node, topic_name), 
        path_(),
        has_received_path_(false)
    {}

    bool hasValidPath() const { return has_received_path_; }
    const PathMsg& get_path() const { return path_; }

private:    
    PathMsg path_;
    bool has_received_path_;
    
    void cb_message(const typename PathMsg::SharedPtr msg) override {
        path_ = *msg;
        has_received_path_ = true;
    }
};
}  // namespace tod_gl






