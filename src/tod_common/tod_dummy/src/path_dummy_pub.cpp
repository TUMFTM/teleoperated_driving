#include "tod_dummy/path_dummy_pub.hpp"
using namespace tod_dummy;


PathDummyPub::PathDummyPub()
: Node("path_dummy_pub")
{
  publisher_ = this->create_publisher<autoware_auto_planning_msgs::msg::Path>("output/path", 1);
  
  // Timer to call the timer_callback function every 500ms
  timer_ = this->create_wall_timer(
    500ms, std::bind(&PathDummyPub::timer_callback, this));
}

void PathDummyPub::timer_callback()
{

  auto path_msg = std::make_unique<autoware_auto_planning_msgs::msg::Path>();
  

  for (int i = 0; i < 100; ++i)
  {
    autoware_auto_planning_msgs::msg::PathPoint point;
    point.pose.position.x = i * 0.1;  // Increment x position
    point.pose.position.y = i * 0.1;  // Increment y position
    point.pose.position.z = 0.0;      // Z position remains constant
    
    path_msg->points.push_back(point);
  }

  publisher_->publish(std::move(path_msg));

  // Optionally log the message
  // RCLCPP_INFO(this->get_logger(), "Published path with 100 points");
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathDummyPub>());
  rclcpp::shutdown();
  return 0;
}
