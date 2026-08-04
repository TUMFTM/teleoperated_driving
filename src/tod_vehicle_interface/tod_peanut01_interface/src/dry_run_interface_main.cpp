#include "rclcpp/rclcpp.hpp"
#include "tod_peanut01_interface/dry_run_interface_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tod_peanut01_interface::DryRunInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
