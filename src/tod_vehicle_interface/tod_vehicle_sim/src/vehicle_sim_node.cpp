/**
 * @file vehicle_sim_node.cpp
 * @brief Simple kinematic bicycle model simulating a vehicle interface.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_vehicle_sim
 */

#include <memory>

#include "tod_vehicle_sim/vehicle_model.hpp"

#include "tod_core/param_set/VehicleParameters.hpp"

#include "tod_vehicle_msgs/msg/primary_control_cmd.hpp"
#include "tod_vehicle_msgs/msg/secondary_control_cmd.hpp"
#include "tod_vehicle_msgs/msg/primary_vehicle_state.hpp"
#include "tod_vehicle_msgs/msg/secondary_vehicle_state.hpp"
#include "tod_status_msgs/msg/status.hpp"

#include <rclcpp/rclcpp.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace tod_vehicle_sim {
/**
 * @ingroup tod_vehicle_sim
 */

using namespace std::chrono_literals;   // for ms

/**
 * @brief Vehicle interface emulation using a single track model.
 */
class VehicleSimNode : public rclcpp::Node
{
    public:
        VehicleSimNode() : Node("VehicleSimNode")
        {
            // set time
            prev_ = this->get_clock()->now();

            // create subscriptions
            primary_control_subs_ = this->create_subscription<tod_vehicle_msgs::msg::PrimaryControlCmd>(
                "actuation/to_actuation/primary_control_cmd", 10, [this](const tod_vehicle_msgs::msg::PrimaryControlCmd::SharedPtr msg){this->handle_primary_control(msg);});

            secondary_control_subs_ = this->create_subscription<tod_vehicle_msgs::msg::SecondaryControlCmd>(
                "actuation/to_actuation/secondary_control_cmd", 10, 
                [this](const tod_vehicle_msgs::msg::SecondaryControlCmd::SharedPtr msg){this->handle_secondary_control(msg);});

            status_subs_ = this->create_subscription<tod_status_msgs::msg::Status>(
                "input/vehicle_status", 10, 
                [this](const tod_status_msgs::msg::Status::SharedPtr msg){this->handle_status(msg);});

            // create publisher
            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("sensing/from_sensing/odom",10);
            primary_vehicle_state_pub_ = this->create_publisher<tod_vehicle_msgs::msg::PrimaryVehicleState>("actuation/from_actuation/primary_vehicle_state", 10);
            secondary_vehicle_state_pub_ = this->create_publisher<tod_vehicle_msgs::msg::SecondaryVehicleState>("actuation/from_actuation/secondary_vehicle_state", 10);

            // declare and get the config path
            this->declare_parameter<std::string>("config_path", "");
            std::string config_path;
            
            if(! this->get_parameter("config_path", config_path)) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Failed to retrieve 'config_path' parameter. Ensure it is set in the launch file.");
            }

            // create the parameter handler
            veh_param_handler_ =
                std::make_unique<tod_core::param_set::Vehicle>(this, config_path + "/vehicle_config/");

            // publish data every 10 ms
            timer_ = this->create_wall_timer(10ms, [this](){this->publish_data();});

            if (!this->get_parameter("vehicleID", vehicle_id_))
                RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << ": Could not set param /vehicleID - using "
                                                   << vehicle_id_);
            
            vehModel_.reset_initial_position(0.0, 0.0, 0.0); // x, y, z
        }

    private:
        // subcriptions
        rclcpp::Subscription<tod_vehicle_msgs::msg::PrimaryControlCmd>::SharedPtr primary_control_subs_;
        rclcpp::Subscription<tod_vehicle_msgs::msg::SecondaryControlCmd>::SharedPtr secondary_control_subs_;
        rclcpp::Subscription<tod_status_msgs::msg::Status>::SharedPtr status_subs_;
        
        // publisher
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<tod_vehicle_msgs::msg::PrimaryVehicleState>::SharedPtr primary_vehicle_state_pub_;
        rclcpp::Publisher<tod_vehicle_msgs::msg::SecondaryVehicleState>::SharedPtr secondary_vehicle_state_pub_;

        // timer
        rclcpp::TimerBase::SharedPtr timer_;

        // published data
        nav_msgs::msg::Odometry base_link_odom_;
        tod_vehicle_msgs::msg::PrimaryVehicleState primary_vehicle_state_msg_;
        tod_vehicle_msgs::msg::SecondaryVehicleState secondary_vehicle_state_msg_;

        // safe the time in each execution step
        rclcpp::Time prev_;

        // primary control commands
        float swa_ = 0;
        float velocity_des_ = 0;
        double integral_ = 0.0;
        double output_ = 0.0;
        double velocity_input_ = 0.0;

        // vehicle parameter handler
        std::unique_ptr<tod_core::param_set::Vehicle> veh_param_handler_;
        std::string vehicle_id_{"edgar"};
        
        // vehicle model
        VehicleModel vehModel_;

        // callback when primary control signals are received
        void handle_primary_control(const tod_vehicle_msgs::msg::PrimaryControlCmd::SharedPtr msg) {
            swa_          = msg->steering_wheel_angle;
            velocity_des_ = msg->velocity;
        }
        
        // callback when secondary control signals are received
        void handle_secondary_control(const tod_vehicle_msgs::msg::SecondaryControlCmd::SharedPtr msg) {
            secondary_vehicle_state_msg_.header.stamp  = this->get_clock()->now();
            secondary_vehicle_state_msg_.honk          = msg->honk;
            secondary_vehicle_state_msg_.wiper         = msg->wiper;
            secondary_vehicle_state_msg_.head_light    = msg->head_light;
            secondary_vehicle_state_msg_.indicator     = msg->indicator;
            secondary_vehicle_state_msg_.flash_light   = msg->flash_light;
            secondary_vehicle_state_msg_.gear_position = msg->gear_position;
        }
        
        // callback when status messages are received
        void handle_status(const tod_status_msgs::msg::Status::SharedPtr msg) {
            static uint8_t prevConnStatus{tod_status_msgs::msg::Status::TOD_STATUS_IDLE};
            uint8_t currConnStatus = msg->tod_status;
            if (currConnStatus == tod_status_msgs::msg::Status::TOD_STATUS_IDLE
                && prevConnStatus != tod_status_msgs::msg::Status::TOD_STATUS_IDLE) {
                // reset vehicle position on disconnect
                // vehModel_.reset_initial_position(0.0, 0.0, 0.0);
            }
            prevConnStatus = currConnStatus;
        }

        /*
        * @brief Takens input commands from the operator / control concept and translate that into vehicle movement and data
        */
        void publish_data() {
            // Update vehModel in case vehicleID changed
            vehModel_.set_params(veh_param_handler_->get_distance_front_axle(), 
                                veh_param_handler_->get_distance_rear_axle(), 
                                veh_param_handler_->get_max_rwa_rad(), 
                                veh_param_handler_->get_max_swa_rad());

            // get the current time
            rclcpp::Time curr = rclcpp::Time(this->get_clock()->now());

            // calculate the vehicle motion
            calculate_vehicle_motion((curr-prev_).seconds());

            // calculate and publish odometry for base_link
            // convention: base_link is middle of rear axle for ackermann steering vehicles
            base_link_odom_.header.stamp     = this->get_clock()->now();
            base_link_odom_.header.frame_id  = "map";
            base_link_odom_.child_frame_id   = "base_link";  
            
            // pose is given in the global coordinate frame (here "map")
            base_link_odom_.pose.pose.position.x = vehModel_.get_x()
                - veh_param_handler_->get_distance_rear_axle() * std::cos(vehModel_.get_psi());
            base_link_odom_.pose.pose.position.y = vehModel_.get_y()
                - veh_param_handler_->get_distance_rear_axle() * std::sin(vehModel_.get_psi());
            base_link_odom_.pose.pose.position.z = 0.0;
            
            tf2::Quaternion quaternion;
            quaternion.setRPY(0.0, 0.0, vehModel_.get_psi());
            base_link_odom_.pose.pose.set__orientation(tf2::toMsg(quaternion));
            
            // twist is given in the vehicles fixed frame (here "child_frame_id")
            base_link_odom_.twist.twist.linear.x  = vehModel_.get_velocity_x();
            base_link_odom_.twist.twist.linear.y  = vehModel_.get_velocity_y();
            base_link_odom_.twist.twist.linear.z  = 0.0;
            base_link_odom_.twist.twist.angular.x = 0.0;
            base_link_odom_.twist.twist.angular.y = 0.0;
            base_link_odom_.twist.twist.angular.z = vehModel_.get_psi_p();
    
            // publish the data
            primary_vehicle_state_pub_->publish(primary_vehicle_state_msg_);
            secondary_vehicle_state_pub_->publish(secondary_vehicle_state_msg_);
            odom_pub_->publish(base_link_odom_);

            // safe the time for the next iteration
            prev_ = curr;
        }

        /*
        * @brief Simple PI controller to simulate the vehicle motion
        */
        void calculate_vehicle_motion(double dt_s) {
            // PI Controller for velocity to duplicate EDGAR behavior
            double error_ = velocity_des_ - vehModel_.get_velocity_x();
            double kp = 2.5;
            double ki = 1;
            double integral_ = integral_ + error_ * dt_s;
            double output_ = kp * error_ + ki * integral_;
            velocity_input_ += output_ * dt_s;
                        
            vehModel_.update_position(velocity_input_, swa_, secondary_vehicle_state_msg_.gear_position, dt_s);
            primary_vehicle_state_msg_.header.stamp = this->get_clock()->now();
            primary_vehicle_state_msg_.velocity = float(vehModel_.get_velocity_x());
            if (secondary_vehicle_state_msg_.gear_position == eGearPosition::GEARPOSITION_REVERSE) {
                primary_vehicle_state_msg_.velocity *= (-1.0f);
            }
            primary_vehicle_state_msg_.acceleration = vehModel_.get_acceleration_x();
            primary_vehicle_state_msg_.steering_wheel_angle = float(tod_helper::Vehicle::Model::rwa2swa(
                vehModel_.get_steering_angle(), 
                veh_param_handler_->get_max_swa_rad(), 
                veh_param_handler_->get_max_rwa_rad()));
        }
};

} // namespace tod_vehicle_sim

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tod_vehicle_sim::VehicleSimNode>());
    rclcpp::shutdown();
    return 0;
}