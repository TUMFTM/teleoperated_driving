/**
 * @file tod_service_forwarder.hpp
 * @author Simon Hoffmann
 * @brief This file contains the implementation of a service forwarder class that forwards ROS service requests between a sender and a receiver over TCP. The class handles synchronous and asynchronous service forwarding based on the connection status, and it includes mechanisms for connecting, disconnecting, and handling service requests and responses.
 * @version 1.0
 *
 * @copyright TUMFTM 2020
  */
#pragma once

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <tod_status_msgs/msg/status.hpp>
#include <tod_network_protocols/base_protocol.hpp>
#include <tod_network/tod_service_utils.hpp>

namespace tod_network {

using std::placeholders::_1;
using std::placeholders::_2;

template <typename ServiceType>
class ServiceForwarder : public rclcpp::Node{
    
public:
    /**
     * @brief Construct a new Service Forwarder object
     * 
     * @param[in] node_name name of the ros node
     * @param[in] service_name name of the service
     * @param[in] forwarder_in_vehicle whether the forwarder is in vehicle
     * @param[in] sender unique ptr to the sender
     * @param[in] receiver unqiue ptr to the receiver
     */
    ServiceForwarder(const std::string& node_name, const std::string& service_name, bool forwarder_in_vehicle, std::unique_ptr<tod_network_protocols::BaseSender>&& sender, std::unique_ptr<tod_network_protocols::BaseReceiver>&& receiver)
        :Node(std::string("service_forwarder")), _sender(std::move(sender)), _receiver(std::move(receiver)), _service_name(service_name), _forwarder_in_vehicle(forwarder_in_vehicle) {
        // create status topic subscriber
        std::string statusTopic = (_forwarder_in_vehicle) ? "input/vehicle_status" : "input/operator_status";
        _status_subs = this->create_subscription<tod_status_msgs::msg::Status>(statusTopic, 1, std::bind(&ServiceForwarder::status_message_received, this, _1));

        // Set up proxy service
        _service = this->create_service<ServiceType>(
            service_name,
            std::bind(&ServiceForwarder<ServiceType>::service_callback, this, _1, _2)
            // &service_callback
        );
        RCLCPP_INFO_STREAM(this->get_logger(), "Service: " << service_name << " is set up.");
    }


    /**
     * @brief Destroy the Service Forwarder New object
     * 
     */
    ~ServiceForwarder() {
        disconnect();
    }
    

    /**
     * @brief ros service callback for service forwarding
     * 
     * @param request the to-be-forwarded request
     * @param response the response from the receiver
     */
    void service_callback(const std::shared_ptr<typename ServiceType::Request> request, std::shared_ptr<typename ServiceType::Response> response) {
        // Exit the callback early if the service forwarder is not connected
        if (!_connected) {
            RCLCPP_ERROR(this->get_logger(), "Service is not connected. Unable to process request.");
            return;
        }

        // Serialize the service request and send it with the sender
        auto serializedRequest = serialize<typename ServiceType::Request>(*request);
        int nofBytesSent = this->_sender->send_data(serializedRequest);
        RCLCPP_INFO(this->get_logger(), "Sending service request, sent %d bytes", nofBytesSent);

        /* ==================
        Synchronous receiving
        ================== */
        try {
            std::vector<uint8_t> data = _receiver->receive();
            if (!data.empty()) {
                    if (deserialize<typename ServiceType::Response>(data, response.get()) == 1) 
                    {
                        RCLCPP_INFO(this->get_logger(), "Received service response");
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Failed to deserialize response");
                    }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Empty response data.");
            }
        } catch(const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Receiver lost connection.");
            disconnect();
            return;
        }
    }

    /**
     * @brief Connect the service forwarder
     * 
     * @param[in] listener_ip IP address of the listener
     * @return int 0 (if success), 1 (if failure)
     */
    int connect(const std::string& listener_ip){
        _listener_ip = listener_ip;
        /*
        synchronously connecting to the listener's receiver
        Retrying 10 times (default)
        If it cannot connect to listener's receiver, it prints out error msg and returns.
        */
        bool sender_connected {false};  // whether sender is connected
        for (uint8_t i=0; i < _connect_try; i++){
            try {
                _sender->change_destination(listener_ip);
                sender_connected = true;
                break;
            } catch (std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Service listener receiver is not available, retrying after 100ms. Error msg: %s", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        // Print out error msg and return, if the sender cannot connect to listener's receiver
        if (!sender_connected) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Service [" << _service_name << "] failed to connect to listener's receiver.");
            
            return 1;
        }

        // waiting the listener's sender to connect to forwarder's receiver
        RCLCPP_INFO_STREAM(this->get_logger(), "Service [" << _service_name << "] is waiting for listener's sender to connec to forwarder's receiver...");
        _receiver->waiting_for_client_connect();
        RCLCPP_INFO_STREAM(this->get_logger(), "Service [" << _service_name << "] is connected to listener's sender.");

        _connected = true;
        return 0;
    }


    /**
     * @brief Disconnect the service forwarder
     * 
     */
    void disconnect() {
        _sender->disconnect();
        _receiver->disconnect();
        _connected = false;
    }


    /**
     * @brief Get the connection state of the service forwarder
     * 
     * @return true if connected
     * @return false if disconnected
     */
    bool is_connected() {
        return _connected; 
    } 


private:
    /**
     * @brief Subscription callback of the status msg
     * Only when the tod_vehicle and the tod_operator are connected, the service is then forwardered.
     * 
     * @param[in] msg 
     */
    void status_message_received(const tod_status_msgs::msg::Status& msg) {
        bool new_connection_status = msg.tod_status == tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION || msg.tod_status == tod_status_msgs::msg::Status::TOD_STATUS_UPLINK_ONLY;  // true if new connection status is connected
        uint8_t new_control_mode = _forwarder_in_vehicle ? msg.vehicle_control_mode : msg.operator_control_mode;
        std::string new_listener_ip = _forwarder_in_vehicle ? msg.operator_ip_address : msg.vehicle_ip_address;

        // When the connection status is different
        if (new_connection_status != _connected) {
            if (!_connected) {
                connect(new_listener_ip);
            } else {
                disconnect();
            }
        }

        // When the listener IP is different
        if (_connected && new_listener_ip != _listener_ip) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Changing to new IP: " << new_listener_ip << std::endl);
            disconnect();
            connect(new_listener_ip);
        }
    }


    /**
     * @brief unqiue ptr to the sender
     * 
     */
    std::unique_ptr<tod_network_protocols::BaseSender> _sender{nullptr};


    /**
     * @brief unique ptr to the receiver
     * 
     */
    std::unique_ptr<tod_network_protocols::BaseReceiver> _receiver{nullptr};


    /**
     * @brief shared ptr to the ros service
     * 
     */
    typename rclcpp::Service<ServiceType>::SharedPtr _service{nullptr};


    /**
     * @brief shared ptr to the status subscriber
     * 
     */
    rclcpp::Subscription<tod_status_msgs::msg::Status>::SharedPtr _status_subs;


    /**
     * @brief name the service
     * 
     */
    std::string _service_name;


    /**
     * @brief connection status of the service forwarder
     * 
     */
    bool _connected{false};


    /**
     * @brief whether the forwarder is in vehicle
     * 
     */
    bool _forwarder_in_vehicle;


    /**
     * @brief How many times the service forwarder try to connect
     * 
     */
    uint8_t _connect_try{10};


    /**
     * @brief IP address of the service listener
     * 
     */
    std::string _listener_ip {"127.0.0.1"};  // default localhost

};  // class ServiceForwarder

}  // namespace tod_network
