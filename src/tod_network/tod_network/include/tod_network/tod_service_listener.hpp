/**
 * @file tod_service_listener.hpp
 * @author Simon Hoffmann
 * @brief This file contains the implementation of a service listener class that listens to ROS service requests between a sender and a receiver over TCP. The class handles synchronous and asynchronous service forwarding based on the connection status, and it includes mechanisms for connecting, disconnecting, and handling service requests and responses.
 * @version 1.0
 *
 * @copyright TUMFTM 2020
  */
#pragma once

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <tod_status_msgs/msg/status.hpp>
#include <tod_network_protocols/base_protocol.hpp>
#include <tod_network/tod_service_utils.hpp>


namespace tod_network {

using std::placeholders::_1;
using std::placeholders::_2;

template <typename ServiceType>
class ServiceListener: public rclcpp::Node {

public:
    /**
     * @brief Construct a new Service Listener object
     * 
     * @param[in] node_name name of the ros node
     * @param[in] service_name name of the ros service
     * @param[in] listener_in_vehicle whether the listener is in vehicle
     * @param[in] sender unique ptr to the sender
     * @param[in] receiver unqiue ptr to receiver
     */
    ServiceListener(const std::string& node_name, const std::string& service_name, bool listener_in_vehicle, std::unique_ptr<tod_network_protocols::BaseSender>&& sender, std::unique_ptr<tod_network_protocols::BaseReceiver>&& receiver)
        :Node(std::string("service_listener")), _sender(std::move(sender)), _receiver(std::move(receiver)), _service_name(service_name), _listener_in_vehicle(listener_in_vehicle) {
        std::string status_topic = (_listener_in_vehicle) ? "input/vehicle_status" : "input/operator_status";
        _status_subs = this->create_subscription<tod_status_msgs::msg::Status>(status_topic, 1, std::bind(&ServiceListener::status_message_received, this, _1));
        RCLCPP_INFO_STREAM(this->get_logger(), "Service: " << service_name << " is set up.");

        try {
            _client= this->create_client<ServiceType>(service_name);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in creating client: %s, service will be unavailable", e.what());
        }
    }


    /**
     * @brief Destroy the Service Listener New object
     * 
     */
    ~ServiceListener() { stop_listening(); }


    /**
     * @brief Setup the listening thread
     * 
     */
    void start_listening() {
        if (!_thread) {
            _thread = std::make_unique<std::thread>([this]() { listen(); });
        }
    }


    /**
     * @brief Stop the listening thread
     * 
     */
    void stop_listening() {
        if (_thread && _thread->joinable()) {
            _thread->join();
        }
        _thread = nullptr;
    }


    /**
     * @brief Connect the listener
     * 
     * @param[in] forwarder_ip IP address of the service forwarder
     * @return int, 0 if success, 1 if failure
     */
    int connect(const std::string& forwarder_ip){
        _forwarder_ip = forwarder_ip;
        // Waiting the forwarder's sender to connect to the receiver
        RCLCPP_INFO_STREAM(this->get_logger(), "Service [" << _service_name << "] is waiting for the forwarder's sender to connect...");
        _receiver->waiting_for_client_connect();
        RCLCPP_INFO_STREAM(this->get_logger(), "Service [" << _service_name << "] is connected to forwarder's sender.");

        /*
        synchronously connecting to the listener's receiver
        Retrying 10 times (default)
        If it cannot connect to listener's receiver, it prints out error msg and returns.
        */
        bool sender_connected {false};
        for (uint8_t i=0; i < _connect_retry; i++){
            try {
                _sender->change_destination(forwarder_ip);
                sender_connected = true;
                break;
            } catch (std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Service forwarder receiver is not available, retrying after 200ms. Error msg: %s", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        }

        if (!sender_connected) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Service [" << _service_name << "] failed to connect to forwarder's receiver.");
            return 1;
        }
        
        if (!_thread) {
            start_listening(); 
        }
        _connected = true;
        RCLCPP_INFO_STREAM(this->get_logger(), "Service [" << _service_name << "] is connected.");
        return 0;
    }


    /**
     * @brief Disconnect the listener
     * 
     */
    void disconnect() {
        _receiver->disconnect();
        _sender->disconnect();
        _connected = false;
    }



    /**
     * @brief Get the connection status of the listener
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
    void status_message_received(const tod_status_msgs::msg::Status &msg) {
        bool new_connection_status = msg.tod_status == tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION || msg.tod_status == tod_status_msgs::msg::Status::TOD_STATUS_UPLINK_ONLY;  // true if new connection status is connected
        // uint8_t new_control_mode = _listener_in_vehicle ? msg.vehicle_control_mode : msg.operator_control_mode;
        std::string new_forwarder_ip = _listener_in_vehicle ? msg.operator_ip_address : msg.vehicle_ip_address;

        // When the connection status is different
        if (new_connection_status != _connected) {
            if (!_connected) {
                connect(new_forwarder_ip);
            } else {
                disconnect();
                stop_listening();
            }
        }

        // When the listener IP is different
        if (_connected && new_forwarder_ip != _forwarder_ip) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Changing to new IP: " << new_forwarder_ip << std::endl);
            disconnect();
            stop_listening();
            connect(new_forwarder_ip);
        }
    }

    /**
     * @brief Spin up a thread to continously listen to the port and receive msgs
     * 
     */
    void listen() {
        while (rclcpp::ok()) {
            try {
                if (!_client->wait_for_service(std::chrono::seconds(1))) {
                    RCLCPP_ERROR(this->get_logger(), "Service %s not available", _service_name.c_str());
                }

                if (!_connected) {
                    RCLCPP_WARN(this->get_logger(), "Service listener is not connected.");
                    break;
                }

                /*
                Try to catch runtime error from tcp-based receiver
                tcp-based receiver will throw a std::runtime_error (EOF) if the socket is broken.
                */
                std::vector<uint8_t> data;
                try {
                    data = _receiver->receive();
                } catch (std::runtime_error& e) {
                    RCLCPP_ERROR(this->get_logger(), "Listener's receiver is disconnected and cannot get service request. ");
                    disconnect();
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    continue;
                }
                auto request = std::make_shared<typename ServiceType::Request>(); 
                
                try {
                    if (deserialize<typename ServiceType::Request>(data, request.get()) == 1) {
                        RCLCPP_INFO(this->get_logger(), "Received service request");
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Failed to deserialize request");
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Service callback exception: %s", e.what());
                }
                auto future_response = _client->async_send_request(request);
                RCLCPP_INFO(this->get_logger(), "Send Request to Vehicle Node for %s service response", _service_name.c_str());


                if (future_response.wait_for(_service_timeout) == std::future_status::ready) {
                    auto response = future_response.get();
                    auto serializedResponse = serialize<typename ServiceType::Response>(*response); // response should be shared_ptr
                    int nofBytesSent = _sender->send_data(serializedResponse);
                    RCLCPP_INFO(this->get_logger(), "Service %s response sent with msg size: %d", _service_name.c_str(),nofBytesSent); 
                } 
                else {
                    // error handling by service forwarder 
                    RCLCPP_ERROR(this->get_logger(), "Failed to get  %s service response:", _service_name.c_str());
                    // return an empty response
                    auto response = std::make_shared<typename ServiceType::Response>();
                    auto serializedResponse = serialize<typename ServiceType::Response>(*response); // response should be shared_ptr
                    int nofBytesSent = _sender->send_data(serializedResponse);
                    RCLCPP_INFO(this->get_logger(), "Service %s response sent with msg size: %d", _service_name.c_str(),nofBytesSent); 
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error in setupListen: %s", e.what());
            }
        }
    }


    /**
     * @brief unique ptr to the receiver
     * 
     */
    std::unique_ptr<tod_network_protocols::BaseReceiver> _receiver{nullptr};


    /**
     * @brief unique ptr to the sender
     * 
     */
    std::unique_ptr<tod_network_protocols::BaseSender> _sender{nullptr};


    /**
     * @brief unique ptr to the listening thread
     * 
     */
    std::unique_ptr<std::thread> _thread{nullptr};


    /**
     * @brief name of the service
     * 
     */
    std::string _service_name;


    /**
     * @brief state whether the listener is connected
     * 
     */
    bool _connected{false};


    /**
     * @brief How many times does the listener try to reconnect
     * 
     */
    uint8_t _connect_retry {10};


    /**
     * @brief ros service timeout
     * 
     */
    std::chrono::duration<int> _service_timeout = std::chrono::seconds(3);


    /**
     * @brief whether listener is in vehilce
     * 
     */
    bool _listener_in_vehicle;


    /**
     * @brief shared ptr to the ros service client
     * 
     */
    typename rclcpp::Client<ServiceType>::SharedPtr _client{nullptr};


    /**
     * @brief shared ptr to the status subscriber
     * 
     */
    rclcpp::Subscription<tod_status_msgs::msg::Status>::SharedPtr _status_subs;


    /**
     * @brief IP address of the service forwarder
     * 
     */
    std::string _forwarder_ip{"127.0.0.1"};  // default localhost

};  // class ServiceListener

}  // namespace tod_network
