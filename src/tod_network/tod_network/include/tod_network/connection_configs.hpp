/**
 * @file connection_configs.hpp
 * @author Nils Gehrke
 * @brief Config for network connections
 * @version 1.0
 *
 * @copyright TUMFTM 2024
  */
#pragma once
#include <string>

namespace tod_network {

static const int RX_MQTT = 1883; // same for operator and vehicle
static const std::string AutoboxIp = "192.168.140.6"; // not for open source

enum VehiclePorts { // only what is sent via udp
    RX_PRIMARYCONTROL_COMMAND = 70000,
    RX_SAFECORRIDORCONTROL_COMMAND = 70001,             // NOT FOR OPEN SOURCE
    RX_OPERATOR_STATUS = 70002,
    RX_SECONDARY_COMMAND = 70003,
    RX_PATHGUIDANCE_REQUEST = 70020,                    // NOT FOR OPEN SOURCE
    RX_PATHGUIDANCE_APPROVAL = 70021,                   // NOT FOR OPEN SOURCE
    RX_PERCMOD_REQUEST = 55030,                         // PERCEPTION MODIFICATION
    RX_PERCMOD_VELOCITY_CHANGE = 55032,                 // PERCEPTION MODIFICATION
    RX_PERCMOD_APPROVAL = 55031,                        // PERCEPTION MODIFICATION
    RX_SAFETY_DRIVER_STATUS_AUTOBOX = 60000,
    RX_VEHICLEDATA_AUTOBOX = 60001,
    RX_VIDEO_RTSP = 8554,
    RX_VEHICLE_AUDIO = 62000,                   // NOT FOR OPEN SOURCE
    RX_PATHGUIDANCE_PATH_VEHICLE = 53000 ,
    RX_TRAJECTORY_GUIDANCE_STATE_VEHICLE = 54000
};

enum AutoboxPorts {
    RX_DIRECTCONTROL_AUTOBOX_COMMAND = 30000
};

// Separate from vehicle/operator construct since interaction could be other way around as well
enum ServicePorts {
    // Always needs 2 ports - reciever @Niklas TODO: Add port range for services
    RX_FORWARD_RECEIVE_VIDEOCONFIG = 60100,  // Sender Receiver Communication
    RX_RESPOND_LISTEN_VIDEOCONFIG =  60101, //  Responder Listener Communcation
    RX_FORWARD_RECEIVE_VIDEOPARAMETER = 60200,  // Sender Receiver Communication
    RX_RESPOND_LISTEN_VIDEOPARAMETER =  60201, //  Responder Listener Communcation
    RX_FORWARD_RECEIVE_MONITORSTATUS =  60300, //  Sender Receiver Communcation
    RX_RESPOND_LISTEN_MONITORSTATUS =  60301, //  Responder Listener Communcation
    RX_FORWARD_RECEIVE_PACKETCAPTURE =  60400, //  Sender Receiver Communcation
    RX_RESPOND_LISTEN_PACKETCAPTURE =  60401, //  Responder Listener Communcation
    RX_FORWARD_RECEIVE_MONITOR_SERVICE =  60500, //  Responder Listener Communcation
    RX_RESPOND_LISTEN_MONITOR_SERVICE =  60501, //  Responder Listener Communcation
};


enum OperatorPorts { // only what is sent via udp
    RX_LIDAR_OBJECTLIST = 50000,
    // 50001 occupied on CarPC4  - NOT FOR OPEN SOURCE
    RX_VEHICLESTATE_VEHICLEDATA = 50002,
    RX_VEHICLE_STATUS = 50003,
    RX_VEHICLESTATE_ODOMETRY = 50005,
    RX_VEHICLESTATE_GPS = 50006,
    RX_SHAREDCONTROL_POLYGON = 50007,                   // NOT FOR OPEN SOURCE
    RX_SHAREDCONTROL_COMMAND = 50008,                   // NOT FOR OPEN SOURCE
    RX_SHAREDCONTROL_MPC_LOG = 50009,                   // NOT FOR OPEN SOURCE
    RX_SHAREDCONTROL_OBJECTS = 50010,                   // NOT FOR OPEN SOURCE
    RX_BITRATE_PREDICTIONS = 50011,
    RX_SHAREDCONTROL_SHADOW_COMMAND = 50012,            // NOT FOR OPEN SOURCE
    RX_SHAREDCONTROL_SVC_LOG = 50013,                   // NOT FOR OPEN SOURCE
    RX_PATHGUIDANCE_RESPONSE = 50020,                   // NOT FOR OPEN SOURCE
    RX_PERCMOD_RESPONSE = 50030,                        // PERCEPTION MODIFICATION
    RX_PERCMOD_GRIDMAP = 50031,                         // PERCEPTION MODIFICATION
    RX_PERCMOD_OBJECTS = 50032,                         // PERCEPTION MODIFICATION
    RX_GATE_STATE = 50040,
    RX_LIDAR_DATA_RANGE_FROM = 50100,
    RX_LIDAR_DATA_RANGE_TO = 50199,
    RX_LIDAR_OBJECTS_RANGE_FROM = 50200,
    RX_LIDAR_OBJECTS_RANGE_TO = 50299,
    RX_LIDAR_OBJECT_MARKER_RANGE_FROM = 50300,
    RX_LIDAR_OBJECT_MARKER_RANGE_TO = 50399,
    RX_ENV_MODEL_RANGE_FROM = 50400,
    RX_ENV_MODEL_RANGE_TO = 50499,
    RX_AUTOWARE_LANE = 51010,                           // NOT FOR OPEN SOURCE
    RX_OPERATOR_AUDIO = 52000,                          // NOT FOR OPEN SOURCE
    RX_ROUTE = 52100,                                    // PERCEPTION MODIFICATION
    RX_NETWORK_METRICS = 52200,
    RX_TRAJECTORY_GUIDANCE_STATE_OPERATOR = 54001
};

/* DEPRECATED
namespace MqttTopics {
static const std::string DesiredVideoConfig{"/Operator/Video/DesiredVideoConfig"};
static const std::string ActualVideoConfig{"/Vehicle/Video/ActualVideoConfig"};
static const std::string DesiredBitrateConfig{"/Operator/Video/DesiredBitrateConfig"};
static const std::string ActualBitrateConfig{"/Vehicle/Video/ActualBitrateConfig"};
};*/

}; // namespace tod_network
