/**
 * @file filter_definitions.hpp
 * @brief filter definitions for libpcap
 * @copyright 2024 TUMFTM
 */

#ifndef FILTER_DEFINITIONS_HPP
#define FILTER_DEFINITIONS_HPP

#include <string>

namespace filter_config {

   /*
        PORT MAPPING:

        554/TCP   -> RTSP
        554/UDP   -> RTSP
        1883/TCP  -> MQTT
        5201/TCP  -> IPERF3_PORT
        ICMP Type 8 -> ICMP_ECHO_REQUEST
        ICMP Type 0 -> ICMP_ECHO_REPLY
   */

    const std::string DEFAULT_IP_FILTER = "ip";
    const std::string TCP_UDP_FILTER = "ip and (tcp or udp)";
    const std::string RTSP_MQTT_FILTER = "ip and (tcp or udp) and (port 554 or port 1883)";
    const std::string TCP_UDP_FILTER_EXCLUDING_IPERF3 = "(tcp or udp or icmp) and not (tcp port 5201)";
    const std::string TCP_UDP_FILTER_EXCLUDING_PING_AND_IPERF3 = "ip and (tcp or udp) and not (tcp port 5201 or icmp[icmptype] == 8 or icmp[icmptype] == 0)";
    const std::string RTSP_MQTT_FILTER_EXCLUDING_PING_AND_IPERF3 = "ip and (tcp or udp) and (port 554 or port 1883) and not (tcp port 5201 or icmp[icmptype] == 8 or icmp[icmptype] == 0)";

    // set the active filter to be used by libpcap
    const std::string ACTIVE_FILTER = TCP_UDP_FILTER_EXCLUDING_PING_AND_IPERF3;
}

#endif // FILTER_DEFINITIONS_HPP