/**
 * @file network_tester.hpp
 * @brief header file for network_tester
 * @copyright 2024 TUMFTM
 */

#ifndef NETWORK_TESTER_HPP
#define NETWORK_TESTER_HPP

#include "tod_network_monitoring/config.hpp"

#include <string>
#include <optional>

struct BandwidthStats {
    long long bitrate_mbps; // megabits per second
    long long transferred_bytes;

    BandwidthStats() : bitrate_mbps(0), transferred_bytes(0) {}
};

/**
 * @brief Class to test network bandwidth and latency
 * @ingroup tod_network_monitoring
 */
class NetworkTester {
public:
    /**
     * @brief constructor
     * 
     */
    NetworkTester();
    
    /**
     * @brief measure latency
     * 
     * @param hostname 
     * @param update_time_interval 
     * @param ping_timeout 
     * @return std::optional<float> average latency as float
     */
    std::optional<float> test_latency(const std::string& hostname, const float& update_time_interval, const int& ping_timeout);

    /**
     * @brief measure bandwidth
     * 
     * @param hostname 
     * @param port 
     * @param duration_s amount of SECONDS the bandwidth test should be executed
     * @param is_reverse 0 = client sends, server receives, 1 = server sends, client receives
     * @return BandwidthStats struct
     */
    BandwidthStats test_bandwidth(const std::string& hostname, int port, int duration_s, bool is_reverse);

    /**
     * @brief destructor
     * 
     */
    ~NetworkTester();
};

#endif // NETWORK_TESTER_HPP