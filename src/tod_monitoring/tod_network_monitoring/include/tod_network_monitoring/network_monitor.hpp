/**
 * @file network_monitor.hpp
 * @brief header file for network_monitor
 * @copyright 2024 TUMFTM
 */

#ifndef NETWORK_MONITOR_HPP
#define NETWORK_MONITOR_HPP

#include "tod_network_monitoring/config.hpp"

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdint>

#define RALQ_DECAY 0.75 // value for speed of decay

struct NetworkData {
    unsigned long long rx_packets, tx_packets;
    unsigned long long rx_bytes, tx_bytes;

    NetworkData() : rx_packets(0), tx_packets(0), rx_bytes(0), tx_bytes(0) {}
};

struct NetworkMetrics {
    double rx_bitrate_mbps;
    double tx_bitrate_mbps;
    double rx_packets_s;
    double tx_packets_s;
    double latency;
    double link_quality;

    NetworkMetrics() : rx_bitrate_mbps(0.0), tx_bitrate_mbps(0.0), rx_packets_s(0.0), tx_packets_s(0.0), latency(0.0), link_quality(0.0) {}
};


struct LinkMetrics {
    double last_latency;
    double link_quality;
    uint16_t icmp_seq; // ICMP_SEQ_MAX_VALUE = 65535
    unsigned int total_packets_lost;

    LinkMetrics() : last_latency(0.0), link_quality(0.0), icmp_seq(0), total_packets_lost(0) {}
};

struct LinkEstimator {
    uint16_t last_seq;
    std::chrono::time_point<std::chrono::steady_clock> last_timestamp;
    double p_k;
    double q_k;
    double link_quality;

    LinkEstimator() : last_seq(0), last_timestamp(std::chrono::steady_clock::now()), p_k(0.0), q_k(0.0), link_quality(0.0) {}
};

/**
 * @brief Class for monitoring the network on a specific interface with information
 * @ingroup tod_network_monitoring
 */
class NetworkMonitor {
public:
    /**
     * @brief Constructor
     * 
     */
    NetworkMonitor();

   /**
    * @brief start monitoring the network on a specific interface.
    * provides information about latency, link quality, throughput, etc.
    * stores the data in network_metrics and other members
    * @param hostname 
    * @param interface 
    * @param update_timeout 
    * @param update_time_interval 
    * @return int =0, if success
    *             >0, otherwise
    */
    int start_monitoring(const std::string& hostname, const std::string& interface, const float update_timeout, const float update_time_interval);

   /**
    * @brief return the current monitoring status
    * 
    * @return int =0, if success
    *             >0, otherwise
    */
    int stop_monitoring();

   /**
    * @brief return the current monitoring status
    * 
    * @return false, if no capture is running
    * @return true, otherwise
    */
    bool is_monitoring();

   /**
    * @brief return the most recent metrics gathered
    * 
    * @return NetworkMetrics struct
    */
    NetworkMetrics get_network_metrics();

   /**
    * @brief return a human readable string representation of the NetworkMetrics struct
    * 
    * @param metrics 
    * @return string in a readable format
    */
    std::string format_network_metrics(const NetworkMetrics& metrics);

    /**
     * @brief Destructor
     * 
     */
    ~NetworkMonitor();

private:
    std::mutex data_mutex_;
    std::thread link_quality_thread_;
    std::thread throughput_thread_;

    std::atomic<bool> keep_running_ = true;

    NetworkMetrics network_metrics_;
    LinkMetrics link_metrics_;
    LinkEstimator link_estimator_;

   /**
    * @brief start monitoring a specific link and the corresponding metrics
    * 
    * @param hostname 
    * @param update_timeout 
    * @param update_time_interval 
    * @return int =0, if success
    *             >0, otherwise
    */
    int link_quality_monitoring(const std::string& hostname, const float update_timeout, const float update_time_interval);

   /**
    * @brief start monitoring the current throughput of the network
    * results based on information provided by the kernel
    * 
    * @param interface 
    * @return int =0, if success
    *             >0, otherwise
    */
    int throughput_monitoring(const std::string& interface);

    /**
     * @brief update the link quality estimator and
     * store the output in the link_estimator struct
     * 
     * @param new_seq 
     */
    void update_link_estimator(uint16_t new_seq);

    /**
     * @brief parse network data from /proc/net/dev
     * use internally in monitor_network_interface_throughput_thread
     * 
     * @param interface 
     * @return NetworkData struct
     */
    NetworkData parse_network_data(const std::string& interface);
};

#endif // NETWORK_MONITOR_HPP