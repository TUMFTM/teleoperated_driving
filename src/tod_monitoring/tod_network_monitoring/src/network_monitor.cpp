/**
 * @file network_monitor.cpp
 * @brief to monitor KPIs during operation like
 *        - avg latency
 *        - packet loss
 *        - current throughput
 *
 *        this component may run as a background service
 *        DURING ACTIVE OPERATION
 * @copyright 2024 TUMFTM
 */

#include "tod_network_monitoring/network_monitor.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <chrono>
#include <iomanip>
#include <thread>
#include <mutex>
#include <deque>
#include <regex>
#include <memory>
#include <array>
#include <vector>
#include <cstdint>
#include <math.h>

//================================================================================
// Public Functions
//================================================================================

NetworkMonitor::NetworkMonitor() {}

NetworkMonitor::~NetworkMonitor() {

    if (throughput_thread_.joinable()) {
        throughput_thread_.join();
    }
    if (link_quality_thread_.joinable()) {
        link_quality_thread_.join();
    }
}

int NetworkMonitor::start_monitoring(const std::string& hostname, const std::string& interface, const float update_timeout, const float update_time_interval) {

    try {
        keep_running_.store(true);

        std::cout << "starting network monitoring with the following parameters:\n"
                  << "hostname: " << hostname << "\n"
                  << "interface: " << interface << "\n"
                  << "update timeout: " << std::fixed << std::setprecision(2) << update_timeout << " seconds\n"
                  << "update time interval: " << std::fixed << std::setprecision(2) << update_time_interval << " seconds\n"
                  << "monitoring is now active..." << std::endl;

        link_quality_thread_ = std::thread(&NetworkMonitor::link_quality_monitoring, this, hostname, update_timeout, update_time_interval);
        throughput_thread_ = std::thread(&NetworkMonitor::throughput_monitoring, this, interface);

        return 0;

    } catch (const std::system_error& e) {
        std::cerr << "failed to start monitoring threads: " << e.what() << std::endl;

        return 1;
    }

}

int NetworkMonitor::link_quality_monitoring(const std::string& hostname, const float update_timeout, const float update_time_interval) {

    const std::regex success_pattern("(icmp_seq=)([0-9]+)(.*)(time=)([0-9.]+)", std::regex::optimize);
    const std::regex error_pattern("(Destination Host Unreachable|Unknown host|Name or service not known)", std::regex::optimize);

    // restart ping if something went downhill
    while (keep_running_.load()) {

        std::string ping_cmd = "ping -W " + std::to_string(update_timeout) + " -i " + std::to_string(update_time_interval) + " " + hostname;
        FILE* pipe = popen(ping_cmd.c_str(), "r");

        if (!pipe) {
            std::cerr << "error starting ping - restarting" << std::endl;
            continue;
        }

        char buffer[256];
        while (keep_running_.load() && fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            std::string output = buffer;

            // check buffer integrity
            if (output.empty() || output.find('\n') == std::string::npos) {
                std::cerr << "corrupt data read from pipe" << std::endl;
                continue;
            }

            std::smatch match;

            if (std::regex_search(output, match, success_pattern) && match.size() > 5) {
                try {
                    unsigned long icmp_seq_tmp = std::stoul(match.str(2));

                    /*
                        NOTE
                        - when related ICMP sequence numbers rise above 65535,
                        they exceed capacity of an uint16_t,
                        leading to an integer overflow.
                        - this is perfectly fine, as RALQ relies on modulo operations.
                        - the 'ping' utility also stores icmp_seq in 16bit unsigned integers,
                        therefore both icmp_seq values are aligned.
                        ->> 'the poor devs modulo trick'
                    */

                    std::lock_guard<std::mutex> guard(data_mutex_);

                    link_metrics_.icmp_seq = static_cast<uint16_t>(icmp_seq_tmp);
                    link_metrics_.last_latency = std::stod(match.str(5));

                    update_link_estimator(link_metrics_.icmp_seq);

                    network_metrics_.latency = link_metrics_.last_latency;
                    network_metrics_.link_quality = link_metrics_.link_quality;

                } catch (const std::exception& e) {
                    std::cerr << "error parsing ping output: " << e.what() << std::endl;
                }
            } else if (std::regex_search(output, match, error_pattern)) {
                std::cerr << "ping error: " << output << std::endl;
                break;  // breaking here will restart the command
            }
        }

        if (pclose(pipe) == -1) {
            std::cerr << "error closing pipe" << std::endl;
            return 1;
        }
    }

    return 0;
}

int NetworkMonitor::throughput_monitoring(const std::string& interface) {

    while (keep_running_.load()) {
        auto start = std::chrono::steady_clock::now();

        NetworkData start_data = parse_network_data(interface);

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(UPDATE_TIME_INTERVAL * 1000)));

        NetworkData end_data = parse_network_data(interface);

        // calculate elapsed time in seconds
        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();

        // ensure elapsed time is not zero to avoid division by zero (bit sketchy)
        if (elapsed == 0) {
            elapsed = 1;
        }

        double rx_bytes_diff = static_cast<double>(end_data.rx_bytes - start_data.rx_bytes);
        double tx_bytes_diff = static_cast<double>(end_data.tx_bytes - start_data.tx_bytes);

        // calculate rates
        double rx_bytes_rate = rx_bytes_diff / elapsed / 1024; // convert to KBps
        double tx_bytes_rate = tx_bytes_diff / elapsed / 1024; // convert to KBps
        double rx_packets_rate = static_cast<double>(end_data.rx_packets - start_data.rx_packets) / elapsed;
        double tx_packets_rate = static_cast<double>(end_data.tx_packets - start_data.tx_packets) / elapsed;

        std::lock_guard<std::mutex> guard(data_mutex_);

        // convert bytes to bits for bitrate and handle conversion to Mbps if necessary
        network_metrics_.rx_bitrate_mbps = (rx_bytes_diff / elapsed * 8) / 1e6; // convert to Mbps
        network_metrics_.tx_bitrate_mbps = (tx_bytes_diff / elapsed * 8) / 1e6; // convert to Mbps
        network_metrics_.rx_packets_s = rx_packets_rate;
        network_metrics_.tx_packets_s = tx_packets_rate;
    }

    return 0;
}

int NetworkMonitor::stop_monitoring() {

    if (!this->is_monitoring()) {
        std::cerr << "no monitoring running. call start_monitoring() to open a new session" << std::endl;
        return 1;
    }

    // stop monitoring and join both threads
    keep_running_.store(false);
    link_quality_thread_.join();
    throughput_thread_.join();

    // reset
    link_metrics_ = LinkMetrics();
    link_estimator_ = LinkEstimator();
    network_metrics_ = NetworkMetrics();

    std::cout << "succesfully joined threads and stopped monitoring" << std::endl;

    return 0;
}

bool NetworkMonitor::is_monitoring() {

    // monitoring is already running, if at least one thread is joinable
    if (link_quality_thread_.joinable() || throughput_thread_.joinable()) {
        return 1;
    }

    return 0;
}

NetworkMetrics NetworkMonitor::get_network_metrics() {

    std::lock_guard<std::mutex> guard(data_mutex_);

    return this->network_metrics_;
}

std::string NetworkMonitor::format_network_metrics(const NetworkMetrics& metrics) {
    std::ostringstream oss;
    oss << "Network Metrics:\n"
        << "Receive Bitrate: " << metrics.rx_bitrate_mbps << " Mbps\n"
        << "Transmit Bitrate: " << metrics.tx_bitrate_mbps << " Mbps\n"
        << "Receive Packets/s: " << metrics.rx_packets_s << "\n"
        << "Transmit Packets/s: " << metrics.tx_packets_s << "\n"
        << "Latency: " << metrics.latency << " ms\n"
        << "Link Quality: " << metrics.link_quality * 100 << "%\n"; // in percent
    return oss.str();
}

//================================================================================
// Private Functions
//================================================================================

/*
 * parse the contents from /proc/net/dev
 */
NetworkData NetworkMonitor::parse_network_data(const std::string& interface) {
    // the file /proc/net/dev keeps track of received/transmitted frames
    std::ifstream file("/proc/net/dev");

    std::string line;
    NetworkData data{};

    // we are parsing the contents of /proc/net/dev to fit our needs
    while (std::getline(file, line)) {
        if (line.find(interface) != std::string::npos) {
            std::istringstream iss(line);
            std::string iface;
            iss >> iface; // name of the interface
            iss >> data.rx_bytes >> data.rx_packets; // RX bytes, packets

            // skip next 6 fields to reach TX bytes
            // dummy is only used to trash the unnecessary information
            unsigned long long dummy;
            for (int i = 0; i < 6; ++i) iss >> dummy;

            iss >> data.tx_bytes >> data.tx_packets; // TX bytes, packets
            break;
        }
    }

    return data;
}

/*
 * function to update the current link quality estimation using the
 * Rate Adaptive Link Quality Estimator (RALQ)
 * approach
 *
 * assume that the necessary values are initialized in the
 * link_estimator struct already
 *
 * params:
 *  - lseq: the last received sequence number
 */
void NetworkMonitor::update_link_estimator(uint16_t new_seq) {

    uint16_t missed = 0;
    if(link_estimator_.last_seq < 65535) {
        missed = new_seq - link_estimator_.last_seq - 1;
        link_metrics_.total_packets_lost += missed;
    }

    auto cur_timestamp = std::chrono::steady_clock::now();

    auto time_delta = cur_timestamp - link_estimator_.last_timestamp;
    auto time_delta_s = std::chrono::duration<double>(time_delta).count();

    /*
     * the Rate Adaptive Link Quality Estimator
     *
     @inproceedings{leclaire2016,
        author = {Leclaire, Maurice and Gunther, Stephan and Lienen, Marten and Riemensberger, Maximilian and Carle, Georg},
        year = {2016},
        month = {11},
        pages = {732-740},
        title = {Rate-Adaptive Link Quality Estimation for Coded Packet Networks},
        doi = {10.1109/LCN.2016.124}
        }
     *
     */

    double p_k_new = (link_estimator_.p_k + 1) * exp(-RALQ_DECAY * (time_delta_s));
	double q_k_new = (link_estimator_.q_k + missed) * exp(-RALQ_DECAY * (time_delta_s));

    double link_quality_new = (double) (p_k_new / (p_k_new + q_k_new));

    link_estimator_.last_seq = new_seq;
    link_estimator_.last_timestamp = cur_timestamp; // assuming we can neglect computational time for intermediate operations
    link_estimator_.p_k = p_k_new;
    link_estimator_.q_k = q_k_new;
    link_estimator_.link_quality = link_quality_new;

    link_metrics_.link_quality = link_estimator_.link_quality;
}
