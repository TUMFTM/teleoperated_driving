/**
 * @file test.cpp
 * @brief for general purpose testing
 * @copyright 2024 TUMFTM
 */

#include "tod_network_monitoring/network_monitor.hpp"
#include "tod_network_monitoring/network_tester.hpp"
#include "tod_network_monitoring/packet_logger.hpp"
#include "tod_network_monitoring/config.hpp"

#include <stdlib.h>
#include <string>
#include <iostream>
#include <unistd.h>

/*
    Latency test
*/
int test_latency() {

    NetworkTester network_tester;
    std::string hostname = HOSTNAME;
    float update_time_interval = UPDATE_TIME_INTERVAL;
    int ping_timeout = PING_TIMEOUT;

    std::optional<float> latency = network_tester.test_latency(hostname, update_time_interval, ping_timeout);

    if (latency.has_value()) {
        std::cout << "Average latency to host '" << hostname << "':\n" << latency.value() << "ms" << std::endl;
        return 0;
    }

    std::cout << "Could not obtain latency to host '" << hostname << "'" << std::endl;
    return -1;
}

/*
    Bandwidth test for upload: client -> server
*/
int test_bandwidth_upload() {

    NetworkTester network_tester;
    std::string hostname = HOSTNAME;
    int port = 5201;
    int duration_s = BANDWIDTH_MEASUREMENT_TIME;
    bool is_reverse = 0;

    BandwidthStats res = network_tester.test_bandwidth(HOSTNAME, port, duration_s, is_reverse);

    std::cout << "BandwidthStats Upload:" << std::endl;
    std::cout << "bitrate_mbs: " << res.bitrate_mbps << std::endl;
    std::cout << "transferred_bytes:" << res.transferred_bytes << std::endl;

    return 0;
}

/*
    Bandwidth test for download: client <- server
*/
int test_bandwidth_download() {

    NetworkTester network_tester;
    std::string hostname = HOSTNAME;
    int port = 5201;
    int duration_s = BANDWIDTH_MEASUREMENT_TIME;
    bool is_reverse = 1;

    BandwidthStats res = network_tester.test_bandwidth(HOSTNAME, port, duration_s, is_reverse);

    std::cout << "BandwidthStats Download:" << std::endl;
    std::cout << "bitrate_mbs: " << res.bitrate_mbps << std::endl;
    std::cout << "transferred_bytes:" << res.transferred_bytes << std::endl;

    return 0;
}

/*
    Monitoring test
*/
int test_network_monitor() {

    NetworkMonitor network_monitor;
    std::string hostname = HOSTNAME;
    std::string interface = NETWORK_INTERFACE;
    float update_timeout = UPDATE_TIMEOUT;
    float update_time_interval = UPDATE_TIME_INTERVAL;

    network_monitor.start_monitoring(hostname, interface, update_timeout, update_time_interval);
    sleep(1);
    network_monitor.stop_monitoring();

    network_monitor.start_monitoring(hostname, interface, update_timeout, update_time_interval);
    sleep(1);
    network_monitor.stop_monitoring();

    return 0;
}

/*
    Monitoring test with stdout output
*/
int test_network_monitor_with_output() {

    NetworkMonitor network_monitor;
    std::string hostname = HOSTNAME;
    std::string interface = NETWORK_INTERFACE;
    float update_timeout = UPDATE_TIMEOUT;
    float update_time_interval = UPDATE_TIME_INTERVAL;

    auto refresh_rate = std::chrono::milliseconds(500);

    network_monitor.start_monitoring(hostname, interface, update_timeout, update_time_interval);

    auto start = std::chrono::high_resolution_clock::now();
    int count = 0;

    while (true) {
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start);

        if (elapsed.count() >= 7) break; // stop after 7 seconds

        NetworkMetrics metrics = network_monitor.get_network_metrics();
        std::string formatted = network_monitor.format_network_metrics(metrics);

        std::cout << formatted;
        std::cout << "--------------------------------\n";

        std::this_thread::sleep_for(refresh_rate);
        count++;
    }

    network_monitor.stop_monitoring();
    return 0;
}

/*
    PacketLogger test
*/
int test_packet_logger() {

    PacketLogger packet_logger;
    std::string interface = NETWORK_INTERFACE;
    std::string logging_directory = LOGGING_DIRECTORY;

    int res = packet_logger.start_capture(interface, logging_directory);
    sleep(4);
    res = packet_logger.stop_capture();

    std::cout << res << std::endl;

    res = packet_logger.start_capture(interface, logging_directory);
    sleep(1);
    res = packet_logger.stop_capture();

    std::cout << res << std::endl;

    return res;
}

int main() {

    // test_latency();

    // test_bandwidth_download();

    // test_bandwidth_upload();

    // test_network_monitor();

    // test_network_monitor_with_output();

    test_packet_logger();

    return 0;
}
