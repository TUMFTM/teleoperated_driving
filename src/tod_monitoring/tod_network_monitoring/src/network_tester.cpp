/**
 * @file network_tester.cpp
 * @brief on startup to execute measurements like
 *        - maximum bandwidth (upload/download)
 *        - avg latency
 *
 *        fthis component should only be called
 *        AT CONNECTION ESTABLISHMENT
 *        to minimize network load
 *        and prevent congestion
 * @copyright 2024 TUMFTM
 */

#include "tod_network_monitoring/network_tester.hpp"

#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <regex>
#include <optional>
#include <array>

//================================================================================
// Public Functions
//================================================================================

NetworkTester::NetworkTester() {}

NetworkTester::~NetworkTester() {}

/*
 * test the average latency to $hostname,
 * where average is calculated from $count RTT measurements
 *
 * RETURN: (float) latency, if sucessful
 *         NULL           , otherwise
 */
std::optional<float> NetworkTester::test_latency(const std::string& hostname, const float& update_time_interval, const int& ping_timeout) {

    /*
        first, call ping with params:
            -w timeout: the amount of seconds before ping will timeout and exit
            -c count: the amount of ICMP packets to be sent and averaged
            -i interval: the amount of seconds between ICMP packets
            hostname: the target host
        second, pipe to tail:
            -1: gets the last line
        third, pipe to awk:
            -F: use / as separator
            {print $5}: takes the element after the 5th operator
        finally:
            we get the average latency from the ping command
    */

    const int count = 10;

    std::string command = "ping -w " + std::to_string(ping_timeout) +
                                " -c " + std::to_string(count) +
                                " -i " + std::to_string(update_time_interval) +
                                " " + hostname +
                                " | tail -1 | awk -F'/' '{print $5}'";
    std::array<char, 128> buffer;
    std::string result;

    // access output from the ping command through a pipe
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        std::cerr << "could not start command:\n" << command << std::endl;
        return {};
    }

    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
        result += buffer.data();
    }

    // check for errors
    auto returnCode = pclose(pipe);

    if (returnCode != 0) {
        std::cerr << "command failed with return code " << returnCode << std::endl;
        return {};
    }

    // convert the result to a float
    float avgLatency;
    std::stringstream ss(result);
    ss >> avgLatency;
    if (ss.fail()) {
        return {};
    }

    return avgLatency;
}

/*
 * test the maximum possible throughput to the given host on the given port
 * if is_reverse is present, invert client <> server | sender <> receiver
 *
 * RETURN: (BandwidthStats) populated struct  , if sucessful
 *         (BandwidthStats) unpopulated struct, otherwise
 */
BandwidthStats NetworkTester::test_bandwidth(const std::string& hostname, int port, int duration_s, bool is_reverse) {

    BandwidthStats result;

    /*
     * generate iperf3 command
     * usage:
     *      -c [hostname]  >> act as client and connect to [hostname]
     *      -f m           >> print bandwidth in Megabits/s
     *      -p [port]      >> connect to [port], default = 5201
     *      -i [seconds]   >> sets reporting interval to stdout, 0 = disable
     *      -t [seconds]   >> sets duration of test to [seconds]
     *      -R             >> server sends, client receives
     */
    std::ostringstream cmd;
    cmd << "iperf3 -c " << hostname << " -p " << port << " -f m -i 0 -t " << duration_s;
    if (is_reverse) {
        cmd << " -R";
    }

    FILE* pipe = popen(cmd.str().c_str(), "r");
    if (!pipe) {
        std::cerr << "Couldn't start command:\n" << cmd.str() << std::endl;
        return result;
    }

    char buffer[512];
    std::string line;

    while (fgets(buffer, sizeof(buffer), pipe) != NULL) {
        line = buffer;

        // check if the line contains 'sender' information
        // this is the case if iperf3 succesfully finished execution
        if (line.find("sender") != std::string::npos) {
            std::regex regex_pattern(R"(\[\s*\d+\]\s*\d+\.\d+-\d+\.\d+\s*sec\s*(\d+\.\d+|\d+)\s*(G|M|K)?Bytes\s*(\d+\.\d+|\d+)\s*Mbits/sec)");
            std::smatch matches;

            if (std::regex_search(line, matches, regex_pattern) && matches.size() == 4) {
                double transferred = std::stod(matches[1].str());
                std::string unit = matches[2].str();
                double bitrate = std::stod(matches[3].str());

                // convert to bytes
                if (unit == "G") {
                    transferred *= 1024 * 1024 * 1024;
                } else if (unit == "M") {
                    transferred *= 1024 * 1024;
                } else if (unit == "K") {
                    transferred *= 1024;
                }

                result.transferred_bytes = static_cast<long long>(transferred);
                result.bitrate_mbps = static_cast<long long>(bitrate);

            } else {
                std::cerr << "Error while parsing iperf3 output:\n" << line << std::endl;
            }
            break;
        }
    }

    pclose(pipe);
    return result;
}

//================================================================================
// Private Functions
//================================================================================
