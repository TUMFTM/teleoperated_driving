/**
 * @file packet_logger.hpp
 * @brief header file for packet_logger
 * @copyright 2024 TUMFTM
 */

#ifndef PACKET_LOGGER_HPP
#define PACKET_LOGGER_HPP

#include "tod_network_monitoring/config.hpp"

#include <string>
#include <atomic>
#include <thread>
#include <pcap.h>

/**
 * @brief Class capturing packets for post-mortem analysis
 *        - capture packets in .pcap format
 *        - set packet filter using filter expressions
 *          defined in filter_definitions.h
 *        - output .pcap file will be timestamped 
 * @ingroup tod_network_monitoring
 */
class PacketLogger {
public:
    /**
     * @brief constructor
     * 
     */
    PacketLogger();

   /**
    * @brief start a new capture session
    * 
    * @param interface the network interface to capture on
    * @param logging_directory 
    * @return int =0, if success
    *             >0, otherwise
    */
    int start_capture(const std::string& interface, const std::string& logging_directory);

   /**
    * @brief stop the current capture session
    * 
    * @return int =0, if success
    *             >0, otherwise
    */
    int stop_capture();

   /**
    * @brief return the current capture status
    * 
    * @return false if no capture is running
    * @return true otherwise
    */
    bool is_running();

    /**
     * @brief destructor
     * 
     */
    ~PacketLogger();

private:
    std::string logging_directory_ = LOGGING_DIRECTORY;
    std::thread capture_thread_;
    std::atomic<bool> keep_running_ = true;

    pcap_t *handle_ = nullptr;
    pcap_dumper_t *dumper_ = nullptr;

    /**
     * @brief generates a filepath which includes a timestamp
     * 
     * @param dir 
     * @return std::string 
     */
    std::string get_timestamped_filename(const std::string& dir);

};

#endif // PACKET_LOGGER_HPP