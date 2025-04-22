/**
 * @file packet_logger.cpp
 * @brief module to allow capturing packets for post-mortem analysis
 *        - capture packets in .pcap format
 *        - set packet filter using filter expressions
 *          defined in filter_definitions.h
 *        - output .pcap file will be timestamped
 *
 *        this component may run as a background service
 *        DURING ACTIVE OPERATION
 * @copyright 2024 TUMFTM
 */

#include "tod_network_monitoring/packet_logger.hpp"
#include "tod_network_monitoring/filter_definitions.hpp"

#include <iostream>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <unistd.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/select.h>

//================================================================================
// Public Functions
//================================================================================

PacketLogger::PacketLogger() {}

PacketLogger::~PacketLogger() {

    /*
        IMPORTANT NOTE:
            - capture_thread_ will ONLY finish once stop_capture() is called
            - stop_capture() takes care of freeing resources
            - otherwise, if we destroy early, the capture also stops

        MAKE SURE TO CALL stop_capture() BEFORE DESTROYING AN INSTANCE
    */
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
}

int PacketLogger::start_capture(const std::string& interface, const std::string& logging_directory) {

    if (this->is_running()) {
        std::cerr << "capture already running. call stop_capture() before opening a new session" << std::endl;
        return 1;
    }

    // create directory for logs, if not yet present
    mkdir(logging_directory.c_str(), 0777);

    char error_buffer[PCAP_ERRBUF_SIZE];
    handle_ = pcap_open_live(interface.c_str(), BUFSIZ, 1, 1000, error_buffer);

    if (handle_ == nullptr) {
        std::cerr << "could not open device: " << error_buffer << std::endl;
        return 2;
    }

    struct bpf_program fp;
    const char* filter = filter_config::ACTIVE_FILTER.c_str(); // select active filter in filter_definitions.h

    if (pcap_compile(handle_, &fp, filter, 0, PCAP_NETMASK_UNKNOWN) == -1) {
        std::cerr << "could not parse filter " << filter << ": " << pcap_geterr(handle_) << std::endl;
        return 2;
    }

    if (pcap_setfilter(handle_, &fp) == -1) {
        std::cerr << "could not install filter " << filter << ": " << pcap_geterr(handle_) << std::endl;
        return 2;
    }

    std::string filename = get_timestamped_filename(logging_directory);
    std::cerr << "trace will be captured in:\n" << filename << std::endl;

    dumper_ = pcap_dump_open(handle_, filename.c_str());
    if (dumper_ == nullptr) {
        pcap_close(handle_);
        handle_ = nullptr;
        std::cerr << "could not open file for writing: " << pcap_geterr(handle_) << std::endl;
        return 2;
    }

    // this synchronizes execution and prevents multiple concurrent capture sessions
    keep_running_.store(true);

    capture_thread_ = std::thread([&](){

        int fd = pcap_get_selectable_fd(handle_);

        struct pcap_pkthdr *header;
        const u_char *packet;

        while (keep_running_.load()) {

            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(fd, &fds); // only use file descriptor returned by pcap

            struct timeval tv;
            tv.tv_sec = 1;
            tv.tv_usec = 0;

            /*
                we need to have a timeout in place as to prevent the edge case,
                where no packets have been captured. this must be done using select()
                to check the file descriptor returned above, as otherwise we would
                not be able to join this thread again.
            */
            int res = select(fd + 1, &fds, nullptr, nullptr, &tv);

            if (res > 0) {
                int pcap_res = pcap_next_ex(handle_, &header, &packet);

                if (pcap_res == 1) {
                    pcap_dump((u_char *)dumper_, header, packet);
                    std::cout << "captured a packet with length of [" << header->len << "] bytes" << std::endl;
                }

                else if (pcap_res == -1) {
                    std::cerr << "error reading packet: " << pcap_geterr(handle_) << std::endl;
                    break;
                }
            } else if (res == 0) {
                // select() timeout, do nothing
            }

            else {
                // actual select() error occurred
                std::cerr << "select error" << std::endl;
                break;
            }
        }
    });

    return 0;
}

int PacketLogger::stop_capture() {

    if (!this->is_running()) {
        std::cerr << "no capture running. call start_capture() to open a new session" << std::endl;
        return 1;
    }

    // stop capturing and wait for thread to dump the last packet
    keep_running_.store(false);
    capture_thread_.join();
    std::cerr << "joined capture thread" << std::endl;

    // cleanup and store trace
    if (dumper_ != nullptr) {
        pcap_dump_close(dumper_);
        dumper_ = nullptr;
        std::cerr << "dumper closed" << std::endl;
    }

    if (handle_ != nullptr) {
        pcap_close(handle_);
        handle_ = nullptr;
        std::cerr << "pcap closed" << std::endl;
    }

    return 0;
}

bool PacketLogger::is_running() {

    // capture is already running, if thread is joinable
    if (capture_thread_.joinable()) {
        return 1;
    }

    return 0;
}

//================================================================================
// Private Functions
//================================================================================

std::string PacketLogger::get_timestamped_filename(const std::string& dir) {
    time_t now = time(nullptr);
    tm *ltm = localtime(&now);

    /*
        OUTPUT FORMAT:
        TRACE_DD-MM-YYYY_HH:MM:SS.pcap
    */
    std::stringstream filename;
    filename << dir << "/TRACE_";
    filename << std::setfill('0') << std::setw(2) << ltm->tm_mday << "-";
    filename << std::setfill('0') << std::setw(2) << 1 + ltm->tm_mon << "-";
    filename << std::setfill('0') << std::setw(4) << 1900 + ltm->tm_year << "_";
    filename << std::setfill('0') << std::setw(2) << ltm->tm_hour << ":";
    filename << std::setfill('0') << std::setw(2) << ltm->tm_min << ":";
    filename << std::setfill('0') << std::setw(2) << ltm->tm_sec << ".pcap";

    return filename.str();
}
