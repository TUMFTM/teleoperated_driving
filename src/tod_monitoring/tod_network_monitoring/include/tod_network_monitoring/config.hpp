/**
 * @file config.hpp
 * @brief default configs for the network_tester, network_monitor and packet_logger
 * @copyright 2024 TUMFTM
 */

#ifndef CONFIG_HPP
#define CONFIG_HPP

/*
 * this will be the target vehicle
 * either use
 *  - its IP address
 *  - the hostname (ensure there is a valid mapping between the hostname and its actual address)
 */
#define HOSTNAME "127.0.0.1"

/*
 * the network interface, where traffic is expected from and to $HOSTNAME
 */
#define NETWORK_INTERFACE "eth0"

/*
 * the base path where log files are stored
 */
#define LOGGING_DIRECTORY "/var/log/tod_network_monitoring"

/*
 * we use this timeout periode to configure the behaviour of awaiting
 * responses like ICMP_ECHO_RESPONSE
 *
 * UNIT = SECONDS
 */
#define UPDATE_TIMEOUT 0.3

/*
 * we use this time interval to configure the time steps
 * where we take periodic measurements
 * (e.g. from a network interface)
 *
 * the value should be set to a fraction of UPDATE_TIMEOUT
 * typically half or less
 *
 * UNIT = SECONDS
 */
#define UPDATE_TIME_INTERVAL 0.1

/*
 * we use this timeout value to break
 * after PING_TIMEOUT seconds
 * if no response was observed
 *
 * UNIT = SECONDS
 */
#define PING_TIMEOUT 4

/*
 * the port to which the iperf3 server opens a TCP port
 * and the client connects to
 *
 * when doing UDP measurements, iperf3 will still connect
 * to the same TCP port and execute
 * a TCP handshake before opening the same port for UDP
 */
#define IPERF3_PORT 5201

/*
 * we use this duration for the bandwidth test
 * (e.g. how long should the client/server send data until
 * we obtain an average value)
 *
 * this value should be set to at least 5 seconds,
 * but can vary based on network conditions
 * higher value => more accurate estimation
 *
 * UNIT = SECONDS
 */
#define BANDWIDTH_MEASUREMENT_TIME 5

// alternative for typesafe
// constexpr char HOSTNAME[] = "hostname";

#endif // CONFIG_HPP