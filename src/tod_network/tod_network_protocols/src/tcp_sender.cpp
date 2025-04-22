/**
 * @file tcp_sender.cpp
 * @author Simon Hoffmann
 * @brief This file contains the implementation of a TCP sender class that handles sending data to a specified server. The `TcpSender` class provides the necessary functionality for establishing a connection, sending data to a remote server, and managing the server address configuration. It also includes mechanisms to change the destination IP address and port.
 * 
 * The `TcpSender` class is designed for use in network communication systems where data needs to be transmitted over TCP. It allows you to send data to a specific destination, change the destination address if needed, and manage socket connections in a thread-safe manner.
 * 
 * The class also includes methods for printing connection specifications and disconnecting from the socket. It ensures that the socket is properly managed through its internal functions, such as creating and closing the socket.
 * 
 * @version 1.0
 *
 * @copyright TUMFTM 2020
 */
#include "tod_network_protocols/tcp_sender.hpp"

using namespace tod_network_protocols;

TcpSender::TcpSender(const int destPort, const std::string& ip) {
    _serv_addr.sin_family = AF_INET;
    create_socket();
    change_destination(ip, destPort);
}

TcpSender::~TcpSender() { close(_sock); }

int TcpSender::send_data(std::vector<uint8_t>& data) {
    try {
        if (_sock < 0) {
            throw std::runtime_error("Socket is not connected.");
        }

        // Create Byte vector for size information
        size_t sizeOfVector = data.size();
        // TCP Package must be not greater than 65535 bytes (due to checksum)
        std::vector<uint8_t> sizePrefix(sizeof(sizeOfVector));
        for (size_t i = 0; i < sizeof(sizeOfVector); ++i) {
            // Writing off data indices bytewise
            sizePrefix[i] = (sizeOfVector >> (i * 8)) & 0xFF;
        }
        data.insert(data.begin(), sizePrefix.begin(), sizePrefix.end());

        // Ensure the whole data is sent
        ssize_t totalSent = 0;
        ssize_t bytesLeft = data.size();
        const uint8_t* dataPtr = data.data();

        while (totalSent < data.size()) {
            // MSG_NOSIGNAL Flag prevents error throwing and returns -1 instead
            ssize_t sent = send(_sock, dataPtr + totalSent, bytesLeft, MSG_NOSIGNAL);
            if (sent == -1) {
                if (errno == EPIPE || errno == ECONNRESET || errno == ETIMEDOUT) {
                    // Handle broken pipe, connection reset, and timeout errors
                    std::cerr << "TCPSender: Connection issue, attempting to reconnect..." << std::endl;
                    close_socket();  // Close the socket
                    create_socket(); // Create a new socket
                    connect_to_server(); // Attempt to reconnect
                    throw std::runtime_error("Failed to send data due to connection issue, attempted to reconnect.");
                } else {
                    // Handle other send errors
                    std::cerr << "TCPSender: Failed to send data: " << strerror(errno) << std::endl;
                    close_socket();  // Close the socket
                    throw std::runtime_error("Failed to send data: " + std::string(strerror(errno)));
                }
            } else if (sent == 0) {
                // Handle case where send returns 0 indicating the connection is closed
                std::cerr << "TCPSender: Connection closed by the receiver." << std::endl;
                close_socket();  // Close the socket
                throw std::runtime_error("Failed to send data, connection closed by the receiver.");
            }
            totalSent += sent;
            bytesLeft -= sent;
        }

        if (bytesLeft > 0) {
            std::cerr << "Mismatch in total sent: " << totalSent << " and bytesLeft: " << bytesLeft << std::endl;
        }
        return totalSent;
    } catch (const std::exception& e) {
        // Handle exceptions
        std::cerr << "TCPSender: Exception caught in send_data: " << e.what() << std::endl;
        return -1; // Indicate failure
    }
}

void TcpSender::change_destination(const std::string& ip, const int port){
    close_socket();
    std::cout << "TCPSender: Change IP Adresse" << std::endl;
    //set the entire struct _serv_addr to zero to make shure 
    //we set it appropriately afterwards
    if (port >= 0){
        std::memset(&_serv_addr, 0, sizeof(_serv_addr));
        _serv_addr.sin_family = AF_INET;
        _serv_addr.sin_port = htons(port);
    }
    create_socket();
    change_ip_address(ip);
    connect_to_server();
}

 void TcpSender::print_connection_specs(){
    std::cout << "Destination IP Address " << inet_ntoa(_serv_addr.sin_addr) << std::endl;
    auto port = ntohs(_serv_addr.sin_port);
    std::cout << "Destination Port " << port << std::endl;
}

std::string TcpSender::get_destination_ip(){
    return std::string(inet_ntoa(_serv_addr.sin_addr));
}

void TcpSender::connect_to_server(){
    while (connect(_sock, (struct sockaddr*)&_serv_addr, sizeof(_serv_addr)) == -1) {
        std::cerr << "TCPSender: Server not found trying again ..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

void TcpSender::change_ip_address(const std::string& ip){
    if (inet_pton(AF_INET, ip.c_str(), &_serv_addr.sin_addr) <= 0) {
        std::cerr << "TCPSender: Invalid address/ Address not supported" << std::endl;
    }
}

void TcpSender::create_socket() {
    uint connection_tried = 0;
    while ((_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0 && (connection_tried < MAX_TRIES)) {
        std::cerr << "TCPSender: Socket creation error, trying again .... \n" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        connection_tried++;
    }
    if (_sock < 0) {
        throw std::runtime_error("TCPSender: Socket creation error after trying multiple times.");
    }
}

void TcpSender::close_socket() {
    if (_sock >= 0) {
        std::cout << "TCPSender: Closing Socket" << std::endl;
        close(_sock);
    }
    _sock = -1;

}

// Override virtual method in base protocol, otherwise error at compiling
void TcpSender::disconnect() {}
