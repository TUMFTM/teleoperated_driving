/**
 * @file tcp_receiver.cpp
 * @author Simon Hoffmann
 * @brief This file contains the implementation of a TCP receiver class that handles synchronous and asynchronous data reception. It also defines a RAII-based `SocketGuard` class to ensure proper socket closure. The `TcpReceiver` class listens for incoming TCP connections, handles receiving data both synchronously and asynchronously, and provides functions for managing the client connection lifecycle.
 * 
 * The `TcpReceiver` class uses the `BaseReceiver` interface and is designed to be used in network communication systems where data is transmitted over TCP. It includes mechanisms to handle connection timeouts and asynchronous reception using `std::future`. The `SocketGuard` class is used to ensure proper socket management and avoid resource leaks.
 *
 * @version 1.0
 *
 * @copyright TUMFTM 2020
 */
#include "tod_network_protocols/tcp_receiver.hpp"
#include <fcntl.h>

using namespace tod_network_protocols;

bool TcpReceiver::waiting_for_client_connect(){
        waiting_for_client_to_connect(); 
        return _connected;
}

void TcpReceiver::disconnect() {
    std::cerr << "TCPReceiver: Closing TCP Server" << std::endl;
    if (_client_socket != -1) {
        std::cerr << "TCPReceiver: Closing client socket" << std::endl;
        close(_client_socket);
        _client_socket = -1;
    }
    _connected = false; 
}

std::vector<uint8_t> TcpReceiver::receive(){
    if(!_connected) return std::vector<uint8_t>();
    std::vector<uint8_t> data;
    while (_connected) {
        size_t size_of_payload = receive_msg_size();
        if (size_of_payload > 0) {
            return receive_payload(size_of_payload); 
        }
    }
    std::cerr << "TCPReceiver: DISCONNECTED" << std::endl;
    return data; //ToDO
}

std::vector<uint8_t> TcpReceiver::receive_with_timeout(int timeout_ms) {
    if (!_connected) return std::vector<uint8_t>();
    std::vector<uint8_t> data;
    auto start_time = std::chrono::steady_clock::now();
    bool timeout = false;
    while (_connected && !timeout) {
        size_t size_of_payload = receive_msg_size();
        if (size_of_payload > 0) {
            return receive_payload(size_of_payload);  // Return the payload if successfully received
        }
        int elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count();
        timeout = elapsed_time >= timeout_ms;
    }
    if(!_connected) std::cerr << "TCPReceiver: DISCONNECTED" << std::endl;
    if(timeout) std::cerr << "TCPReceiver: RECEIVE TIMEOUT" << std::endl;
    return data;  // Return empty vector on disconnection
}

std::future<void> TcpReceiver::async_receive(std::function<void(const std::vector<uint8_t>&)> callback){
    return std::async(std::launch::async, [this, callback]() {
        std::vector<uint8_t> data = this->receive();
        if (!data.empty()) {
            callback(data);
        }
    });
}

void TcpReceiver::waiting_for_client_to_connect() {
    std::cerr << "TCPReceiver: WAITING FOR CLIENT TO CONNECT" << std::endl;
    int opt = 1;
    int server_fd = -1;

    while (true) {
        std::cerr << "TCPReceiver: Creating server socket..." << std::endl;

        // Creating a socket file descriptor
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd == -1) {
            std::cerr << "TCPReceiver: Can't create a socket!" << std::endl;
            // Sleep for 100 milliseconds before retrying
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Ensure the server socket is closed properly when leaving the scope of the function
        SocketGuard server_fd_guard(server_fd);

        // Setting socket options to allow reuse of the address and port
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)) == -1) {
            std::cerr << "TCPReceiver: Can't set socket options!" << std::endl;
            // Sleep for 100 milliseconds before retrying
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Initialize the address structure
        struct sockaddr_in address;
        // make shure address is set to 0 prior to filling in values
        memset(&address, 0, sizeof(address));
        // addig address
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(_destPort);

        std::cerr << "TCPReceiver: Binding socket to address..." << std::endl;
        // Binding the socket to the specified IP address and port
        if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) == -1) {
            std::cerr << "TCPReceiver: Can't bind to IP/port!" << std::endl;
            // Sleep for 100 milliseconds before retrying
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        std::cerr << "TCPReceiver: Marking socket for listening..." << std::endl;
        // Mark the socket as passive, indicating it will be used to accept incoming connection requests
        if (listen(server_fd, SOMAXCONN) == -1) {
            std::cerr << "TCPReceiver: Can't listen on socket!" << std::endl;
            // Sleep for 100 milliseconds before retrying
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Initialize client address structure
        sockaddr_in client;
        socklen_t clientSize = sizeof(client);
        std::cerr << "TCPReceiver: Accepting client connection..." << std::endl;

        // Set server socket to non-blocking mode
        int flags = fcntl(server_fd, F_GETFL, 0);
        fcntl(server_fd, F_SETFL, flags | O_NONBLOCK);

        // Polling loop to accept connections
        while (true) {
            // Accept client connection
            _client_socket = accept(server_fd, (struct sockaddr*)&client, &clientSize);
            if (_client_socket == -1) {
                // Check if there are no pending connections
                if (errno == EWOULDBLOCK || errno == EAGAIN) {
                    // No pending connections, wait a bit before trying again
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                } else {
                    // Error occurred while accepting connection
                    std::cerr << "TCPReceiver: Problem with client connection!" << std::endl;
                    break;
                }
            } else {
                // Successfully accepted a client connection
                _connected = true;
                std::cerr << "TCPReceiver: Client connected from address: " << inet_ntoa(client.sin_addr)
                          << " and port: " << ntohs(client.sin_port) << std::endl;
                // Close the server socket as it's no longer needed
                close(server_fd);
                return;
            }
        }
    }
}

size_t TcpReceiver::receive_msg_size() {
    size_t size_of_payload {0};
    uint8_t buffer[sizeof(size_of_payload)];
    size_t bytesReceived = 0;

    while (bytesReceived < sizeof(size_of_payload)) {
        int readSize = recv(_client_socket, buffer + bytesReceived, sizeof(size_of_payload) - bytesReceived, 0);
        if (readSize <= 0) {
            std::cerr << "TCPReceiver: There was a connection issue. Receive size not matching" << std::endl;
            _connected = false;
            return 0;
        }
        bytesReceived += readSize;
    }

    std::memcpy(&size_of_payload, buffer, sizeof(size_of_payload));
    return size_of_payload;
}

std::vector<uint8_t> TcpReceiver::receive_payload(size_t size_of_payload) {
    int receivedBytes = 0;
    std::vector<uint8_t> data(size_of_payload);

    // make sure that the full byte count is received
    // prevent eadlock by _connected function
    while (receivedBytes < size_of_payload && _connected) {
        int bufferSize = std::min(MAXLINE, static_cast<int>(size_of_payload - receivedBytes));
        uint8_t buffer[bufferSize];

        int sizeReceived = recv(_client_socket, buffer, bufferSize, 0);
        if (sizeReceived <= 0) {
            std::cerr << "TCPReceiver: There was a connection issue, size not matching." << std::endl;
            _connected = false;
            return std::vector<uint8_t>(); // Return an empty vector to indicate failure.
        }

        std::copy(buffer, buffer + sizeReceived, data.begin() + receivedBytes);
        receivedBytes += sizeReceived;
    }

    if (receivedBytes != size_of_payload) {
        std::cerr << "TCPReceiver: Received bytes not matching the expected payload size." << std::endl;
        _connected = false;
        return std::vector<uint8_t>(); // Return an empty vector to indicate failure.
    }

    return data;
}