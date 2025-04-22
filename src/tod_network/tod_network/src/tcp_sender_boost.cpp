#include "tod_network/tcp_sender_boost.hpp"

namespace tod_network {

    int TcpSenderBoost::send_data(std::vector<uint8_t>& data) {
        boost::system::error_code ec;
        std::cout << "sending data " << std::endl;
        std::size_t length = _socket.write_some(boost::asio::buffer(data), ec);
        if (ec) {
            std::cerr << "Send failed: " << ec.message() << std::endl;
            return -1;
        }
        return length;
    }

    void TcpSenderBoost::change_destination(const std::string& ip, const int port = -1) {
        if (port != -1) {
            _dest_port = port;
        }
        _dest_ip = ip;
        std::cout << "Connecting to ip:  " << ip << " port: " << _dest_port << std::endl;
        tcp::resolver::query query(ip, std::to_string(_dest_port));
        auto endpoint_iterator = _resolver.resolve(query);
        connect_to_server(endpoint_iterator);
    }

    std::string TcpSenderBoost::get_destination_ip() {
        return _dest_ip;
    }

    void TcpSenderBoost::disconnect() {
        boost::system::error_code ec;
        _socket.shutdown(tcp::socket::shutdown_both, ec); // Disable both sending and receiving on the socket
        if (ec) {
            std::cerr << "Shutdown failed: " << ec.message() << std::endl;
        }
        _socket.close(ec);
        if (ec) {
            std::cerr << "Socket close failed: " << ec.message() << std::endl;
        }
    }

    void TcpSenderBoost::connect_to_server(tcp::resolver::iterator endpoint_iterator) {
        boost::asio::connect(_socket, endpoint_iterator);
    }

    void TcpSenderBoost::close_socket() {
        _socket.close();
    }

} // namespace tod_network