#include "tod_network/tcp_receiver_boost.hpp"

namespace tod_network {

    std::vector<uint8_t> TcpReceiverBoost::receive() {
        std::vector<uint8_t> data(MAXLINE);
        boost::system::error_code ec;
        size_t length = _socket.read_some(boost::asio::buffer(data), ec);
        if (!ec) {
            data.resize(length);
            return data;
        } else {
            throw std::runtime_error("Receive error: " + ec.message());
        }
    }

    std::future<void> TcpReceiverBoost::async_receive(std::function<void(const std::vector<uint8_t>&)> callback) {
        return std::async(std::launch::async, [this,callback]() {
            try {
                std::vector<uint8_t> data = receive();
                callback(data);
            } catch (...) {
                std::promise<void> p;
                p.set_exception(std::current_exception());
                throw;
            }
        });
    }

    bool TcpReceiverBoost::waiting_for_client_connect() {
        std::cout << "waiting_for_client_connect " << std::endl;

        boost::system::error_code ec;
        _acceptor.accept(_socket, ec);
        if (!ec) {
            _connected = true;
            std::cout << "Connection established" << std::endl;
        } else {
            std::cerr << "Accept failed: " << ec.message() << std::endl;
            _connected = false;
        }
        return _connected;
    }

    void TcpReceiverBoost::disconnect() {
        boost::system::error_code ec;
        _socket.close(ec);
        if (ec) {
            std::cerr << "Failed to close socket: " << ec.message() << std::endl;
        }
    }

    boost::asio::io_context& TcpReceiverBoost::get_io() {
        return _io_context;
    }
} // namespace tod_network