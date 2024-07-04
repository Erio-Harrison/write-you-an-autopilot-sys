#include "network_comm/zeromq_adapter.hpp"
#include <sodium.h>
#include <stdexcept>
#include <iostream>

namespace network_comm {

ZeroMQAdapter::ZeroMQAdapter(zmq::socket_type type)
    : context_(1), socket_(context_, type) {}

ZeroMQAdapter::~ZeroMQAdapter() {
    disconnect();
}

void ZeroMQAdapter::bind(const std::string& address) {
    try {
        socket_.bind(address);
        std::cout << "Bind successful" << std::endl;
    } catch (const zmq::error_t& e) {
        std::cerr << "ZeroMQ bind failed: " << e.what() << std::endl;
        std::cerr << "Error code: " << e.num() << std::endl;
        throw std::runtime_error("ZeroMQ bind failed: " + std::string(e.what()));
    }
}

void ZeroMQAdapter::connect(const std::string& address) {
    try {
        socket_.connect(address);
        std::cout << "Connect successful" << std::endl;
    } catch (const zmq::error_t& e) {
        std::cerr << "ZeroMQ connect failed: " << e.what() << std::endl;
        std::cerr << "Error code: " << e.num() << std::endl;
        throw std::runtime_error("ZeroMQ connect failed: " + std::string(e.what()));
    }
}

void ZeroMQAdapter::disconnect() {
    socket_.close();
}

void ZeroMQAdapter::send(const std::vector<uint8_t>& data) {
    try {
        socket_.send(zmq::buffer(data), zmq::send_flags::none);
    } catch (const zmq::error_t& e) {
        throw std::runtime_error("ZeroMQ send failed: " + std::string(e.what()));
    }
}

std::vector<uint8_t> ZeroMQAdapter::receive() {
    try {
        zmq::message_t message;
        auto result = socket_.recv(message, zmq::recv_flags::none);
        if (result) {
            return std::vector<uint8_t>(static_cast<uint8_t*>(message.data()),
                                        static_cast<uint8_t*>(message.data()) + message.size());
        }
    } catch (const zmq::error_t& e) {
        throw std::runtime_error("ZeroMQ receive failed: " + std::string(e.what()));
    }
    return {};
}

} // namespace network_comm
