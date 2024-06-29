#include "network_comm/zeromq_adapter.hpp"
#include <stdexcept>

namespace network_comm {

ZeroMQAdapter::ZeroMQAdapter() 
    : context_(std::make_unique<zmq::context_t>(1)),
      socket_(std::make_unique<zmq::socket_t>(*context_, ZMQ_REQ)) {}

ZeroMQAdapter::~ZeroMQAdapter() {
    disconnect();
}

void ZeroMQAdapter::send(const std::vector<uint8_t>& data) {
    zmq::message_t message(data.data(), data.size());
    socket_->send(message, zmq::send_flags::none);
}

std::vector<uint8_t> ZeroMQAdapter::receive() {
    zmq::message_t message;
    auto result = socket_->recv(message);
    if (!result) {
        throw std::runtime_error("Failed to receive message");
    }
    return std::vector<uint8_t>(static_cast<uint8_t*>(message.data()), 
                                static_cast<uint8_t*>(message.data()) + message.size());
}

void ZeroMQAdapter::connect(const std::string& address) {
    socket_->connect(address);
}

void ZeroMQAdapter::disconnect() {
    if (socket_) {
        socket_->close();
    }
    socket_.reset();
    context_.reset();
}

} // namespace network_comm