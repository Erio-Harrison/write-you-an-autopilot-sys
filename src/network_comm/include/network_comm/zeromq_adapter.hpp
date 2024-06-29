#pragma once

#include "communication_interface.hpp"
#include <zmq.hpp>
#include <memory>

namespace network_comm {

class ZeroMQAdapter : public CommunicationInterface {
public:
    ZeroMQAdapter();
    ~ZeroMQAdapter() override;

    void send(const std::vector<uint8_t>& data) override;
    std::vector<uint8_t> receive() override;
    void connect(const std::string& address) override;
    void disconnect() override;

private:
    std::unique_ptr<zmq::context_t> context_;
    std::unique_ptr<zmq::socket_t> socket_;
};

} // namespace network_comm