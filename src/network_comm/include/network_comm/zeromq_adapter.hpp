#pragma once
#include "communication_interface.hpp"
#include <zmq.hpp>
#include <string>

namespace network_comm {

class ZeroMQAdapter : public CommunicationInterface {
public:
    ZeroMQAdapter(zmq::socket_type type);
    ~ZeroMQAdapter() override;

    void connect(const std::string& address) override;
    void bind(const std::string& address);

    void disconnect() override;
    void send(const std::vector<uint8_t>& data) override;
    std::vector<uint8_t> receive() override;

private:
    zmq::context_t context_;
    zmq::socket_t socket_;
    bool is_server_;
};

} // namespace network_comm