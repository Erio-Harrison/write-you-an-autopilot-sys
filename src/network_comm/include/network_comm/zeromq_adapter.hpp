#pragma once
#include "communication_interface.hpp"
#include <zmq.hpp>

namespace network_comm{
class ZeroMQAdapter : public CommunicationInterface{
    public:
        ZeroMQAdapter();
        ~ZeroMQAdapter() override;

        void connect(const std::string& address) override;
        void disconnect() override;
        void send(const std::vector<uint8_t>& data) override; 
        std::vector<uint8_t> receive() override;

    private:
        zmq::context_t context_;
        zmq::socket_t socket_;
};
}