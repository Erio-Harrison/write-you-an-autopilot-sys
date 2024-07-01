#pragma once
#include <vector>
#include <string>

namespace network_comm {
class CommunicationInterface{
    public:
        virtual ~CommunicationInterface() = default;
        
        virtual void connect(const std::string& address) = 0;
        virtual void bind(const std::string& address) = 0;
        virtual void disconnect() = 0;
        virtual void send(const std::vector<uint8_t>& data) = 0; 
        virtual std::vector<uint8_t> receive() = 0;
};
}