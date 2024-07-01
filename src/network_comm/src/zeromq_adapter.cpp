#include "network_comm/zeromq_adapter.hpp"

namespace network_comm{

ZeroMQAdapter::ZeroMQAdapter() : context_(1), socket_(context_, ZMQ_REQ) {}

ZeroMQAdapter:: ~ZeroMQAdapter(){
    disconnect();
}

void ZeroMQAdapter::connect(const std::string& address){
    socket_.connect(address);
}

void ZeroMQAdapter::disconnect(){
    socket_.close();
}

void ZeroMQAdapter:: send(const std::vector<uint8_t>& data){
    socket_.send(zmq::buffer(data),zmq::send_flags::none);
}

std::vector<uint8_t> ZeroMQAdapter::receive(){
    zmq::message_t message;
    auto result = socket_.recv(message,zmq::recv_flags::none);
    if(result){
        return std::vector<uint8_t>(static_cast<uint8_t*> (message.data()),
                                    static_cast<uint8_t*> (message.data() + message.size()));
    }

    return {};
}

}