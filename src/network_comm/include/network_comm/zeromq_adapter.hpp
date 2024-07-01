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

    // 设置加密密钥
    void setServerKey(const std::string& key);
    void setPublicKey(const std::string& key);
    void setSecretKey(const std::string& key);

private:
    zmq::context_t context_;
    zmq::socket_t socket_;
    bool is_server_;
    std::string server_key_;
    std::string public_key_;
    std::string secret_key_;
};

} // namespace network_comm