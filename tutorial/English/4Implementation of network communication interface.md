# Network communication interface

In fact, if the server is also using **ROS2** for communication, we do not need to use an additional network library (ROS2's powerful DDS mechanism provides us with efficient distributed communication), but in real scenarios, the server rarely uses ROS2 for development, so a dedicated network communication module will be implemented in our system.

Generally speaking, we will use the `connect` method on the client to connect to the server's address, and use the `bind` method on the server to bind an address so that the client can connect. Both the client and the server can use `send` and `receive` to send and receive data respectively.

```bash
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
```

If we use the **ZeroMQ** network library, remember to install the related libraries:

```bash
sudo apt-get install libzmq3-dev
```
Related dependencies also need to be added in **CMakeLists.txt** and **package.xml**.

Inherit this interface in **zeromq_adapter.hpp**, and then implement the corresponding adapter:

```bash
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
```

The specific implementation is placed in **zeromq_adapter.cpp**:

```bash
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
```

OK, now that the basic mechanism is set up, we can start developing the client and server.

There are many benefits to setting up interfaces. Here are a few network module-related scenarios for you to think about:

1. Some projects may need to support multiple communication protocols. For example, a device may need to support both TCP and UDP protocols. By defining a common interface and implementing specific classes for each protocol, the communication protocol can be dynamically selected at runtime without modifying the business logic code.

```bash
std::unique_ptr<network_comm::CommunicationInterface> comm;

if (useTCP) {
    comm = std::make_unique<network_comm::TCPCommunication>();
} else {
    comm = std::make_unique<network_comm::UDPCommunication>();
}

BusinessLogic logic(std::move(comm));
logic.execute();
```

2. The project may need to increase the security of network communication at a later stage, such as adding TLS/SSL support. By defining a common interface, the existing communication implementation can be easily replaced or extended to support encryption without modifying the business logic code.

```bash
// Before adding TLS support
auto comm = std::make_unique<network_comm::TCPCommunication>();

// After adding TLS support
auto comm = std::make_unique<network_comm::SecureTCPCommunication>();

```

3. We may choose to use ZeroMQ in the early stage of the project, and may want to replace it with MQTT later.

```bash
// Use ZeroMQ before replacement
auto comm = std::make_unique<network_comm::ZeroMQAdapter>(zmq::socket_type::req);

// Use MQTT after replacement
auto comm = std::make_unique<network_comm::MQTTAdapter>();
```

Currently our project uses the **ZeroMQ** network library. What if we want to replace it with **boost_asio**?

