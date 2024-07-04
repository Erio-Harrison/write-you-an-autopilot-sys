#pragma once
#include "communication_interface.hpp"
#include <boost/asio.hpp>
#include <string>
#include <memory>

namespace network_comm {

class BoostAsioAdapter : public CommunicationInterface {
public:
    BoostAsioAdapter();
    ~BoostAsioAdapter() override;

    void connect(const std::string& address) override;
    void bind(const std::string& address) override;
    void disconnect() override;
    void send(const std::vector<uint8_t>& data) override;
    std::vector<uint8_t> receive() override;

private:
    boost::asio::io_context io_context_;
    std::unique_ptr<boost::asio::ip::tcp::socket> socket_;
    std::unique_ptr<boost::asio::ip::tcp::acceptor> acceptor_;
    bool is_server_;
};

} // namespace network_comm