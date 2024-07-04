#include "network_comm/boost_asio_adapter.hpp"
#include <iostream>
#include <stdexcept>

namespace network_comm {

BoostAsioAdapter::BoostAsioAdapter()
    : socket_(std::make_unique<boost::asio::ip::tcp::socket>(io_context_)),
      is_server_(false) {}

BoostAsioAdapter::~BoostAsioAdapter() {
    disconnect();
}

void BoostAsioAdapter::connect(const std::string& address) {
    try {
        boost::asio::ip::tcp::resolver resolver(io_context_);
        auto endpoint = resolver.resolve(address, "0");
        boost::asio::connect(*socket_, endpoint);
        is_server_ = false;
        std::cout << "Connect successful" << std::endl;
    } catch (const boost::system::system_error& e) {
        std::cerr << "Boost.Asio connect failed: " << e.what() << std::endl;
        throw std::runtime_error("Boost.Asio connect failed: " + std::string(e.what()));
    }
}

void BoostAsioAdapter::bind(const std::string& address) {
    try {
        boost::asio::ip::tcp::resolver resolver(io_context_);
        auto endpoint = *resolver.resolve(address, "0").begin();
        acceptor_ = std::make_unique<boost::asio::ip::tcp::acceptor>(io_context_, endpoint);
        acceptor_->accept(*socket_);
        is_server_ = true;
        std::cout << "Bind successful" << std::endl;
    } catch (const boost::system::system_error& e) {
        std::cerr << "Boost.Asio bind failed: " << e.what() << std::endl;
        throw std::runtime_error("Boost.Asio bind failed: " + std::string(e.what()));
    }
}

void BoostAsioAdapter::disconnect() {
    if (socket_->is_open()) {
        socket_->close();
    }
    if (acceptor_ && acceptor_->is_open()) {
        acceptor_->close();
    }
}

void BoostAsioAdapter::send(const std::vector<uint8_t>& data) {
    try {
        boost::asio::write(*socket_, boost::asio::buffer(data));
    } catch (const boost::system::system_error& e) {
        throw std::runtime_error("Boost.Asio send failed: " + std::string(e.what()));
    }
}

std::vector<uint8_t> BoostAsioAdapter::receive() {
    try {
        std::vector<uint8_t> data(1024);  // Adjust buffer size as needed
        size_t length = socket_->read_some(boost::asio::buffer(data));
        data.resize(length);
        return data;
    } catch (const boost::system::system_error& e) {
        throw std::runtime_error("Boost.Asio receive failed: " + std::string(e.what()));
    }
}

} // namespace network_comm