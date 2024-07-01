#include <rclcpp/rclcpp.hpp>
#include <network_comm/zeromq_adapter.hpp>
#include <nlohmann/json.hpp>
#include "auto_drive_msgs/msg/vehicle_state.hpp"
#include "auto_drive_network_bridge/secure_communication.hpp"
#include <thread>
#include <mutex>
#include <vector>
#include <memory>
#include <atomic>

class Connection {
public:
    Connection(std::unique_ptr<SecureCommunication> comm)
        : secure_comm_(std::move(comm)), is_active_(true) {}

    bool send(const std::string& message) {
        return secure_comm_->send(message);
    }

    std::string receive() {
        return secure_comm_->receive();
    }

    void close() {
        is_active_ = false;
    }

    bool is_active() const {
        return is_active_;
    }

private:
    std::unique_ptr<SecureCommunication> secure_comm_;
    std::atomic<bool> is_active_;
};

class MockServerNode : public rclcpp::Node {
public:
    MockServerNode() : Node("mock_server_node"), should_exit_(false) {
        listener_thread_ = std::thread(&MockServerNode::listen_for_connections, this);

        comm_ = std::make_unique<network_comm::ZeroMQAdapter>();
        try {
            comm_->bind("tcp://0.0.0.0:5555");
            RCLCPP_INFO(this->get_logger(), "Mock Server Node bound to port 5555");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind: %s", e.what());
            return;
        }

        connection_handler_thread_ = std::thread(&MockServerNode::handle_connections, this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MockServerNode::serverLoop, this));
    }

    ~MockServerNode() {
        should_exit_ = true;
        if (listener_thread_.joinable()) {
            listener_thread_.join();
        }
        if (connection_handler_thread_.joinable()) {
            connection_handler_thread_.join();
        }
    }

private:
    void listen_for_connections() {
        while (!should_exit_) {
            auto secure_comm = std::make_unique<SecureCommunication>();
            if (secure_comm->initialize(true) && secure_comm->accept(8080)) {
                RCLCPP_INFO(this->get_logger(), "Accepted new connection");
                std::lock_guard<std::mutex> lock(connections_mutex_);
                connections_.push_back(std::make_unique<Connection>(std::move(secure_comm)));
            }
        }
    }

    void handle_connections() {
        while (!should_exit_) {
            std::vector<std::unique_ptr<Connection>> active_connections;
            {
                std::lock_guard<std::mutex> lock(connections_mutex_);
                for (auto it = connections_.begin(); it != connections_.end();) {
                    if ((*it)->is_active()) {
                        active_connections.push_back(std::move(*it));
                        it = connections_.erase(it);
                    } else {
                        ++it;
                    }
                }
            }

            for (auto& conn : active_connections) {
                try {
                    std::string received = conn->receive();
                    if (!received.empty()) {
                        process_message(received, *conn);
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error processing connection: %s", e.what());
                    conn->close();
                }
            }

            {
                std::lock_guard<std::mutex> lock(connections_mutex_);
                for (auto& conn : active_connections) {
                    if (conn->is_active()) {
                        connections_.push_back(std::move(conn));
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void process_message(const std::string& message, Connection& conn) {
        try {
            auto j = nlohmann::json::parse(message);
            RCLCPP_INFO(this->get_logger(), "Received vehicle state: x=%f, y=%f, yaw=%f",
                j["position_x"].get<double>(),
                j["position_y"].get<double>(),
                j["yaw"].get<double>());

            // Echo back the received data with a small modification
            j["position_x"] = j["position_x"].get<double>() + 1.0;
            std::string response = j.dump();
            conn.send(response);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing message: %s", e.what());
        }
    }

    void serverLoop() {
        try {
            auto received_data = comm_->receive();
            if (!received_data.empty()) {
                std::string json_str(received_data.begin(), received_data.end());
                auto j = nlohmann::json::parse(json_str);

                RCLCPP_INFO(this->get_logger(), "Received vehicle state via ZeroMQ: x=%f, y=%f, yaw=%f",
                    j["position_x"].get<double>(),
                    j["position_y"].get<double>(),
                    j["yaw"].get<double>());

                // Echo back the received data with a small modification
                j["position_x"] = j["position_x"].get<double>() + 1.0;
                json_str = j.dump();
                std::vector<uint8_t> response_data(json_str.begin(), json_str.end());
                comm_->send(response_data);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in server loop: %s", e.what());
        }
    }

    std::unique_ptr<network_comm::CommunicationInterface> comm_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::unique_ptr<Connection>> connections_;
    std::mutex connections_mutex_;
    std::thread listener_thread_;
    std::thread connection_handler_thread_;
    std::atomic<bool> should_exit_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockServerNode>());
    rclcpp::shutdown();
    return 0;
}