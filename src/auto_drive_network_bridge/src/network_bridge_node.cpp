#include "auto_drive_network_bridge/network_bridge_node.hpp"

NetworkBridgeNode::NetworkBridgeNode() : Node("network_bridge_node"),
    last_send_time_(this->now()),
    send_test_messages_(false),
    latest_state_(nullptr)
    {
    // Initialize ROS2 subscription to receive vehicle state messages
    vehicle_state_sub_ = this->create_subscription<auto_drive_msgs::msg::VehicleState>(
        "vehicle_state", 10, std::bind(&NetworkBridgeNode::vehicleStateCallback, this, std::placeholders::_1));

    // Initialize ZeroMQ communication
    comm_ = std::make_unique<network_comm::ZeroMQAdapter>(zmq::socket_type::dealer);
    try {
        comm_->connect("tcp://localhost:5555");  // Connect to the remote server
        RCLCPP_INFO(this->get_logger(), "Connected to ZeroMQ server at localhost:5555");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to ZeroMQ server: %s", e.what());
        return;
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&NetworkBridgeNode::timerCallback, this));

    send_test_messages_ = false;

    RCLCPP_INFO(this->get_logger(), "Sending initial test message");
    sendTestMessage();
    RCLCPP_INFO(this->get_logger(), "Test timer created");
}

void NetworkBridgeNode::vehicleStateCallback(const auto_drive_msgs::msg::VehicleState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_ = msg;
}

void NetworkBridgeNode::timerCallback() {
    auto now = this->now();
    if (send_test_messages_) {
        sendTestMessage();
    } else if ((now - last_send_time_).seconds() >= 1.0) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (latest_state_) {
            sendVehicleState(latest_state_);
            last_send_time_ = now;
        }
    }
}

void NetworkBridgeNode::sendVehicleState(const auto_drive_msgs::msg::VehicleState::SharedPtr msg) {
    try {
        nlohmann::json j = {
            {"message_type", "vehicle_state"},
            {"position_x", msg->position_x},
            {"position_y", msg->position_y},
            {"yaw", msg->yaw},
            {"velocity", msg->velocity},
            {"acceleration", msg->acceleration}
        };
        std::string json_str = j.dump();

        RCLCPP_INFO(this->get_logger(), "Sending vehicle state: %s", json_str.c_str());
        std::vector<uint8_t> data(json_str.begin(), json_str.end());
        comm_->send(data);

        auto zmq_data = comm_->receive();
        if (!zmq_data.empty()) {
            std::string response_str(zmq_data.begin(), zmq_data.end());
            RCLCPP_INFO(this->get_logger(), "Received response: %s", response_str.c_str());
            processReceivedData(response_str);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in sendVehicleState: %s", e.what());
    }
}

void NetworkBridgeNode::processReceivedData(const std::string& json_str) {
    try {
        auto j = nlohmann::json::parse(json_str);
        auto remote_state = std::make_unique<auto_drive_msgs::msg::VehicleState>();
        remote_state->position_x = j["position_x"];
        remote_state->position_y = j["position_y"];
        remote_state->yaw = j["yaw"];
        remote_state->velocity = j["velocity"];
        remote_state->acceleration = j["acceleration"];

        RCLCPP_INFO(this->get_logger(), "Processed received vehicle state: x=%f, y=%f, yaw=%f",
                    remote_state->position_x, remote_state->position_y, remote_state->yaw);

        // Here you can publish the received state to a ROS topic if needed
    } catch (const nlohmann::json::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error parsing JSON data: %s", e.what());
    }
}

void NetworkBridgeNode::sendTestMessage() {
    RCLCPP_INFO(this->get_logger(), "Timer triggered, preparing to send test message");
    try {
        nlohmann::json test_msg = {
            {"message_type", "test"},
            {"timestamp", this->now().seconds()}
        };
        std::string json_str = test_msg.dump();
        std::vector<uint8_t> data(json_str.begin(), json_str.end());
        
        RCLCPP_INFO(this->get_logger(), "Sending test message: %s", json_str.c_str());
        comm_->send(data);
        RCLCPP_INFO(this->get_logger(), "Test message sent, waiting for response");

        // Immediately receive the response
        auto response = comm_->receive();
        if (!response.empty()) {
            std::string response_str(response.begin(), response.end());
            RCLCPP_INFO(this->get_logger(), "Received response for test message: %s", response_str.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Received empty response for test message");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in sendTestMessage: %s", e.what());
    }
    RCLCPP_INFO(this->get_logger(), "sendTestMessage completed");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NetworkBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}