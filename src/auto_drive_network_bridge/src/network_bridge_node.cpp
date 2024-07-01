#include "auto_drive_network_bridge/network_bridge_node.hpp"
#include <nlohmann/json.hpp>

NetworkBridgeNode::NetworkBridgeNode() : Node("network_bridge_node") {
    // Initialize secure communication
    if (!secure_comm_.initialize(false)) {  // Initialize as client
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize secure communication");
        return;
    }
    if (!secure_comm_.connect("localhost", 8080)) {  // Connect to mock server
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to mock server");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Secure connection established with mock server");

    // Initialize ROS2 subscription to receive vehicle state messages
    vehicle_state_sub_ = this->create_subscription<auto_drive_msgs::msg::VehicleState>(
        "vehicle_state", 10, std::bind(&NetworkBridgeNode::vehicleStateCallback, this, std::placeholders::_1));

    // Initialize ZeroMQ communication
    comm_ = std::make_unique<network_comm::ZeroMQAdapter>();
    try {
        comm_->connect("tcp://localhost:5555");  // Connect to the remote server
        RCLCPP_INFO(this->get_logger(), "Connected to ZeroMQ server at localhost:5555");
    } catch (const zmq::error_t& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to ZeroMQ server: %s", e.what());
        return;
    }

    // Create a timer to periodically receive data from the remote server
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&NetworkBridgeNode::receiveRemoteData, this));
}

void NetworkBridgeNode::vehicleStateCallback(const auto_drive_msgs::msg::VehicleState::SharedPtr msg) {
    try {
        // Serialize the vehicle state message to JSON
        nlohmann::json j = {
            {"position_x", msg->position_x},
            {"position_y", msg->position_y},
            {"yaw", msg->yaw},
            {"velocity", msg->velocity},
            {"acceleration", msg->acceleration}
        };
        std::string json_str = j.dump();

        // Send data using secure communication
        if (!secure_comm_.send(json_str)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send data over secure communication");
            return;
        }

        // Send data using ZeroMQ
        std::vector<uint8_t> data(json_str.begin(), json_str.end());
        comm_->send(data);

        RCLCPP_INFO(this->get_logger(), "Vehicle state sent successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing vehicle state: %s", e.what());
    }
}

void NetworkBridgeNode::receiveRemoteData() {
    try {
        // Receive data from the secure communication
        std::string secure_data = secure_comm_.receive();
        if (!secure_data.empty()) {
            processReceivedData(secure_data, "Secure");
        }

        // Receive data from the ZeroMQ server
        auto zmq_data = comm_->receive();
        if (!zmq_data.empty()) {
            std::string json_str(zmq_data.begin(), zmq_data.end());
            processReceivedData(json_str, "ZeroMQ");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error receiving data: %s", e.what());
    }
}

void NetworkBridgeNode::processReceivedData(const std::string& json_str, const std::string& source) {
    try {
        auto j = nlohmann::json::parse(json_str);
        auto remote_state = std::make_unique<auto_drive_msgs::msg::VehicleState>();
        remote_state->position_x = j["position_x"];
        remote_state->position_y = j["position_y"];
        remote_state->yaw = j["yaw"];
        remote_state->velocity = j["velocity"];
        remote_state->acceleration = j["acceleration"];

        RCLCPP_INFO(this->get_logger(), "Received vehicle state from %s: x=%f, y=%f, yaw=%f",
                    source.c_str(), remote_state->position_x, remote_state->position_y, remote_state->yaw);

        // Here you can publish the received state to a ROS topic if needed
    } catch (const nlohmann::json::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error parsing JSON data from %s: %s", source.c_str(), e.what());
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NetworkBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}