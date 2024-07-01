#include "auto_drive_network_bridge/network_bridge_node.hpp"
#include <nlohmann/json.hpp>

NetworkBridgeNode::NetworkBridgeNode() : Node("network_bridge_node") {

    // Initialize ROS2 subscription to receive vehicle state messages
    vehicle_state_sub_ = this->create_subscription<auto_drive_msgs::msg::VehicleState>(
        "vehicle_state", 10, std::bind(&NetworkBridgeNode::vehicleStateCallback, this, std::placeholders::_1));

    comm_ = std::make_unique<network_comm::ZeroMQAdapter>();
    try {
        comm_->connect("tcp://localhost:5555");// Connect to the remote server
        RCLCPP_INFO(this->get_logger(), "Connected to server at localhost:5555");
    } catch (const zmq::error_t& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect: %s", e.what());
        return;
    }

    // Create a timer to periodically receive data from the remote server
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&NetworkBridgeNode::receiveRemoteData, this));
}

void NetworkBridgeNode::vehicleStateCallback(const auto_drive_msgs::msg::VehicleState::SharedPtr msg) {

    try{
        // Serialize the vehicle state message to JSON
        nlohmann::json j;
        j["position_x"] = msg->position_x;
        j["position_y"] = msg->position_y;
        j["yaw"] = msg->yaw;
        j["velocity"] = msg->velocity;
        j["acceleration"] = msg->acceleration;
        std::string json_str = j.dump();
        
        // Convert JSON string to byte vector and send it over the network
        std::vector<uint8_t> data(json_str.begin(), json_str.end());
        comm_->send(data);
    }catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error sending data: %s", e.what());
    }
}

void NetworkBridgeNode::receiveRemoteData() {

    try{
        // Receive data from the remote server
        auto received_data = comm_->receive();
        if (!received_data.empty()) {
            // Deserialize the received JSON string to a VehicleState message
            std::string json_str(received_data.begin(), received_data.end());
            auto j = nlohmann::json::parse(json_str);
            auto remote_state = std::make_unique<auto_drive_msgs::msg::VehicleState>();
            remote_state->position_x = j["position_x"];
            remote_state->position_y = j["position_y"];
            remote_state->yaw = j["yaw"];
            remote_state->velocity = j["velocity"];
            remote_state->acceleration = j["acceleration"];
            
            // Process the received data as needed
            RCLCPP_INFO(this->get_logger(), "Received vehicle state: x=%f, y=%f, yaw=%f",
                        remote_state->position_x, remote_state->position_y, remote_state->yaw);
        }
    }catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error receiving data: %s", e.what());
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NetworkBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
