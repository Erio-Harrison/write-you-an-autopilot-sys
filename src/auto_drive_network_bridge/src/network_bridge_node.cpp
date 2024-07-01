#include "auto_drive_network_bridge/network_bridge_node.hpp"
#include <nlohmann/json.hpp>

NetworkBridgeNode::NetworkBridgeNode() : Node("network_bridge_node") {

    vehicle_state_sub_ = this->create_subscription<auto_drive_msgs::msg::VehicleState>(

        "vehicle_state", 10, std::bind(&NetworkBridgeNode::vehicleStateCallback, this, std::placeholders::_1));

    remote_vehicle_state_pub_ = this->create_publisher<auto_drive_msgs::msg::VehicleState>(

        "remote_vehicle_state", 10);

    timer_ = this->create_wall_timer(

        std::chrono::milliseconds(100),

        std::bind(&NetworkBridgeNode::receiveRemoteData, this));

    comm_ = std::make_unique<network_comm::ZeroMQAdapter>();

    comm_->connect("tcp://localhost:5555");  // 连接到远程服务器

}

void NetworkBridgeNode::vehicleStateCallback(const auto_drive_msgs::msg::VehicleState::SharedPtr msg) {

    nlohmann::json j;

    j["position_x"] = msg->position_x;

    j["position_y"] = msg->position_y;

    j["yaw"] = msg->yaw;

    j["velocity"] = msg->velocity;

    j["acceleration"] = msg->acceleration;

    std::string json_str = j.dump();

    std::vector<uint8_t> data(json_str.begin(), json_str.end());

    comm_->send(data);

}

void NetworkBridgeNode::receiveRemoteData() {

    auto received_data = comm_->receive();

    if (!received_data.empty()) {

        std::string json_str(received_data.begin(), received_data.end());

        auto j = nlohmann::json::parse(json_str);

        auto remote_state = std::make_unique<auto_drive_msgs::msg::VehicleState>();

        remote_state->position_x = j["position_x"];

        remote_state->position_y = j["position_y"];

        remote_state->yaw = j["yaw"];

        remote_state->velocity = j["velocity"];

        remote_state->acceleration = j["acceleration"];

        remote_vehicle_state_pub_->publish(std::move(remote_state));

    }

}

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<NetworkBridgeNode>());

    rclcpp::shutdown();

    return 0;

}