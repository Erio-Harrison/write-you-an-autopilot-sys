#include "auto_drive_network_bridge/network_bridge_node.hpp"
#include <iostream>

namespace auto_drive_network_bridge {

NetworkBridgeNode::NetworkBridgeNode() : Node("network_bridge_node") {
    // 创建 ZeroMQ 适配器
    comm_ = std::make_unique<network_comm::ZeroMQAdapter>();
    
    // 从参数获取连接地址，如果没有设置，使用默认值
    std::string connect_address = this->declare_parameter("connect_address", "tcp://localhost:5555");
    comm_->connect(connect_address);

    // 创建订阅
    vehicle_state_sub_ = this->create_subscription<auto_drive_msgs::msg::VehicleState>(
        "vehicle_state", 10, 
        std::bind(&NetworkBridgeNode::vehicleStateCallback, this, std::placeholders::_1));

    // 创建发布器
    control_command_pub_ = this->create_publisher<auto_drive_msgs::msg::ControlCommand>(
        "control_command", 10);

    // 创建定时器用于接收数据
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&NetworkBridgeNode::receiveData, this));

    RCLCPP_INFO(this->get_logger(), "Network Bridge Node has been initialized.");
}

void NetworkBridgeNode::vehicleStateCallback(const auto_drive_msgs::msg::VehicleState::SharedPtr msg) {
    std::vector<uint8_t> data = serializeMessage(*msg);
    try {
        comm_->send(data);
        RCLCPP_DEBUG(this->get_logger(), "Sent vehicle state data.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send data: %s", e.what());
    }
}

void NetworkBridgeNode::receiveData() {
    try {
        std::vector<uint8_t> received_data = comm_->receive();
        if (!received_data.empty()) {
            processReceivedData(received_data);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to receive data: %s", e.what());
    }
}

std::vector<uint8_t> NetworkBridgeNode::serializeMessage(const auto_drive_msgs::msg::VehicleState& msg) {
    // 这里应该实现消息序列化逻辑
    // 为简单起见，这里只是将一些数据放入vector
    std::vector<uint8_t> serialized_data;
    serialized_data.push_back(static_cast<uint8_t>(msg.position_x));
    serialized_data.push_back(static_cast<uint8_t>(msg.position_y));
    // ... 添加其他字段
    return serialized_data;
}

void NetworkBridgeNode::processReceivedData(const std::vector<uint8_t>& data) {
    // 这里应该实现数据处理逻辑
    // 为简单起见，这里只是创建一个ControlCommand消息并发布
    auto control_command = std::make_unique<auto_drive_msgs::msg::ControlCommand>();
    if (!data.empty()) {
        control_command->steering_angle = static_cast<double>(data[0]);
        if (data.size() > 1) {
            control_command->acceleration = static_cast<double>(data[1]);
        }
    }
    control_command_pub_->publish(std::move(control_command));
}

} // namespace auto_drive_network_bridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<auto_drive_network_bridge::NetworkBridgeNode>());
    rclcpp::shutdown();
    return 0;
}