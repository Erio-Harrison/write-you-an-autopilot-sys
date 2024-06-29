#pragma once

#include "rclcpp/rclcpp.hpp"
#include "network_comm/zeromq_adapter.hpp"
#include "auto_drive_msgs/msg/vehicle_state.hpp"
#include "auto_drive_msgs/msg/control_command.hpp"
#include <memory>

namespace auto_drive_network_bridge {

class NetworkBridgeNode : public rclcpp::Node {
public:
    NetworkBridgeNode();

private:
    void vehicleStateCallback(const auto_drive_msgs::msg::VehicleState::SharedPtr msg);
    void receiveData();
    std::vector<uint8_t> serializeMessage(const auto_drive_msgs::msg::VehicleState& msg);
    void processReceivedData(const std::vector<uint8_t>& data);

    std::unique_ptr<network_comm::CommunicationInterface> comm_;
    rclcpp::Subscription<auto_drive_msgs::msg::VehicleState>::SharedPtr vehicle_state_sub_;
    rclcpp::Publisher<auto_drive_msgs::msg::ControlCommand>::SharedPtr control_command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace auto_drive_network_bridge