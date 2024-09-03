#ifndef AUTO_DRIVE_NETWORK_BRIDGE_NETWORK_BRIDGE_NODE_HPP_
#define AUTO_DRIVE_NETWORK_BRIDGE_NETWORK_BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <network_comm/zeromq_adapter.hpp>
#include <auto_drive_msgs/msg/vehicle_state.hpp>
#include <nlohmann/json.hpp>

class NetworkBridgeNode : public rclcpp::Node {
public:
    NetworkBridgeNode();

private:
    void vehicleStateCallback(const auto_drive_msgs::msg::VehicleState::SharedPtr msg);
    void processReceivedData(const std::string& json_str);
    void sendTestMessage();

    rclcpp::Subscription<auto_drive_msgs::msg::VehicleState>::SharedPtr vehicle_state_sub_;
    std::unique_ptr<network_comm::ZeroMQAdapter> comm_;
    rclcpp::TimerBase::SharedPtr test_timer_;
};

#endif  // AUTO_DRIVE_NETWORK_BRIDGE_NETWORK_BRIDGE_NODE_HPP_