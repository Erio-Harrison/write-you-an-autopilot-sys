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
    void timerCallback();
    void sendVehicleState(const auto_drive_msgs::msg::VehicleState::SharedPtr msg);

    rclcpp::Subscription<auto_drive_msgs::msg::VehicleState>::SharedPtr vehicle_state_sub_;
    std::unique_ptr<network_comm::ZeroMQAdapter> comm_;

    rclcpp::Time last_send_time_;
    rclcpp::TimerBase::SharedPtr timer_;
    auto_drive_msgs::msg::VehicleState::SharedPtr latest_state_;

    std::mutex state_mutex_;
    std::atomic<bool> send_test_messages_;
};

#endif  // AUTO_DRIVE_NETWORK_BRIDGE_NETWORK_BRIDGE_NODE_HPP_