#pragma once
#include <rclcpp/rclcpp.hpp>
#include <auto_drive_msgs/msg/vehicle_state.hpp>
#include <network_comm/zeromq_adapter.hpp>

class NetworkBridgeNode : public rclcpp::Node {
public:
    NetworkBridgeNode();

private:
    void vehicleStateCallback(const auto_drive_msgs::msg::VehicleState::SharedPtr msg);
    void receiveRemoteData();

    rclcpp::Subscription<auto_drive_msgs::msg::VehicleState>::SharedPtr vehicle_state_sub_;
    rclcpp::Publisher<auto_drive_msgs::msg::VehicleState>::SharedPtr remote_vehicle_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<network_comm::ZeroMQAdapter> comm_;
};