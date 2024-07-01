#include <rclcpp/rclcpp.hpp>
#include <network_comm/zeromq_adapter.hpp>
#include <nlohmann/json.hpp>
#include "auto_drive_msgs/msg/vehicle_state.hpp"

class MockServerNode : public rclcpp::Node {
public:

    MockServerNode() : Node("mock_server_node") {
        comm_ = std::make_unique<network_comm::ZeroMQAdapter>();
        try {
            comm_->bind("tcp://0.0.0.0:5555");  // 绑定到所有接口的5555端口
            RCLCPP_INFO(this->get_logger(), "Mock Server Node bound to port 5555");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind: %s", e.what());
            return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MockServerNode::serverLoop, this));
    }

    void serverLoop() {

    try{
        auto received_data = comm_->receive();
        if (!received_data.empty()) {
            std::string json_str(received_data.begin(), received_data.end());
            auto j = nlohmann::json::parse(json_str);

            RCLCPP_INFO(this->get_logger(), "Received vehicle state: x=%f, y=%f, yaw=%f",
                j["position_x"].get<double>(),
                j["position_y"].get<double>(),
                j["yaw"].get<double>());

            // Echo back the received data with a small modification
            j["position_x"] = j["position_x"].get<double>() + 1.0;
            json_str = j.dump();
            std::vector<uint8_t> response_data(json_str.begin(), json_str.end());
            comm_->send(response_data);
            }
        }catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in server loop: %s", e.what());
        }
    }

    std::unique_ptr<network_comm::CommunicationInterface> comm_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockServerNode>());
    rclcpp::shutdown();
    return 0;
}