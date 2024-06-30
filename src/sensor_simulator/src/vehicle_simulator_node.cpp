#include "rclcpp/rclcpp.hpp"
#include "auto_drive_msgs/msg/path.hpp"
#include "auto_drive_msgs/msg/vehicle_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>
#include <vector>

class VehicleSimulator : public rclcpp::Node {
public:
    VehicleSimulator() : Node("vehicle_simulator"), current_index_(0) {
        path_sub_ = this->create_subscription<auto_drive_msgs::msg::Path>(
            "planned_path", 10, std::bind(&VehicleSimulator::pathCallback, this, std::placeholders::_1));

        vehicle_state_pub_ = this->create_publisher<auto_drive_msgs::msg::VehicleState>("vehicle_state", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&VehicleSimulator::timerCallback, this));

        // 初始化车辆状态（左上角）
        current_state_.position_x = -10.0;
        current_state_.position_y = 10.0;
        current_state_.yaw = 0.0;
        current_state_.velocity = 0.0;
        current_state_.acceleration = 0.0;
    }

private:
    void pathCallback(const auto_drive_msgs::msg::Path::SharedPtr msg) {
        path_ = msg->poses;
        current_index_ = 0;
        RCLCPP_INFO(this->get_logger(), "Received new path with %zu points", path_.size());
    }

    void timerCallback() {
        if (path_.empty() || current_index_ >= path_.size()) return;

        auto current_pose = path_[current_index_];
        auto next_pose = (current_index_ + 1 < path_.size()) ? path_[current_index_ + 1] : current_pose;

        // 计算速度和加速度
        double dt = 0.1;  // 假设时间步长为0.1秒
        double dx = next_pose.pose.position.x - current_state_.position_x;
        double dy = next_pose.pose.position.y - current_state_.position_y;

        double distance = std::sqrt(dx*dx + dy*dy);
        double new_velocity = distance / dt;
        double acceleration = (new_velocity - current_state_.velocity) / dt;

        // 更新车辆状态
        current_state_.position_x += dx;
        current_state_.position_y += dy;
        current_state_.yaw = std::atan2(dy, dx);
        current_state_.velocity = new_velocity;
        current_state_.acceleration = acceleration;

        // 发布车辆状态
        auto vehicle_state_msg = std::make_unique<auto_drive_msgs::msg::VehicleState>();
        *vehicle_state_msg = current_state_;
        vehicle_state_pub_->publish(std::move(vehicle_state_msg));

        RCLCPP_INFO(this->get_logger(), "Vehicle state: x=%.2f, y=%.2f, yaw=%.2f, velocity=%.2f, acceleration=%.2f",
                    current_state_.position_x, current_state_.position_y, current_state_.yaw,
                    current_state_.velocity, current_state_.acceleration);

        current_index_++;

        // 检查是否到达终点（右下角）
        if (current_index_ >= path_.size()) {
            RCLCPP_INFO(this->get_logger(), "Vehicle has reached the destination");
        }
    }

    rclcpp::Subscription<auto_drive_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<auto_drive_msgs::msg::VehicleState>::SharedPtr vehicle_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::PoseStamped> path_;
    size_t current_index_;
    auto_drive_msgs::msg::VehicleState current_state_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleSimulator>());
    rclcpp::shutdown();
    return 0;
}