#include "rclcpp/rclcpp.hpp"
#include "auto_drive_msgs/msg/path.hpp"
#include "auto_drive_msgs/msg/vehicle_state.hpp"
#include "auto_drive_msgs/msg/control_command.hpp"
#include <cmath>

class VehicleControlNode : public rclcpp::Node {
public:
    VehicleControlNode() : Node("vehicle_control_node"), current_path_index_(0), 
                           prev_error_(0.0), integral_error_(0.0) {
        path_sub_ = this->create_subscription<auto_drive_msgs::msg::Path>(
            "planned_path", 10, std::bind(&VehicleControlNode::pathCallback, this, std::placeholders::_1));
        
        vehicle_state_pub_ = this->create_publisher<auto_drive_msgs::msg::VehicleState>("vehicle_state", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&VehicleControlNode::controlLoop, this));
        
        // 初始化车辆状态（左上角）
        current_state_.position_x = -10.0;
        current_state_.position_y = -10.0;
        current_state_.yaw = 0.0;
        current_state_.velocity = 0.0;
        current_state_.acceleration = 0.0;

        // PID 参数
        kp_ = 1.0;  // 比例系数
        ki_ = 0.1;  // 积分系数
        kd_ = 0.05; // 微分系数
    }

private:
    void pathCallback(const auto_drive_msgs::msg::Path::SharedPtr msg) {
        current_path_ = *msg;
        current_path_index_ = 0;
        RCLCPP_INFO(this->get_logger(), "Received new path with %zu points", current_path_.poses.size());
    }

    void controlLoop() {
        if (current_path_.poses.empty() || current_path_index_ >= current_path_.poses.size()) return;

        auto target_pose = current_path_.poses[current_path_index_];
        
        // 计算误差
        double dx = target_pose.pose.position.x - current_state_.position_x;
        double dy = target_pose.pose.position.y - current_state_.position_y;
        double distance = std::sqrt(dx*dx + dy*dy);
        double target_yaw = std::atan2(dy, dx);

        // PID 控制
        double error = distance;
        integral_error_ += error * 0.1;  // 假设时间步长为0.1秒
        double derivative_error = (error - prev_error_) / 0.1;
        
        double control_signal = kp_ * error + ki_ * integral_error_ + kd_ * derivative_error;
        
        // 限制最大控制信号
        double max_control = 0.1;
        control_signal = std::clamp(control_signal, -max_control, max_control);

        // 更新位置
        current_state_.position_x += control_signal * std::cos(target_yaw);
        current_state_.position_y += control_signal * std::sin(target_yaw);

        // 更新朝向
        double yaw_diff = target_yaw - current_state_.yaw;
        current_state_.yaw += std::clamp(yaw_diff, -0.1, 0.1);  // 限制转向速度

        // 更新速度和加速度
        current_state_.velocity = control_signal / 0.1;  // 假设时间步长为0.1秒
        current_state_.acceleration = (current_state_.velocity - prev_velocity_) / 0.1;

        // 发布更新后的车辆状态
        auto vehicle_state_msg = std::make_unique<auto_drive_msgs::msg::VehicleState>();
        *vehicle_state_msg = current_state_;
        vehicle_state_pub_->publish(std::move(vehicle_state_msg));

        RCLCPP_INFO(this->get_logger(), "Vehicle state: x=%.2f, y=%.2f, yaw=%.2f, velocity=%.2f",
                    current_state_.position_x, current_state_.position_y, current_state_.yaw,
                    current_state_.velocity);

        // 检查是否到达当前目标点
        if (distance < 0.1) {
            current_path_index_++;
            integral_error_ = 0.0;  // 重置积分误差
            if (current_path_index_ >= current_path_.poses.size()) {
                RCLCPP_INFO(this->get_logger(), "Vehicle has reached the destination");
            }
        }

        // 更新上一次的误差和速度
        prev_error_ = error;
        prev_velocity_ = current_state_.velocity;
    }

    rclcpp::Subscription<auto_drive_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<auto_drive_msgs::msg::VehicleState>::SharedPtr vehicle_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    auto_drive_msgs::msg::Path current_path_;
    auto_drive_msgs::msg::VehicleState current_state_;
    size_t current_path_index_;

    // PID 控制相关变量
    double kp_, ki_, kd_;
    double prev_error_;
    double integral_error_;
    double prev_velocity_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleControlNode>());
  rclcpp::shutdown();
  return 0;
}