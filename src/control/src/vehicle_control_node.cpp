#include "rclcpp/rclcpp.hpp"
#include "auto_drive_msgs/msg/path.hpp"
#include "auto_drive_msgs/msg/vehicle_state.hpp"
#include "auto_drive_msgs/msg/control_command.hpp"

class VehicleControlNode : public rclcpp::Node {
public:
  VehicleControlNode() : Node("vehicle_control_node") {
    // 订阅路径规划结果和车辆状态
    path_sub_ = this->create_subscription<auto_drive_msgs::msg::Path>(
      "planned_path", 10, std::bind(&VehicleControlNode::pathCallback, this, std::placeholders::_1));
    state_sub_ = this->create_subscription<auto_drive_msgs::msg::VehicleState>(
      "vehicle_state", 10, std::bind(&VehicleControlNode::stateCallback, this, std::placeholders::_1));
    
    // 发布控制命令
    control_pub_ = this->create_publisher<auto_drive_msgs::msg::ControlCommand>("control_command", 10);
    
    // 创建定时器,定期执行控制计算
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&VehicleControlNode::controlLoop, this));
  }

private:
  void pathCallback(const auto_drive_msgs::msg::Path::SharedPtr msg) {
    // 处理接收到的路径
    current_path_ = *msg;
  }

  void stateCallback(const auto_drive_msgs::msg::VehicleState::SharedPtr msg) {
    // 更新当前车辆状态
    current_state_ = *msg;
  }

  void controlLoop() {
    // 实现控制算法,如PID控制
    auto_drive_msgs::msg::ControlCommand cmd;
    // ... 计算转向角、加速度等
    control_pub_->publish(cmd);
  }

  rclcpp::Subscription<auto_drive_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<auto_drive_msgs::msg::VehicleState>::SharedPtr state_sub_;
  rclcpp::Publisher<auto_drive_msgs::msg::ControlCommand>::SharedPtr control_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  auto_drive_msgs::msg::Path current_path_;
  auto_drive_msgs::msg::VehicleState current_state_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleControlNode>());
  rclcpp::shutdown();
  return 0;
}