#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "auto_drive_msgs/msg/localization_result.hpp"
#include "auto_drive_msgs/msg/vehicle_state.hpp"
#include <Eigen/Dense>

class GPSIMUFusionNode : public rclcpp::Node {
public:
  GPSIMUFusionNode() : Node("gps_imu_fusion_node") {
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "gps", 10, std::bind(&GPSIMUFusionNode::gpsCallback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&GPSIMUFusionNode::imuCallback, this, std::placeholders::_1));
    
    loc_pub_ = this->create_publisher<auto_drive_msgs::msg::LocalizationResult>("localization", 10);
    vehicle_state_pub_ = this->create_publisher<auto_drive_msgs::msg::VehicleState>("vehicle_state", 10);
    
    initializeEKF();
  }

private:
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    updateEKF(msg);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    predictEKF(msg);
  }

  void initializeEKF() {
    // 初始化扩展卡尔曼滤波器
    // ...
  }

  void predictEKF(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    // EKF预测步骤
    // ...
    publishState();
  }

  void updateEKF(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg) {
    // EKF更新步骤
    // ...
    publishState();
  }

  void publishState() {
    auto_drive_msgs::msg::LocalizationResult loc_result;
    // 填充 loc_result ...
    loc_pub_->publish(loc_result);

    auto_drive_msgs::msg::VehicleState vehicle_state;
    // 从 EKF 状态填充 vehicle_state
    vehicle_state.position_x = state_(0);
    vehicle_state.position_y = state_(1);
    vehicle_state.yaw = state_(2);
    vehicle_state.velocity = state_(3);
    vehicle_state.acceleration = state_(4);  // 如果 EKF 状态包含加速度
    vehicle_state_pub_->publish(vehicle_state);
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<auto_drive_msgs::msg::LocalizationResult>::SharedPtr loc_pub_;
  rclcpp::Publisher<auto_drive_msgs::msg::VehicleState>::SharedPtr vehicle_state_pub_;

  Eigen::VectorXd state_;
  Eigen::MatrixXd covariance_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSIMUFusionNode>());
  rclcpp::shutdown();
  return 0;
}