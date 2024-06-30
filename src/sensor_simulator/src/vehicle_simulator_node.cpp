#include "rclcpp/rclcpp.hpp"
#include "auto_drive_msgs/msg/path.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>
#include <vector>

class VehicleSimulator : public rclcpp::Node {
public:
    VehicleSimulator() : Node("vehicle_simulator"), current_index_(0) {
        path_sub_ = this->create_subscription<auto_drive_msgs::msg::Path>(
            "planned_path", 10, std::bind(&VehicleSimulator::pathCallback, this, std::placeholders::_1));
        
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&VehicleSimulator::timerCallback, this));
    }

private:
    void pathCallback(const auto_drive_msgs::msg::Path::SharedPtr msg) {
        path_ = msg->poses;
        current_index_ = 0;
    }

    void timerCallback() {
        if (path_.empty() || current_index_ >= path_.size()) return;

        auto current_pose = path_[current_index_];
        auto next_pose = (current_index_ + 1 < path_.size()) ? path_[current_index_ + 1] : current_pose;

        // 计算速度和加速度（这里使用简化的计算）
        double dt = 0.1;  // 假设时间步长为0.1秒
        double dx = next_pose.pose.position.x - current_pose.pose.position.x;
        double dy = next_pose.pose.position.y - current_pose.pose.position.y;
        double dz = next_pose.pose.position.z - current_pose.pose.position.z;
        
        double vx = dx / dt;
        double vy = dy / dt;
        double vz = dz / dt;

        // 发布GPS数据
        sensor_msgs::msg::NavSatFix gps_msg;
        gps_msg.header.stamp = this->now();
        gps_msg.header.frame_id = "gps";
        // 使用简化的坐标转换（这里假设1度约等于111km）
        gps_msg.latitude = current_pose.pose.position.y / 111000.0;
        gps_msg.longitude = current_pose.pose.position.x / (111000.0 * std::cos(gps_msg.latitude * M_PI / 180.0));
        gps_msg.altitude = current_pose.pose.position.z;
        gps_pub_->publish(gps_msg);

        // 发布IMU数据
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu";
        imu_msg.linear_acceleration.x = vx / dt;
        imu_msg.linear_acceleration.y = vy / dt;
        imu_msg.linear_acceleration.z = vz / dt + 9.81;  // 加上重力加速度
        imu_msg.angular_velocity.z = std::atan2(vy, vx);  // 简化的角速度计算
        imu_msg.orientation = current_pose.pose.orientation;
        imu_pub_->publish(imu_msg);

        current_index_++;
    }

    rclcpp::Subscription<auto_drive_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::PoseStamped> path_;
    size_t current_index_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleSimulator>());
    rclcpp::shutdown();
    return 0;
}