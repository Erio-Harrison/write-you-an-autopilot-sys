#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <cmath>

class IMUSimulator : public rclcpp::Node {
public:
    IMUSimulator() : Node("imu_simulator"), count_(0) {
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&IMUSimulator::timer_callback, this));
    }

private:
    void timer_callback() {
        auto message = sensor_msgs::msg::Imu();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "imu";
        
        // 模拟简单的圆周运动
        double angular_velocity = 0.1;  // 弧度/秒
        
        message.angular_velocity.z = angular_velocity;
        message.linear_acceleration.x = -angular_velocity * angular_velocity * 10.0 * std::sin(angular_velocity * count_ * 0.01);
        message.linear_acceleration.y = angular_velocity * angular_velocity * 10.0 * std::cos(angular_velocity * count_ * 0.01);
        message.linear_acceleration.z = 9.81;  // 重力加速度

        publisher_->publish(message);
        count_++;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUSimulator>());
    rclcpp::shutdown();
    return 0;
}