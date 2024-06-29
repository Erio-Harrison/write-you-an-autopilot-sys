#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <cmath>

class GPSSimulator : public rclcpp::Node {
public:
    GPSSimulator() : Node("gps_simulator"), count_(0) {
        publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GPSSimulator::timer_callback, this));
    }

private:
    void timer_callback() {
        auto message = sensor_msgs::msg::NavSatFix();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "gps";
        
        // 模拟一个简单的圆形轨迹
        double radius = 10.0;  // 米
        double angular_velocity = 0.1;  // 弧度/秒
        
        message.latitude = 40.0 + radius * std::cos(angular_velocity * count_) / 111000.0;  // 简化的度到米的转换
        message.longitude = -80.0 + radius * std::sin(angular_velocity * count_) / 111000.0;
        message.altitude = 100.0;  // 固定高度

        publisher_->publish(message);
        count_++;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSSimulator>());
    rclcpp::shutdown();
    return 0;
}