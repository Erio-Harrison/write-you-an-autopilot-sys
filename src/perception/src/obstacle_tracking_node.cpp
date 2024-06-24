#include <rclcpp/rclcpp.hpp>
#include "auto_drive_msgs/msg/obstacle.hpp"

class ObstacleTrackingNode : public rclcpp::Node
{
public:
    ObstacleTrackingNode() : Node("obstacle_tracking")
    {
        // 创建订阅者，订阅检测到的障碍物
        obstacle_sub_ = this->create_subscription<auto_drive_msgs::msg::Obstacle>(
            "detected_obstacles", 10, 
            std::bind(&ObstacleTrackingNode::obstacleCallback, this, std::placeholders::_1));

        // 创建发布者，发布跟踪后的障碍物
        tracked_obstacle_pub_ = this->create_publisher<auto_drive_msgs::msg::Obstacle>("tracked_obstacles", 10);

        RCLCPP_INFO(this->get_logger(), "Obstacle Tracking Node has been started.");
    }

private:
    void obstacleCallback(const auto_drive_msgs::msg::Obstacle::SharedPtr msg)
    {
        // 这里应该实现障碍物跟踪的逻辑
        // 为了演示，我们只是简单地转发接收到的障碍物消息，并添加一些跟踪信息

        auto tracked_msg = *msg;
        tracked_msg.velocity.x = 0.5;  // 假设障碍物在x方向有0.5m/s的速度
        tracked_msg.velocity.y = 0.0;
        tracked_msg.velocity.z = 0.0;

        tracked_obstacle_pub_->publish(tracked_msg);
        RCLCPP_INFO(this->get_logger(), "Tracked and published an obstacle with ID: %d", msg->id);
    }

    rclcpp::Subscription<auto_drive_msgs::msg::Obstacle>::SharedPtr obstacle_sub_;
    rclcpp::Publisher<auto_drive_msgs::msg::Obstacle>::SharedPtr tracked_obstacle_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleTrackingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}