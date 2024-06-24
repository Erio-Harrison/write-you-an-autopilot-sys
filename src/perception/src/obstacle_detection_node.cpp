#include <rclcpp/rclcpp.hpp>
#include "auto_drive_msgs/msg/obstacle.hpp"
#include <random>

class ObstacleDetectionNode : public rclcpp::Node
{
public:
    ObstacleDetectionNode() : Node("obstacle_detection"), gen(rd()), dis(-10.0, 10.0)
    {
        // 创建发布者，发布检测到的障碍物
        detected_obstacle_pub_ = this->create_publisher<auto_drive_msgs::msg::Obstacle>("detected_obstacles", 10);
        // 定时器定期生成随机障碍物
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ObstacleDetectionNode::publishRandomObstacle, this));

        RCLCPP_INFO(this->get_logger(), "Obstacle Detection Node has been started.");
    }

private:
    void publishRandomObstacle()
    {
        auto msg = auto_drive_msgs::msg::Obstacle();
        msg.id = obstacle_id_++;
        msg.position.x = dis(gen);
        msg.position.y = dis(gen);
        msg.position.z = 0.0;

        detected_obstacle_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published random obstacle with ID: %d at position (%.2f, %.2f)", msg.id, msg.position.x, msg.position.y);
    }

    rclcpp::Publisher<auto_drive_msgs::msg::Obstacle>::SharedPtr detected_obstacle_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int obstacle_id_ = 0;

    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<> dis;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
