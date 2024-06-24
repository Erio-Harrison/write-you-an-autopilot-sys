#include <rclcpp/rclcpp.hpp>
#include "auto_drive_msgs/msg/obstacle.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

class ObstacleDetectionNode : public rclcpp::Node 
{
public:
    ObstacleDetectionNode() : Node("obstacle_detection")
    {
        // 创建订阅者，订阅点云数据
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "simulated_pointcloud", 10, 
            std::bind(&ObstacleDetectionNode::pointcloud_callback, this, std::placeholders::_1));

        // 创建发布者，用于发布检测到的障碍物
        detected_obstacle_pub_ = this->create_publisher<auto_drive_msgs::msg::Obstacle>("detected_obstacles", 10);

        RCLCPP_INFO(this->get_logger(), "Obstacle Detection Node has been started.");
    }

private:
    struct Point {
        double x, y, z;
        Point(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    };

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::vector<Point> points;
        
        // 解析点云数据
        const float* data = reinterpret_cast<const float*>(&msg->data[0]);
        for (unsigned int i = 0; i < msg->width * msg->height; ++i) {
            Point p(data[i*3], data[i*3+1], data[i*3+2]);
            points.push_back(p);
        }

        // 执行聚类
        std::vector<Point> clusters = kMeansClustering(points, 5);  // 假设我们想要5个聚类

        // 发布聚类结果作为障碍物
        for (const auto& cluster : clusters) {
            auto obstacle_msg = auto_drive_msgs::msg::Obstacle();
            obstacle_msg.id = obstacle_id_++;
            obstacle_msg.position.x = cluster.x;
            obstacle_msg.position.y = cluster.y;
            obstacle_msg.position.z = cluster.z;

            detected_obstacle_pub_->publish(obstacle_msg);
            RCLCPP_INFO(this->get_logger(), "Published clustered obstacle with ID: %d at position (%.2f, %.2f, %.2f)",
                        obstacle_msg.id, obstacle_msg.position.x, obstacle_msg.position.y, obstacle_msg.position.z);
        }
    }

    std::vector<Point> kMeansClustering(const std::vector<Point>& points, int k, int maxIterations = 100) 
    {
        std::vector<Point> centroids(k);
        for (int i = 0; i < k; ++i) {
            centroids[i] = points[i];  // Initialize centroids with first k points
        }

        for (int iter = 0; iter < maxIterations; ++iter) {
            std::vector<std::vector<Point>> clusters(k);

            // Assign points to nearest centroid
            for (const auto& point : points) {
                int nearest = 0;
                double minDist = std::numeric_limits<double>::max();
                for (int i = 0; i < k; ++i) {
                    double dist = std::hypot(point.x - centroids[i].x, point.y - centroids[i].y, point.z - centroids[i].z);
                    if (dist < minDist) {
                        minDist = dist;
                        nearest = i;
                    }
                }
                clusters[nearest].push_back(point);
            }

            // Recalculate centroids
            bool changed = false;
            for (int i = 0; i < k; ++i) {
                if (clusters[i].empty()) continue;
                Point newCentroid;
                for (const auto& p : clusters[i]) {
                    newCentroid.x += p.x;
                    newCentroid.y += p.y;
                    newCentroid.z += p.z;
                }
                newCentroid.x /= clusters[i].size();
                newCentroid.y /= clusters[i].size();
                newCentroid.z /= clusters[i].size();
                if (newCentroid.x != centroids[i].x || newCentroid.y != centroids[i].y || newCentroid.z != centroids[i].z) {
                    changed = true;
                    centroids[i] = newCentroid;
                }
            }

            if (!changed) break;  // Convergence achieved
        }

        return centroids;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<auto_drive_msgs::msg::Obstacle>::SharedPtr detected_obstacle_pub_;
    int obstacle_id_ = 0;
};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}