#include <rclcpp/rclcpp.hpp>
#include "auto_drive_msgs/msg/obstacle.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

class ObstacleDetectionNode : public rclcpp::Node 
{
public:
    ObstacleDetectionNode() : Node("obstacle_detection")
    {
        // Create a subscriber to subscribe to point cloud data
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "simulated_pointcloud", 10, 
            std::bind(&ObstacleDetectionNode::pointcloud_callback, this, std::placeholders::_1));

        // Create a publisher to publish detected obstacles
        detected_obstacle_pub_ = this->create_publisher<auto_drive_msgs::msg::Obstacle>("detected_obstacles", 10);

        RCLCPP_INFO(this->get_logger(), "Obstacle Detection Node has been started.");
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::vector<geometry_msgs::msg::Point> points;
        
        // Parsing point cloud data
        const float* data = reinterpret_cast<const float*>(&msg->data[0]);
        for (unsigned int i = 0; i < msg->width * msg->height; ++i) {
            geometry_msgs::msg::Point p;
            p.x = data[i*3];
            p.y = data[i*3+1];
            p.z = data[i*3+2];
            points.push_back(p);
        }

        // Perform clustering
        std::vector<geometry_msgs::msg::Point> clusters = kMeansClustering(points, 5);  // Suppose we want 5 clusters

        // Publish clustering results as obstacles
        for (const auto& cluster : clusters) {
            auto obstacle_msg = auto_drive_msgs::msg::Obstacle();
            obstacle_msg.id = obstacle_id_++;
            obstacle_msg.position = cluster;

            detected_obstacle_pub_->publish(obstacle_msg);
            RCLCPP_INFO(this->get_logger(), "Published clustered obstacle with ID: %d at position (%.2f, %.2f, %.2f)",
                        obstacle_msg.id, obstacle_msg.position.x, obstacle_msg.position.y, obstacle_msg.position.z);
        }
    }

    std::vector<geometry_msgs::msg::Point> kMeansClustering(const std::vector<geometry_msgs::msg::Point>& points, int k, int maxIterations = 50) 
    {
        std::vector<geometry_msgs::msg::Point> centroids(k);
        for (int i = 0; i < k; ++i) {
            centroids[i] = points[i];  // Initialize centroids with first k points
        }

        for (int iter = 0; iter < maxIterations; ++iter) {
            std::vector<std::vector<geometry_msgs::msg::Point>> clusters(k);

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
                geometry_msgs::msg::Point newCentroid;
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
