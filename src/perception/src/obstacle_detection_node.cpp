#include <rclcpp/rclcpp.hpp>
#include "auto_drive_msgs/msg/obstacle.hpp"
#include <random>
#include <vector>
#include <cmath>
#include <algorithm>

class ObstacleDetectionNode : public rclcpp::Node {
public:
    ObstacleDetectionNode() : Node("obstacle_detection"), gen(rd()), dis(-10.0, 10.0)
    {
        detected_obstacle_pub_ = this->create_publisher<auto_drive_msgs::msg::Obstacle>("detected_obstacles", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ObstacleDetectionNode::publishClusteredObstacles, this));

        RCLCPP_INFO(this->get_logger(), "Obstacle Detection Node has been started.");
    }

private:
    struct Point {
        double x, y;
        Point(double x = 0, double y = 0) : x(x), y(y) {}
    };

    std::vector<Point> generateRandomPoints(int n) {
        std::vector<Point> points;
        for (int i = 0; i < n; ++i) {
            points.emplace_back(dis(gen), dis(gen));
        }
        return points;
    }

    std::vector<Point> kMeansClustering(const std::vector<Point>& points, int k, int maxIterations = 100) {
        std::vector<Point> centroids(k);
        for (int i = 0; i < k; ++i) {
            centroids[i] = points[i]; // Initialize centroids with first k points
        }

        for (int iter = 0; iter < maxIterations; ++iter) {
            std::vector<std::vector<Point>> clusters(k);

            // Assign points to nearest centroid
            for (const auto& point : points) {
                int nearest = 0;
                double minDist = std::numeric_limits<double>::max();
                for (int i = 0; i < k; ++i) {
                    double dist = std::hypot(point.x - centroids[i].x, point.y - centroids[i].y);
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
                }
                newCentroid.x /= clusters[i].size();
                newCentroid.y /= clusters[i].size();
                if (newCentroid.x != centroids[i].x || newCentroid.y != centroids[i].y) {
                    changed = true;
                    centroids[i] = newCentroid;
                }
            }

            if (!changed) break; // Convergence achieved
        }

        return centroids;
    }

    void publishClusteredObstacles()
    {
        std::vector<Point> points = generateRandomPoints(50); // Generate 50 random points
        std::vector<Point> clusters = kMeansClustering(points, 5); // Cluster into 5 groups

        for (const auto& cluster : clusters) {
            auto msg = auto_drive_msgs::msg::Obstacle();
            msg.id = obstacle_id_++;
            msg.position.x = cluster.x;
            msg.position.y = cluster.y;
            msg.position.z = 0.0;

            detected_obstacle_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published clustered obstacle with ID: %d at position (%.2f, %.2f)", 
                        msg.id, msg.position.x, msg.position.y);
        }
    }

    rclcpp::Publisher<auto_drive_msgs::msg::Obstacle>::SharedPtr detected_obstacle_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int obstacle_id_ = 0;

    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<> dis;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}