#include <rclcpp/rclcpp.hpp>
#include "auto_drive_msgs/msg/obstacle.hpp"
#include "auto_drive_msgs/msg/path.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>

class AutoDriveVisualizerNode : public rclcpp::Node {
public:
    AutoDriveVisualizerNode() : Node("auto_drive_visualizer") {
        detected_obstacles_sub_ = this->create_subscription<auto_drive_msgs::msg::Obstacle>(
            "detected_obstacles", 10, 
            std::bind(&AutoDriveVisualizerNode::detectedObstacleCallback, this, std::placeholders::_1));
        
        tracked_obstacles_sub_ = this->create_subscription<auto_drive_msgs::msg::Obstacle>(
            "tracked_obstacles", 10, 
            std::bind(&AutoDriveVisualizerNode::trackedObstacleCallback, this, std::placeholders::_1));
        
        path_sub_ = this->create_subscription<auto_drive_msgs::msg::Path>(
            "planned_path", 10, 
            std::bind(&AutoDriveVisualizerNode::pathCallback, this, std::placeholders::_1));
        
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_markers", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("visualization_path", 10);
    }

private:
    void detectedObstacleCallback(const auto_drive_msgs::msg::Obstacle::SharedPtr msg) {
        publishObstacleMarker(msg, 0, 1.0, 0.0, 0.0,"detected_obstacles"); // Red for detected obstacles
    }

    void trackedObstacleCallback(const auto_drive_msgs::msg::Obstacle::SharedPtr msg) {
        publishObstacleMarker(msg, 1, 0.0, 1.0, 0.0,"tracked_obstacles"); // Green for tracked obstacles
    }

    void pathCallback(const auto_drive_msgs::msg::Path::SharedPtr msg) {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->now();

        for (const auto& pose : msg->poses) {
            path.poses.push_back(pose);
        }

        path_pub_->publish(path);
    }

    void publishObstacleMarker(const auto_drive_msgs::msg::Obstacle::SharedPtr msg, int id_offset, float r, float g, float b,const std::string& ns) {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = msg->id + id_offset * 1000; // Use offset to differentiate detected and tracked obstacles
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position = msg->position;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.7;
        marker.scale.y = 0.7;
        marker.scale.z = 0.7;
        
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        
        marker_array.markers.push_back(marker);
        marker_pub_->publish(marker_array);
    }

    rclcpp::Subscription<auto_drive_msgs::msg::Obstacle>::SharedPtr detected_obstacles_sub_;
    rclcpp::Subscription<auto_drive_msgs::msg::Obstacle>::SharedPtr tracked_obstacles_sub_;
    rclcpp::Subscription<auto_drive_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoDriveVisualizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}