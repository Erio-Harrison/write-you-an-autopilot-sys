#include <rclcpp/rclcpp.hpp>
#include "auto_drive_msgs/msg/obstacle.hpp"
#include "auto_drive_msgs/msg/path.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include "auto_drive_msgs/msg/localization_result.hpp"

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

        localization_sub_ = this->create_subscription<auto_drive_msgs::msg::LocalizationResult>(
            "localization", 10, 
            std::bind(&AutoDriveVisualizerNode::localizationCallback, this, std::placeholders::_1));
        
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_markers", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("visualization_path", 10);
        vehicle_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("vehicle_marker", 10);
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

    void localizationCallback(const auto_drive_msgs::msg::LocalizationResult::SharedPtr msg) {
        publishVehicleMarker(msg);
    }

    void publishObstacleMarker(const auto_drive_msgs::msg::Obstacle::SharedPtr msg, int id_offset, float r, float g, float b,const std::string& ns) {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = msg->id;
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

    void publishVehicleMarker(const auto_drive_msgs::msg::LocalizationResult::SharedPtr msg) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";  // 确保这与RViz中的Fixed Frame一致
        marker.header.stamp = this->now();
        marker.ns = "vehicle";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // 直接使用本地坐标
        marker.pose.position.x = msg->latitude;  // x坐标存储在latitude字段中
        marker.pose.position.y = msg->longitude; // y坐标存储在longitude字段中
        marker.pose.position.z = msg->altitude;  // z坐标存储在altitude字段中
        
        marker.pose.orientation.x = msg->orientation_x;
        marker.pose.orientation.y = msg->orientation_y;
        marker.pose.orientation.z = msg->orientation_z;
        marker.pose.orientation.w = msg->orientation_w;
        
        marker.scale.x = 2.0;  // 箭头长度
        marker.scale.y = 0.2;  // 箭头宽度
        marker.scale.z = 0.2;  // 箭头高度
        
        marker.color.r = 1.0;  // 红色
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        marker.lifetime = rclcpp::Duration::from_seconds(0);  // 永久显示
        
        vehicle_marker_pub_->publish(marker);
    }

    rclcpp::Subscription<auto_drive_msgs::msg::Obstacle>::SharedPtr detected_obstacles_sub_;
    rclcpp::Subscription<auto_drive_msgs::msg::Obstacle>::SharedPtr tracked_obstacles_sub_;
    rclcpp::Subscription<auto_drive_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<auto_drive_msgs::msg::LocalizationResult>::SharedPtr localization_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vehicle_marker_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoDriveVisualizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}