#include <rclcpp/rclcpp.hpp>
#include "auto_drive_msgs/msg/obstacle.hpp"
#include "auto_drive_msgs/msg/path.hpp"
#include "auto_drive_msgs/msg/vehicle_state.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

        vehicle_state_sub_ = this->create_subscription<auto_drive_msgs::msg::VehicleState>(
            "vehicle_state", 10, 
            std::bind(&AutoDriveVisualizerNode::vehicleStateCallback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_markers", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("visualization_path", 10);
        vehicle_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("vehicle_marker", 10);
    }

private:
    void detectedObstacleCallback(const auto_drive_msgs::msg::Obstacle::SharedPtr msg) {
        publishObstacleMarker(msg, 1.0, 0.0, 0.0,"detected_obstacles"); // Red for detected obstacles
    }

    void trackedObstacleCallback(const auto_drive_msgs::msg::Obstacle::SharedPtr msg) {
        publishObstacleMarker(msg, 0.0, 1.0, 0.0,"tracked_obstacles"); // Green for tracked obstacles
    }

    void vehicleStateCallback(const auto_drive_msgs::msg::VehicleState::SharedPtr msg) {
        publishVehicleMarker(msg);
    }

/**
 * The function `pathCallback` subscribes to a message containing a path, transforms it into a new
 * message with a specified frame ID, and publishes it.
 * 
 * @param msg auto_drive_msgs::msg::Path::SharedPtr msg
 */
    void pathCallback(const auto_drive_msgs::msg::Path::SharedPtr msg) {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->now();

        for (const auto& pose : msg->poses) {
            path.poses.push_back(pose);
        }

        path_pub_->publish(path);
    }

    /**
     * The function `publishObstacleMarker` publishes a visualization marker representing an obstacle
     * with specified color and namespace.
     * 
     * @param msg `msg` is a shared pointer to an `Obstacle` message from the `auto_drive_msgs`
     * package. This message likely contains information about an obstacle detected by a system, such
     * as its ID and position.
     * @param r The `r` parameter in the `publishObstacleMarker` function represents the red component
     * of the color used for visualizing the obstacle marker. It is a float value that should be
     * between 0.0 and 1.0, indicating the intensity of the red color in the marker.
     * @param g The parameter `g` in the `publishObstacleMarker` function represents the green
     * component of the RGB color used for visualizing the obstacle marker. It is a float value ranging
     * from 0.0 (no green) to 1.0 (full green intensity). By adjusting the `g
     * @param b The parameter `b` in the `publishObstacleMarker` function represents the blue component
     * of the color of the obstacle marker that will be visualized. It is used to set the blue value of
     * the marker's color in the RGB color space.
     * @param ns The `ns` parameter in the `publishObstacleMarker` function stands for the namespace of
     * the marker. Namespaces are used to group related markers together, allowing you to organize and
     * manage them more effectively. By setting a namespace for a marker, you can easily identify and
     * manipulate markers that belong to
     */
    void publishObstacleMarker(const auto_drive_msgs::msg::Obstacle::SharedPtr msg, float r, float g, float b,const std::string& ns) {
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
        
        marker.lifetime = rclcpp::Duration::from_seconds(5);
        marker_array.markers.push_back(marker);
        marker_pub_->publish(marker_array);
    }


    /**
     * The function `publishVehicleMarker` creates and publishes a visualization marker representing a
     * vehicle's position and orientation in a 3D space.
     * 
     * @param msg The `msg` parameter in the `publishVehicleMarker` function is of type
     * `auto_drive_msgs::msg::VehicleState::SharedPtr`. This parameter contains information about the
     * state of a vehicle, such as its position and orientation.
     */
    void publishVehicleMarker(const auto_drive_msgs::msg::VehicleState::SharedPtr msg) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "vehicle";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::MODIFY;

        marker.pose.position.x = msg->position_x;
        marker.pose.position.y = msg->position_y;
        marker.pose.position.z = 0.0;  // Assume the vehicle is on the ground plane

        tf2::Quaternion q;
        q.setRPY(0, 0, msg->yaw);  //Set vehicle orientation
        marker.pose.orientation = tf2::toMsg(q);

        marker.scale.x = 2.0;  // Arrow Length
        marker.scale.y = 0.5;  // Arrow Width
        marker.scale.z = 0.5;  // Arrow height

        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;  // blue
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        vehicle_marker_pub_->publish(marker);
    }
    
    rclcpp::Subscription<auto_drive_msgs::msg::Obstacle>::SharedPtr detected_obstacles_sub_;
    rclcpp::Subscription<auto_drive_msgs::msg::Obstacle>::SharedPtr tracked_obstacles_sub_;
    rclcpp::Subscription<auto_drive_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<auto_drive_msgs::msg::VehicleState>::SharedPtr vehicle_state_sub_;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vehicle_marker_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoDriveVisualizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}