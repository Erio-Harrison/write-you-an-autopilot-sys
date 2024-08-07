#include <rclcpp/rclcpp.hpp>
#include "auto_drive_msgs/msg/obstacle.hpp"
#include <Eigen/Dense>
#include <unordered_map>

class KalmanFilter {
public:
    KalmanFilter() 
        : A(6, 6), H(3, 6), Q(6, 6), R(3, 3), P(6, 6), K(6, 3), I(6, 6) {
        // State transition matrix
        A << 1, 0, 0, dt, 0, 0,
             0, 1, 0, 0, dt, 0,
             0, 0, 1, 0, 0, dt,
             0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 1;

        // Observation Matrix
        H << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0;

        // Process noise covariance
        Q = Eigen::MatrixXd::Identity(6, 6) * 0.1;

        // Measurement noise covariance
        R = Eigen::MatrixXd::Identity(3, 3) * 0.1;

        // Initial estimate error covariance
        P = Eigen::MatrixXd::Identity(6, 6);

        I = Eigen::MatrixXd::Identity(6, 6);

        x = Eigen::VectorXd::Zero(6);
    }

    void predict() {
        x = A * x;
        P = A * P * A.transpose() + Q;
    }

    void update(const Eigen::Vector3d& z) {
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        x = x + K * (z - H * x);
        P = (I - K * H) * P;
    }

    Eigen::VectorXd getState() const { return x; }

private:
    static constexpr double dt = 0.1;  // Time Step
    Eigen::MatrixXd A, H, Q, R, P, K, I;
    Eigen::VectorXd x;
};

class ObstacleTrackingNode : public rclcpp::Node {
public:
    ObstacleTrackingNode() : Node("obstacle_tracking") {
        obstacle_sub_ = this->create_subscription<auto_drive_msgs::msg::Obstacle>(
            "detected_obstacles", 10, 
            std::bind(&ObstacleTrackingNode::obstacleCallback, this, std::placeholders::_1));

        tracked_obstacle_pub_ = this->create_publisher<auto_drive_msgs::msg::Obstacle>("tracked_obstacles", 10);

        RCLCPP_INFO(this->get_logger(), "Obstacle Tracking Node has been started.");
    }

private:
    /**
     * The `obstacleCallback` function tracks and publishes the position and velocity of obstacles
     * using a Kalman filter.
     * 
     * @param msg The `msg` parameter in the `obstacleCallback` function is of type
     * `auto_drive_msgs::msg::Obstacle::SharedPtr`, which is a shared pointer to an `Obstacle` message.
     * This message likely contains information about an obstacle detected by a sensor, such as its ID,
     * position
     */
    void obstacleCallback(const auto_drive_msgs::msg::Obstacle::SharedPtr msg) {
        auto it = trackers.find(msg->id);
        if (it == trackers.end()) {
            // New obstacle, initialize tracker
            trackers[msg->id] = std::make_unique<KalmanFilter>();
        }

        KalmanFilter& kf = *trackers[msg->id];

        //predict
        kf.predict();

        // renew
        Eigen::Vector3d measurement(msg->position.x, msg->position.y, msg->position.z);
        kf.update(measurement);

        // Get the updated status
        Eigen::VectorXd state = kf.getState();

        // Publish tracking results
        auto tracked_msg = *msg;
        tracked_msg.position.x = state(0);
        tracked_msg.position.y = state(1);
        tracked_msg.position.z = state(2);
        tracked_msg.velocity.x = state(3);
        tracked_msg.velocity.y = state(4);
        tracked_msg.velocity.z = state(5);
        
        tracked_obstacle_pub_->publish(tracked_msg);
        RCLCPP_INFO(this->get_logger(), "Tracked and published an obstacle with ID: %d", msg->id);
    }

    rclcpp::Subscription<auto_drive_msgs::msg::Obstacle>::SharedPtr obstacle_sub_;
    rclcpp::Publisher<auto_drive_msgs::msg::Obstacle>::SharedPtr tracked_obstacle_pub_;
    std::unordered_map<int, std::unique_ptr<KalmanFilter>> trackers;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleTrackingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}