#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "auto_drive_msgs/msg/localization_result.hpp"
#include "auto_drive_msgs/msg/vehicle_state.hpp"
#include <Eigen/Dense>
#include <cmath>

class GPSIMUFusionNode : public rclcpp::Node {
public:
    GPSIMUFusionNode() : Node("gps_imu_fusion_node"), ekf_initialized_(false) {
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps", 10, std::bind(&GPSIMUFusionNode::gpsCallback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10, std::bind(&GPSIMUFusionNode::imuCallback, this, std::placeholders::_1));
        
        loc_pub_ = this->create_publisher<auto_drive_msgs::msg::LocalizationResult>("localization", 10);
        vehicle_state_pub_ = this->create_publisher<auto_drive_msgs::msg::VehicleState>("vehicle_state", 10);
        
        initializeEKF();
        RCLCPP_INFO(this->get_logger(), "GPS IMU Fusion Node has been started.");
    }

private:
    void initializeEKF() {
        // 状态向量: [x, y, z, vx, vy, vz, qw, qx, qy, qz]
        state_ = Eigen::VectorXd::Zero(10);
        P_ = Eigen::MatrixXd::Identity(10, 10) * 100.0;  // 初始协方差
        Q_ = Eigen::MatrixXd::Identity(10, 10) * 0.1;    // 过程噪声
        R_gps_ = Eigen::MatrixXd::Identity(3, 3) * 5.0;  // GPS测量噪声
        R_imu_ = Eigen::MatrixXd::Identity(4, 4) * 0.1;  // IMU测量噪声 (仅状态)
    }

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (!ekf_initialized_) {
            initializeState(*msg);
            ekf_initialized_ = true;
            return;
        }

        Eigen::Vector3d gps_measurement = gpsToLocalCoordinates(*msg);
        updateEKF(gps_measurement);
        publishState();
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (!ekf_initialized_) return;

        predictEKF(*msg);
        
        Eigen::Vector4d imu_orientation(msg->orientation.w, msg->orientation.x, 
                                        msg->orientation.y, msg->orientation.z);
        updateEKFOrientation(imu_orientation);

        publishState();
    }

    void initializeState(const sensor_msgs::msg::NavSatFix& first_gps) {
        ref_lat_ = first_gps.latitude;
        ref_lon_ = first_gps.longitude;
        ref_alt_ = first_gps.altitude;
        
        state_.segment<3>(0) = Eigen::Vector3d::Zero();  // 初始位置设为原点
        state_.segment<4>(6) = Eigen::Vector4d(1, 0, 0, 0);  // 初始朝向
    }

    Eigen::Vector3d gpsToLocalCoordinates(const sensor_msgs::msg::NavSatFix& gps_msg) {
        const double R = 6371000; // 地球半径（米）
        double lat_diff = gps_msg.latitude - ref_lat_;
        double lon_diff = gps_msg.longitude - ref_lon_;
        double alt_diff = gps_msg.altitude - ref_alt_;

        double x = R * std::cos(ref_lat_ * M_PI / 180.0) * lon_diff * M_PI / 180.0;
        double y = R * lat_diff * M_PI / 180.0;
        double z = alt_diff;

        // 将坐标映射到 -10 到 10 的范围
        const double scale = 20.0 / std::max({std::abs(x), std::abs(y), std::abs(z)});
        return Eigen::Vector3d(x * scale, y * scale, z * scale);
    }

    void predictEKF(const sensor_msgs::msg::Imu& imu_msg) {
        double dt = 0.1;  // 假设固定的时间步长，实际应该计算

        // 更新速度
        state_[3] += imu_msg.linear_acceleration.x * dt;
        state_[4] += imu_msg.linear_acceleration.y * dt;
        state_[5] += imu_msg.linear_acceleration.z * dt;

        // 更新位置
        state_[0] += state_[3] * dt;
        state_[1] += state_[4] * dt;
        state_[2] += state_[5] * dt;

        // 更新方向（这里需要一个更复杂的四元数积分方法）
        // 这只是一个简化的示例
        Eigen::Quaterniond q(state_[6], state_[7], state_[8], state_[9]);
        Eigen::Quaterniond dq(1, imu_msg.angular_velocity.x * dt / 2,
                                imu_msg.angular_velocity.y * dt / 2,
                                imu_msg.angular_velocity.z * dt / 2);
        q = q * dq;
        q.normalize();
        state_[6] = q.w();
        state_[7] = q.x();
        state_[8] = q.y();
        state_[9] = q.z();

        // 更新协方差
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(10, 10);
        F.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3) * dt;
        P_ = F * P_ * F.transpose() + Q_;
    }

    void updateEKF(const Eigen::Vector3d& gps_measurement) {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 10);
        H.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);

        Eigen::MatrixXd K = P_ * H.transpose() * (H * P_ * H.transpose() + R_gps_).inverse();
        state_ += K * (gps_measurement - state_.segment<3>(0));
        P_ = (Eigen::MatrixXd::Identity(10, 10) - K * H) * P_;
    }

    void updateEKFOrientation(const Eigen::Vector4d& imu_orientation) {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 10);
        H.block<4, 4>(0, 6) = Eigen::MatrixXd::Identity(4, 4);

        Eigen::MatrixXd K = P_ * H.transpose() * (H * P_ * H.transpose() + R_imu_).inverse();
        state_.segment<4>(6) += K * (imu_orientation - state_.segment<4>(6));
        P_ = (Eigen::MatrixXd::Identity(10, 10) - K * H) * P_;

        // 归一化四元数
        state_.segment<4>(6).normalize();
    }

    std::tuple<double, double, double> localToGPS(double x, double y, double z) {
        // 这个函数需要实现本地坐标到GPS坐标的转换
        // 这里是一个简化的示例，实际实现可能更复杂
        double lat = ref_lat_ + y / 111000.0;  // 简化：1度纬度约111km
        double lon = ref_lon_ + x / (111000.0 * std::cos(ref_lat_ * M_PI / 180.0));
        double alt = ref_alt_ + z;
        return {lat, lon, alt};
    }

    std::tuple<double, double, double> quaternionToAngularVelocity(const Eigen::Vector4d& q) {
        // 这个函数需要根据四元数的变化率来计算角速度
        // 这里返回一个空实现，实际应该根据四元数的导数计算角速度
        return {0.0, 0.0, 0.0};
    }

    void publishState() {
        auto loc_result = std::make_unique<auto_drive_msgs::msg::LocalizationResult>();

        // 使用状态向量中的本地坐标
        loc_result->latitude = state_[0];  // 将x坐标存储在latitude字段中
        loc_result->longitude = state_[1]; // 将y坐标存储在longitude字段中
        loc_result->altitude = state_[2];  // 将z坐标存储在altitude字段中

        // 填充位置协方差（这里简单地使用 P_ 矩阵的相应部分）
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                loc_result->position_covariance[i*3 + j] = P_(i, j);
            }
        }

        loc_result->orientation_w = state_[6];
        loc_result->orientation_x = state_[7];
        loc_result->orientation_y = state_[8];
        loc_result->orientation_z = state_[9];

        loc_result->linear_velocity_x = state_[3];
        loc_result->linear_velocity_y = state_[4];
        loc_result->linear_velocity_z = state_[5];

        // 假设我们从四元数计算角速度
        auto [ang_vel_x, ang_vel_y, ang_vel_z] = quaternionToAngularVelocity(state_.segment<4>(6));
        loc_result->angular_velocity_x = ang_vel_x;
        loc_result->angular_velocity_y = ang_vel_y;
        loc_result->angular_velocity_z = ang_vel_z;

        loc_pub_->publish(std::move(loc_result));

        // VehicleState 消息保持不变
        auto vehicle_state = std::make_unique<auto_drive_msgs::msg::VehicleState>();
        vehicle_state->position_x = state_[0];
        vehicle_state->position_y = state_[1];
        vehicle_state->yaw = 2 * std::atan2(state_[9], state_[6]);  // 从四元数计算偏航角
        vehicle_state->velocity = std::hypot(state_[3], state_[4]);
        
        // 计算加速度（如果状态向量中没有加速度，可能需要从IMU数据或其他来源获取）
        vehicle_state->acceleration = 0.0;  // 这里需要根据实际情况计算或获取加速度

        vehicle_state_pub_->publish(std::move(vehicle_state));
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<auto_drive_msgs::msg::LocalizationResult>::SharedPtr loc_pub_;
    rclcpp::Publisher<auto_drive_msgs::msg::VehicleState>::SharedPtr vehicle_state_pub_;

    Eigen::VectorXd state_;
    Eigen::MatrixXd P_, Q_, R_gps_, R_imu_;
    bool ekf_initialized_;
    double ref_lat_, ref_lon_, ref_alt_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSIMUFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}