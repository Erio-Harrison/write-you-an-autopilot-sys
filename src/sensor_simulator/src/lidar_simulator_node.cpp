#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <random>
#include <vector>

class LidarSimulatorNode : public rclcpp::Node
{
public:
    LidarSimulatorNode() : Node("lidar_simulator"), gen(rd()), dis(-10.0, 10.0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("simulated_pointcloud", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&LidarSimulatorNode::publish_simulated_pointcloud, this));
    }

private:
    void publish_simulated_pointcloud()
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = this->now();
        cloud.header.frame_id = "lidar_frame";
        cloud.height = 1;
        cloud.width = 50;  // 50 points
        cloud.is_dense = true;
        //大端小端，false为小端
        cloud.is_bigendian = false;
        cloud.fields.resize(3);
        
        cloud.fields[0].name = "x";
        cloud.fields[0].offset = 0;
        cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[0].count = 1;
        
        cloud.fields[1].name = "y";
        cloud.fields[1].offset = 4;
        cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[1].count = 1;
        
        cloud.fields[2].name = "z";
        cloud.fields[2].offset = 8;
        cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[2].count = 1;

        cloud.point_step = 12;  // 4 bytes per float, 3 floats per point
        cloud.row_step = cloud.point_step * cloud.width;
        cloud.data.resize(cloud.row_step * cloud.height);

        for (size_t i = 0; i < cloud.width; ++i)
        {
            float x = dis(gen);
            float y = dis(gen);
            float z = dis(gen) / 2.0;  // z range is half of x and y

            memcpy(&cloud.data[i * cloud.point_step + 0], &x, sizeof(float));
            memcpy(&cloud.data[i * cloud.point_step + 4], &y, sizeof(float));
            memcpy(&cloud.data[i * cloud.point_step + 8], &z, sizeof(float));
        }

        publisher_->publish(cloud);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<> dis;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSimulatorNode>());
    rclcpp::shutdown();
    return 0;
}