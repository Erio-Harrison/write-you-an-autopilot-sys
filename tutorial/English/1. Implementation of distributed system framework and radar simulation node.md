![result](/asset/system_design.png)

# Introduction to distributed system framework

As mentioned in the introduction, our system is broadly divided into three main modules: perception, planning, and control. For ease of development, we often subdivide these modules further into submodules. For instance, the perception module is further divided into the detection module (ObstacleDetectionNode) and the tracking module (ObstacleTrackingNode). The former is used to simulate initial obstacle detection, while the latter is used to simulate tracking obstacles.

In the data points detected by hardware, there are often many invalid data points. By separating these functions, we can perform initial processing in the detection module and then track the actual valid data points in the tracking module. For example, in our current system's simplified implementation, we use a simple K-means clustering algorithm in the detection module to filter the massive point cloud dataset and publish the valid data points to the tracking module. In the tracking module, we use a Kalman filter to predict and track the data.

Of course, all data originates from hardware. Here, we simulate data from hardware using the LidarSimulatorNode.

The ObstacleTrackingNode passes the tracked data to the path planning module (PathPlanningNode). The PathPlanningNode makes path plans based on the received obstacle data (we are currently using a simple A algorithm*) and then sends the planned path data to the vehicle control module (VehicleControlNode). The VehicleControlNode calculates commands (acceleration, deceleration, steering, etc.) based on the path plan and the current vehicle state, and updates the vehicle state (position, speed, acceleration, etc.).

All autonomous vehicles can receive information from the VehicleControlNode via the network communication module (NetworkBridgeNode) and then communicate with the simulated central server (MockServerNode).

The specific data types involved in the data transfer process are the built-in data types of ROS2, which will be detailed when the code is implemented (of course, looking directly at the architecture diagram should also help understand their general usage). We want to visualize this entire process, so ObstacleDetectionNode, ObstacleTrackingNode, PathPlanningNode, and VehicleControlNode will pass data to the AutoDriveVisualizerNode to let us see the results of our work.

By fully utilizing the data distribution service (DDS) of ROS2, we can achieve a distributed architecture. For instance, we can run the ObstacleDetectionNode on computer A and the ObstacleTrackingNode on computer B. These two nodes will discover each other through DDS's automatic discovery mechanism and communicate via the publish-subscribe pattern.

# lidar_simulator_node

Now let's start the code part. Assuming you have installed the ROS2 environment, let's create a workspace (that is, create a folder):

```bash
mkdir -p write-you-an-autopilot-sys/src
```

```bash
cd src
```

All functional modules in the ROS2 environment are implemented through dedicated functional packages. To create a new functional package, use the following syntax:

```bash
ros2 pkg create <The name of the new feature package> --build-type {cmake,ament_cmake,ament_python} --dependencies <Dependencies>
```

**pkg**: Indicates the functions related to the function package

**create**：Indicates the creation of a function package

**build-type**：Indicates whether the newly created function package is C++ or Python. If C++ or C is used, follow ament_cmake here. If Python is used, follow ament_python

**package_name**：The name of the new feature package

Let's create a package called `sensor_simulator` for `lidar_simulator_node`：

```bash
ros2 pkg create sensor_simulator --build-type ament_cmake --dependencies rclcpp
```

Enter the workspace **src** of this function package, create **lidar_simulator_node.cpp**, and then you can start writing C++ code happily! !

**lidar_simulator_node.cpp**The implementation logic is very simple, using random numbers to generate data. Think about it, our Node is simulating hardware data, so what data do we need?  let's first look at these three lines of code in the main function:

```bash
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSimulatorNode>());
    rclcpp::shutdown();
    return 0;
}
```

Here, we first call the `rclcpp::init` function to initialize the ROS2 environment, then use `std::make_shared<LidarSimulatorNode>()` to create a **LidarSimulatorNode** object and call the `rclcpp::spin` function to enter the event loop. The `rclcpp::spin` function will continue to run until the node is shut down or the program is interrupted. When the node no longer needs to run, call the `rclcpp::shutdown` function to clean up resources and shut down the ROS2 environment.

Okay, so our focus is to construct a **LidarSimulatorNode** object, so the first thing we need to think about is what member variables should be in this object.

When the `rclcpp::spin` function runs and enters the event loop, what this node is doing is generating and publishing simulated hardware data. Pay attention to two things: **generating data** and **publishing data**.

So we should have a member variable for generating data and a member variable for publishing data. Let's first introduce the template classes for **publishing data** and **receiving data** in ROS2, so that we can focus on the development of module functions later without introducing this part of the syntax:

## Publisher

> In ROS2, publishing data is done through the template class `rclcpp::Publisher<data type>::SharedPtr`, for example:
>
> ```bash
> rclcpp::Publisher<data type>::SharedPtr publisher = this->create_publisher<example>("code", 10);
> ```
>
> 1. `rclcpp::Publisher<data type>`: This is a template class in ROS2, used to create a publisher. The data type is the type of message you want to publish, such as `std_msgs::msg::String` or `sensor_msgs::msg::Image`, etc.
>
> 2. `publisher`: This is a smart pointer (SharedPtr) pointing to the `rclcpp::Publisher` object, used to manage the life cycle of the publisher.
>
> 3. `this->create_publisher<example>("code", 10)`: This is to call the `create_publisher` method of the current object (usually a node) to create a publisher. `example` is the type of message you want to publish, "code" is the topic name of the publisher, and 10 is the queue size, which indicates the number of messages that the publisher can store.
>
> Then call `publisher->publish(data_example)`, so that this publisher will publish the data `data_example` to the ROS2 system.

## Subscriber

> In ROS2, data subscription is done through the `rclcpp::Subscription<Data Type>::SharedPtr` template class, for example:
>
> ```bash
> rclcpp::Subscription<Data Type>::SharedPtr subscription = this->create_subscription<example>("code", 10, std::bind(&ClassName::callback, this, std::placeholders::_1));
> ```
>
> 1. `rclcpp::Subscription<Data Type>`: This is a template class in ROS2, used to create subscribers. Data type is the type of message you want to subscribe to, such as `std_msgs::msg::String` or `sensor_msgs::msg::Image`, etc.
>
> 2. `subscription`: This is a smart pointer (SharedPtr) pointing to the `rclcpp::Subscription` object, used to manage the life cycle of the subscriber.
>
> 3. `this->create_subscription<example>("code", 10, std::bind(&ClassName::callback, this, std::placeholders::_1))`: This is to call the `create_subscription` method of the current object (usually a node) to create a subscriber. `example` is the message type you want to subscribe to, "code" is the name of the subscriber's topic, and 10 is the queue size, which indicates the number of messages that the subscriber can store. `std::bind(&ClassName::callback, this, std::placeholders::_1)` is the method to bind the callback function, which will be called when a message is received.
>
> In the callback function `callback`, you can handle the received message. For example:
>
> ```cpp
> void ClassName::callback(const example::SharedPtr msg) {
> // Process the received message
> }
> ```

## Note

Whether it is a publisher or a subscriber, the data type they pass needs to be a built-in type of ROS2, because the underlying transmission mechanism of ROS2 is a byte stream. If we want to pass our own custom data type, we need to manually complete serialization and deserialization.

Okay, after understanding how ROS2 publishes and receives data, we start to consider how to generate data.

From the perspective of actual hardware, the data we collect from devices such as LiDAR sensors and depth cameras is usually a data type of **point cloud data**. In the ROS2 environment, the `PointCloud2` message type is used to receive and process this data (i.e. `sensor_msgs::msg::PointCloud2`). This message type has the following basic elements:

1. header (std_msgs::Header)

- stamp: timestamp, indicating the acquisition time of the point cloud data

- frame_id: string, indicating the coordinate system of the point cloud data

2. height (uint32)

- The height of the point cloud. For unordered point clouds, this is usually set to 1

3. width (uint32)

- The width of the point cloud. For unordered point clouds, this represents the total number of points

4. fields (sensor_msgs::PointField[])

- The data structure describing each point. Each PointField contains:

- 1. name: field name (such as "x", "y", "z", "intensity", etc.)

- 2. offset: byte offset of the field in the point structure

- 3. datatype: data type (such as FLOAT32, UINT8, etc.)

- 4. count: number of elements in the field

5. is_bigendian (bool)

- Indicates whether the data is stored in big endian format

6. point_step (uint32)

- number of bytes occupied by a single point

7. row_step (uint32)

- number of bytes occupied by a row of points (for ordered point clouds)

8. data (uint8[])

- actual point cloud data, stored as a continuous byte array

9. is_dense (bool)

- Indicates whether the point cloud contains invalid points

What we mainly need are a lot of three-dimensional coordinates (x, y, z), which are the positions of obstacles detected by the hardware in the simulated reality. As for other more detailed data, you can further improve it according to your own ideas.

Generate random numbers through the combination of these three member variables:

```bash
std::random_device rd;
std::mt19937 gen;
std::uniform_real_distribution<> dis;
```

Note that the data field in `sensor_msgs::msg::PointCloud2` stores the byte array of point cloud data. Use `memcpy` to quickly copy data by directly operating memory. Since our data volume is large, this method is very suitable. So we write the function for generating data:

```bash
void publish_simulated_pointcloud()
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = this->now();
    cloud.header.frame_id = "lidar_frame";
    cloud.height = 1;
    cloud.width = 50;
    cloud.is_dense = true;
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

    for (size_t i = 0; i < cloud.height * cloud.width; ++i)
    {
        float x = dis(gen);
        float y = dis(gen);
        float z = dis(gen) / 2.0;

        memcpy(&cloud.data[i * cloud.point_step + 0], &x, sizeof(float));
        memcpy(&cloud.data[i * cloud.point_step + 4], &y, sizeof(float));
        memcpy(&cloud.data[i * cloud.point_step + 8], &z, sizeof(float));
    }

    publisher_->publish(cloud);
}
```

This function is also one of our member variables. We put `publisher_->publish(cloud)` directly in this function that generates data. After each set of data is generated, it is immediately published to the ROS2 system.

Another member variable is needed to control the frequency of our data publishing:

> `rclcpp::TimerBase::SharedPtr timer_` This member variable is used to store a smart pointer to the `rclcpp::TimerBase` object. `rclcpp::TimerBase` is a timer class in ROS2 that is used to trigger a callback function at a specific time interval. During initialization, the timer is bound to a callback function and a time interval is set. For example:
>
> ```cpp
> timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&NodeClass::callback_function, this));
> ```
>
> This means that when we run this node using `rclcpp::spin(std::make_shared<NodeClass>())`, the `callback_function` function will be called every second. In this way, we can perform some tasks regularly, such as publishing sensor data or performing periodic calculations.

Initialize member variables in the constructor. We generate a set of data every second. The coordinate range of these data is between (-10, 10) for later visualization.

```bash
LidarSimulatorNode() : Node("lidar_simulator"), gen(rd()), dis(-10.0, 10.0)
{
publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("simulated_pointcloud", 10);
timer_ = this->create_wall_timer(
std::chrono::seconds(1),
std::bind(&LidarSimulatorNode::publish_simulated_pointcloud, this));
}
```

Okay, here, our radar simulation node `lidar_simulator_node` is implemented. Pay attention to modify **cmakelist** and **package.xml** and add related dependencies.