# Detection module and clustering algorithm

The detection module does three things: receiving data->filtering data->publishing data

So we first need to create a member variable for each receiver and publisher:

```bash
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
rclcpp::Publisher<auto_drive_msgs::msg::Obstacle>::SharedPtr detected_obstacle_pub_;
```

The topic published by the previous simulated data node is **simulated_pointcloud**. When we initialize the subscriber here, we should have the same topic:

```bash
pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "simulated_pointcloud", 10, 
            std::bind(&ObstacleDetectionNode::pointcloud_callback, this, std::placeholders::_1));
```

The detection module will publish data to the tracking module later:

```bash
detected_obstacle_pub_ = this->create_publisher<auto_drive_msgs::msg::Obstacle>("detected_obstacles", 10);
```

The function to filter data is called after ObstacleDetectionNode receives the point cloud data: `std::bind(&ObstacleDetectionNode::pointcloud_callback)`. The focus of this module is on how to filter data. In our project, we use a clustering algorithm called k-means. In fact, you can use your creativity here to use **better algorithms** or introduce **deep learning models**, etc.

In addition, you can see a line `RCLCPP_INFO(this->get_logger(), "Obstacle Detection Node has been started.")`, which uses the RCLCPP_INFO macro in the rclcpp library (a C++ client library of ROS 2) to output log information in the C++ program. This is to let us see that our Node can work properly. Hey, this thing that outputs log information is very useful, we often use it for debugging.

When we pass the x, y, and z coordinates, we use the built-in type of ROS2: `Point`. Let's use this example to see a very obvious benefit of using built-in types, which can reduce our manual serialization and deserialization work:

```bash
#include <geometry_msgs/msg/point.hpp>

// ...

std::vector<geometry_msgs::msg::Point> points;

void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    const float* data = reinterpret_cast<const float*>(&msg->data[0]);
    for (unsigned int i = 0; i < msg->width * msg->height; ++i) {
        geometry_msgs::msg::Point p;
        p.x = data[i*3];
        p.y = data[i*3+1];
        p.z = data[i*3+2];
        points.push_back(p);
    }

    // ... 处理点云数据 ...

    // 发布障碍物
    auto obstacle_msg = auto_drive_msgs::msg::Obstacle();
    obstacle_msg.position = cluster; // 直接赋值，不需要额外处理
    detected_obstacle_pub_->publish(obstacle_msg);
}
```

```bash
struct Point {
    float x;
    float y;
    float z;
};

// ...

std::vector<Point> points;

void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    const float* data = reinterpret_cast<const float*>(&msg->data[0]);
    for (unsigned int i = 0; i < msg->width * msg->height; ++i) {
        Point p;
        p.x = data[i*3];
        p.y = data[i*3+1];
        p.z = data[i*3+2];
        points.push_back(p);
    }

    // ... 处理点云数据 ...

    // 发布障碍物
    auto obstacle_msg = auto_drive_msgs::msg::Obstacle();
    // 需要手动转换Point到geometry_msgs::msg::Point
    obstacle_msg.position.x = cluster.x;
    obstacle_msg.position.y = cluster.y;
    obstacle_msg.position.z = cluster.z;
    detected_obstacle_pub_->publish(obstacle_msg);
}
```

Note that we use a new type here: `auto_drive_msgs::msg::Obstacle`, which is our own definition of Obstacle.msg under auto_drive_msgs/msg:

```bash
# Unique identifier for the obstacle
uint32 id

# 3D position of the obstacle
geometry_msgs/Point position

# Velocity of the obstacle (if it's moving)
geometry_msgs/Vector3 velocity
```

We can combine the built-in types of ROS2 in this way to form a certain data structure we want. We only need to add relevant dependencies in cmakelist and package.xml when using it. For example, here, I define an id, a position and a velocity for the Obstacle data structure.

Next, let's learn about the k-means algorithm and its significance in our project. First, I recommend this website that visualizes the k-means algorithm process: [!k-means](https://www.naftaliharris.com/blog/visualizing-k-means-clustering/)

Why do we use clustering algorithms? Because in the actual business scenarios of autonomous driving, our hardware may recognize different objects, and the system will classify them when processing different data, such as other vehicles, pedestrians, animals, buildings, etc., and then make different behavior predictions for these different categories.

In our project, we can use `k-means` to extract a few more representative points from the large amount of point cloud data generated, and then predict their position changes, which can reduce the amount of calculation of our system. You can see the following picture:

![result](/asset/k-means.png)

In this example, we divide these numerous data points into five categories, and take their centroids as representative points to send to the tracking module.

When we use the `k-means` algorithm to classify in the project, we also divide it into five categories. Of course, you can decide how to process the data yourself. This is just a starting point.

The implementation steps of the `k-means` algorithm are roughly divided into:

1. Initialization: Select k initial center points (centroids). This is usually randomly selected, but other methods can also be used. I took the first k points for convenience

2. Distribution: Calculate the distance from all other points to these k points, and assign all points to a cluster among these k points

3. Update: For these k clusters, we recalculate the centroid, and the new centroid is the center position of all points

4. Repeat steps 2 and 3 until the termination condition is met

5. There are generally two termination conditions: reaching the maximum number of cycles or the centroid no longer changes

First parse the point cloud data and put them in a `vector`, then we will implement the `k-means` algorithm step by step

```bash
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
```

What we finally return is the k centroid points of a large number of point cloud data points:

```bash
std::vector<geometry_msgs::msg::Point> centroids(k);
```

1. Initialization

I just took the first k points:

```bash
for (int i = 0; i < k; ++i) {
    centroids[i] = points[i];  // Initialize centroids with first k points
}
```

2. Allocation

```bash
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
```

3. Update

```bash
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
```

4. Repeat (I set it to 50 times)

```bash
std::vector<geometry_msgs::msg::Point> kMeansClustering(const std::vector<geometry_msgs::msg::Point>& points, int k, int maxIterations = 50) 
```

5. Termination condition (looped 50 times or the center of mass position no longer changes)

```bash
if (!changed) break;  // Convergence achieved
```

Return our result: `return centroids`

After calling the `k-means` algorithm (`std::vector<geometry_msgs::msg::Point> clusters = kMeansClustering(points, 5);`), we publish our k representatives. Note that we also give these five landmarks a tag (obstacle_id_ ).

Good!! Data filtering is completed. Every time we receive a set of point cloud data, we divide them into k categories and publish them to the system:

```bash
for (const auto& cluster : clusters) {
    auto obstacle_msg = auto_drive_msgs::msg::Obstacle();
    obstacle_msg.id = obstacle_id_++;
    obstacle_msg.position = cluster;

    detected_obstacle_pub_->publish(obstacle_msg);
}
```

# Tracking module and deep learning

This part is actually not related to computer knowledge. I hope to complete this part in combination with **deep learning**. Friends who are learning can ignore this part for now. In short, after this module subscribes to data from the prediction module, it undergoes data prediction and then sends the predicted data to our system.

# Implementation of visualization node `auto_drive_visualizer_node`

In the visualization node module, we need to use RViz2, which is a visualization tool for ROS2. What we need to do is to subscribe to the corresponding topic in the system, and then use the `visualization_msgs::msg` data type to publish to the system, and then RViz2 will subscribe to the topic of these visualization messages and complete the rendering.

Everyone can understand the types of our various data structures, and then make numerical modifications to the corresponding components, and then the final visualization process can be completed. For example, the first thing we need to use is `visualization_msgs::msg::Marker`

It contains the following main fields:
- header: standard message header, including timestamp and coordinate system information
- ns: namespace, used to organize and distinguish different markers
- id: unique identifier of the marker
- type: marker type (such as CUBE, SPHERE, ARROW, etc.)
- action: operation performed on the marker (such as ADD, DELETE, MODIFY)
- pose: position and orientation of the marker
- scale: size of the marker
- color: color of the marker (RGBA)
- lifetime: lifetime of the marker
- frame_locked: whether it is locked in a specific coordinate system