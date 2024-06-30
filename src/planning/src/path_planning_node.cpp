#include <rclcpp/rclcpp.hpp>
#include "auto_drive_msgs/msg/obstacle.hpp"
#include "auto_drive_msgs/msg/path.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

struct AStarNode {
    int x, y;
    double g, h;
    AStarNode* parent;

    // Default constructor
    AStarNode()
        : x(0), y(0), g(0.0), h(0.0), parent(nullptr) {}

    // Parameterized constructor
    AStarNode(int x, int y, double g, double h, AStarNode* parent = nullptr)
        : x(x), y(y), g(g), h(h), parent(parent) {}

    double f() const { return g + h; }

    bool operator>(const AStarNode& other) const {
        return f() > other.f();
    }
};

class PathPlanningNode : public rclcpp::Node
{
public:
    PathPlanningNode() : Node("path_planning")
    {
        // 订阅跟踪后的障碍物信息
        obstacle_sub_ = this->create_subscription<auto_drive_msgs::msg::Obstacle>(
            "tracked_obstacles", 10, 
            std::bind(&PathPlanningNode::obstacleCallback, this, std::placeholders::_1));

        // 发布规划后的路径
        path_pub_ = this->create_publisher<auto_drive_msgs::msg::Path>("planned_path", 10);
        RCLCPP_INFO(this->get_logger(), "Path Planning Node has been started.");
    }

private:
    void obstacleCallback(const auto_drive_msgs::msg::Obstacle::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received obstacle with ID: %d at position (%.2f, %.2f)", msg->id, msg->position.x, msg->position.y);

        auto path_msg = std::make_unique<auto_drive_msgs::msg::Path>();
        path_msg->header.stamp = this->now();
        path_msg->header.frame_id = "map";

        // 假设起点和终点
        int start_x = -10, start_y = -10;
        int end_x = 10, end_y = 10;

        // 执行A*算法来计算路径
        std::vector<std::pair<int, int>> path = aStar(start_x, start_y, end_x, end_y, msg->position);

        for (const auto& [x, y] : path)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg->header;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_msg->poses.push_back(pose);
        }

        path_pub_->publish(std::move(path_msg));
        RCLCPP_INFO(this->get_logger(), "Published path with A* algorithm.");
    }

    // A*算法实现
    std::vector<std::pair<int, int>> aStar(int start_x, int start_y, int end_x, int end_y, const geometry_msgs::msg::Point& obstacle)
    {
        auto heuristic = [](int x1, int y1, int x2, int y2) {
            return std::hypot(x2 - x1, y2 - y1);
        };

        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
        std::unordered_set<int> closed_list;
        std::unordered_map<int, AStarNode> all_nodes;

        AStarNode start(start_x, start_y, 0.0, heuristic(start_x, start_y, end_x, end_y));
        open_list.push(start);
        all_nodes[start_x * 100 + start_y] = start;

        int directions[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

        while (!open_list.empty())
        {
            AStarNode current = open_list.top();
            open_list.pop();
            int current_id = current.x * 100 + current.y;

            if (current.x == end_x && current.y == end_y)
            {
                std::vector<std::pair<int, int>> path;
                AStarNode* node = &all_nodes[current_id];
                while (node != nullptr)
                {
                    path.emplace_back(node->x, node->y);
                    node = node->parent;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            closed_list.insert(current_id);

            for (auto& direction : directions)
            {
                int new_x = current.x + direction[0];
                int new_y = current.y + direction[1];
                int neighbor_id = new_x * 100 + new_y;

                if (closed_list.find(neighbor_id) != closed_list.end())
                {
                    continue;
                }

                // 跳过障碍物位置
                if (new_x == static_cast<int>(obstacle.x) && new_y == static_cast<int>(obstacle.y))
                {
                    continue;
                }

                double tentative_g = current.g + 1.0;
                double h = heuristic(new_x, new_y, end_x, end_y);

                if (all_nodes.find(neighbor_id) == all_nodes.end() || tentative_g < all_nodes[neighbor_id].g)
                {
                    AStarNode neighbor(new_x, new_y, tentative_g, h, &all_nodes[current_id]);
                    open_list.push(neighbor);
                    all_nodes[neighbor_id] = neighbor;
                }
            }
        }

        return {}; // 未找到路径
    }

    rclcpp::Subscription<auto_drive_msgs::msg::Obstacle>::SharedPtr obstacle_sub_;
    rclcpp::Publisher<auto_drive_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlanningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}