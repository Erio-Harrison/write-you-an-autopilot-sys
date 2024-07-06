# 规划模块（A*算法的实现）

在我们得到障碍物的数据之后，我们需要在地图上规划出一条最好的路径出来。简单的模型就是，从初始坐标运动到终止坐标，不触碰障碍物。

依旧是基本的发布者和订阅者模型，不多说～～这里需要关注`geometry_msgs::msg::PoseStamped`这个数据类型，我们会用它来表示带有时间戳的三维空间中的位置和方向，它有两部分组成：Header和Pose。

1. Header (std_msgs::msg::Header):

stamp: 时间戳，表示这个位姿信息被记录或创建的时间。
frame_id: 字符串，表示这个位姿所参照的坐标系。


2. Pose (geometry_msgs::msg::Pose):

- Position (geometry_msgs::msg::Point):

    x: double, x 坐标
    y: double, y 坐标
    z: double, z 坐标

- Orientation (geometry_msgs::msg::Quaternion):

    x: double, 四元数的 x 分量
    y: double, 四元数的 y 分量
    z: double, 四元数的 z 分量
    w: double, 四元数的 w 分量（实部）

我们已经知道`Point`的作用了，至于`Quaternion`:

>四元数是一种用于表示3D空间中旋转的数学工具，由四个分量组成：x, y, z, w。它可以表示为：q = w + xi + yj + zk，其中i, j, >k是虚数单位。

我们可以用它来表示机器人或者车辆的朝向，这里我们为了简化模型，单纯的把四元数的 w 分量为 1，这表示没有旋转（单位四元数）。其他分量（x, y, z）默认为 0。

我们项目当中的A*算法是基础版本，假设我们的车辆是在一个方格图当中行驶，并且行驶路线也是沿着格子线，在实际业务场景里，我们的车辆要考虑更多可能，一起学习的朋友可以根据自己的兴趣进一步完善。A*算法的基本原理可以参考这个： [A*算法的原理](https://www.bilibili.com/video/BV18h411e7dH/?spm_id_from=333.337.search-card.all.click&vd_source=b284da586203e097a82c06c5405042de)。值得一提的是，我们可以把A*算法看成是广度优先搜索算法和启发函数结合的一种算法，在稍后的实现里我们也可以看到这一点。

好，接下来我们来实现A*算法，首先明确A*算法的实现步骤：

1. 初始化：

- 创建开放列表（open list）和闭合列表（closed list）。
- 将起始节点加入开放列表。


2. 主循环：
当开放列表不为空时，重复以下步骤：
a. 从开放列表中选择f值最小的节点作为当前节点。f = g + h，其中：

- g 是从起点到当前节点的实际代价。
- h 是从当前节点到目标的估计代价（启发函数）。

b. 如果当前节点是目标节点，算法结束，返回路径。
c. 否则：

- 将当前节点从开放列表移到闭合列表。
- 对当前节点的每个相邻节点：

    如果该相邻节点在闭合列表中，忽略它。
    如果该相邻节点不在开放列表中，将其添加到开放列表。
    如果该相邻节点已经在开放列表中，检查是否这条路径到该相邻节点更好（g值更小）。如果是，更新该节点的父节点和g值。

3. 路径重建：

当找到目标节点时，从目标节点开始，通过每个节点的父节点指针，反向追踪到起始节点，即可得到完整路径。


4. 返回结果：

- 如果找到路径，返回该路径。
- 如果开放列表为空且未找到目标，则无路径可达，返回失败。

然后看我们这里的代码实现：

首先我们定义Node，复写了 **>** 这个操作符，这是为了方便后面我们利用优先队列实现最小堆：

```bash
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
```

1. 初始化：


```bash
auto heuristic = [](int x1, int y1, int x2, int y2) {
    return std::hypot(x2 - x1, y2 - y1);
};

std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
std::unordered_set<int> closed_list;
std::unordered_map<int, AStarNode> all_nodes;

AStarNode start(start_x, start_y, 0.0, heuristic(start_x, start_y, end_x, end_y));
open_list.push(start);
all_nodes[start_x * 100 + start_y] = start;
```

注意这里`all_nodes[start_x * 100 + start_y] = start`，我们给每个node附上一个唯一存在的标记，它可以是随机的，但是这里有一个限制条件，就是我们的网格必须是小于100x100，不然这个标记可能不唯一。

定义一个数组，之后用于遍历，来模拟上下左右的方向

```bash
int directions[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
```

2. 主循环：

开启主循环的代码, 从开放列表中选择f值最小的节点作为当前节点：

```bash
while (!open_list.empty())
{
    AStarNode current = open_list.top();
    open_list.pop();
    int current_id = current.x * 100 + current.y;
```

如果当前节点是目标节点，算法结束，返回路径：

```bash
if (current.x == end_x && current.y == end_y)
    {
        // 这里我们先空着，后面再解释怎么做
    }
```

将当前节点加入闭合列表：

```bash
closed_list.insert(current_id);
```

对当前相邻的每个节点：

需要检查节点是否在闭合列表中或是障碍物

```bash
for (auto& direction : directions)
    {
        int new_x = current.x + direction[0];
        int new_y = current.y + direction[1];
        int neighbor_id = new_x * 100 + new_y;

        if (closed_list.find(neighbor_id) != closed_list.end())
        {
            continue;
        }

        // Jump over obstacle position
        if (new_x == static_cast<int>(obstacle.x) && new_y == static_cast<int>(obstacle.y))
        {
            continue;
        }
```

计算到相邻节点的新g值和h值
如果是新节点或找到更好的路径，更新节点信息并加入开放列表

```bash
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
```

3. 路径重建：

```bash
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
```

4. 返回结果：

如果循环结束没找到路径，返回空路径:

```bash
return {}; // Path not found
```

事实上，即便是一个单独的算法，我们也可以从很多角度去改进它，比如，我们从追踪模块得到的速度，是否可以作为改进我们启发函数的一个因素？这方面一起学习的朋友可以充分发挥自己的才能～


# 控制层面（PID的实现）

发布者和订阅者～hh，老掉牙的东西，我们只需要关注PID的实现