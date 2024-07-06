# Planning module (implementation of A* algorithm)

After we get the obstacle data, we need to plan the best path on the map. The simple model is to move from the initial coordinate to the final coordinate without touching the obstacle.

It is still the basic publisher and subscriber model, no more to say~~Here we need to pay attention to the data type of `geometry_msgs::msg::PoseStamped`, which we will use to represent the position and direction in the three-dimensional space with a timestamp. It consists of two parts: Header and Pose.

1. Header (std_msgs::msg::Header):

stamp: timestamp, indicating the time when this pose information is recorded or created.
frame_id: string, indicating the coordinate system referenced by this pose.

2. Pose (geometry_msgs::msg::Pose):

- Position (geometry_msgs::msg::Point):

x: double, x coordinate
y: double, y coordinate
z: double, z coordinate

- Orientation (geometry_msgs::msg::Quaternion):

x: double, x component of quaternion
y: double, y component of quaternion
z: double, z component of quaternion
w: double, w component of quaternion (real part)

We already know the function of `Point`, as for `Quaternion`:

>Quaternion is a mathematical tool for representing rotation in 3D space, consisting of four components: x, y, z, w. It can be expressed as: q = w + xi + yj + zk, where i, j, >k are imaginary units.

We can use it to represent the orientation of a robot or vehicle. Here, in order to simplify the model, we simply set the w component of the quaternion to 1, which means there is no rotation (unit quaternion). The other components (x, y, z) default to 0.

The A* algorithm in our project is a basic version. Assume that our vehicle is driving in a grid map and the driving route is also along the grid line. In actual business scenarios, our vehicles need to consider more possibilities. Friends who study together can further improve according to their own interests. The basic principle of the A* algorithm can refer to this: [Principle of the A* Algorithm](https://www.bilibili.com/video/BV18h411e7dH/?spm_id_from=333.337.search-card.all.click&vd_source=b284da586203e097a82c06c5405042de). It is worth mentioning that we can regard the A* algorithm as an algorithm that combines the breadth-first search algorithm and the heuristic function, which we can also see in the later implementation.

Okay, let's implement the A* algorithm. First, let's clarify the implementation steps of the A* algorithm:

OK, let's implement the A* algorithm. First, let's clarify the implementation steps of the A* algorithm:

1. Initialization:

- Create an open list and a closed list.

- Add the starting node to the open list.

2. Main loop:

When the open list is not empty, repeat the following steps:

a. Select the node with the smallest f value from the open list as the current node. f = g + h, where:

- g is the actual cost from the starting point to the current node.

- h is the estimated cost from the current node to the target (heuristic function).

b. If the current node is the target node, the algorithm ends and returns the path.

c. Otherwise:

- Move the current node from the open list to the closed list.

- For each neighboring node of the current node:

If the neighboring node is in the closed list, ignore it.

If the neighboring node is not in the open list, add it to the open list.

If the neighboring node is already in the open list, check whether this path to the neighboring node is better (smaller g value). If so, update the parent node and g value of the node.

3. Path reconstruction:

When the target node is found, start from the target node, trace back to the starting node through the parent node pointer of each node, and you can get the complete path.

4. Return result:

- If the path is found, return the path.

- If the open list is empty and the target is not found, there is no path to reach, and return failure.

Then look at our code implementation here:

First, we define Node and override the **>** operator. This is to facilitate the use of priority queues to implement the minimum heap later:

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

1. Initialization:

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

Note that `all_nodes[start_x * 100 + start_y] = start`. We attach a unique tag to each node. It can be random, but there is a restriction that our grid must be smaller than 100x100, otherwise the tag may not be unique.

Define an array and then use it to traverse to simulate the up, down, left, and right directions

```bash
int directions[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
```

2. Main loop:

Code to start the main loop, select the node with the smallest f value from the open list as the current node:

```bash
while (!open_list.empty())
{
    AStarNode current = open_list.top();
    open_list.pop();
    int current_id = current.x * 100 + current.y;
```

If the current node is the target node, the algorithm ends and returns the path:

```bash
if (current.x == end_x && current.y == end_y)
    {
        // 这里我们先空着，后面再解释怎么做
    }
```

Add the current node to the closed list:

```bash
closed_list.insert(current_id);
```

For each currently adjacent node:

It is necessary to check whether the node is in the closed list or an obstacle

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

Calculate the new g and h values ​​of the adjacent nodes. If it is a new node or a better path is found, update the node information and add it to the open list.

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

3. Path reconstruction:

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
4. Return result:

If the loop ends without finding a path, an empty path is returned:

```bash
return {}; // Path not found
```

In fact, even if it is a single algorithm, we can improve it from many angles. For example, can the speed we get from the tracking module be used as a factor to improve our heuristic function? Friends who study together in this regard can give full play to their talents~

# Control level (PID implementation)

Publishers and subscribers~hh, old stuff, we only need to focus on the implementation of PID
