# Simulate the server

Okay, finally let's build a simulated server.

Note that although we encapsulate the server as a ROS2 node here, the actual communication method is network communication. The reason for doing this is just to make it easier for everyone to run the system in a closed loop (Python can run a ROS2 node by adding a few lines, which is really convenient!! I will talk about it at the end).

Basic initialization, let's first explain the role of these member variables:

1. comm_: network communication adapter.
2. timer_: timer is used for periodic tasks.
3. receiver_thread_: independent thread that receives messages.
4. worker_threads_: a collection of worker threads, that is, a thread pool.
5. should_exit_: an atomic Boolean variable used to indicate when a thread stops.
6. message_queue_: a message queue used to store received messages.
7. queue_mutex_: a mutex that protects the message queue.
8. comm_mutex_: a mutex that protects network communication operations.

Initialization of thread pool:

```bash
for (int i = 0; i < 4; ++i) { // Using 4 worker threads, adjust as needed
worker_threads_.emplace_back(&MockServerNode::workerLoop, this);
}
```

In the constructor, start 4 worker threads and add them to worker_threads_. You can adjust the number of threads as needed.

Thread receiving messages:

```bash
void receiverLoop() {
while (!should_exit_) {
try {
auto received_data = comm_->receive();
if (!received_data.empty()) {
std::string json_str(received_data.begin(), received_data.end());
std::lock_guard<std::mutex> lock(queue_mutex_);
message_queue_.push(json_str);
}
} catch (const std::exception& e) {
RCLCPP_ERROR(this->get_logger(), "Error in receiver loop: %s", e.what());
}
}
}
```
`receiverLoop` method runs in a separate thread, continuously receives data from the network and pushes it into the message queue, using queue_mutex_ to protect the message_queue_

Worker thread processes messages:

```bash# Simulate the server

Okay, finally let's build a simulated server.

Note that although we encapsulate the server as a ROS2 node here, the actual communication method is network communication. The reason for doing this is just to make it easier for everyone to run the system in a closed loop (Python can run a ROS2 node by adding a few lines, which is really convenient!! I will talk about it at the end).

Basic initialization, let's first explain the role of these member variables:

1. comm_: network communication adapter.
2. timer_: timer is used for periodic tasks.
3. receiver_thread_: independent thread that receives messages.
4. worker_threads_: a collection of worker threads, that is, a thread pool.
5. should_exit_: an atomic Boolean variable used to indicate when a thread stops.
6. message_queue_: a message queue used to store received messages.
7. queue_mutex_: a mutex that protects the message queue.
8. comm_mutex_: a mutex that protects network communication operations.

Initialization of thread pool:

```bash
for (int i = 0; i < 4; ++i) { // Using 4 worker threads, adjust as needed
worker_threads_.emplace_back(&MockServerNode::workerLoop, this);
}
```

In the constructor, start 4 worker threads and add them to worker_threads_. You can adjust the number of threads as needed.

Thread receiving messages:

```bash
void receiverLoop() {
while (!should_exit_) {
try {
auto received_data = comm_->receive();
if (!received_data.empty()) {
std::string json_str(received_data.begin(), received_data.end());
std::lock_guard<std::mutex> lock(queue_mutex_);
message_queue_.push(json_str);
}
} catch (const std::exception& e) {
RCLCPP_ERROR(this->get_logger(), "Error in receiver loop: %s", e.what());
}
}
}
```
`receiverLoop` method runs in a separate thread, continuously receives data from the network and pushes it into the message queue, using queue_mutex_ to protect the message_queue_

Worker thread processes messages:

```bash

void workerLoop() {
while (!should_exit_) {
std::string message;
{
std::lock_guard<std::mutex> lock(queue_mutex_);
if (!message_queue_.empty()) {
message = message_queue_.front();
message_queue_.pop();
}
}
if (!message.empty()) {
process_message(message);
} else {
std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
}
}
```

`workerLoop` method runs in each worker thread, takes messages from the message queue and processes them. If the message queue is empty, the thread will sleep for a while to reduce CPU usage.

Processing message:

```bash
void process_message(const std::string& json_str) {
try {
auto j = nlohmann::json::parse(json_str);
RCLCPP_INFO(this->get_logger(), "Received vehicle state: x=%f, y=%f, yaw=%f",
j["position_x"].get<double>(),
j["position_y"].get<double>(),
j["yaw"].get<double>());

// Echo back the received data with a small modification
j["position_x"] = j["position_x"].get<double>() + 1.0;
std::string response = j.dump();
std::vector<uint8_t> response_data(response.begin(), response.end());

std::lock_guard<std::mutex> lock(comm_mutex_);
comm_->send(response_data);
} catch (const std::exception& e) {
RCLCPP_ERROR(this->get_logger(), "Error processing message: %s", e.what());
}
}
```

`process_message` method parses the received JSON message, processes the data, and sends back the modified data, using comm_mutex_ to protect the network communication operation.

OK! ! The server is done! ! !

Finally, let's run the demo. You can use ROS2 to run Node specifically, or you can write a program specifically. There are two programs in our project, one for running all the Nodes we have written, and the other for running the network communication module, which can be seen under the launch folder.

Run in the terminal:

```bash
ros2 launch launch/auto_drive_system.launch.py
```
You can see a demo come out, and then friends who are learning can make a good-looking demo by modifying the data flow in the system

Because we debug a lot of code, we may not see the output displayed on the **shell** after the network module runs successfully, so we wrote a special one:

```bash
ros2 launch launch/network_communication.launch.py
```
As long as you see this output:

![result](/asset/network.png)

It means that it runs successfully. The following outputs are because we did not establish more network communication mechanisms, so there are small errors. The specific reasons can also be seen from the client and server codes.

void workerLoop() {
while (!should_exit_) {# Simulate the server

Okay, finally let's build a simulated server.

Note that although we encapsulate the server as a ROS2 node here, the actual communication method is network communication. The reason for doing this is just to make it easier for everyone to run the system in a closed loop (Python can run a ROS2 node by adding a few lines, which is really convenient!! I will talk about it at the end).

Basic initialization, let's first explain the role of these member variables:

1. comm_: network communication adapter.
2. timer_: timer is used for periodic tasks.
3. receiver_thread_: independent thread that receives messages.
4. worker_threads_: a collection of worker threads, that is, a thread pool.
5. should_exit_: an atomic Boolean variable used to indicate when a thread stops.
6. message_queue_: a message queue used to store received messages.
7. queue_mutex_: a mutex that protects the message queue.
8. comm_mutex_: a mutex that protects network communication operations.

Initialization of thread pool:

```bash
for (int i = 0; i < 4; ++i) { // Using 4 worker threads, adjust as needed
worker_threads_.emplace_back(&MockServerNode::workerLoop, this);
}
```

In the constructor, start 4 worker threads and add them to worker_threads_. You can adjust the number of threads as needed.

Thread receiving messages:

```bash
void receiverLoop() {
while (!should_exit_) {
try {
auto received_data = comm_->receive();
if (!received_data.empty()) {
std::string json_str(received_data.begin(), received_data.end());
std::lock_guard<std::mutex> lock(queue_mutex_);
message_queue_.push(json_str);
}
} catch (const std::exception& e) {
RCLCPP_ERROR(this->get_logger(), "Error in receiver loop: %s", e.what());
}
}
}
```
`receiverLoop` method runs in a separate thread, continuously receives data from the network and pushes it into the message queue, using queue_mutex_ to protect the message_queue_

Worker thread processes messages:

```bash

void workerLoop() {
while (!should_exit_) {
std::string message;
{
std::lock_guard<std::mutex> lock(queue_mutex_);
if (!message_queue_.empty()) {
message = message_queue_.front();
message_queue_.pop();
}
}
if (!message.empty()) {
process_message(message);
} else {
std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
}
}
```

`workerLoop` method runs in each worker thread, takes messages from the message queue and processes them. If the message queue is empty, the thread will sleep for a while to reduce CPU usage.

Processing message:

```bash
void process_message(const std::string& json_str) {
try {
auto j = nlohmann::json::parse(json_str);
RCLCPP_INFO(this->get_logger(), "Received vehicle state: x=%f, y=%f, yaw=%f",
j["position_x"].get<double>(),
j["position_y"].get<double>(),
j["yaw"].get<double>());

// Echo back the received data with a small modification
j["position_x"] = j["position_x"].get<double>() + 1.0;
std::string response = j.dump();
std::vector<uint8_t> response_data(response.begin(), response.end());

std::lock_guard<std::mutex> lock(comm_mutex_);
comm_->send(response_data);
} catch (const std::exception& e) {
RCLCPP_ERROR(this->get_logger(), "Error processing message: %s", e.what());
}
}
```

`process_message` method parses the received JSON message, processes the data, and sends back the modified data, using comm_mutex_ to protect the network communication operation.

OK! ! The server is done! ! !

Finally, let's run the demo. You can use ROS2 to run Node specifically, or you can write a program specifically. There are two programs in our project, one for running all the Nodes we have written, and the other for running the network communication module, which can be seen under the launch folder.

Run in the terminal:

```bash
ros2 launch launch/auto_drive_system.launch.py
```
You can see a demo come out, and then friends who are learning can make a good-looking demo by modifying the data flow in the system

Because we debug a lot of code, we may not see the output displayed on the **shell** after the network module runs successfully, so we wrote a special one:

```bash
ros2 launch launch/network_communication.launch.py
```
As long as you see this output:

![result](/asset/network.png)

It means that it runs successfully. The following outputs are because we did not establish more network communication mechanisms, so there are small errors. The specific reasons can also be seen from the client and server codes.
std::string message;
{
std::lock_guard<std::mutex> lock(queue_mutex_);
if (!message_queue_.empty()) {
message = message_queue_.front();
message_queue_.pop();
}
}
if (!message.empty()) {
process_message(message);
} else {
std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
}
}
```

`workerLoop` method runs in each worker thread, takes messages from the message queue and processes them. If the message queue is empty, the thread will sleep for a while to reduce CPU usage.

Processing message:

```bash
void process_message(const std::string& json_str) {
try {
auto j = nlohmann::json::parse(json_str);
RCLCPP_INFO(this->get_logger(), "Received vehicle state: x=%f, y=%f, yaw=%f",
j["position_x"].get<double>(),
j["position_y"].get<double>(),
j["yaw"].get<double>());

// Echo back the received data with a small modification
j["position_x"] = j["position_x"].get<double>() + 1.0;
std::string response = j.dump();
std::vector<uint8_t> response_data(response.begin(), response.end());

std::lock_guard<std::mutex> lock(comm_mutex_);
comm_->send(response_data);
} catch (const std::exception& e) {
RCLCPP_ERROR(this->get_logger(), "Error processing message: %s", e.what());
}
}
```

`process_message` method parses the received JSON message, processes the data, and sends back the modified data, using comm_mutex_ to protect the network communication operation.

OK! ! The server is done! ! !

Finally, let's run the demo. You can use ROS2 to run Node specifically, or you can write a program specifically. There are two programs in our project, one for running all the Nodes we have written, and the other for running the network communication module, which can be seen under the launch folder.

Run in the terminal:

```bash
ros2 launch launch/auto_drive_system.launch.py
```
You can see a demo come out, and then friends who are learning can make a good-looking demo by modifying the data flow in the system

Because we debug a lot of code, we may not see the output displayed on the **shell** after the network module runs successfully, so we wrote a special one:

```bash
ros2 launch launch/network_communication.launch.py
```
As long as you see this output:

![result](/asset/network.png)

It means that it runs successfully. The following outputs are because we did not establish more network communication mechanisms, so there are small errors. The specific reasons can also be seen from the client and server codes.