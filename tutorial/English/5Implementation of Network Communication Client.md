# Implementation of network communication client

In the field of unmanned driving, the client is naturally an unmanned car driving on the road.

In our project, the node responsible for network communication (`NetworkBridgeNode()`) will initialize the network adapter we have implemented and call `connect` to establish a connection with the server:

```bash
// Initialize ZeroMQ communication
comm_ = std::make_unique<network_comm::ZeroMQAdapter>(zmq::socket_type::req);
try {
comm_->connect("tcp://localhost:5555"); // Connect to the remote server
RCLCPP_INFO(this->get_logger(), "Connected to ZeroMQ server at localhost:5555");
} catch (const std::exception& e) {
RCLCPP_ERROR(this->get_logger(), "Failed to connect to ZeroMQ server: %s", e.what());
return;
}
```

Once this node receives the topic about vehicle state (`vehicle_state`), it will call the callback function and use the `json` library to serialize the data in our system:

```bash
nlohmann::json j = {
{"position_x", msg->position_x},
{"position_y", msg->position_y},
{"yaw", msg->yaw},
{"velocity", msg->velocity},
{"acceleration", msg->acceleration}
};
std::string json_str = j.dump();

// Send data using ZeroMQ
std::vector<uint8_t> data(json_str.begin(), json_str.end());
```
Remember to install the third-party library for json (you can also use other libraries):

```bash
sudo apt-get install nlohmann-json3-dev
```

And modify CMakeLists.txt and package.xml

Then send to the server:

```bash
comm_->send(data);
```

The client and server can communicate with each other, and the specific implementation method can give full play to your creativity.