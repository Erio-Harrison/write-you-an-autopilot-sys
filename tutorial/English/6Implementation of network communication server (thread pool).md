# Simulate the server

Okay, finally let's build a simulated server.

Note that although we encapsulate the server as a ROS2 node here, the actual communication method is network communication. The reason for doing this is just to make it easier for everyone to run the system to form a closed loop (Python can run a ROS2 node by adding a few lines, which is really convenient!! I will talk about it at the end).

Basic initialization, let's first explain the role of these member variables:

1. comm_: Network communication adapter.

2. timer_: Timer is used for periodic tasks.

3. receiver_thread_: Independent thread that receives messages.

4. should_exit_: Atomic Boolean variable used to indicate when the thread stops.

5. thread_pool_: Thread pool, establish a producer-consumer model to handle work

6. comm_mutex_: Mutex lock that protects network communication operations.

When our unmanned car wants to communicate with the server over the network, the system first needs to establish a socket connection based on the address and port number:

```bash
comm_ = std::make_unique<network_comm::ZeroMQAdapter>(zmq::socket_type::rep);
        try {
            comm_->bind("tcp://0.0.0.0:5555");
            RCLCPP_INFO(this->get_logger(), "Mock Server Node bound to port 5555");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind: %s", e.what());
            return;
        }
```

We have received data, so we start the following thread to process the received data, which will call the `receiverLoop()` function:

```bash
receiver_thread_ = std::thread(&MockServerNode::receiverLoop, this);
```

Here `this` is a member function pointer: &MockServerNode::receiverLoop is a member function pointer of the MockServerNode class. When we want to execute a member function of a class in a thread, we need to use this syntax.

It is equivalent to the following lambda expression:

```bash
receiver_thread_ = std::thread([this]() { this->receiverLoop(); });
```

When we stop running this mock_server_node, we need to adjust should_exit_ to true to end the `receiverLoop()` thread, otherwise our join() function will not be able to exit while waiting for the thread to complete its work:

```bash
~MockServerNode() {
    should_exit_ = true;
    if (receiver_thread_.joinable()) {
        receiver_thread_.join();
    }
}
```

When we receive a message from the client (that is, the driverless car), we will parse the message into a JSON string and then put this part of the information into the thread pool for processing (`thread_pool_.enqueue(&MockServerNode::process_message, this, json_str);`, whose implementation we will talk about last). The method that the thread pool processes each string is `process_message(const std::string& json_str)`:

```bash
void receiverLoop() {
        while (!should_exit_) {
            try {
                auto received_data = comm_->receive();
                if (!received_data.empty()) {
                    std::string json_str(received_data.begin(), received_data.end());
                    thread_pool_.enqueue(&MockServerNode::process_message, this, json_str);
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error in receiver loop: %s", e.what());
            }
        }
    }
```

The `process_message` method parses the received JSON message, processes the data, and sends the modified data back, using comm_mutex_ to protect network communication operations:

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

Then comes the implementation of the thread pool. First, let's look at the member variables of the thread pool:

1. workers: We put all the threads in the thread pool into a container, which makes it easier for us to manage these threads

2. tasks: used to store pending tasks, that is, the tasks taken out and run in each of our threads

3. queue_mutex: ensure the thread safety of the task queue

4. condition: used for thread wakeup and waiting

5. stop: control the overall operation of the thread pool

In the constructor, the thread pool starts num_threads worker threads and adds them to worker_threads_. We adjust the number of threads as needed. The number of threads here is related to the frequency of calling `process_message` and the working time of `process_message`. We can further expand on the stress test to find the optimal number of threads.

Here is a brief introduction to the difference between emplace_back and push_back in C++:

Generally speaking, emplace_back is more efficient than push_back because it constructs objects directly in the container's memory space, avoiding the creation of temporary objects. Push_back first creates a temporary object and then copies or moves it to the container.

>emplace_back can construct objects by passing parameters only, mainly because it uses variable parameter templates to accept arbitrary parameters, and then directly constructs objects in the correct memory location through perfect forwarding and placement new >. This method avoids the creation of intermediate temporary objects and improves efficiency, especially for complex objects.

Let's look at the constructor code:

```bash
ThreadPool(size_t num_threads) : stop(false) {
    for(size_t i = 0; i < num_threads; ++i) {
        workers.emplace_back([this] {
            while(true) {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(this->queue_mutex);
                    this->condition.wait(lock, [this] { 
                        return this->stop || !this->tasks.empty(); 
                    });
                    if(this->stop && this->tasks.empty()) {
                        return;
                    }
                    task = std::move(this->tasks.front());
                    this->tasks.pop();
                }
                task();
            }
        });
    }
}
```

The thread pool destructor needs to change the stop flag to true. Each working thread will check it periodically. Once it becomes true, the thread will return and exit. The other is the basic join:

```bash
~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true;
        }
        condition.notify_all();
        for(std::thread &worker: workers) {
            worker.join();
        }
    }
```

Now the good thread has basic "consumers" and "producers", that is, how to add tasks to the thread pool?

First of all, it needs to be clear that what the "producer" does is to add the task to tasks after `thread_pool_.enqueue(&MockServerNode::process_message, this, json_str);`. Let's look at its implementation step by step:

```bash
template<class F, class... Args>
auto enqueue(F&& f, Args&&... args)
```

This is a function template. F is the type of function to be executed, Args... is the parameters to be passed to the function, and perfect forwarding is achieved using universal reference (&&).

```bash
-> std::future<typename std::result_of<F(Args...)>::type>
```

Here, `std::result_of` is used to deduce the return type of function F, and return a `std::future`, which allows asynchronous access to the result of the task.

```bash
using return_type = typename std::result_of<F(Args...)>::type;
```

This is to simplify the code for later writing.

```bash
auto task = std::make_shared<std::packaged_task<return_type()>>(
std::bind(std::forward<F>(f), std::forward<Args>(args)...)
);
```

Use `std::packaged_task` to encapsulate the task, `std::bind` and `std::forward` are used for perfect forwarding parameters, and `std::make_shared` is used to create a shared pointer and manage the life cycle of the task.

```bash
std::future<return_type> res = task->get_future();
```
Get the task result asynchronously. This line of code is used together with the above one, regardless of whether the task returns a value or not.

```bash
{
    std::unique_lock<std::mutex> lock(queue_mutex);
    if(stop) {
        throw std::runtime_error("enqueue on stopped ThreadPool");
    }
    tasks.emplace([task](){ (*task)(); });
}
```

This is adding a task queue, using a mutex to protect the task queue, checking whether the thread pool has stopped, and adding the task to the queue. Use lambda expressions to wrap the task for later execution.

```bash
condition.notify_one();
```
Notify the waiting thread.

```bash
return res;
```
Allow the caller to get the result of the task later.

Okay, the thread pool is also completed! ! ! The entire server is also completed~~~~~

Finally, run the demo. You can use ROS2 to run Node specifically, or you can write a program specifically. There are two programs in our project, one for running all the Nodes we have written, and the other for running the network communication module, which can be seen under the launch folder.

Run in the terminal:

```bash
ros2 launch launch/auto_drive_system.launch.py
```
You can see a demo come out, and then friends who are learning can make a good-looking demo by modifying the data flow in the system

Because we debug a lot of code, we may not see the output displayed on the shell after the network module runs successfully, so we wrote a special one:

```bash
ros2 launch launch/network_communication.launch.py
```
As long as you see this output:

![result](/asset/network.png)

It means that it runs successfully. The following outputs are because we did not establish more network communication mechanisms, so there are small errors. The specific reasons can also be seen from the client and server codes.