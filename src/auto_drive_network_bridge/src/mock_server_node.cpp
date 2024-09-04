#include <rclcpp/rclcpp.hpp>
#include <network_comm/zeromq_adapter.hpp>
#include <nlohmann/json.hpp>
#include <thread>
#include <mutex>
#include <vector>
#include <queue>
#include <functional>
#include <condition_variable>
#include <future>

class ThreadPool {
public:
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

    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args) 
        -> std::future<typename std::result_of<F(Args...)>::type> {
        using return_type = typename std::result_of<F(Args...)>::type;

        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );
            
        std::future<return_type> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            if(stop) {
                throw std::runtime_error("enqueue on stopped ThreadPool");
            }
            tasks.emplace([task](){ (*task)(); });
        }
        condition.notify_one();
        return res;
    }
    
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

private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;
};


class MockServerNode : public rclcpp::Node {
public:
    MockServerNode() : Node("mock_server_node"), should_exit_(false), thread_pool_(4) {
        comm_ = std::make_unique<network_comm::ZeroMQAdapter>(zmq::socket_type::rep);
        try {
            comm_->bind("tcp://0.0.0.0:5556");
            RCLCPP_INFO(this->get_logger(), "Mock Server Node bound to port 5556");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind: %s", e.what());
            return;
        }

        receiver_thread_ = std::thread(&MockServerNode::receiverLoop, this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MockServerNode::timerCallback, this));
    }

    ~MockServerNode() {
        should_exit_ = true;
        if (receiver_thread_.joinable()) {
            receiver_thread_.join();
        }
    }

private:
    void receiverLoop() {
        while (!should_exit_) {
            try {
                RCLCPP_INFO(this->get_logger(), "Waiting for incoming message");
                auto received_data = comm_->receive();
                if (!received_data.empty()) {
                    std::string json_str(received_data.begin(), received_data.end());
                    RCLCPP_INFO(this->get_logger(), "Received message: %s", json_str.c_str());
                    thread_pool_.enqueue(&MockServerNode::process_message, this, json_str);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Received empty message");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error in receiver loop: %s", e.what());
            }
        }
    }

    void process_message(const std::string& json_str) {
        RCLCPP_INFO(this->get_logger(), "Processing message: %s", json_str.c_str());
        try {
            auto j = nlohmann::json::parse(json_str);
            
            if (j["message_type"] == "test") {
                RCLCPP_INFO(this->get_logger(), "Received test message: %s", json_str.c_str());
                
                // Creating a Response
                nlohmann::json response = {
                    {"message_type", "test_response"},
                    {"received_timestamp", j["timestamp"]},
                    {"response_timestamp", this->now().seconds()}
                };
                
                std::string response_str = response.dump();
                std::vector<uint8_t> response_data(response_str.begin(), response_str.end());
                
                RCLCPP_INFO(this->get_logger(), "Sending response: %s", response_str.c_str());
                
                std::lock_guard<std::mutex> lock(comm_mutex_);
                comm_->send(response_data);
                RCLCPP_INFO(this->get_logger(), "Response sent");
            } else if (j["message_type"] == "vehicle_state") {
                RCLCPP_INFO(this->get_logger(), "Received vehicle state: %s", json_str.c_str());
                
                // Echo back the received data with a small modification
                j["position_x"] = j["position_x"].get<double>() + 1.0;
                j["message_type"] = "vehicle_state_response";
                std::string response_str = j.dump();
                std::vector<uint8_t> response_data(response_str.begin(), response_str.end());
                
                RCLCPP_INFO(this->get_logger(), "Sending response: %s", response_str.c_str());
                
                std::lock_guard<std::mutex> lock(comm_mutex_);
                comm_->send(response_data);
                RCLCPP_INFO(this->get_logger(), "Response sent");
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown message type received");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing message: %s", e.what());
        }
        RCLCPP_INFO(this->get_logger(), "Message processing completed");
    }

    void timerCallback() {
        // This callback is empty but can be used for periodic tasks if needed
    }

    std::unique_ptr<network_comm::ZeroMQAdapter> comm_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread receiver_thread_;
    std::atomic<bool> should_exit_;
    std::mutex comm_mutex_;
    ThreadPool thread_pool_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockServerNode>());
    rclcpp::shutdown();
    return 0;
}