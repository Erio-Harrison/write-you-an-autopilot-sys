#include <rclcpp/rclcpp.hpp>
#include <network_comm/zeromq_adapter.hpp>
#include <nlohmann/json.hpp>
#include "auto_drive_msgs/msg/vehicle_state.hpp"
#include <thread>
#include <mutex>
#include <vector>
#include <memory>
#include <atomic>
#include <queue>

class MockServerNode : public rclcpp::Node {
public:
    MockServerNode() : Node("mock_server_node"), should_exit_(false) {
    comm_ = std::make_unique<network_comm::ZeroMQAdapter>(zmq::socket_type::rep);
        try {
            comm_->bind("tcp://0.0.0.0:5555");
            RCLCPP_INFO(this->get_logger(), "Mock Server Node bound to port 5555");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind: %s", e.what());
            return;
        }

        receiver_thread_ = std::thread(&MockServerNode::receiverLoop, this);
        
        // Start worker threads
        for (int i = 0; i < 4; ++i) {  // Using 4 worker threads, adjust as needed
            worker_threads_.emplace_back(&MockServerNode::workerLoop, this);
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MockServerNode::timerCallback, this));
    }

    ~MockServerNode() {
        should_exit_ = true;
        if (receiver_thread_.joinable()) {
            receiver_thread_.join();
        }
        for (auto& thread : worker_threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
    }

private:
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

    void timerCallback() {
        // This callback is empty but can be used for periodic tasks if needed
    }

    std::unique_ptr<network_comm::ZeroMQAdapter> comm_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread receiver_thread_;
    std::vector<std::thread> worker_threads_;
    std::atomic<bool> should_exit_;
    std::queue<std::string> message_queue_;
    std::mutex queue_mutex_;
    std::mutex comm_mutex_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockServerNode>());
    rclcpp::shutdown();
    return 0;
}