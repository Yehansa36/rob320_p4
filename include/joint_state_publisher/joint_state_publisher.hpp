#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rix/core/common.hpp"
#include "rix/ipc/signal.hpp"
#include "rix/msg/sensor/JS.hpp"
#include "rix/msg/sensor/JointState.hpp"
#include "rix/rix.hpp"
#include "rix/robot/robot_model.hpp"

class JointStatePublisher : public rix::core::Node {
   public:
    JointStatePublisher(const rix::ipc::Endpoint &rixhub_endpoint, 
                        std::shared_ptr<rix::robot::RobotModel> robot_model,
                        double rate, 
                        const std::vector<std::string> &topics,
                        const rix::ipc::Endpoint &publisher_endpoint = rix::ipc::Endpoint("127.0.0.1", 0),
                        const std::vector<rix::ipc::Endpoint> &subscriber_endpoints = {},
                        rix::core::ServerFactory server_factory = rix::core::Node::make_server_default,
                        rix::core::ClientFactory client_factory = rix::core::Node::make_client_default);

   private:
    void joint_state_callback(const rix::msg::sensor::JS &js);
    void timer_callback(const rix::core::Timer::Event &event);

    std::shared_ptr<rix::robot::RobotModel> robot_;
    mutable std::mutex robot_mutex_;
    std::shared_ptr<rix::core::Publisher> joint_state_pub_;
    std::shared_ptr<rix::core::Timer> timer_;
};