#include "joint_state_publisher/joint_state_publisher.hpp"

JointStatePublisher::JointStatePublisher(const rix::ipc::Endpoint &rixhub_endpoint,
                                         std::shared_ptr<rix::robot::RobotModel> robot, double rate,
                                         const std::vector<std::string> &topics,
                                         const rix::ipc::Endpoint &publisher_endpoint,
                                         const std::vector<rix::ipc::Endpoint> &subscriber_endpoints,
                                         rix::core::ServerFactory server_factory,
                                         rix::core::ClientFactory client_factory)
    : Node("joint_state_publisher", rixhub_endpoint, server_factory, client_factory), robot_(robot) {
    if (!ok()) {
        rix::util::Log::error << "Failed to initialize node" << std::endl;
        shutdown();
        return;
    }

    if (!robot_->is_valid()) {
        rix::util::Log::error << "Failed to load JRDF." << std::endl;
        shutdown();
        return;
    }

    joint_state_pub_ = create_publisher<rix::msg::sensor::JS>("/joint_states", publisher_endpoint);
    if (!joint_state_pub_->ok()) {
        rix::util::Log::error << "Failed to advertise topic joint_states" << std::endl;
        shutdown();
        return;
    }

    bool has_subscriber_endpoints = !subscriber_endpoints.empty() && subscriber_endpoints.size() == topics.size();
    size_t i = 0;
    for (const std::string &topic : topics) {
        auto sub = create_subscriber<rix::msg::sensor::JS>(
            topic, std::bind(&JointStatePublisher::joint_state_callback, this, std::placeholders::_1),
            (has_subscriber_endpoints) ? subscriber_endpoints[i++] : rix::ipc::Endpoint("127.0.0.1", 0));
        if (!sub->ok()) {
            rix::util::Log::error << "Failed to subscribe to topic " << topic << std::endl;
            shutdown();
            return;
        }
    }

    timer_ = create_timer(rix::util::Duration(1.0 / rate),
                          std::bind(&JointStatePublisher::timer_callback, this, std::placeholders::_1));
}

/**< TODO: Implement the joint_state_callback method. */
void JointStatePublisher::joint_state_callback(const rix::msg::sensor::JS &msg) {
    return;
}

/**< TODO: Implement the timer_callback method. */
void JointStatePublisher::timer_callback(const rix::core::Timer::Event &event) {
    return;
} 