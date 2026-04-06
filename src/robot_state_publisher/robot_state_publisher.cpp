#include "robot_state_publisher/robot_state_publisher.hpp"

RobotStatePublisher::RobotStatePublisher(const rix::ipc::Endpoint& rixhub_endpoint,
                                         std::shared_ptr<rix::robot::RobotModel> robot, const std::string& topic,
                                         double rate, bool ignore_timestamps,
                                         const rix::ipc::Endpoint& tf_endpoint,
                                         const rix::ipc::Endpoint& js_endpoint,
                                         rix::core::ServerFactory server_factory,
                                         rix::core::ClientFactory client_factory)
    : Node("robot_state_publisher", rixhub_endpoint, server_factory, client_factory),
      robot_(robot),
      period_(1.0 / rate),
      tf_broadcaster_(*this, "/tf", tf_endpoint),
      ignore_timestamps_(ignore_timestamps) {
    if (!robot_->is_valid()) {
        Log::error << "Failed to load JRDF." << std::endl;
        shutdown();
        return;
    } 

    if (!tf_broadcaster_.ok()) {
        Log::error << "Failed to create TF broadcaster" << std::endl;
        shutdown();
        return;
    }

    auto joint_states_sub = create_subscriber<JS>(
        topic, std::bind(&RobotStatePublisher::joint_state_callback, this, std::placeholders::_1), js_endpoint);

    if (!joint_states_sub->ok()) {
        Log::error << "Failed to subscribe to topic joint_states" << std::endl;
        shutdown();
        return;
    }
}

/**< TODO: Implement joint_state_callback method */
void RobotStatePublisher::joint_state_callback(const rix::msg::sensor::JS& msg) {
    return;
}