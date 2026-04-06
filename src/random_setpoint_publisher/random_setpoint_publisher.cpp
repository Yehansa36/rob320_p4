#include "random_setpoint_publisher/random_setpoint_publisher.hpp"

RandomSetpointPublisher::RandomSetpointPublisher(
    const rix::ipc::Endpoint &rixhub_endpoint, std::shared_ptr<rix::robot::RobotModel> robot_model, double rate,
    const rix::ipc::Endpoint &publisher_endpoint)
    : Node("random_setpoint_publisher", rixhub_endpoint), robot_(robot_model) {
    if (!ok()) {
        shutdown();
        return;
    }

    joint_state_pub_ = create_publisher<rix::msg::sensor::JS>("/random_joint_states");
    if (!joint_state_pub_->ok()) {
        shutdown();
        return;
    }

    // Set to default joint states
    current_state_ = setpoint_ = robot_->get_joint_states();

    setpoint_update_timer_ =
        create_timer(rix::util::Duration(1.0 / rate),
                     std::bind(&RandomSetpointPublisher::setpoint_update_callback, this, std::placeholders::_1));

    publish_joint_states_timer_ =
        create_timer(rix::util::Duration(1.0 / 30.0),
                     std::bind(&RandomSetpointPublisher::publish_joint_states_callback, this, std::placeholders::_1));
}

void RandomSetpointPublisher::publish_joint_states_callback(const rix::core::Timer::Event &event) {
    for (size_t i = 0; i < setpoint_.joint_states.size(); ++i) {
        current_state_.joint_states[i].position += 1.0 / 10.0 * (setpoint_.joint_states[i].position - current_state_.joint_states[i].position);
    }
    joint_state_pub_->publish(current_state_);
}

void RandomSetpointPublisher::setpoint_update_callback(const rix::core::Timer::Event &event) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<double> dist(0.0, 1.0);

    for (auto &state : setpoint_.joint_states) {
        auto joint = robot_->get_joint(state.name);
        if (!joint->is_mimic()) {
            state.position = joint->limits().lower + dist(gen) * (joint->limits().upper - joint->limits().lower);
        }
    }
}