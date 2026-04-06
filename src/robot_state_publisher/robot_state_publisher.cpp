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
    // Case 1: Ignore timestamps mode
    // Publish every time we get a message
    rix::util::Time current_time(msg.stamp.sec, msg.stamp.nsec);
    if (ignore_timestamps_) {
        // Update robot model with new joint states
        for (const auto& joint_state : msg.joint_states) {
            auto joint = robot_->get_joint(joint_state.name);
            if (joint) {
                joint->set_state(joint_state);
            }
        }
        
        // Get transforms for each joint based on new state
        rix::msg::geometry::TF tf = robot_->get_transforms();
        
        // Broadcast transforms immediately
        tf_broadcaster_.send(tf);
        
        // Update last publish time
        //last_publish_time_ = msg.stamp;
        last_publish_time_ = current_time;
        return;
    }
    
    // Case 2: Rate-limited mode (default)
    // Only publish if enough time has passed
    
    // Calculate time since last publish
    //rix::util::Duration time_since_publish = msg.stamp - last_publish_time_;
    rix::util::Duration time_since_publish = current_time - last_publish_time_;

    // If we haven't waited long enough, skip publishing
    if (time_since_publish < period_) {
        // Not enough time, return without publishing
        return;
    }
    
    // Enough time has passed, update and publish
    
    // Update robot model with new joint states
    for (const auto& joint_state : msg.joint_states) {
        auto joint = robot_->get_joint(joint_state.name);
        if (joint) {
            joint->set_state(joint_state);
        }
    }
    
    // Get transforms for each joint based on new state
    rix::msg::geometry::TF tf = robot_->get_transforms();
    
    // Broadcast transforms
    tf_broadcaster_.send(tf);
    
    // Record the time of this publish
    last_publish_time_ = current_time;
}