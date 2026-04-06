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

class RandomSetpointPublisher : public rix::core::Node {
   public:
    RandomSetpointPublisher(const rix::ipc::Endpoint &rixhub_endpoint, 
                        std::shared_ptr<rix::robot::RobotModel> robot_model,
                        double rate, 
                        const rix::ipc::Endpoint &publisher_endpoint = rix::ipc::Endpoint("127.0.0.1", 0));

   private:
    void update_setpoint();
    void setpoint_update_callback(const rix::core::Timer::Event &event);
    void publish_joint_states_callback(const rix::core::Timer::Event &event);

    std::shared_ptr<rix::robot::RobotModel> robot_;
    std::shared_ptr<rix::core::Publisher> joint_state_pub_;
    std::shared_ptr<rix::core::Timer> setpoint_update_timer_;
    std::shared_ptr<rix::core::Timer> publish_joint_states_timer_;
    rix::msg::sensor::JS setpoint_;
    rix::msg::sensor::JS current_state_;
};