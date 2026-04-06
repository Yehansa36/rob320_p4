#include <string>
#include <thread>
#include <unordered_map>

#include "rix/ipc/signal.hpp"
#include "rix/msg/geometry/TF.hpp"
#include "rix/msg/sensor/JS.hpp"
#include "rix/rix.hpp"
#include "rix/robot/eigen_util.hpp"
#include "rix/robot/robot_model.hpp"
#include "rix/tf/transform_broadcaster.hpp"

using namespace rix::util;
using namespace rix::core;
using namespace rix::robot;

using JS = rix::msg::sensor::JS;
using TF = rix::msg::geometry::TF;

class RobotStatePublisher : public rix::core::Node {
   public:
    RobotStatePublisher(const rix::ipc::Endpoint &rixhub_endpoint, 
                        std::shared_ptr<rix::robot::RobotModel> robot, 
                        const std::string& topic, 
                        double rate,
                        bool ignore_timestamps,
                        const rix::ipc::Endpoint& tf_endpoint = rix::ipc::Endpoint("127.0.0.1", 0),
                        const rix::ipc::Endpoint& js_endpoint = rix::ipc::Endpoint("127.0.0.1", 0),
                        rix::core::ServerFactory server_factory = rix::core::Node::make_server_default,
                        rix::core::ClientFactory client_factory = rix::core::Node::make_client_default);

   private:
    void joint_state_callback(const rix::msg::sensor::JS &msg);

    std::shared_ptr<rix::robot::RobotModel> robot_;
    rix::util::Time last_publish_time_;
    rix::msg::geometry::TF static_tf_;
    rix::tf::TransformBroadcaster tf_broadcaster_;
    rix::util::Duration period_;
    bool ignore_timestamps_;
};