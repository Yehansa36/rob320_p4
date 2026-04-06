#pragma once

#include "rix/core/node.hpp"
#include "rix/core/subscriber.hpp"
#include "rix/msg/geometry/Point.hpp"
#include "rix/msg/geometry/Pose.hpp"
#include "rix/msg/geometry/Quaternion.hpp"
#include "rix/msg/geometry/TF.hpp"
#include "rix/msg/geometry/Vector3.hpp"
#include "rix/msg/sensor/PointCloud.hpp"
#include "rix/robot/msg_util.hpp"
#include "rix/tf/frame_graph.hpp"
#include "rix/util/log.hpp"

namespace rix {
namespace tf {

class TransformListener {
   public:
    TransformListener(std::shared_ptr<rix::core::Node> node,
                      const rix::util::Duration &duration = rix::util::Duration(10.0), const std::string &topic = "/tf",
                      const rix::ipc::Endpoint &endpoint = rix::ipc::Endpoint("127.0.0.1", 0));
    TransformListener(rix::core::Node &node, const rix::util::Duration &duration = rix::util::Duration(10.0),
                      const std::string &topic = "/tf",
                      const rix::ipc::Endpoint &endpoint = rix::ipc::Endpoint("127.0.0.1", 0));
    TransformListener(const TransformListener &other);
    TransformListener &operator=(const TransformListener &other);

    bool get_transform(const std::string &target_frame, const std::string &source_frame, rix::util::Time time,
                       rix::msg::geometry::TransformStamped &transform) const;

    bool transform_point(const std::string &target_frame, const std::string &source_frame, rix::util::Time time,
                         const rix::msg::geometry::Point &point, rix::msg::geometry::Point &transformed_point) const;
    bool transform_point_cloud(const std::string &target_frame, const std::string &source_frame, rix::util::Time time,
                               const rix::msg::sensor::PointCloud &point_cloud,
                               rix::msg::sensor::PointCloud &transformed_point_cloud) const;
    bool transform_pose(const std::string &target_frame, const std::string &source_frame, rix::util::Time time,
                        const rix::msg::geometry::Pose &pose, rix::msg::geometry::Pose &transformed_pose) const;
    bool transform_quaternion(const std::string &target_frame, const std::string &source_frame, rix::util::Time time,
                              const rix::msg::geometry::Quaternion &quaternion,
                              rix::msg::geometry::Quaternion &transformed_quaternion) const;
    bool transform_vector(const std::string &target_frame, const std::string &source_frame, rix::util::Time time,
                          const rix::msg::geometry::Vector3 &vector,
                          rix::msg::geometry::Vector3 &transformed_vector) const;

    bool ok() const;

    const FrameGraph &graph() const;
    rix::util::Duration duration() const;
    void set_duration(const rix::util::Duration &duration);

   private:
    std::shared_ptr<rix::core::Subscriber> subscriber_;
    FrameGraph graph_;
    rix::util::Duration duration_;
};

}  // namespace tf
}  // namespace rix