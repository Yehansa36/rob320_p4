
#include "rix/tf/transform_listener.hpp"

#include <Eigen/Geometry>

#include "rix/robot/eigen_util.hpp"

namespace rix {
namespace tf {

TransformListener::TransformListener(std::shared_ptr<rix::core::Node> node, const rix::util::Duration &duration,
                                     const std::string &topic, const rix::ipc::Endpoint &endpoint)
    : TransformListener(*node, duration, topic, endpoint) {}

TransformListener::TransformListener(rix::core::Node &node, const rix::util::Duration &duration,
                                     const std::string &topic, const rix::ipc::Endpoint &endpoint)
    : duration_(duration),
      graph_("world", duration),
      subscriber_(node.create_subscriber<rix::msg::geometry::TF>(
          topic,
          [this](const rix::msg::geometry::TF &msg) {
              if (!graph_.update(msg)) {
                  rix::util::Log::warn << "Failed to update transform from TF message." << std::endl;
              }
          },
          endpoint)) {}

TransformListener::TransformListener(const TransformListener &other)
    : subscriber_(other.subscriber_), graph_(other.graph_), duration_(other.duration_) {}

TransformListener &TransformListener::operator=(const TransformListener &other) {
    if (this != &other) {
        subscriber_ = other.subscriber_;
        graph_ = other.graph_;
        duration_ = other.duration_;
    }
    return *this;
}

bool TransformListener::get_transform(const std::string &target_frame, const std::string &source_frame,
                                      rix::util::Time time, rix::msg::geometry::TransformStamped &transform) const {
    return graph_.get_transform(target_frame, source_frame, time, transform);
}

bool TransformListener::transform_point(const std::string &target_frame, const std::string &source_frame,
                                        rix::util::Time time, const rix::msg::geometry::Point &point,
                                        rix::msg::geometry::Point &transformed_point) const {
    rix::msg::geometry::TransformStamped transform;
    if (!graph_.get_transform(target_frame, source_frame, time, transform)) {
        return false;
    }
    Eigen::Affine3d eigen_transform = rix::robot::msg_to_eigen(transform.transform);

    Eigen::Vector3d eigen_point = rix::robot::msg_to_eigen(point);
    Eigen::Vector3d transformed_eigen_point = eigen_transform * eigen_point;
    transformed_point.x = transformed_eigen_point.x();
    transformed_point.y = transformed_eigen_point.y();
    transformed_point.z = transformed_eigen_point.z();
    return true;
}

bool TransformListener::transform_point_cloud(const std::string &target_frame, const std::string &source_frame,
                                              rix::util::Time time, const rix::msg::sensor::PointCloud &point_cloud,
                                              rix::msg::sensor::PointCloud &transformed_point_cloud) const {
    transformed_point_cloud.header = point_cloud.header;
    transformed_point_cloud.header.frame_id = target_frame;
    transformed_point_cloud.points.resize(point_cloud.points.size());
    transformed_point_cloud.channels.resize(point_cloud.channels.size());

    rix::msg::geometry::TransformStamped transform;
    if (!graph_.get_transform(target_frame, source_frame, time, transform)) {
        return false;
    }
    Eigen::Affine3d eigen_transform = rix::robot::msg_to_eigen(transform.transform);

    for (size_t i = 0; i < point_cloud.points.size(); ++i) {
        const auto &point = point_cloud.points[i];
        auto &t_point = transformed_point_cloud.points[i];

        Eigen::Vector3d eigen_point = rix::robot::msg_to_eigen(point);
        Eigen::Vector3d transformed_eigen_point = eigen_transform * eigen_point;
        t_point.x = transformed_eigen_point.x();
        t_point.y = transformed_eigen_point.y();
        t_point.z = transformed_eigen_point.z();
    }
    return true;
}

bool TransformListener::transform_pose(const std::string &target_frame, const std::string &source_frame,
                                       rix::util::Time time, const rix::msg::geometry::Pose &pose,
                                       rix::msg::geometry::Pose &transformed_pose) const {
    rix::msg::geometry::TransformStamped transform;
    if (!graph_.get_transform(target_frame, source_frame, time, transform)) {
        return false;
    }
    Eigen::Affine3d eigen_transform = rix::robot::msg_to_eigen(transform.transform);

    Eigen::Affine3d pose_transform =
        Eigen::Translation3d(rix::robot::msg_to_eigen(pose.position)) * rix::robot::msg_to_eigen(pose.orientation);
    Eigen::Affine3d transformed_pose_affine = eigen_transform * pose_transform;

    const Eigen::Vector3d &transformed_position = transformed_pose_affine.translation();
    Eigen::Quaterniond transformed_orientation(transformed_pose_affine.rotation());

    transformed_pose.position.x = transformed_position.x();
    transformed_pose.position.y = transformed_position.y();
    transformed_pose.position.z = transformed_position.z();
    transformed_pose.orientation = rix::robot::eigen_to_msg(transformed_orientation);

    return true;
}

bool TransformListener::transform_quaternion(const std::string &target_frame, const std::string &source_frame,
                                             rix::util::Time time, const rix::msg::geometry::Quaternion &quaternion,
                                             rix::msg::geometry::Quaternion &transformed_quaternion) const {
    rix::msg::geometry::TransformStamped transform;
    if (!graph_.get_transform(target_frame, source_frame, time, transform)) {
        return false;
    }
    Eigen::Affine3d eigen_transform = rix::robot::msg_to_eigen(transform.transform);

    Eigen::Quaterniond eigen_quaternion = rix::robot::msg_to_eigen(quaternion);
    Eigen::Quaterniond transformed_eigen_quaternion(eigen_transform.rotation() * eigen_quaternion);
    transformed_quaternion = rix::robot::eigen_to_msg(transformed_eigen_quaternion);
    return true;
}

bool TransformListener::transform_vector(const std::string &target_frame, const std::string &source_frame,
                                         rix::util::Time time, const rix::msg::geometry::Vector3 &vector,
                                         rix::msg::geometry::Vector3 &transformed_vector) const {
    rix::msg::geometry::TransformStamped transform;
    if (!graph_.get_transform(target_frame, source_frame, time, transform)) {
        return false;
    }
    Eigen::Affine3d eigen_transform = rix::robot::msg_to_eigen(transform.transform);

    Eigen::Vector3d eigen_vector = rix::robot::msg_to_eigen(vector);
    Eigen::Vector3d transformed_eigen_vector = eigen_transform.linear() * eigen_vector;
    transformed_vector = rix::robot::eigen_to_msg(transformed_eigen_vector);
    return true;
}

bool TransformListener::ok() const { return subscriber_->ok(); }

const FrameGraph &TransformListener::graph() const { return graph_; }

rix::util::Duration TransformListener::duration() const { return duration_; }

void TransformListener::set_duration(const rix::util::Duration &duration) { duration_ = duration; }

}  // namespace tf
}  // namespace rix