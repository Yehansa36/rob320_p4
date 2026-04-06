#pragma once

#include <Eigen/Geometry>

#include "rix/msg/geometry/Transform.hpp"
#include "rix/msg/geometry/Vector3.hpp"
#include "rix/msg/geometry/Point.hpp"

namespace rix {
namespace robot {

rix::msg::geometry::Transform eigen_to_msg(const Eigen::Affine3d &T);
rix::msg::geometry::Vector3 eigen_to_msg(const Eigen::Vector3d &v);
rix::msg::geometry::Quaternion eigen_to_msg(const Eigen::Quaterniond &v);

Eigen::Affine3d msg_to_eigen(const rix::msg::geometry::Transform &T);
Eigen::Vector3d msg_to_eigen(const rix::msg::geometry::Vector3 &v);
Eigen::Vector3d msg_to_eigen(const rix::msg::geometry::Point &p);
Eigen::Quaterniond msg_to_eigen(const rix::msg::geometry::Quaternion &v);

rix::msg::geometry::Transform interpolate(const rix::msg::geometry::Transform &t1, const rix::msg::geometry::Transform &t2, double ratio);
rix::msg::geometry::Quaternion interpolate(const rix::msg::geometry::Quaternion &q1, const rix::msg::geometry::Quaternion &q2, double ratio);
rix::msg::geometry::Vector3 interpolate(const rix::msg::geometry::Vector3 &v1, const rix::msg::geometry::Vector3 &v2, double ratio);

}  // namespace robot
}  // namespace rix