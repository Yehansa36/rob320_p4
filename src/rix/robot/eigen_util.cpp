#include "rix/robot/eigen_util.hpp"

namespace rix {
namespace robot {

rix::msg::geometry::Transform eigen_to_msg(const Eigen::Affine3d &T) {
    rix::msg::geometry::Transform t;
    const Eigen::Vector3d &translation = T.translation();
    t.translation.x = translation.x();
    t.translation.y = translation.y();
    t.translation.z = translation.z();

    Eigen::Quaterniond q(T.linear());
    t.rotation.w = q.w();
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();
    return t;
}

rix::msg::geometry::Vector3 eigen_to_msg(const Eigen::Vector3d &v) {
    rix::msg::geometry::Vector3 vec;
    vec.x = v.x();
    vec.y = v.y();
    vec.z = v.z();
    return vec;
}

rix::msg::geometry::Quaternion eigen_to_msg(const Eigen::Quaterniond &q) {
    rix::msg::geometry::Quaternion quat;
    quat.w = q.w();
    quat.x = q.x();
    quat.y = q.y();
    quat.z = q.z();
    return quat;
}

Eigen::Affine3d msg_to_eigen(const rix::msg::geometry::Transform &t) {
    Eigen::Affine3d T(Eigen::Affine3d::Identity());
    T.translation() = Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
    Eigen::Quaterniond q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
    T.linear() = q.toRotationMatrix();
    return T;
}

Eigen::Vector3d msg_to_eigen(const rix::msg::geometry::Vector3 &v) { return Eigen::Vector3d(v.x, v.y, v.z); }

Eigen::Vector3d msg_to_eigen(const rix::msg::geometry::Point &p) { return Eigen::Vector3d(p.x, p.y, p.z); }

Eigen::Quaterniond msg_to_eigen(const rix::msg::geometry::Quaternion &q) {
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

rix::msg::geometry::Transform interpolate(const rix::msg::geometry::Transform &t1,
                                          const rix::msg::geometry::Transform &t2, double ratio) {
    rix::msg::geometry::Transform t;
    t.translation = interpolate(t1.translation, t2.translation, ratio);
    t.rotation = interpolate(t1.rotation, t2.rotation, ratio);
    return t;
}

rix::msg::geometry::Quaternion interpolate(const rix::msg::geometry::Quaternion &q1,
                                           const rix::msg::geometry::Quaternion &q2, double ratio) {
    Eigen::Quaterniond q_prev = rix::robot::msg_to_eigen(q1);
    Eigen::Quaterniond q_next = rix::robot::msg_to_eigen(q2);
    Eigen::Quaterniond q_interp = q_prev.slerp(ratio, q_next);
    return rix::robot::eigen_to_msg(q_interp);
}

rix::msg::geometry::Vector3 interpolate(const rix::msg::geometry::Vector3 &v1, const rix::msg::geometry::Vector3 &v2,
                                        double ratio) {
    Eigen::Vector3d x_prev = rix::robot::msg_to_eigen(v1);
    Eigen::Vector3d x_next = rix::robot::msg_to_eigen(v2);
    Eigen::Vector3d x_interp = x_prev + ratio * (x_next - x_prev);
    return rix::robot::eigen_to_msg(x_interp);
}

}  // namespace robot
}  // namespace rix