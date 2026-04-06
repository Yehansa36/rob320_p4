#include "rix/robot/joint.hpp"

#include <Eigen/Geometry>

#include "rix/robot/eigen_util.hpp"
#include "rix/util/log.hpp"

namespace rix {
namespace robot {

Joint::Joint(const rix::msg::geometry::Vector3 &axis, const rix::msg::geometry::Transform &origin,
             const Joint::Type &type, const JointLimits &limits, const JointDynamics &dynamics, const JointMimic &mimic,
             const std::string &name, const std::string &parent, const std::string &child)
    : position_(0.0),
      velocity_(0.0),
      effort_(0.0),
      type_(type),
      limits_(limits),
      dynamics_(dynamics),
      mimic_(mimic),
      name_(name),
      parent_(parent),
      child_(child),
      axis_(axis),
      origin_(origin) {}

Joint::Joint(const Joint &other)
    : position_(other.position_),
      velocity_(other.velocity_),
      effort_(other.effort_),
      type_(other.type_),
      limits_(other.limits_),
      dynamics_(other.dynamics_),
      mimic_(other.mimic_),
      name_(other.name_),
      parent_(other.parent_),
      child_(other.child_),
      axis_(other.axis_),
      origin_(other.origin_) {}

Joint &Joint::operator=(const Joint &other) {
    if (this != &other) {
        position_ = other.position_;
        velocity_ = other.velocity_;
        effort_ = other.effort_;
        type_ = other.type_;
        limits_ = other.limits_;
        dynamics_ = other.dynamics_;
        mimic_ = other.mimic_;
        name_ = other.name_;
        parent_ = other.parent_;
        child_ = other.child_;
        axis_ = other.axis_;
        origin_ = other.origin_;
    }
    return *this;
}

bool Joint::in_bounds(double position) const { return position >= limits_.lower && position <= limits_.upper; }

double Joint::clamp(double position) const {
    if (type_ == Type::CONTINUOUS) {
        // Wrap to (-pi, pi] without clamping to joint limits
        position = std::fmod(position + M_PI, 2 * M_PI);
        if (position < 0) position += 2 * M_PI;
        position -= M_PI;
        return position;
    } else {
        // Clamp for revolute or prismatic joints
        if (position < limits_.lower) {
            return limits_.lower;
        } else if (position > limits_.upper) {
            return limits_.upper;
        }
        return position;
    }
}

const std::string &Joint::name() const { return name_; }
const std::string &Joint::parent() const { return parent_; }
const std::string &Joint::child() const { return child_; }
Joint::Type Joint::type() const { return type_; }
const JointLimits &Joint::limits() const { return limits_; }
const JointDynamics &Joint::dynamics() const { return dynamics_; }

bool Joint::is_mimic() const { return !mimic_.name.empty(); }
const JointMimic &Joint::mimic() const { return mimic_; }

double Joint::position() const {
    if (is_mimic()) {
        auto parent_state = mimic_.joint->get_state();
        return parent_state.position * mimic_.multiplier + mimic_.offset;
    }
    return position_;
}
double Joint::velocity() const {
    if (is_mimic()) {
        auto parent_state = mimic_.joint->get_state();
        return parent_state.velocity * mimic_.multiplier + mimic_.offset;
    }
    return velocity_;
}
double Joint::effort() const {
    if (is_mimic()) {
        auto parent_state = mimic_.joint->get_state();
        return parent_state.velocity * mimic_.multiplier + mimic_.offset;
    }
    return velocity_;
}

const rix::msg::geometry::Vector3 &Joint::axis() const { return axis_; }
const rix::msg::geometry::Transform &Joint::origin() const { return origin_; }

/**< TODO: Implement transform method. */
rix::msg::geometry::Transform Joint::transform() const {
    return {};
}

rix::msg::sensor::JointState Joint::get_state() const {
    rix::msg::sensor::JointState js;
    js.name = name_;
    if (is_mimic()) {
        auto parent_state = mimic_.joint->get_state();
        js.position = parent_state.position * mimic_.multiplier + mimic_.offset;
        js.velocity = parent_state.velocity * mimic_.multiplier + mimic_.offset;
        js.effort = parent_state.effort * mimic_.multiplier + mimic_.offset;
        return js;
    }
    js.position = position_;
    js.velocity = velocity_;
    js.effort = effort_;
    return js;
}

void Joint::set_state(double position, double velocity, double effort) {
    if (is_mimic()) {
        rix::util::Log::warn << "Cannot set state of mimic joint: \"" << name_ << "\"!";
        return;
    }
    position_ = position;
    velocity_ = velocity;
    effort_ = effort;
}

void Joint::set_state(const rix::msg::sensor::JointState &joint_state) {
    if (is_mimic()) {
        rix::util::Log::warn << "Cannot set state of mimic joint: \"" << name_ << "\"!";
        return;
    }
    position_ = joint_state.position;
    velocity_ = joint_state.velocity;
    effort_ = joint_state.effort;
}

}  // namespace robot
}  // namespace rix