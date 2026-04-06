#pragma once

#include <iostream>
#include <memory>

#include "rix/msg/geometry/Transform.hpp"
#include "rix/msg/geometry/Vector3.hpp"
#include "rix/msg/sensor/JointState.hpp"
#include "rix/robot/msg_util.hpp"

namespace rix {
namespace robot {

class JointDynamics {
   public:
    JointDynamics() {};
    double damping;
    double friction;
};

class JointLimits {
   public:
    JointLimits() {};
    double lower;
    double upper;
    double effort;
    double velocity;
};

class Joint; // Forward declaration

class JointMimic {
   public:
    JointMimic() {};
    double offset;
    double multiplier;
    std::string name;
    mutable std::shared_ptr<Joint> joint;
};

class Joint {
   public:
    enum Type { UNKNOWN, FIXED, CONTINUOUS, REVOLUTE, PRISMATIC };

    Joint(const rix::msg::geometry::Vector3 &axis = rix::robot::vector3_zeros(),
          const rix::msg::geometry::Transform &origin = rix::robot::transform_identity(), 
          const Type &type = FIXED,
          const JointLimits &limits = {},
          const JointDynamics &dynamics = {},
          const JointMimic &mimic = {},
          const std::string &name = "",
          const std::string &parent = "",
          const std::string &child = "");
    Joint(const Joint &j);
    Joint &operator=(const Joint &j);

    bool in_bounds(double position) const;
    double clamp(double position) const;

    const std::string &name() const;
    const std::string &parent() const;
    const std::string &child() const;
    Type type() const;
    const JointLimits &limits() const;
    const JointDynamics &dynamics() const;
    
    bool is_mimic() const;
    const JointMimic &mimic() const;

    double position() const;  // rad or m
    double velocity() const;  // rad/s or m/s
    double effort() const;    // Nm or N
    rix::msg::sensor::JointState get_state() const;
    void set_state(double position, double velocity, double effort);
    void set_state(const rix::msg::sensor::JointState &js);

    const rix::msg::geometry::Vector3 &axis() const;      // Axis of rotation or translation
    const rix::msg::geometry::Transform &origin() const;  // Transform from parent link frame to joint frame
    rix::msg::geometry::Transform transform() const;      // Transform based on current joint state
    
   private:
    double position_;  // The current position (rad or m)
    double velocity_;  // The current velocity (rad/s or m/s)
    double effort_;    // The current effort (Nm or N)

    Type type_;                        // The type of the joint (Fixed, Continuous, Revolute, or Prismatic)
    JointLimits limits_;                    // The limits on the joint
    JointDynamics dynamics_;                // The dynamics of the joint
    JointMimic mimic_;                      // The mimic parameters of the joint
    std::string name_;                      // The name of the joint
    std::string parent_;                    // The name of the joint's parent link
    std::string child_;                     // The name of the joint's child link
    rix::msg::geometry::Vector3 axis_;      // The unit vector along the actuation axis in the parent link frame
    rix::msg::geometry::Transform origin_;  // The transform from parent frame to child frame when the position is 0
};

}  // namespace robot
}  // namespace rix