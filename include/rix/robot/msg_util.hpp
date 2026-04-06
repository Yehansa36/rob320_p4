#pragma once

#include "rix/msg/geometry/Inertia.hpp"
#include "rix/msg/geometry/Transform.hpp"
#include "rix/msg/geometry/Vector3.hpp"

namespace rix {
namespace robot {

inline rix::msg::geometry::Vector3 vector3_zeros() {
    rix::msg::geometry::Vector3 vec;
    vec.x = 0.0;
    vec.y = 0.0;
    vec.z = 0.0;
    return vec;
}

inline rix::msg::geometry::Transform transform_identity() {
    rix::msg::geometry::Transform t;
    t.translation = vector3_zeros();
    t.rotation.w = 1.0;
    t.rotation.x = 0.0;
    t.rotation.y = 0.0;
    t.rotation.z = 0.0;
    return t;
}

inline rix::msg::geometry::Inertia default_inertia() {
    rix::msg::geometry::Inertia i;
    i.mass = 0;
    i.center_of_mass = vector3_zeros();
    i.ixx = 1;
    i.iyy = 1;
    i.izz = 1;
    i.ixy = 0;
    i.ixz = 0;
    i.iyz = 0;
    return i;
}

}  // namespace robot
}  // namespace rix