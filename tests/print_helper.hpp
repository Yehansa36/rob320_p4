#pragma once

#include <iostream>
#include "rix/msg/sensor/JointState.hpp"
#include "rix/msg/geometry/TransformStamped.hpp"

static inline void print_transform(const rix::msg::geometry::TransformStamped &t) {
    std::cout << t.header.frame_id << " -> " << t.child_frame_id << "\n";
    std::cout << "  Translation:\n";
    std::cout << "    x: " << t.transform.translation.x << "\n";
    std::cout << "    y: " << t.transform.translation.y << "\n";
    std::cout << "    z: " << t.transform.translation.z << "\n";
    std::cout << "  Rotation:\n";
    std::cout << "    w: " << t.transform.rotation.w << "\n";
    std::cout << "    x: " << t.transform.rotation.x << "\n";
    std::cout << "    y: " << t.transform.rotation.y << "\n";
    std::cout << "    z: " << t.transform.rotation.z << "\n";
    std::cout << std::endl;
}

static inline void print_joint_state(const rix::msg::sensor::JointState &j) {
    std::cout << j.name << "\n";
    std::cout << "  Position: " << j.position << "\n";
    std::cout << "  Velocity: " << j.velocity << "\n";
    std::cout << "  Effort: " << j.effort << "\n";
    std::cout << std::endl;
}