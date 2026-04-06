#pragma once

#include <memory>

#include "rix/msg/geometry/Inertia.hpp"
#include "rix/msg/geometry/Transform.hpp"
#include "rix/msg/standard/Color.hpp"
#include "rix/robot/msg_util.hpp"

namespace rix {
namespace robot {

class Geometry {
   public:
    enum { SPHERE, BOX, CYLINDER, MESH } type;

    virtual ~Geometry() {}
};

class Sphere : public Geometry {
   public:
    Sphere() { type = SPHERE; };
    double radius;
};

class Box : public Geometry {
   public:
    Box() { type = BOX; };
    rix::msg::geometry::Vector3 dim;
};

class Cylinder : public Geometry {
   public:
    Cylinder() { type = CYLINDER; };
    double length;
    double radius;
};

class Mesh : public Geometry {
   public:
    Mesh() { type = MESH; };
    std::string filename;
    rix::msg::geometry::Vector3 scale;
};

class Material {
   public:
    Material() {};
    std::string name;
    std::string texture_filename;
    rix::msg::standard::Color color;
};

class Inertial {
   public:
    Inertial() {};
    rix::msg::geometry::Transform origin;
    double mass;
    double ixx, ixy, ixz, iyy, iyz, izz;
};

class Visual {
   public:
    Visual() {};
    rix::msg::geometry::Transform origin;
    std::shared_ptr<Geometry> geometry;
    Material material;
};

class Collision {
   public:
    Collision() {};
    rix::msg::geometry::Transform origin;
    std::shared_ptr<Geometry> geometry;
};

class Link {
   public:
    Link(const std::vector<Visual> &visuals = {},
         const std::vector<Collision> &collisions = {},
         const Inertial &inertial = {},
         const std::string &name = "",
         const std::string &parent = "",
         const std::vector<std::string> &children = {});

    Link(const Link &other);
    Link &operator=(const Link &other);

    bool is_root() const;
    bool is_end_effector() const;

    const std::string &name() const;
    const std::string &parent() const;
    const std::vector<std::string> &children() const;

    const Inertial &inertial() const;                  // Transform from link frame to center of mass frame
    const std::vector<Visual> &visuals() const;        // Transform from link frame to visual frame
    const std::vector<Collision> &collisions() const;  // Transform from link frame to collision frame
    rix::msg::geometry::Inertia get_inertia() const;  // Get the inertia as a message type

   private:
    std::vector<Visual> visuals_;        // The transform from the link frame to the visual frame
    std::vector<Collision> collisions_;  // The transform from the link frame to the collision frame
    Inertial inertial_;                  // The transform from the link frame to the center of mass frame
    std::string name_;                   // The name of the link
    std::string parent_;                 // The name of the parent joint
    std::vector<std::string> children_;  // The names of the child joints (empty of end effector)
};

}  // namespace robot
}  // namespace rix