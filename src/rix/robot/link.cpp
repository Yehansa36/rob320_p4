#include "rix/robot/link.hpp"

namespace rix {
namespace robot {

Link::Link(const std::vector<Visual> &visuals,
           const std::vector<Collision> &collisions,
           const Inertial &inertial,
           const std::string &name,
           const std::string &parent,
           const std::vector<std::string> &children)
    : visuals_(visuals),
      collisions_(collisions),
      inertial_(inertial),
      name_(name),
      parent_(parent),
      children_(children) {}

Link::Link(const Link &other)
    : visuals_(other.visuals_),
      collisions_(other.collisions_),
      inertial_(other.inertial_),
      name_(other.name_),
      parent_(other.parent_),
      children_(other.children_) {}

Link &Link::operator=(const Link &other) {
    if (this != &other) {
        visuals_ = other.visuals_;
        collisions_ = other.collisions_;
        inertial_ = other.inertial_;
        name_ = other.name_;
        parent_ = other.parent_;
        children_ = other.children_;
    }
    return *this;
}

bool Link::is_root() const { return parent_.empty(); }
bool Link::is_end_effector() const { return children_.empty(); }

const std::string &Link::name() const { return name_; }
const std::string &Link::parent() const { return parent_; }
const std::vector<std::string> &Link::children() const { return children_; }

const Inertial &Link::inertial() const { return inertial_; }
const std::vector<Visual> &Link::visuals() const { return visuals_; }
const std::vector<Collision> &Link::collisions() const { return collisions_; }

rix::msg::geometry::Inertia Link::get_inertia() const {
    rix::msg::geometry::Inertia inertia;
    inertia.mass = inertial_.mass;
    inertia.center_of_mass = inertial_.origin.translation;
    inertia.ixx = inertial_.ixx;
    inertia.ixy = inertial_.ixy;
    inertia.ixz = inertial_.ixz;
    inertia.iyy = inertial_.iyy;
    inertia.iyz = inertial_.iyz;
    inertia.izz = inertial_.izz;
    return inertia;
}

}  // namespace robot
}  // namespace rix