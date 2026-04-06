#include "rix/robot/robot_model.hpp"

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include "rix/robot/eigen_util.hpp"

#include <set>
#include <algorithm>

using Json = nlohmann::json;

namespace rix {
namespace robot {

namespace detail { 

bool convert_json_sphere(const Json &src, Sphere &dst);
bool convert_json_box(const Json &src, Box &dst);
bool convert_json_cylinder(const Json &src, Cylinder &dst);
bool convert_json_mesh(const Json &src, Mesh &dst);
bool convert_json_material(const Json &src, Material &dst, const std::map<std::string, Material> &materials);
bool convert_json_origin(const Json &src, rix::msg::geometry::Transform &dst);
bool convert_json_inertial(const Json &src, Inertial &dst);
bool convert_json_geometry(const Json &src, std::shared_ptr<Geometry> &dst);
bool convert_json_visual(const Json &src, Visual &dst);
bool convert_json_collision(const Json &src, Collision &dst);
bool convert_json_joint_dynamics(const Json &src, JointDynamics &dst);
bool convert_json_joint_limits(const Json &src, JointLimits &dst);
std::shared_ptr<Joint> convert_json_joint(const Json &src);
std::shared_ptr<Link> convert_json_link(const Json &src, const std::map<std::string, std::string> &parent_map,
                                        const std::map<std::string, std::vector<std::string>> &children_map,
                                        const std::map<std::string, Material> &materials);
void parse_jrdf(Json &json, std::map<std::string, std::shared_ptr<Joint>> &joints,
                std::map<std::string, std::shared_ptr<Link>> &links, std::string &root);

}  // namespace detail

RobotModel RobotModel::from_json(const std::string &json_str) {
    RobotModel robot;
    Json json = Json::parse(json_str);
    detail::parse_jrdf(json, robot.joints, robot.links, robot.root);
    return robot;
}

RobotModel::RobotModel() : root(""), world_to_root(transform_identity()) {}

RobotModel::RobotModel(const std::string &file_path) : world_to_root(transform_identity()) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        return;
    }
    Json json = Json::parse(file);
    detail::parse_jrdf(json, joints, links, root);
}

RobotModel::RobotModel(const RobotModel &other)
    : joints(other.joints), links(other.links), root(other.root), world_to_root(other.world_to_root) {}

RobotModel &RobotModel::operator=(const RobotModel &other) {
    if (this != &other) {
        joints = other.joints;
        links = other.links;
        root = other.root;
        world_to_root = other.world_to_root;
        return *this;
    }
    return *this;
}

bool RobotModel::is_valid() const { return !joints.empty() && !links.empty() && !root.empty(); }

size_t RobotModel::get_joint_count() const { return joints.size(); }
size_t RobotModel::get_link_count() const { return links.size(); }

std::vector<std::string> RobotModel::get_joint_names() const {
    std::vector<std::string> names;
    for (auto j : joints) {
        names.push_back(j.second->name());
    }
    return names;
}

std::vector<std::string> RobotModel::get_link_names() const {
    std::vector<std::string> names;
    for (auto l : links) {
        names.push_back(l.second->name());
    }
    return names;
}

bool RobotModel::has_joint(const std::string &name) const { return joints.find(name) != joints.end(); }
bool RobotModel::has_link(const std::string &name) const { return links.find(name) != links.end(); }

std::shared_ptr<Joint> RobotModel::get_joint(const std::string &name) const {
    auto it = joints.find(name);
    if (it == joints.end()) {
        return nullptr;
    }
    return it->second;
}

std::shared_ptr<Link> RobotModel::get_link(const std::string &name) const {
    auto it = links.find(name);
    if (it == links.end()) {
        return nullptr;
    }
    return it->second;
}

std::shared_ptr<Link> RobotModel::get_root() const { return links.at(root); }

std::vector<std::shared_ptr<Link>> RobotModel::get_end_effectors() const {
    std::vector<std::shared_ptr<Link>> end_effectors;
    for (const auto &pair : links) {
        if (pair.second->is_end_effector()) {
            end_effectors.push_back(pair.second);
        }
    }
    return end_effectors;
}

/**< TODO: Implement the get_joints_in_chain method */
std::vector<std::shared_ptr<Joint>> RobotModel::get_joints_in_chain(const std::string &link_name) const {
     std::vector<std::shared_ptr<Joint>> chain;
    
    // Find the joint that connects to this link (this is the last joint in chain)
    std::shared_ptr<Joint> current_joint = nullptr;
    for (const auto& [name, joint] : joints) { 
        if (joint->child() == link_name) {
            current_joint = joint;
            break;
        }
    }
    
    // If link not found, return empty chain
    if (!current_joint) {
        return chain;
    }
    
    // Traverse backwards from link to root
    chain.push_back(current_joint);
    std::string parent_link = current_joint->parent();
    
    // Stop when we reach the root
    while (parent_link != root) {
        // Find joint connecting to this parent_link
        std::shared_ptr<Joint> parent_joint = nullptr;
        for (const auto& [name, joint] : joints) {  
            if (joint->child() == parent_link) {
                parent_joint = joint;
                break;
            }
        }
        
        // If we can't find the parent joint, stop
        if (!parent_joint) break;
        
        chain.push_back(parent_joint);
        parent_link = parent_joint->parent();
    }
    
    // Reverse to get root-to-link order
    std::reverse(chain.begin(), chain.end());
    
    return chain;
}

/**< TODO: Implement the get_transforms method */
rix::msg::geometry::TF RobotModel::get_transforms() const {
    rix::msg::geometry::TF tf;
    
    // Add world → root transform
    rix::msg::geometry::TransformStamped world_to_root_ts;
    world_to_root_ts.header.frame_id = "world";
    world_to_root_ts.child_frame_id = root;
    world_to_root_ts.transform = world_to_root;
    //world_to_root_ts.header.stamp = rix::util::Time::now();
    tf.transforms.push_back(world_to_root_ts);
    
    // Add transforms for all joints in proper order (parent before child)
    std::set<std::string> added_links = {"world", root};
    
    bool added_new = true;
    while (added_new && added_links.size() < (links.size() + 2)) {
        added_new = false;
        
        for (const auto& [name, joint] : joints) {  
            if (added_links.find(joint->parent()) == added_links.end()) {
                continue;
            }
            
            // Check if child already added (avoid duplicates)
            if (added_links.find(joint->child()) != added_links.end()) {
                continue;  // Child already added
            }
            
            // Create and add transform for this joint
            rix::msg::geometry::TransformStamped transform;
            transform.header.frame_id = joint->parent();
            transform.child_frame_id = joint->child();
            transform.transform = joint->transform();
            //transform.header.stamp = rix::util::Time::now();
            
            tf.transforms.push_back(transform);
            added_links.insert(joint->child());
            added_new = true;
        }
    }
    
    return tf;
}

/**< TODO: Implement the get_joint_states method */
rix::msg::sensor::JS RobotModel::get_joint_states() const {
    rix::msg::sensor::JS js;
    
    // Add joint state for each joint
    for (const auto& [name, joint] : joints) {  
        js.joint_states.push_back(joint->get_state());
    }
    
    return js;
}

rix::msg::geometry::TF RobotModel::get_static_transforms() const {
    rix::msg::geometry::TF tf;
    rix::msg::geometry::TransformStamped transform;
    transform.header.stamp = rix::util::Time::now().to_msg();
    transform.header.seq = 0;

    size_t index = 0;
    for (const auto &pair : links) {
        const std::string &link_name = pair.first;
        std::shared_ptr<Link> link = pair.second;

        transform.header.frame_id = link_name;
        transform.child_frame_id = link_name + "/intertial";
        transform.transform = link->inertial().origin;
        tf.transforms.push_back(transform);
        transform.header.seq++;

        size_t visual_index = 0;
        for (const auto &visual : link->visuals()) {
            transform.child_frame_id = link_name + "/visual/" + std::to_string(visual_index);
            transform.transform = visual.origin;
            tf.transforms.push_back(transform);
            transform.header.seq++;
            visual_index++;
        }

        size_t collision_index = 0;
        for (const auto &collision : link->collisions()) {
            transform.child_frame_id = link_name + "/collision/" + std::to_string(collision_index);
            transform.transform = collision.origin;
            tf.transforms.push_back(transform);
            transform.header.seq++;
            collision_index++;
        }
    }
    return tf;
}

void RobotModel::set_state(const rix::msg::sensor::JS &js) {
    for (const auto &joint_state : js.joint_states) {
        set_state(joint_state);
    }
}

void RobotModel::set_state(const rix::msg::sensor::JointState &js) {
    set_state(js.name, js.position, js.velocity, js.effort);
}

void RobotModel::set_state(const std::string &joint_name, double position, double velocity, double effort) {
    auto it = joints.find(joint_name);
    if (it == joints.end()) {
        return;
    }
    it->second->set_state(position, velocity, effort);
}

void RobotModel::set_world_to_root(const rix::msg::geometry::Transform &transform) { world_to_root = transform; }

const rix::msg::geometry::Transform &RobotModel::get_world_to_root() const { return world_to_root; }

namespace detail {

bool convert_json_sphere(const Json &src, Sphere &dst) {
    if (!src.is_object()) return false;
    if (!src.contains("radius")) return false;
    if (!src.at("radius").is_number()) return false;
    double radius = src.at("radius").template get<double>();
    dst.radius = radius;
    return true;
}

bool convert_json_box(const Json &src, Box &dst) {
    if (!src.is_object()) return false;
    if (!src.contains("size")) return false;
    if (!src.at("size").is_array()) return false;
    if (src.at("size").size() != 3) return false;
    if (!src.at("size")[0].is_number()) return false;
    if (!src.at("size")[1].is_number()) return false;
    if (!src.at("size")[2].is_number()) return false;

    dst.dim.x = src.at("size")[0].template get<double>();
    dst.dim.y = src.at("size")[1].template get<double>();
    dst.dim.z = src.at("size")[2].template get<double>();
    return true;
}

bool convert_json_cylinder(const Json &src, Cylinder &dst) {
    if (!src.is_object()) return false;
    if (!src.contains("radius")) return false;
    if (!src.at("radius").is_number()) return false;
    if (!src.contains("length")) return false;
    if (!src.at("length").is_number()) return false;
    dst.radius = src.at("radius").template get<double>();
    dst.length = src.at("length").template get<double>();
    return true;
}

bool convert_json_mesh(const Json &src, Mesh &dst) {
    if (!src.is_object()) return false;
    if (!src.contains("filename")) return false;
    if (!src.at("filename").is_string()) return false;
    dst.filename = src.at("filename").template get<std::string>();

    dst.scale.x = dst.scale.y = dst.scale.z = 1;
    if (src.contains("scale")) {
        if (!src.at("scale").is_array()) return false;
        if (src.at("scale").size() != 3) return false;
        if (!src.at("scale")[0].is_number()) return false;
        if (!src.at("scale")[1].is_number()) return false;
        if (!src.at("scale")[2].is_number()) return false;
        dst.scale.x = src.at("scale")[0].template get<double>();
        dst.scale.y = src.at("scale")[1].template get<double>();
        dst.scale.z = src.at("scale")[2].template get<double>();
    }
    return true;
}

bool convert_json_material(const Json &src, Material &dst, const std::map<std::string, Material> &materials) {
    if (!src.is_object()) return false;
    if (!src.contains("name")) return false;
    if (!src.at("name").is_string()) return false;
    std::string name = src.at("name").template get<std::string>();
    auto it = materials.find(name);
    if (it != materials.end()) {
        dst = it->second;
        return true;
    }

    if (src.contains("filename")) {
        if (!src.at("filename").is_string()) return false;
        dst.texture_filename = src.at("filename").template get<std::string>();
    }
    if (src.contains("color")) {
        if (!src.at("color").is_array()) return false;
        if (src.at("color").size() != 4) return false;
        if (!src.at("color")[0].is_number()) return false;
        if (!src.at("color")[1].is_number()) return false;
        if (!src.at("color")[2].is_number()) return false;
        if (!src.at("color")[3].is_number()) return false;
        dst.color.r = src.at("color")[0].template get<float>();
        dst.color.g = src.at("color")[1].template get<float>();
        dst.color.b = src.at("color")[2].template get<float>();
        dst.color.a = src.at("color")[3].template get<float>();
    }
    return true;
}

bool convert_json_origin(const Json &src, rix::msg::geometry::Transform &dst) {
    if (!src.is_object()) return false;
    if (!src.contains("xyz")) return false;
    if (!src.contains("rpy")) return false;
    if (!src.at("xyz").is_array()) return false;
    if (!src.at("rpy").is_array()) return false;
    if (src.at("xyz").size() != 3) return false;
    if (src.at("rpy").size() != 3) return false;
    if (!src.at("xyz")[0].is_number()) return false;
    if (!src.at("xyz")[1].is_number()) return false;
    if (!src.at("xyz")[2].is_number()) return false;
    if (!src.at("rpy")[0].is_number()) return false;
    if (!src.at("rpy")[1].is_number()) return false;
    if (!src.at("rpy")[2].is_number()) return false;

    Eigen::Vector3d xyz_v(src.at("xyz")[0].template get<double>(), src.at("xyz")[1].template get<double>(),
                          src.at("xyz")[2].template get<double>());

    Eigen::Vector3d rpy_v(src.at("rpy")[0].template get<double>(), src.at("rpy")[1].template get<double>(),
                          src.at("rpy")[2].template get<double>());

    Eigen::Affine3d matrix = Eigen::Translation3d(xyz_v) * Eigen::AngleAxisd(rpy_v[0], Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(rpy_v[1], Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(rpy_v[2], Eigen::Vector3d::UnitZ());

    dst = eigen_to_msg(matrix);
    return true;
}

bool convert_json_inertial(const Json &src, Inertial &dst) {
    if (!src.is_object()) return false;
    dst.origin = transform_identity();
    if (src.contains("origin")) {
        if (!convert_json_origin(src.at("origin"), dst.origin)) return false;
    }

    if (!src.contains("mass")) return false;
    if (!src.at("mass").is_number()) return false;
    dst.mass = src.at("mass").template get<double>();

    if (!src.contains("inertia")) return false;
    if (!src.at("inertia").is_object()) return false;
    if (!src.at("inertia").contains("ixx")) return false;
    if (!src.at("inertia").contains("ixy")) return false;
    if (!src.at("inertia").contains("ixz")) return false;
    if (!src.at("inertia").contains("iyy")) return false;
    if (!src.at("inertia").contains("iyz")) return false;
    if (!src.at("inertia").contains("izz")) return false;
    if (!src.at("inertia").at("ixx").is_number()) return false;
    if (!src.at("inertia").at("ixy").is_number()) return false;
    if (!src.at("inertia").at("ixz").is_number()) return false;
    if (!src.at("inertia").at("iyy").is_number()) return false;
    if (!src.at("inertia").at("iyz").is_number()) return false;
    if (!src.at("inertia").at("izz").is_number()) return false;

    dst.ixx = src.at("inertia").at("ixx").template get<double>();
    dst.ixy = src.at("inertia").at("ixy").template get<double>();
    dst.ixz = src.at("inertia").at("ixz").template get<double>();
    dst.iyy = src.at("inertia").at("iyy").template get<double>();
    dst.iyz = src.at("inertia").at("iyz").template get<double>();
    dst.izz = src.at("inertia").at("izz").template get<double>();
    return true;
}

bool convert_json_geometry(const Json &src, std::shared_ptr<Geometry> &dst) {
    if (!src.is_object()) return false;
    if (!src.contains("type")) return false;
    if (!src.at("type").is_string()) return false;
    std::string type = src.at("type").template get<std::string>();
    if (type == "box") {
        auto dst_ptr = std::make_shared<Box>();
        if (!convert_json_box(src, *dst_ptr)) return false;
        dst = dst_ptr;
    } else if (type == "cylinder") {
        auto dst_ptr = std::make_shared<Cylinder>();
        if (!convert_json_cylinder(src, *dst_ptr)) return false;
        dst = dst_ptr;
    } else if (type == "sphere") {
        auto dst_ptr = std::make_shared<Sphere>();
        if (!convert_json_sphere(src, *dst_ptr)) return false;
        dst = dst_ptr;
    } else if (type == "mesh") {
        auto dst_ptr = std::make_shared<Mesh>();
        if (!convert_json_mesh(src, *dst_ptr)) return false;
        dst = dst_ptr;
    } else {
        return false;
    }
    return true;
}

bool convert_json_visual(const Json &src, Visual &dst, const std::map<std::string, Material> &materials) {
    if (!src.is_object()) return false;
    dst.origin = transform_identity();
    if (src.contains("origin")) {
        if (!convert_json_origin(src.at("origin"), dst.origin)) return false;
    }

    if (!src.contains("geometry")) return false;
    if (!convert_json_geometry(src.at("geometry"), dst.geometry)) return false;

    if (src.contains("material")) {
        if (!convert_json_material(src.at("material"), dst.material, materials)) return false;
    }
    return true;
}

bool convert_json_collision(const Json &src, Collision &dst) {
    if (!src.is_object()) return false;
    dst.origin = transform_identity();
    if (src.contains("origin")) {
        if (!convert_json_origin(src.at("origin"), dst.origin)) return false;
    }

    if (!src.contains("geometry")) return false;
    if (!convert_json_geometry(src.at("geometry"), dst.geometry)) return false;
    return true;
}

std::shared_ptr<Link> convert_json_link(const Json &src, const std::map<std::string, std::string> &parent_map,
                                        const std::map<std::string, std::vector<std::string>> &children_map,
                                        const std::map<std::string, Material> &materials) {
    if (!src.is_object()) return nullptr;

    if (!src.contains("name")) return nullptr;
    if (!src.at("name").is_string()) return nullptr;
    std::string name = src.at("name").template get<std::string>();

    auto parent_it = parent_map.find(name);
    std::string parent = "";
    if (parent_it != parent_map.end()) {
        parent = parent_it->second;
    }

    auto children_it = children_map.find(name);
    std::vector<std::string> children;
    if (children_it != children_map.end()) {
        children = children_it->second;
    }

    std::vector<Visual> visuals;
    if (src.contains("visuals")) {
        if (!src.at("visuals").is_array()) return nullptr;
        const size_t num_visuals = src.at("visuals").size();
        visuals.resize(num_visuals);
        for (size_t i = 0; i < num_visuals; i++) {
            if (!convert_json_visual(src.at("visuals")[i], visuals[i], materials)) return nullptr;
        }
    }

    std::vector<Collision> collisions;
    if (src.contains("collisions")) {
        if (!src.at("collisions").is_array()) return nullptr;
        const size_t num_collisions = src.at("collisions").size();
        collisions.resize(num_collisions);
        for (size_t i = 0; i < collisions.size(); i++) {
            if (!convert_json_collision(src.at("collisions")[i], collisions[i])) return nullptr;
        }
    }

    Inertial inertial;
    if (src.contains("inertial")) {
        if (!convert_json_inertial(src.at("inertial"), inertial)) return nullptr;
    }

    return std::make_shared<Link>(visuals, collisions, inertial, name, parent, children);
}

bool convert_json_joint_dynamics(const Json &src, JointDynamics &dst) {
    if (!src.is_object()) return false;
    dst.damping = 0;
    if (src.contains("damping")) {
        if (!src.at("damping").is_number()) return false;
        dst.damping = src.at("damping").template get<double>();
    }
    dst.friction = 0;
    if (src.contains("friction")) {
        if (!src.at("friction").is_number()) return false;
        dst.friction = src.at("friction").template get<double>();
    }
    return true;
}

bool convert_json_joint_mimic(const Json &src, JointMimic &dst) {
    if (!src.is_object()) return false;

    dst.offset = 0;
    if (!src.contains("offset")) return false;
    if (!src.at("offset").is_number()) return false;
    dst.offset = src.at("offset").template get<double>();

    dst.multiplier = 0;
    if (!src.contains("multiplier")) return false;
    if (!src.at("multiplier").is_number()) return false;
    dst.multiplier = src.at("multiplier").template get<double>();

    dst.name = "";
    if (!src.contains("name")) return false;
    if (!src.at("name").is_string()) return false;
    dst.name = src.at("name").template get<std::string>();
    return true;
}

bool convert_json_joint_limits(const Json &src, JointLimits &dst) {
    if (!src.is_object()) return false;

    if (!src.contains("effort")) return false;
    if (!src.at("effort").is_number()) return false;
    dst.effort = src.at("effort").template get<double>();

    if (!src.contains("velocity")) return false;
    if (!src.at("velocity").is_number()) return false;
    dst.effort = src.at("velocity").template get<double>();

    dst.lower = 0;
    if (src.contains("lower")) {
        if (!src.at("lower").is_number()) return false;
        dst.lower = src.at("lower").template get<double>();
    }
    dst.upper = 0;
    if (src.contains("upper")) {
        if (!src.at("upper").is_number()) return false;
        dst.upper = src.at("upper").template get<double>();
    }
    return true;
}

std::shared_ptr<Joint> convert_json_joint(const Json &src) {
    if (!src.is_object()) return nullptr;

    if (!src.contains("name")) return nullptr;
    if (!src.at("name").is_string()) return nullptr;
    std::string name = src.at("name").template get<std::string>();

    if (!src.contains("parent")) return nullptr;
    if (!src.at("parent").is_string()) return nullptr;
    std::string parent = src.at("parent").template get<std::string>();

    if (!src.contains("child")) return nullptr;
    if (!src.at("child").is_string()) return nullptr;
    std::string child = src.at("child").template get<std::string>();

    Joint::Type type;
    if (!src.contains("type")) return nullptr;
    if (!src.at("type").is_string()) return nullptr;
    std::string type_str = src.at("type").template get<std::string>();
    if (type_str == "revolute") {
        type = Joint::Type::REVOLUTE;
    } else if (type_str == "prismatic") {
        type = Joint::Type::PRISMATIC;
    } else if (type_str == "fixed") {
        type = Joint::Type::FIXED;
    } else if (type_str == "continuous") {
        type = Joint::Type::CONTINUOUS;
    } else {
        return nullptr;
    }

    rix::msg::geometry::Vector3 axis;
    bool has_axis = src.contains("axis");
    if (!has_axis && type != Joint::Type::FIXED) return nullptr;  // All joint must have axis besides fixed
    if (has_axis) {
        if (!src.at("axis").is_array()) return nullptr;
        if (src.at("axis").size() != 3) return nullptr;
        if (!src.at("axis")[0].is_number()) return nullptr;
        if (!src.at("axis")[1].is_number()) return nullptr;
        if (!src.at("axis")[2].is_number()) return nullptr;
        axis.x = src.at("axis")[0].template get<double>();
        axis.y = src.at("axis")[1].template get<double>();
        axis.z = src.at("axis")[2].template get<double>();
    }

    rix::msg::geometry::Transform origin = rix::robot::transform_identity();
    if (src.contains("origin")) {
        if (!convert_json_origin(src.at("origin"), origin)) return nullptr;
    }

    JointLimits limits;
    bool has_limits = src.contains("limits");
    if ((type == Joint::Type::REVOLUTE || type == Joint::Type::PRISMATIC) && !has_limits) return nullptr;
    if (has_limits) {
        if (!convert_json_joint_limits(src.at("limits"), limits)) return nullptr;
    }

    JointDynamics dynamics;
    if (src.contains("dynamics")) {
        if (!convert_json_joint_dynamics(src.at("dynamics"), dynamics)) return nullptr;
    }

    JointMimic mimic;
    if (src.contains("mimic")) {
        if (!convert_json_joint_mimic(src.at("mimic"), mimic)) return nullptr;
    }

    return std::make_shared<Joint>(axis, origin, type, limits, dynamics, mimic, name, parent, child);
}

void parse_jrdf(Json &json, std::map<std::string, std::shared_ptr<Joint>> &joints,
                std::map<std::string, std::shared_ptr<Link>> &links, std::string &root) {
    // Parse joints first to get the link's parent/children
    if (!json.is_object()) return;

    // Parse any global materials
    std::map<std::string, Material> materials;
    if (json.contains("materials")) {
        if (!json.at("materials").is_array()) return;
        auto material_array = json.at("materials");
        const size_t num_materials = material_array.size();
        for (size_t i = 0; i < num_materials; i++) {
            Material material;
            if (!convert_json_material(material_array[i], material, materials)) return;
            materials.insert({material.name, material});
        }
    }

    if (!json.contains("joints")) return;
    if (!json.at("joints").is_array()) return;
    auto joint_array = json.at("joints");
    const size_t num_joints = joint_array.size();
    std::map<std::string, std::string> parent_map;
    std::map<std::string, std::vector<std::string>> children_map;
    for (size_t i = 0; i < num_joints; i++) {
        std::shared_ptr<Joint> j = convert_json_joint(joint_array[i]);
        if (!j) return;
        joints.insert({j->name(), j});
        if (parent_map.find(j->child()) != parent_map.end()) return;  // Each link can only have 1 parent
        parent_map.insert({j->child(), j->name()});
        children_map[j->parent()].push_back(j->name());
    }

    // Assign mimic joint pointers
    for (auto j : joints) {
        if (j.second->is_mimic()) {
            j.second->mimic().joint = joints.at(j.second->mimic().name);
        }
    }

    // Parse the links
    if (!json.contains("links")) return;
    if (!json.at("links").is_array()) return;
    auto link_array = json.at("links");
    const size_t num_links = link_array.size();
    for (size_t i = 0; i < num_links; i++) {
        std::shared_ptr<Link> l = convert_json_link(link_array[i], parent_map, children_map, materials);
        if (!l) return;
        if (l->is_root()) root = l->name();
        links.insert({l->name(), l});
    }
    return;
}

}  // namespace detail

}  // namespace robot
}  // namespace rix