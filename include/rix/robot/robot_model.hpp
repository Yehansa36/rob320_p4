#pragma once

#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <stack>
#include <string>
#include <vector>

#include "rix/msg/geometry/TF.hpp"
#include "rix/msg/geometry/TransformStamped.hpp"
#include "rix/msg/geometry/Twist.hpp"
#include "rix/msg/sensor/JS.hpp"
#include "rix/robot/joint.hpp"
#include "rix/robot/link.hpp"
#include "rix/util/log.hpp"
#include "rix/util/time.hpp"

namespace rix {
namespace robot {

class RobotModel {
   public:
    /**
     * @brief Factory method to create a RobotModel from a JSON string
     *
     * @param json_str A string formatted as valid JSON
     * @return RobotModel A RobotModel parsed from the JSON string
     */
    static RobotModel from_json(const std::string &json_str);

    /**
     * @brief Construct a new RobotModel object.
     *
     * @param file_path The pathname of a valid JRDF (JSON Robot Description File)
     */
    RobotModel(const std::string &file_path);

    /**
     * @brief Copy constructor
     *
     * @param other The RobotModel to copy
     */
    RobotModel(const RobotModel &other);

    /**
     * @brief Copy assignment operator
     *
     * @param other The RobotModel to copy
     * @return RobotModel&
     */
    RobotModel &operator=(const RobotModel &other);

    /**
     * @brief Returns true if the JRDF was successfully parsed in the
     * constructor
     *
     */
    bool is_valid() const;

    /**
     * @brief Returns the total number of joints
     *
     */
    size_t get_joint_count() const;

    /**
     * @brief Returns the total number of links
     *
     */
    size_t get_link_count() const;

    /**
     * @brief Returns a vector of joint names in the RobotModel. The joints are
     * not ordered.
     */
    std::vector<std::string> get_joint_names() const;

    /**
     * @brief Returns a vector of link names in the RobotModel. The links are not
     * ordered.
     */
    std::vector<std::string> get_link_names() const;

    /**
     * @brief Returns true if the joint exists, false otherwise.
     *
     * @param name The name of the joint to check.
     * @return true if the joint exists, false otherwise
     */
    bool has_joint(const std::string &name) const;

    /**
     * @brief Returns true if the link exsits, false otherwise.
     *
     * @param name The name of the link to check
     * @return true if the link exists, false otherwise
     */
    bool has_link(const std::string &name) const;

    /**
     * @brief Get the joint object by name. If the joint does not exist, an exception will be thrown.
     *
     * @param name The name of the joint to get.
     * @return A reference to the joint object.
     */
    std::shared_ptr<Joint> get_joint(const std::string &name) const;

    /**
     * @brief Get the link object by name. If the link does not exist, an exception will be thrown.
     *
     * @param name The name of the link to get.
     * @return A reference to the link object.
     */
    std::shared_ptr<Link> get_link(const std::string &name) const;

    /**
     * @brief Return the root link name of the RobotModel. This is the link that has
     * no parent joint.
     *
     */
    std::shared_ptr<Link> get_root() const;

    /**
     * @brief Returns a vector of end effector names in the RobotModel. An end effector
     * is defined as a link that has no children joints.
     *
     */
    std::vector<std::shared_ptr<Link>> get_end_effectors() const;

    /**
     * @brief Returns a vector of joint names in the chain from the root to the given link name.
     * The first element is the root-most joint and the last element is the parent joint of the link.
     *
     * @param link_name The name of the link to get the joint chain for.
     *
     */
    std::vector<std::shared_ptr<Joint>> get_joints_in_chain(const std::string &link_name) const;

    /**
     * @brief Create a TF message from the current state of the RobotModel.
     *
     * @details The TF message will be ordered such that, for any given
     * frame transformation, its parent will appear in the list before itself.
     *
     * For example:
     *          -> C -> D
     * A -> B <
     *          -> E -> F
     *
     * A must come first, then B before C and E, then C before D, and E before F
     *
     * There are several valid ordering of the transforms, a few are:
     *      [A->B, B->C, C->D, B->E, E->F]
     *      [A->B, B->C, B->E, C->D, E->F]
     *      [A->B, B->E, E->F, B->D, C->D]
     *
     * An example of invalid ordering is:
     *      [A->B, B->C, C->D, E->F, B->E]
     * B->E must come before E->F because B is the parent of E.
     *
     * @return rix::msg::geometry::TF
     */
    rix::msg::geometry::TF get_transforms() const;

    /**
     * @brief Create a JS message from the current state of the RobotModel.
     * There is no order to the joint states in the JS message.
     *
     * @return rix::msg::sensor::JS
     */
    rix::msg::sensor::JS get_joint_states() const;

    /**
     * @brief Get the static transforms from the RobotModel. This returns any
     * inertial, visual, or collision frames for each link.
     *
     * @return rix::msg::geometry::TF
     */
    rix::msg::geometry::TF get_static_transforms() const;

    void set_state(const rix::msg::sensor::JS &js);
    void set_state(const rix::msg::sensor::JointState &js);
    void set_state(const std::string &joint_name, double position, double velocity, double effort);

    void set_world_to_root(const rix::msg::geometry::Transform &world_to_root);
    const rix::msg::geometry::Transform &get_world_to_root() const;

   private:
    std::string root;
    std::map<std::string, std::shared_ptr<Joint>> joints;
    std::map<std::string, std::shared_ptr<Link>> links;
    rix::msg::geometry::Transform world_to_root;

    RobotModel();
};

}  // namespace robot
}  // namespace rix