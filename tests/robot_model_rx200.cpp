#include <gtest/gtest.h>

#include "print_helper.hpp"
#include "rix/robot/robot_model.hpp"
#include "robots.hpp"

using namespace rix::robot;

// Need to test: get_joints_in_chain, get_transforms, and get_joint_states

TEST(RobotModel, GetJointsInChainTest) {
    RobotModel robot(RobotModel::from_json(RX200_JRDF));

    auto joints = robot.get_joints_in_chain("/ee_gripper_link");
    ASSERT_EQ(joints.size(), 9);
    EXPECT_EQ(joints[0]->name(), "waist");
    EXPECT_EQ(joints[1]->name(), "shoulder");
    EXPECT_EQ(joints[2]->name(), "elbow");
    EXPECT_EQ(joints[3]->name(), "wrist_angle");
    EXPECT_EQ(joints[4]->name(), "wrist_rotate");
    EXPECT_EQ(joints[5]->name(), "ee_arm");
    EXPECT_EQ(joints[6]->name(), "gripper_bar");
    EXPECT_EQ(joints[7]->name(), "ee_bar");
    EXPECT_EQ(joints[8]->name(), "ee_gripper");

    joints = robot.get_joints_in_chain("/gripper_prop_link");
    ASSERT_EQ(joints.size(), 7);
    EXPECT_EQ(joints[0]->name(), "waist");
    EXPECT_EQ(joints[1]->name(), "shoulder");
    EXPECT_EQ(joints[2]->name(), "elbow");
    EXPECT_EQ(joints[3]->name(), "wrist_angle");
    EXPECT_EQ(joints[4]->name(), "wrist_rotate");
    EXPECT_EQ(joints[5]->name(), "ee_arm");
    EXPECT_EQ(joints[6]->name(), "gripper");

    joints = robot.get_joints_in_chain("/left_finger_link");
    ASSERT_EQ(joints.size(), 9);
    EXPECT_EQ(joints[0]->name(), "waist");
    EXPECT_EQ(joints[1]->name(), "shoulder");
    EXPECT_EQ(joints[2]->name(), "elbow");
    EXPECT_EQ(joints[3]->name(), "wrist_angle");
    EXPECT_EQ(joints[4]->name(), "wrist_rotate");
    EXPECT_EQ(joints[5]->name(), "ee_arm");
    EXPECT_EQ(joints[6]->name(), "gripper_bar");
    EXPECT_EQ(joints[7]->name(), "ee_bar");
    EXPECT_EQ(joints[8]->name(), "left_finger");

    joints = robot.get_joints_in_chain("/right_finger_link");
    ASSERT_EQ(joints.size(), 9);
    EXPECT_EQ(joints[0]->name(), "waist");
    EXPECT_EQ(joints[1]->name(), "shoulder");
    EXPECT_EQ(joints[2]->name(), "elbow");
    EXPECT_EQ(joints[3]->name(), "wrist_angle");
    EXPECT_EQ(joints[4]->name(), "wrist_rotate");
    EXPECT_EQ(joints[5]->name(), "ee_arm");
    EXPECT_EQ(joints[6]->name(), "gripper_bar");
    EXPECT_EQ(joints[7]->name(), "ee_bar");
    EXPECT_EQ(joints[8]->name(), "right_finger");

    joints = robot.get_joints_in_chain("/shoulder_link");
    EXPECT_EQ(joints.size(), 1);
    EXPECT_EQ(joints[0]->name(), "waist");

    joints = robot.get_joints_in_chain("/base_link");
    EXPECT_TRUE(joints.empty());

    joints = robot.get_joints_in_chain("not_a_joint");
    EXPECT_TRUE(joints.empty());
}

TEST(RobotModel, GetJointStatesTest) {
    RobotModel robot(RobotModel::from_json(RX200_JRDF));

    robot.set_state("gripper", M_PI / 4, 0.8, 1.2);
    robot.set_state("wrist_rotate", M_PI / 6, 0.9, 1.1);
    robot.set_state("wrist_angle", -M_PI / 4, 1.0, 1.0);
    robot.set_state("elbow", M_PI / 4, 1.1, 0.9);
    robot.set_state("shoulder", -M_PI / 3, 1.2, 0.8);
    robot.set_state("waist", -M_PI / 3, 1.2, 0.8);

    std::map<std::string, rix::msg::sensor::JointState> correct;
    auto joint_names = robot.get_joint_names();
    for (auto j : joint_names) {
        correct[j] = robot.get_joint(j)->get_state();
    }

    auto js = robot.get_joint_states();

    /*
    ee_arm
      Position: 0
      Velocity: 0
      Effort: 0

    ee_bar
      Position: 0
      Velocity: 0
      Effort: 0

    ee_gripper
      Position: 0
      Velocity: 0
      Effort: 0

    elbow
      Position: 0.785398
      Velocity: 1.1
      Effort: 0.9

    gripper
      Position: 0.785398
      Velocity: 0.8
      Effort: 1.2

    gripper_bar
      Position: 0
      Velocity: 0
      Effort: 0

    left_finger
      Position: 0.0249956
      Velocity: 0.0252
      Effort: 0.0308

    right_finger
      Position: -0.0249956
      Velocity: -0.0252
      Effort: -0.0308

    shoulder
      Position: -1.0472
      Velocity: 1.2
      Effort: 0.8

    waist
      Position: -1.0472
      Velocity: 1.2
      Effort: 0.8

    wrist_angle
      Position: -0.785398
      Velocity: 1
      Effort: 1

    wrist_rotate
      Position: 0.523599
      Velocity: 0.9
      Effort: 1.1
    */

    for (auto j : js.joint_states) {
        print_joint_state(j);
    }

    ASSERT_EQ(js.joint_states.size(), 12);
    EXPECT_GT(js.stamp.sec, 0) << "The timestamp must be set to the current time!";
    EXPECT_GT(js.stamp.nsec, 0) << "The timestamp must be set to the current time!";

    for (auto j : js.joint_states) {
        auto it = correct.find(j.name);
        ASSERT_TRUE(it != correct.end());
        EXPECT_NEAR(j.position, it->second.position, 1e-4);
        EXPECT_NEAR(j.velocity, it->second.velocity, 1e-4);
        EXPECT_NEAR(j.effort, it->second.effort, 1e-4);
    }
}

TEST(RobotModel, GetTransformsTest) {
    RobotModel robot(RobotModel::from_json(RX200_JRDF));

    robot.set_state("gripper", M_PI / 4, 0.8, 1.2);
    robot.set_state("wrist_rotate", M_PI / 6, 0.9, 1.1);
    robot.set_state("wrist_angle", -M_PI / 4, 1.0, 1.0);
    robot.set_state("elbow", M_PI / 4, 1.1, 0.9);
    robot.set_state("shoulder", -M_PI / 3, 1.2, 0.8);
    robot.set_state("waist", -M_PI / 3, 1.2, 0.8);

    auto tf = robot.get_transforms();

    /*
    world -> /base_link
      Translation:
        x: 0
        y: 0
        z: 0
      Rotation:
        w: 1
        x: 0
        y: 0
        z: 0

    /base_link -> /shoulder_link
      Translation:
        x: 0
        y: 0
        z: 0.065
      Rotation:
        w: 0.5
        x: 0
        y: 0
        z: 0.866025

    /shoulder_link -> /upper_arm_link
      Translation:
        x: 0
        y: 0
        z: 0.03891
      Rotation:
        w: 0.866025
        x: 0
        y: 0.5
        z: 0

    /upper_arm_link -> /forearm_link
      Translation:
        x: 0.05
        y: 0
        z: 0.2
      Rotation:
        w: 0
        x: 0.92388
        y: 0
        z: 0.382683

    /forearm_link -> /wrist_link
      Translation:
        x: 0.2
        y: 0
        z: 0
      Rotation:
        w: 0.92388
        x: 0
        y: -0.382683
        z: 0

    /wrist_link -> /gripper_link
      Translation:
        x: 0.065
        y: 0
        z: 0
      Rotation:
        w: -0.258819
        x: 0.965926
        y: 0
        z: 0

    /gripper_link -> /ee_arm_link
      Translation:
        x: 0.043
        y: 0
        z: 0
      Rotation:
        w: 1
        x: 0
        y: 0
        z: 0

    /ee_arm_link -> /gripper_prop_link
      Translation:
        x: 0.0055
        y: 0
        z: 0
      Rotation:
        w: 0.92388
        x: 0.382683
        y: 0
        z: 0

    /ee_arm_link -> /gripper_bar_link
      Translation:
        x: 0
        y: 0
        z: 0
      Rotation:
        w: 1
        x: 0
        y: 0
        z: 0

    /gripper_bar_link -> /fingers_link
      Translation:
        x: 0.023
        y: 0
        z: 0
      Rotation:
        w: 1
        x: 0
        y: 0
        z: 0

    /fingers_link -> /left_finger_link
      Translation:
        x: 0
        y: 0
        z: 0
      Rotation:
        w: 1
        x: 0
        y: 0
        z: 0

    /fingers_link -> /right_finger_link
      Translation:
        x: 0
        y: 0
        z: 0
      Rotation:
        w: 1
        x: 0
        y: 0
        z: 0

    /fingers_link -> /ee_gripper_link
      Translation:
        x: 0.027575
        y: 0
        z: 0
      Rotation:
        w: 1
        x: 0
        y: 0
        z: 0
    */

    ASSERT_EQ(tf.transforms.size(), 13);

    std::map<std::string, size_t> frames;
    frames.insert({"world", -1});

    for (size_t i = 0; i < tf.transforms.size(); ++i) {
        const auto &t = tf.transforms[i];
        print_transform(tf.transforms[i]);
        EXPECT_TRUE(frames.find(t.header.frame_id) != frames.end())
            << "Transforms must be ordered such that, for any given transform, its parent frame appears earlier in the "
               "list. Unable to find \""
            << t.header.frame_id << "\"";
        frames.insert({t.child_frame_id, i});
    }

    ASSERT_EQ(tf.transforms[0].header.frame_id, "world") << "First transform parent frame must be \"world\".";
    ASSERT_EQ(tf.transforms[0].child_frame_id, "/base_link") << "First transform child frame must be the base link.";
    EXPECT_NEAR(tf.transforms[0].transform.translation.x, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[0].transform.translation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[0].transform.translation.z, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[0].transform.rotation.w, 1, 1e-4);
    EXPECT_NEAR(tf.transforms[0].transform.rotation.x, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[0].transform.rotation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[0].transform.rotation.z, 0, 1e-4);

    size_t i = frames["/shoulder_link"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "/base_link");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "/shoulder_link");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0.065, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, 0.5, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0.866025, 1e-4);

    i = frames["/gripper_prop_link"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "/ee_arm_link");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "/gripper_prop_link");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, 0.0055, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, 0.92388, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, 0.382683, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0, 1e-4);

    i = frames["/gripper_bar_link"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "/ee_arm_link");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "/gripper_bar_link");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, 1, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0, 1e-4);

    i = frames["/left_finger_link"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "/fingers_link");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "/left_finger_link");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, 1, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0, 1e-4);

    i = frames["/right_finger_link"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "/fingers_link");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "/right_finger_link");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, 1, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0, 1e-4);

    i = frames["/ee_gripper_link"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "/fingers_link");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "/ee_gripper_link");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, 0.027575, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, 1, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0, 1e-4);

    i = frames["/wrist_link"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "/forearm_link");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "/wrist_link");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, 0.2, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, 0.92388, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, -0.382683, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0, 1e-4);

    i = frames["/fingers_link"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "/gripper_bar_link");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "/fingers_link");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, 0.023, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, 1, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0, 1e-4);

    i = frames["/ee_arm_link"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "/gripper_link");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "/ee_arm_link");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, 0.043, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, 1, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0, 1e-4);

    i = frames["/upper_arm_link"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "/shoulder_link");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "/upper_arm_link");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0.03891, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, 0.866025, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, 0.5, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0, 1e-4);

    i = frames["/forearm_link"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "/upper_arm_link");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "/forearm_link");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, 0.05, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0.2, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, 0.92388, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0.382683, 1e-4);

    i = frames["/gripper_link"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "/wrist_link");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "/gripper_link");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, 0.065, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, -0.258819, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, 0.965926, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, 0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0, 1e-4);
}