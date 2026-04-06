#include <gtest/gtest.h>

#include "print_helper.hpp"
#include "rix/robot/robot_model.hpp"
#include "rix/tf/frame_graph.hpp"

#include "robots.hpp"

using namespace rix::robot;

// Need to test: get_joints_in_chain, get_transforms, and get_joint_states

TEST(RobotModel, GetJointsInChainTest) {
    RobotModel robot(RobotModel::from_json(SIMPLEBOT_JRDF));

    auto joints = robot.get_joints_in_chain("pupil_left");
    ASSERT_EQ(joints.size(), 3);
    EXPECT_EQ(joints[0]->name(), "waist");
    EXPECT_EQ(joints[1]->name(), "eye_socket_left");
    EXPECT_EQ(joints[2]->name(), "eye_lens_left");

    joints = robot.get_joints_in_chain("pupil_right");
    ASSERT_EQ(joints.size(), 3);
    EXPECT_EQ(joints[0]->name(), "waist");
    EXPECT_EQ(joints[1]->name(), "eye_socket_right");
    EXPECT_EQ(joints[2]->name(), "eye_lens_right");

    joints = robot.get_joints_in_chain("tool");
    ASSERT_EQ(joints.size(), 4);
    EXPECT_EQ(joints[0]->name(), "waist");
    EXPECT_EQ(joints[1]->name(), "shoulder");
    EXPECT_EQ(joints[2]->name(), "elbow");
    EXPECT_EQ(joints[3]->name(), "wrist");

    joints = robot.get_joints_in_chain("torso");
    ASSERT_EQ(joints.size(), 1);
    EXPECT_EQ(joints[0]->name(), "waist");

    joints = robot.get_joints_in_chain("base");
    EXPECT_TRUE(joints.empty());

    joints = robot.get_joints_in_chain("not_a_joint");
    EXPECT_TRUE(joints.empty());
}

TEST(RobotModel, GetJointStatesTest) {
    RobotModel robot(RobotModel::from_json(SIMPLEBOT_JRDF));

    robot.set_state("waist", M_PI / 2, 0.8, 1.2);
    robot.set_state("shoulder", M_PI / 6, 0.9, 1.1);
    robot.set_state("elbow", -M_PI / 4, 1.0, 1.0);
    robot.set_state("eye_lens_left", M_PI / 4, 1.1, 0.9);
    robot.set_state("eye_lens_right", -M_PI / 3, 1.2, 0.8);

    std::map<std::string, rix::msg::sensor::JointState> correct;
    auto joint_names = robot.get_joint_names();
    for (auto j : joint_names) {
        correct[j] = robot.get_joint(j)->get_state();
    }

    auto js = robot.get_joint_states();

    /*
    elbow
      Position: -0.785398
      Velocity: 1
      Effort: 1

    eye_lens_left
      Position: 0.785398
      Velocity: 1.1
      Effort: 0.9

    eye_lens_right
      Position: -1.0472
      Velocity: 1.2
      Effort: 0.8

    eye_socket_left
      Position: 0
      Velocity: 0
      Effort: 0

    eye_socket_right
      Position: 0
      Velocity: 0
      Effort: 0

    shoulder
      Position: 0.523599
      Velocity: 0.9
      Effort: 1.1

    waist
      Position: 1.5708
      Velocity: 0.8
      Effort: 1.2

    wrist
      Position: 0
      Velocity: 0
      Effort: 0
    */

    for (auto j : js.joint_states) {
        print_joint_state(j);
    }

    ASSERT_EQ(js.joint_states.size(), 8);
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
    RobotModel robot(RobotModel::from_json(SIMPLEBOT_JRDF));

    robot.set_state("waist", M_PI / 2, 0.0, 0.0);
    robot.set_state("shoulder", M_PI / 6, 0.0, 0.0);
    robot.set_state("elbow", -M_PI / 4, 0.0, 0.0);
    robot.set_state("eye_lens_left", M_PI / 4, 0.0, 0.0);
    robot.set_state("eye_lens_right", -M_PI / 3, 0.0, 0.0);
    auto tf = robot.get_transforms();

    /*
    world -> base
      Translation:
        x: 0
        y: 0
        z: 0
      Rotation:
        w: 1
        x: 0
        y: 0
        z: 0

    base -> torso
      Translation:
        x: 0
        y: 0
        z: 0
      Rotation:
        w: 0.707107
        x: 0
        y: 0
        z: 0.707107

    torso -> upper_arm
      Translation:
        x: 0.0375
        y: 0
        z: 0.3
      Rotation:
        w: 0.965926
        x: 0
        y: 0.258819
        z: 0

    torso -> eyeball_left
      Translation:
        x: 0.0375
        y: -0.0375
        z: 0.45
      Rotation:
        w: 1
        x: 0
        y: 0
        z: 0

    torso -> eyeball_right
      Translation:
        x: -0.0375
        y: -0.0375
        z: 0.45
      Rotation:
        w: 1
        x: 0
        y: 0
        z: 0

    eyeball_right -> pupil_right
      Translation:
        x: 0
        y: 0
        z: 0
      Rotation:
        w: 0.866025
        x: -0.5
        y: 0
        z: 0

    eyeball_left -> pupil_left
      Translation:
        x: 0
        y: 0
        z: 0
      Rotation:
        w: 0.92388
        x: 0
        y: 0
        z: 0.382683

    upper_arm -> forearm
      Translation:
        x: 0.2
        y: 0
        z: 0
      Rotation:
        w: 0.92388
        x: 0
        y: -0.382683
        z: 0

    forearm -> tool
      Translation:
        x: 0.1
        y: 0
        z: 0
      Rotation:
        w: 1
        x: 0
        y: 0
        z: 0
    */

    ASSERT_EQ(tf.transforms.size(), 9);

    std::map<std::string, size_t> frames;
    frames.insert({"world", -1});

    for (size_t i = 0; i < tf.transforms.size(); ++i) {
        const auto &t = tf.transforms[i];
        print_transform(tf.transforms[i]);
        EXPECT_TRUE(frames.find(t.header.frame_id) != frames.end())
            << "Transforms must be ordered such that, for any given transform, its parent frame appears earlier in the "
               "list. Unable to find \"" << t.header.frame_id << "\"";
        frames.insert({t.child_frame_id, i});
    }

    ASSERT_EQ(tf.transforms[0].header.frame_id, "world") << "First transform parent frame must be \"world\".";
    ASSERT_EQ(tf.transforms[0].child_frame_id, "base") << "First transform child frame must be the base link.";
    EXPECT_NEAR(tf.transforms[0].transform.translation.x, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[0].transform.translation.y, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[0].transform.translation.z, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[0].transform.rotation.w, 1, 1e-4);
    EXPECT_NEAR(tf.transforms[0].transform.rotation.x, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[0].transform.rotation.y, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[0].transform.rotation.z, 0.0, 1e-4);

    size_t i = frames["torso"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "base");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "torso");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, 0.707107, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0.707107, 1e-4);

    i = frames["pupil_left"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "eyeball_left");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "pupil_left");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, 0.92388, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0.382683, 1e-4);

    i = frames["pupil_right"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "eyeball_right");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "pupil_right");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, 0.866025, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, -0.5, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0.0, 1e-4);

    i = frames["tool"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "forearm");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "tool");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, 0.1, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, 1, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0.0, 1e-4);

    i = frames["upper_arm"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "torso");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "upper_arm");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, 0.0375, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0.3, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, 0.965926, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, 0.258819, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0.0, 1e-4);

    i = frames["eyeball_left"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "torso");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "eyeball_left");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, 0.0375, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, -0.0375, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0.45, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, 1, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0.0, 1e-4);

    i = frames["eyeball_right"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "torso");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "eyeball_right");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, -0.0375, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, -0.0375, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0.45, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, 1, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0.0, 1e-4);

    i = frames["forearm"];
    EXPECT_EQ(tf.transforms[i].header.frame_id, "upper_arm");
    EXPECT_EQ(tf.transforms[i].child_frame_id, "forearm");
    EXPECT_NEAR(tf.transforms[i].transform.translation.x, 0.2, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.y, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.translation.z, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.w, 0.92388, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.x, 0.0, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.y, -0.382683, 1e-4);
    EXPECT_NEAR(tf.transforms[i].transform.rotation.z, 0.0, 1e-4);
}