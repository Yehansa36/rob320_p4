#include <gtest/gtest.h>

#include "print_helper.hpp"
#include "rix/robot/eigen_util.hpp"
#include "rix/robot/robot_model.hpp"
#include "robots.hpp"
 
using namespace rix::robot;
using namespace rix::msg::geometry;

// Compare two doubles with tolerance
inline bool almost_equal_scalar(double a, double b, double tol = 1e-6) { return std::fabs(a - b) <= tol; }

// Compare Vector3
inline bool almost_equal(const rix::msg::geometry::Vector3 &a, const rix::msg::geometry::Vector3 &b,
                         double tol = 1e-6) {
    return almost_equal_scalar(a.x, b.x, tol) && almost_equal_scalar(a.y, b.y, tol) &&
           almost_equal_scalar(a.z, b.z, tol);
}

// Compare Quaternion (account for double-cover: q and -q are equivalent)
inline bool almost_equal(const rix::msg::geometry::Quaternion &a, const rix::msg::geometry::Quaternion &b,
                         double tol = 1e-6) {
    bool direct = almost_equal_scalar(a.w, b.w, tol) && almost_equal_scalar(a.x, b.x, tol) &&
                  almost_equal_scalar(a.y, b.y, tol) && almost_equal_scalar(a.z, b.z, tol);

    bool negated = almost_equal_scalar(a.w, -b.w, tol) && almost_equal_scalar(a.x, -b.x, tol) &&
                   almost_equal_scalar(a.y, -b.y, tol) && almost_equal_scalar(a.z, -b.z, tol);

    return direct || negated;
}

// Compare Transform
inline bool almost_equal(const rix::msg::geometry::Transform &a, const rix::msg::geometry::Transform &b,
                         double tol = 1e-6) {
    return almost_equal(a.translation, b.translation, tol) && almost_equal(a.rotation, b.rotation, tol);
}

Transform rotation_about_z(double angle) {
    Transform t = transform_identity();
    t.rotation = eigen_to_msg(Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d({0, 0, 1}))));
    return t;
}

Transform translation_along_x(double dist) {
    Transform t = transform_identity();
    t.translation.x = dist;
    t.translation.y = 0;
    t.translation.z = 0;
    return t;
}

// --- FIXED JOINT ---
TEST(JointTransformTest, FixedJointIdentity) {
    Joint j(vector3_zeros(), transform_identity(), Joint::FIXED);
    j.set_state(0.5, 0.0, 0.0);  // Position shouldn't matter

    Transform tf = j.transform();
    EXPECT_TRUE(almost_equal(tf, transform_identity()));
}

// --- REVOLUTE JOINT ---
TEST(JointTransformTest, RevoluteJointRotation) {
    Vector3 axis;
    axis.x = 0;
    axis.y = 0;
    axis.z = 1;
    Joint j(axis, transform_identity(), Joint::REVOLUTE);
    j.set_state(M_PI / 2, 0.0, 0.0);

    Transform tf = j.transform();
    Transform expected = rotation_about_z(M_PI / 2);

    EXPECT_TRUE(almost_equal(tf, expected, 1e-6));
}

// --- CONTINUOUS JOINT ---
TEST(JointTransformTest, ContinuousJointRotation) {
    Vector3 axis;
    axis.x = 0;
    axis.y = 0;
    axis.z = 1;
    Joint j(axis, transform_identity(), Joint::CONTINUOUS);
    j.set_state(M_PI / 4, 0.0, 0.0);

    Transform tf = j.transform();
    Transform expected = rotation_about_z(M_PI / 4);

    EXPECT_TRUE(almost_equal(tf, expected, 1e-6));
}

// --- CONTINUOUS JOINT WITH WRAP ---
TEST(JointTransformTest, ContinuousJointRotationWrap) {
    Vector3 axis;
    axis.x = 0;
    axis.y = 0;
    axis.z = 1;
    Joint j(axis, transform_identity(), Joint::CONTINUOUS);
    j.set_state(2 * M_PI, 0.0, 0.0);

    Transform tf = j.transform();
    Transform expected = transform_identity();  // full rotation should wrap

    EXPECT_TRUE(almost_equal(tf, expected, 1e-6));
}

// --- PRISMATIC JOINT ---
TEST(JointTransformTest, PrismaticJointTranslation) {
    Vector3 axis;
    axis.x = 1;
    axis.y = 0;
    axis.z = 0;
    Joint j(axis, transform_identity(), Joint::PRISMATIC);
    j.set_state(2.0, 0.0, 0.0);

    Transform tf = j.transform();
    Transform expected = translation_along_x(2.0);

    EXPECT_TRUE(almost_equal(tf, expected, 1e-6));
}

// --- ENSURE NO ORIGIN OFFSET ---
TEST(JointTransformTest, OriginOffsetIsNotApplied) {
    Vector3 axis;
    axis.x = 0;
    axis.y = 0;
    axis.z = 1;
    Transform origin = translation_along_x(1.0);
    Joint j(axis, origin, Joint::REVOLUTE);
    j.set_state(M_PI / 2, 0.0, 0.0);

    Transform tf = j.transform();
    Transform expected = rotation_about_z(M_PI / 2);

    EXPECT_TRUE(almost_equal(tf, expected, 1e-6));
}
