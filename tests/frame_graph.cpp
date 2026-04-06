#include "rix/tf/frame_graph.hpp"

#include <gtest/gtest.h>

using namespace rix::tf; 

TEST(FrameGraph, Test) {
    FrameGraph graph("world", rix::util::Duration(1.0));
    EXPECT_EQ(graph.get_leaves().size(), 0);
    EXPECT_EQ(graph.get_root()->name, "world");
    
    FrameGraph::Iterator it = graph.get_root();
    ASSERT_NE(it, graph.end());
    EXPECT_EQ(it->name, "world");

    rix::msg::geometry::TransformStamped transform;
    transform.header.frame_id = "world";
    transform.child_frame_id = "base";
    EXPECT_TRUE(graph.update(transform));
    EXPECT_EQ(graph.exists("base"), true);

    transform.header.frame_id = "base";
    transform.child_frame_id = "upper_arm";
    EXPECT_TRUE(graph.update(transform));
    EXPECT_EQ(graph.exists("upper_arm"), true);
    
    transform.header.frame_id = "upper_arm";
    transform.child_frame_id = "forearm";
    EXPECT_TRUE(graph.update(transform));
    EXPECT_EQ(graph.exists("forearm"), true);

    it = graph.find("base");
    EXPECT_NE(it, graph.end());
    EXPECT_EQ(it->name, "base");
    EXPECT_EQ(it->buffer.duration(), rix::util::Duration(1.0));

    it = graph.find("upper_arm");
    EXPECT_NE(it, graph.end());
    EXPECT_EQ(it->name, "upper_arm");
    EXPECT_EQ(it->buffer.duration(), rix::util::Duration(1.0));

    it = graph.find("forearm");
    EXPECT_NE(it, graph.end());
    EXPECT_EQ(it->name, "forearm");
    EXPECT_EQ(it->buffer.duration(), rix::util::Duration(1.0));
}

TEST(FrameGraph, IteratorTest) {
    FrameGraph graph("world", rix::util::Duration(1.0));

    rix::msg::geometry::TransformStamped transform;
    transform.header.frame_id = "world";
    transform.child_frame_id = "base";
    graph.update(transform);

    transform.header.frame_id = "base";
    transform.child_frame_id = "upper_arm";
    graph.update(transform);
    
    transform.header.frame_id = "upper_arm";
    transform.child_frame_id = "forearm";
    graph.update(transform);

    auto it = graph.find("forearm");
    EXPECT_EQ(it->name, "forearm");
    --it;
    EXPECT_EQ(it->name, "upper_arm");
    --it;
    EXPECT_EQ(it->name, "base");
    --it;
    EXPECT_EQ(it->name, "world");
}

TEST(FrameGraph, UpdateTest) {
    FrameGraph graph("world", rix::util::Duration(1.0));

    rix::msg::geometry::TransformStamped transform;
    transform.header.stamp = rix::util::Time(1.0).to_msg();
    transform.header.frame_id = "world";
    transform.child_frame_id = "base";
    transform.transform.translation.x = 1.0;
    transform.transform.translation.y = 2.0;
    transform.transform.translation.z = 3.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    EXPECT_EQ(graph.update(transform), true);
    EXPECT_EQ(graph.find("base")->buffer.size(), 1);

    transform.header.stamp = rix::util::Time(2.0).to_msg();
    transform.header.frame_id = "base";
    transform.child_frame_id = "upper_arm";
    transform.transform.translation.x = 2.0;
    transform.transform.translation.y = 4.0;
    transform.transform.translation.z = 6.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 1.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 0.0;
    EXPECT_EQ(graph.update(transform), true);
    EXPECT_EQ(graph.find("base")->buffer.size(), 1);
    EXPECT_EQ(graph.find("upper_arm")->buffer.size(), 1);

    transform.header.frame_id = "world";
    transform.child_frame_id = "upper_arm";
    EXPECT_EQ(graph.update(transform), false);
    EXPECT_EQ(graph.find("base")->buffer.size(), 1);
    EXPECT_EQ(graph.find("upper_arm")->buffer.size(), 1);
}

TEST(FrameGraph, LeafTest) {
    FrameGraph graph("world", rix::util::Duration(1.0));

    rix::msg::geometry::TransformStamped transform;
    transform.header.frame_id = "world";
    transform.child_frame_id = "base";
    graph.update(transform);

    transform.header.frame_id = "base";
    transform.child_frame_id = "camera";
    graph.update(transform);

    transform.header.frame_id = "base";
    transform.child_frame_id = "lidar";
    graph.update(transform);

    std::vector<std::string> leaves = graph.get_leaves();
    EXPECT_EQ(leaves.size(), 2);
    EXPECT_TRUE(std::find(leaves.begin(), leaves.end(), "camera") != leaves.end());
    EXPECT_TRUE(std::find(leaves.begin(), leaves.end(), "lidar") != leaves.end());
}

TEST(FrameGraph, AncestorTest) {
    FrameGraph graph("world", rix::util::Duration(1.0));

    rix::msg::geometry::TransformStamped tf;
    tf.header.frame_id = "world";
    tf.child_frame_id = "base";
    graph.update(tf);

    tf.header.frame_id = "base";
    tf.child_frame_id = "arm";
    graph.update(tf);

    tf.header.frame_id = "arm";
    tf.child_frame_id = "hand_a";
    graph.update(tf);
    
    tf.header.frame_id = "hand_a";
    tf.child_frame_id = "camera";
    graph.update(tf);

    tf.header.frame_id = "arm";
    tf.child_frame_id = "hand_b";
    graph.update(tf);
    
    tf.header.frame_id = "hand_b";
    tf.child_frame_id = "lidar";
    graph.update(tf);

    auto ancestor = graph.find_nearest_ancestor("lidar", "camera");
    EXPECT_NE(ancestor, graph.end());
    EXPECT_EQ(ancestor->name, "arm");

    ancestor = graph.find_nearest_ancestor("arm", "hand_b");
    EXPECT_EQ(ancestor->name, "arm");

    ancestor = graph.find_nearest_ancestor("hand_b", "arm");
    EXPECT_EQ(ancestor->name, "arm");

    ancestor = graph.find_nearest_ancestor("base", "lidar");
    EXPECT_EQ(ancestor->name, "base");

    ancestor = graph.find_nearest_ancestor("world", "base");
    EXPECT_EQ(ancestor->name, "world");

    ancestor = graph.find_nearest_ancestor("base", "world");
    EXPECT_EQ(ancestor->name, "world");

    ancestor = graph.find_nearest_ancestor("base", "base");
    EXPECT_EQ(ancestor->name, "base");

    ancestor = graph.find_nearest_ancestor("camera", "camera");
    EXPECT_EQ(ancestor->name, "camera");

    ancestor = graph.find_nearest_ancestor("camera", "hand_b");
    EXPECT_EQ(ancestor->name, "arm");
}

TEST(FrameGraph, DuplicateTransformTest) {
    FrameGraph graph("world", rix::util::Duration(1.0));

    rix::msg::geometry::TransformStamped tf;
    tf.header.stamp = rix::util::Time(1.0).to_msg();
    tf.header.frame_id = "world";
    tf.child_frame_id = "base";
    tf.transform.translation.x = 1.0;

    EXPECT_TRUE(graph.update(tf));
    EXPECT_EQ(graph.find("base")->buffer.size(), 1);

    // Insert transform with same time stamp and frame ids
    tf.transform.translation.x = 2.0;
    EXPECT_TRUE(graph.update(tf));
    auto it = graph.find("base");
    EXPECT_EQ(it->buffer.size(), 1);  // No duplicate entry

    // Transform should be most recent
    rix::msg::geometry::Transform new_tf;
    EXPECT_EQ(it->buffer.get(rix::util::Time(1.0), new_tf), true);
    EXPECT_NEAR(new_tf.translation.x, 2.0, 1e-6);
}

TEST(FrameGraph, GetTransformTest) {
    FrameGraph graph("world", rix::util::Duration(10.0));

    rix::msg::geometry::TransformStamped tf;
    tf.header.stamp = rix::util::Time(5.0).to_msg();
    tf.header.frame_id = "world";
    tf.child_frame_id = "base";
    tf.transform = rix::robot::transform_identity();
    tf.transform.translation.x = 1.0;
    tf.transform.translation.y = 2.0;
    tf.transform.translation.z = 3.0;
    graph.update(tf);

    tf.header.frame_id = "base";
    tf.child_frame_id = "arm";
    tf.transform.translation.x = 1.0;
    tf.transform.translation.y = 1.0;
    tf.transform.translation.z = 0.0;
    graph.update(tf);

    rix::msg::geometry::TransformStamped result;
    bool success = graph.get_transform("arm", "world", rix::util::Time(5.0), result);
    EXPECT_TRUE(success);
    EXPECT_EQ(result.header.frame_id, "world");
    EXPECT_EQ(result.child_frame_id, "arm");
    EXPECT_NEAR(result.transform.translation.x, 2.0, 1e-6);  // 1 (base) + 1 (arm)
    EXPECT_NEAR(result.transform.translation.y, 3.0, 1e-6);  // 2 + 1
}

TEST(FrameGraph, IteratorComparisonTest) {
    FrameGraph graph("world", rix::util::Duration(1.0));
    rix::msg::geometry::TransformStamped tf;

    tf.header.frame_id = "world";
    tf.child_frame_id = "base";
    graph.update(tf);

    tf.header.frame_id = "base";
    tf.child_frame_id = "arm";
    graph.update(tf);

    tf.header.frame_id = "arm";
    tf.child_frame_id = "camera";
    graph.update(tf);

    auto a = graph.find("camera");
    auto b = graph.find("arm");
    auto c = graph.find("base");
    auto d = graph.find("camera");
    auto e = graph.find("world");
    EXPECT_TRUE(e == graph.get_root());

    // Iterator comparisons are only valid within the same joint chain 
    EXPECT_TRUE(a == d);
    EXPECT_FALSE(a == b);
    EXPECT_TRUE(a != b);
    EXPECT_FALSE(a != d);
    
    // greater than means further from root
    EXPECT_TRUE(a > b);
    EXPECT_TRUE(b > c);
    EXPECT_FALSE(b > a);
    EXPECT_FALSE(c > b);

    // less than means closer to root
    EXPECT_TRUE(b < a);
    EXPECT_TRUE(c < b);
    EXPECT_FALSE(a < b);
    EXPECT_FALSE(b < c);

    EXPECT_TRUE(a >= d);
    EXPECT_TRUE(a <= d);
}
