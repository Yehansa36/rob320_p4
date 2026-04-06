#include "rix/tf/transform_buffer.hpp"

#include <gtest/gtest.h>

using namespace rix::tf;

TEST(TransformBuffer, Test) {
    TransformBuffer buffer(rix::util::Duration(1.0));
    EXPECT_EQ(buffer.duration().to_nanoseconds(), 1e9);
    EXPECT_EQ(buffer.size(), 0);
    EXPECT_EQ(buffer.empty(), true);

    rix::msg::geometry::Transform transform;
    EXPECT_EQ(buffer.get(rix::util::Time(0.0), transform), false);
    EXPECT_EQ(buffer.get(rix::util::Time(1.0), transform), false);
}

TEST(TransformBuffer, InsertTest) {
    TransformBuffer buffer(rix::util::Duration(2.0));
    rix::msg::geometry::Transform transform;
    transform.translation.x = 1.0;
    transform.translation.y = 2.0;
    transform.translation.z = 3.0;
    transform.rotation.x = 0.0;
    transform.rotation.y = 0.0;
    transform.rotation.z = 0.0;
    transform.rotation.w = 1.0;

    buffer.insert(rix::util::Time(1.0), transform);
    EXPECT_EQ(buffer.size(), 1);
    EXPECT_EQ(buffer.empty(), false);

    buffer.insert(rix::util::Time(2.0), transform);
    EXPECT_EQ(buffer.size(), 2);
}

TEST(TransformBuffer, InsertDurationTest) {
    TransformBuffer buffer(rix::util::Duration(2.0));
    rix::msg::geometry::Transform transform;
    transform.translation.x = 1.0;
    transform.translation.y = 0.0;
    transform.translation.z = 0.0;
    transform.rotation.x = 0.0;
    transform.rotation.y = 0.0;
    transform.rotation.z = 0.0;
    transform.rotation.w = 1.0;

    buffer.insert(rix::util::Time(1.0), transform);
    EXPECT_EQ(buffer.size(), 1);
    EXPECT_EQ(buffer.empty(), false);

    buffer.insert(rix::util::Time(2.0), transform);
    EXPECT_EQ(buffer.size(), 2);

    buffer.insert(rix::util::Time(4.0), transform);
    EXPECT_EQ(buffer.size(), 2);
}

TEST(TransformBuffer, GetTest) {
    TransformBuffer buffer(rix::util::Duration(2.0));
    rix::msg::geometry::Transform transform;
    transform.translation.x = 1.0;
    transform.translation.y = 0.0;
    transform.translation.z = 0.0;
    transform.rotation.x = 0.0;
    transform.rotation.y = 0.0;
    transform.rotation.z = 0.0;
    transform.rotation.w = 1.0;

    buffer.insert(rix::util::Time(1.0), transform);

    transform.translation.x = 2.0;
    buffer.insert(rix::util::Time(2.0), transform);

    EXPECT_EQ(buffer.size(), 2);
    EXPECT_EQ(buffer.empty(), false);

    rix::msg::geometry::Transform result_transform;

    // Test special case of time at 0 (get the last value)
    EXPECT_EQ(buffer.get(rix::util::Time(0.0), result_transform), true);
    EXPECT_EQ(result_transform.translation.x, 2.0);
    EXPECT_EQ(result_transform.translation.y, 0.0);
    EXPECT_EQ(result_transform.translation.z, 0.0);
    EXPECT_EQ(result_transform.rotation.x, 0.0);
    EXPECT_EQ(result_transform.rotation.y, 0.0);
    EXPECT_EQ(result_transform.rotation.z, 0.0);
    EXPECT_EQ(result_transform.rotation.w, 1.0);

    EXPECT_EQ(buffer.get(rix::util::Time(1.0), result_transform), true);
    EXPECT_EQ(result_transform.translation.x, 1.0);
    EXPECT_EQ(result_transform.translation.y, 0.0);
    EXPECT_EQ(result_transform.translation.z, 0.0);
    EXPECT_EQ(result_transform.rotation.x, 0.0);
    EXPECT_EQ(result_transform.rotation.y, 0.0);
    EXPECT_EQ(result_transform.rotation.z, 0.0);
    EXPECT_EQ(result_transform.rotation.w, 1.0);

    EXPECT_EQ(buffer.get(rix::util::Time(2.0), result_transform), true);
    EXPECT_EQ(result_transform.translation.x, 2.0);
    EXPECT_EQ(result_transform.translation.y, 0.0);
    EXPECT_EQ(result_transform.translation.z, 0.0);
    EXPECT_EQ(result_transform.rotation.x, 0.0);
    EXPECT_EQ(result_transform.rotation.y, 0.0);
    EXPECT_EQ(result_transform.rotation.z, 0.0);
    EXPECT_EQ(result_transform.rotation.w, 1.0);

    EXPECT_EQ(buffer.get(rix::util::Time(3.0), result_transform), true);
    EXPECT_EQ(result_transform.translation.x, 2.0);
    EXPECT_EQ(result_transform.translation.y, 0.0);
    EXPECT_EQ(result_transform.translation.z, 0.0);
    EXPECT_EQ(result_transform.rotation.x, 0.0);
    EXPECT_EQ(result_transform.rotation.y, 0.0);
    EXPECT_EQ(result_transform.rotation.z, 0.0);
    EXPECT_EQ(result_transform.rotation.w, 1.0);
}

TEST(TransformBuffer, InterpolationTest) {
    TransformBuffer buffer(rix::util::Duration(2.0));
    rix::msg::geometry::Transform transform1;
    transform1.translation.x = 1.0;
    transform1.translation.y = 2.0;
    transform1.translation.z = 3.0;
    transform1.rotation.x = 0.0;
    transform1.rotation.y = 0.0;
    transform1.rotation.z = 0.0;
    transform1.rotation.w = 1.0;

    rix::msg::geometry::Transform transform2;
    transform2.translation.x = 2.0;
    transform2.translation.y = 4.0;
    transform2.translation.z = 6.0;
    transform2.rotation.x = 0.0;
    transform2.rotation.y = 1.0;
    transform2.rotation.z = 0.0;
    transform2.rotation.w = 0.0;

    buffer.insert(rix::util::Time(1.0), transform1);
    buffer.insert(rix::util::Time(2.0), transform2);

    rix::msg::geometry::Transform result_transform;
    EXPECT_EQ(buffer.get(rix::util::Time(1.5), result_transform), true);

    EXPECT_EQ(result_transform.translation.x, 1.5);
    EXPECT_EQ(result_transform.translation.y, 3.0);
    EXPECT_EQ(result_transform.translation.z, 4.5);
    EXPECT_NEAR(result_transform.rotation.x, 0.0, 1e-6);
    EXPECT_NEAR(result_transform.rotation.y, 0.7071, 1e-4);
    EXPECT_NEAR(result_transform.rotation.z, 0.0, 1e-6);
    EXPECT_NEAR(result_transform.rotation.w, 0.7071, 1e-4);
}

TEST(TransformBuffer, CopyConstructorAndAssignmentTest) {
    TransformBuffer buffer1(rix::util::Duration(3.0));
    rix::msg::geometry::Transform transform;
    transform.translation.x = 1.0;
    buffer1.insert(rix::util::Time(1.0), transform);

    TransformBuffer buffer2(buffer1);  // Copy constructor
    EXPECT_EQ(buffer2.size(), 1);

    TransformBuffer buffer3;
    buffer3 = buffer1;  // Assignment operator
    EXPECT_EQ(buffer3.size(), 1);

    rix::msg::geometry::Transform result_transform;
    EXPECT_TRUE(buffer2.get(rix::util::Time(1.0), result_transform));
    EXPECT_EQ(result_transform.translation.x, 1.0);
}

TEST(TransformBuffer, ClearTest) {
    TransformBuffer buffer(rix::util::Duration(2.0));
    rix::msg::geometry::Transform transform;
    buffer.insert(rix::util::Time(1.0), transform);
    EXPECT_FALSE(buffer.empty());

    buffer.clear();
    EXPECT_EQ(buffer.size(), 0);
    EXPECT_TRUE(buffer.empty());
}

TEST(TransformBuffer, ChangeDurationTest) {
    TransformBuffer buffer(rix::util::Duration(5.0));
    rix::msg::geometry::Transform transform;
    buffer.insert(rix::util::Time(1.0), transform);
    buffer.insert(rix::util::Time(6.0), transform);
    EXPECT_EQ(buffer.size(), 2);

    buffer.set_duration(rix::util::Duration(3.0));
    buffer.insert(rix::util::Time(9.0), transform);  // Should prune old ones

    EXPECT_EQ(buffer.size(), 2);  // Only transforms at 6.0 and 9.0 remain
}

TEST(TransformBuffer, DuplicateTimestampInsertTest) {
    TransformBuffer buffer(rix::util::Duration(3.0));
    rix::msg::geometry::Transform transform1, transform2;
    transform1.translation.x = 1.0;
    transform2.translation.x = 2.0;

    buffer.insert(rix::util::Time(1.0), transform1);
    buffer.insert(rix::util::Time(1.0), transform2);  // Same timestamp

    EXPECT_EQ(buffer.size(), 1);  // Duplicate timestamps not allowed

    rix::msg::geometry::Transform result_transform;
    EXPECT_TRUE(buffer.get(rix::util::Time(1.0), result_transform));
    EXPECT_EQ(result_transform.translation.x, 2.0);  // Should match last inserted one
}

TEST(TransformBuffer, GetFromEmptyBufferTest) {
    TransformBuffer buffer(rix::util::Duration(2.0));
    rix::msg::geometry::Transform result_transform;
    EXPECT_FALSE(buffer.get(rix::util::Time(1.0), result_transform));
}

TEST(TransformBuffer, InterpolationOutsideDurationTest) {
    TransformBuffer buffer(rix::util::Duration(2.0));
    rix::msg::geometry::Transform transform;

    transform.translation.x = 1.0;
    buffer.insert(rix::util::Time(1.0), transform);
    transform.translation.x = 2.0;
    buffer.insert(rix::util::Time(2.0), transform);
    transform.translation.x = 3.0;
    buffer.insert(rix::util::Time(4.0), transform);  // This will prune (1.0)

    rix::msg::geometry::Transform result;
    EXPECT_TRUE(buffer.get(rix::util::Time(1.5), result));  // Returns earliest transform
    EXPECT_EQ(result.translation.x, 2.0);
}

TEST(TransformBuffer, DataAccessorTest) {
    TransformBuffer buffer(rix::util::Duration(2.0));
    rix::msg::geometry::Transform transform;
    transform.translation.x = 1.23;

    buffer.insert(rix::util::Time(1.0), transform);
    const auto& data = buffer.data();
    ASSERT_EQ(data.size(), 1);
    EXPECT_EQ(data.front().first.to_seconds(), 1.0);
    EXPECT_NEAR(data.front().second.translation.x, 1.23, 1e-4);
}
