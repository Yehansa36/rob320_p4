#include "robot_state_publisher/robot_state_publisher.hpp"

#include <gmock/gmock.h>

#include <thread>

#include "mocks/mock_client.hpp"
#include "mocks/mock_notification.hpp"
#include "mocks/mock_server.hpp"
#include "print_helper.hpp"
#include "rix/core/mediator.hpp"
#include "rix/core/node.hpp"
#include "robots.hpp"

using ::testing::NiceMock;

std::shared_ptr<rix::ipc::interfaces::Server> server_factory(const rix::ipc::Endpoint &endpoint) {
    return std::make_shared<NiceMock<MockServer>>(endpoint);
}

std::shared_ptr<rix::ipc::interfaces::Client> client_factory() { return std::make_shared<NiceMock<MockClient>>(); }

TEST(RobotStatePublisher, Test) {
    auto server_map = std::make_shared<std::map<rix::ipc::Endpoint, NiceMock<MockServer> *>>();
    auto server_mutex = std::make_shared<std::mutex>();
    NiceMock<MockClient>::address = "127.0.0.1";
    NiceMock<MockClient>::server_map = server_map;
    NiceMock<MockServer>::server_map = server_map;
    NiceMock<MockClient>::server_mutex = server_mutex;
    NiceMock<MockServer>::server_mutex = server_mutex;

    {
        rix::ipc::Endpoint rixhub_endpoint("127.0.0.1", rix::core::RIXHUB_PORT);
        auto mediator = std::make_shared<rix::core::Mediator>(rixhub_endpoint, server_factory, client_factory);
        ASSERT_TRUE(mediator->ok());
        std::thread rixhub_thread([&]() { mediator->spin(); });

        {
            auto node = std::make_shared<rix::core::Node>("test", rixhub_endpoint, server_factory, client_factory);
            EXPECT_TRUE(node->ok());

            auto robot = std::make_shared<rix::robot::RobotModel>(rix::robot::RobotModel::from_json(SIMPLEBOT_JRDF));
            auto robot_js = robot->get_joint_states();
            auto robot_tf = robot->get_transforms();

            // Use high publish rate to ensure that spin will always publish
            auto rsp = std::make_shared<RobotStatePublisher>(rixhub_endpoint, robot, "/joint_states", 10.0, true,
                                                             rix::ipc::Endpoint("127.0.0.1", 0), rix::ipc::Endpoint("127.0.0.1", 1), server_factory,
                                                             client_factory);
            EXPECT_TRUE(rsp->ok());

            rix::ipc::Endpoint pub_endpoint("127.0.0.1", 2);
            auto pub = node->create_publisher<rix::msg::sensor::JS>("/joint_states", pub_endpoint);
            EXPECT_TRUE(pub->ok());

            rix::util::sleep_for(rix::util::Duration(0.25));  // Wait for mediator to connect for sub notify
            rsp->spin_once();                                 // Subscribers call connect
            node->spin_once();                                // Publishers call accept
            EXPECT_EQ(pub->get_subscriber_count(), 1);

            std::mutex received_tf_mtx;
            std::vector<rix::msg::geometry::TF> received_tf;
            rix::ipc::Endpoint sub_endpoint("127.0.0.1", 3);
            auto sub = node->create_subscriber<rix::msg::geometry::TF>(
                "/tf",
                [&](const rix::msg::geometry::TF &msg) {
                    std::lock_guard<std::mutex> guard(received_tf_mtx);
                    received_tf.push_back(msg);
                },
                sub_endpoint);
            EXPECT_TRUE(sub->ok());

            rix::util::sleep_for(rix::util::Duration(0.25));  // Wait for mediator to connect for sub notify
            node->spin_once();                                // Subscriber calls connect
            rsp->spin_once();                                 // Publisher calls accept
            EXPECT_EQ(sub->get_publisher_count(), 1);

            received_tf_mtx.lock();
            EXPECT_EQ(received_tf.size(), 0);
            received_tf_mtx.unlock();

            /*
            Now we should have a system that looks like this:

            Test Node                      RSP Node

            Publisher  -> /joint_states -> Subscriber
            Subscriber <- /tf           <- Publisher
            */

            pub->publish(robot_js);  // Publish 0 configuration

            rsp->spin_once();   // Receive the message, update robot model, and publish TF
            node->spin_once();  // Receive the message

            received_tf_mtx.lock();
            EXPECT_EQ(received_tf.size(), 1);

            for (size_t i = 0; i < received_tf[0].transforms.size(); ++i) {
                auto transform_a = received_tf[0].transforms[i];
                auto transform_b = robot_tf.transforms[i];
                EXPECT_EQ(transform_a.header.frame_id, transform_b.header.frame_id);
                EXPECT_EQ(transform_a.child_frame_id, transform_b.child_frame_id);
                EXPECT_EQ(transform_a.transform.translation.x, transform_b.transform.translation.x);
                EXPECT_EQ(transform_a.transform.translation.y, transform_b.transform.translation.y);
                EXPECT_EQ(transform_a.transform.translation.z, transform_b.transform.translation.z);
                EXPECT_EQ(transform_a.transform.rotation.w, transform_b.transform.rotation.w);
                EXPECT_EQ(transform_a.transform.rotation.x, transform_b.transform.rotation.x);
                EXPECT_EQ(transform_a.transform.rotation.y, transform_b.transform.rotation.y);
                EXPECT_EQ(transform_a.transform.rotation.z, transform_b.transform.rotation.z);
            }
            received_tf_mtx.unlock();
        }

        mediator->shutdown();
        rixhub_thread.join();
    }

    NiceMock<MockClient>::server_map = nullptr;
    NiceMock<MockServer>::server_map = nullptr;
    NiceMock<MockClient>::server_mutex = nullptr;
    NiceMock<MockServer>::server_mutex = nullptr;
}