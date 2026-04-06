#include "joint_state_publisher/joint_state_publisher.hpp"

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

TEST(JointStatePublisher, Test) {
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

            // Use high publish rate to ensure that spin will always publish
            auto jsp = std::make_shared<JointStatePublisher>(
                rixhub_endpoint, robot, 1'000'000.0, std::vector<std::string>({"/joint_a", "/joint_b"}),
                rix::ipc::Endpoint("127.0.0.1", 2),
                std::vector<rix::ipc::Endpoint>(
                    {rix::ipc::Endpoint("127.0.0.1", 3), rix::ipc::Endpoint("127.0.0.1", 4)}),
                server_factory, client_factory);

            EXPECT_TRUE(jsp->ok());

            rix::ipc::Endpoint pub_a_endpoint("127.0.0.1", 0);
            auto pub_a = node->create_publisher<rix::msg::sensor::JS>("/joint_a", pub_a_endpoint);
            EXPECT_TRUE(pub_a->ok());

            rix::util::sleep_for(rix::util::Duration(0.25));  // Wait for mediator to connect for sub notify
            jsp->spin_once();                                 // Subscribers call connect
            node->spin_once();                                // Publishers call accept

            rix::ipc::Endpoint pub_b_endpoint("127.0.0.1", 1);
            auto pub_b = node->create_publisher<rix::msg::sensor::JS>("/joint_b", pub_b_endpoint);
            EXPECT_TRUE(pub_b->ok());

            rix::util::sleep_for(rix::util::Duration(0.25));  // Wait for mediator to connect for sub notify
            jsp->spin_once();                                 // Subscribers call connect
            node->spin_once();                                // Publishers call accept

            std::mutex received_js_mtx;
            std::vector<rix::msg::sensor::JS> received_js;
            rix::ipc::Endpoint sub_endpoint("127.0.0.1", 5);
            auto sub = node->create_subscriber<rix::msg::sensor::JS>(
                "/joint_states",
                [&](const rix::msg::sensor::JS &msg) {
                    std::lock_guard<std::mutex> guard(received_js_mtx);
                    received_js.push_back(msg);
                },
                sub_endpoint);
            EXPECT_TRUE(sub->ok());

            rix::util::sleep_for(rix::util::Duration(0.25));  // Wait for mediator to connect for sub notify
            node->spin_once();                                // Subscriber calls connect
            jsp->spin_once();                                 // Publisher calls accept, invokes timer
            node->spin_once();                                // Receives message

            EXPECT_EQ(pub_a->get_subscriber_count(), 1);
            EXPECT_EQ(pub_b->get_subscriber_count(), 1);
            EXPECT_EQ(sub->get_publisher_count(), 1);

            received_js_mtx.lock();
            EXPECT_EQ(received_js.size(), 1);
            for (auto joint_state : received_js[0].joint_states) {
                EXPECT_EQ(joint_state.position, 0.0);
                EXPECT_EQ(joint_state.velocity, 0.0);
                EXPECT_EQ(joint_state.effort, 0.0);
            }
            received_js_mtx.unlock();

            // Now we should have a system that looks like this:
            /*

            Test Node                       JSP Node

            Publisher A -> /joint_a      -> JSP Subscriber 1
            Publisher B -> /joint_b      -> JSP Subscriber 2
            Subscriber  <- /joint_states <- JSP Publisher

            */

            rix::msg::sensor::JS js;
            rix::msg::sensor::JointState joint_state;
            joint_state.name = "waist";
            joint_state.position = 0.5;
            joint_state.velocity = 0.25;
            joint_state.effort = 0.125;
            js.joint_states.push_back(joint_state);
            pub_a->publish(js);

            js.joint_states[0].name = "shoulder";
            js.joint_states[0].position = 1.0;
            pub_b->publish(js);

            jsp->spin_once();   // Receive the messages and update robot model, invoke timer

            auto robot_js = robot->get_joint_states();
            for (auto joint_state : robot_js.joint_states) {
                if (joint_state.name == "shoulder") {
                    EXPECT_EQ(joint_state.position, 1.0);
                    EXPECT_EQ(joint_state.velocity, 0.25);
                    EXPECT_EQ(joint_state.effort, 0.125);
                } else if (joint_state.name == "waist") {
                    EXPECT_EQ(joint_state.position, 0.5);
                    EXPECT_EQ(joint_state.velocity, 0.25);
                    EXPECT_EQ(joint_state.effort, 0.125);
                } else {
                    EXPECT_EQ(joint_state.position, 0.0);
                    EXPECT_EQ(joint_state.velocity, 0.0);
                    EXPECT_EQ(joint_state.effort, 0.0);
                }
            }

            node->spin_once();  // Receive message

            received_js_mtx.lock();
            EXPECT_EQ(received_js.size(), 2);
            for (auto joint_state : received_js[1].joint_states) {
                if (joint_state.name == "shoulder") {
                    EXPECT_EQ(joint_state.position, 1.0);
                    EXPECT_EQ(joint_state.velocity, 0.25);
                    EXPECT_EQ(joint_state.effort, 0.125);
                } else if (joint_state.name == "waist") {
                    EXPECT_EQ(joint_state.position, 0.5);
                    EXPECT_EQ(joint_state.velocity, 0.25);
                    EXPECT_EQ(joint_state.effort, 0.125);
                } else {
                    EXPECT_EQ(joint_state.position, 0.0);
                    EXPECT_EQ(joint_state.velocity, 0.0);
                    EXPECT_EQ(joint_state.effort, 0.0);
                }
            }
            received_js_mtx.unlock();
        }

        mediator->shutdown();
        rixhub_thread.join();
    }

    NiceMock<MockClient>::server_map = nullptr;
    NiceMock<MockServer>::server_map = nullptr;
    NiceMock<MockClient>::server_mutex = nullptr;
    NiceMock<MockServer>::server_mutex = nullptr;
}