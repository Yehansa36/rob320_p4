#pragma once

#include <memory>
#include <mutex>
#include <set>

#include "rix/core/common.hpp"
#include "rix/core/publisher.hpp"
#include "rix/core/subscriber.hpp"
#include "rix/core/timer.hpp"
#include "rix/ipc/client_tcp.hpp"
#include "rix/ipc/server_tcp.hpp"
#include "rix/msg/mediator/NodeInfo.hpp"
#include "rix/util/log.hpp"

namespace rix {
namespace core {

class Node : public interfaces::Spinner {
   public:
    /**
     * @brief Construct a new Node.
     *
     * @param name The name of the node
     * @param rixhub_endpoint The endpoint of the `rixhub` instance
     */
    Node(const std::string &name, const rix::ipc::Endpoint &rixhub_endpoint,
         ServerFactory server_factory = Node::make_server_default,
         ClientFactory client_factory = Node::make_client_default);

    Node(const Node &) = delete;             // Delete copy constructor
    Node &operator=(const Node &) = delete;  // Delete copy assignment operator

    /**
     * @brief Destroy the Node object
     *
     */
    ~Node();

    /**
     * @brief Publisher factory method.
     *
     * @details This is a public, templated function that invokes the private implementation.
     * This is used to keep student code in source files so that solution implementation
     * details are kept in binary format in future projects.
     *
     * @tparam TMsg The message type of the topic
     * @param topic The topic to publish on
     * @param endpoint The endpoint that the publisher server will host on.
     * @return std::shared_ptr<Publisher>
     */
    template <typename TMsg>
    std::shared_ptr<Publisher> create_publisher(const std::string &topic,
                                                const rix::ipc::Endpoint &endpoint = rix::ipc::Endpoint("127.0.0.1",
                                                                                                        0));

    /**
     * @brief Subscriber factory method.
     *
     * @details This is a public, templated function that invokes the private implementation.
     * This is used to keep student code in source files so that solution implementation
     * details are kept in binary format in future projects.
     *
     * @tparam TMsg The message type of the topic
     * @param topic The topic to subscribe to
     * @param callback The callback function to be invoked upon receiving a message from a publisher
     * @param endpoint The endpoint that the subscriber server will host on (used for notification of new publishers).
     * @return std::shared_ptr<Subscriber>
     */
    template <typename TMsg>
    std::shared_ptr<Subscriber> create_subscriber(const std::string &topic, std::function<void(const TMsg &)> callback,
                                                  const rix::ipc::Endpoint &endpoint = rix::ipc::Endpoint("127.0.0.1",
                                                                                                          0));
    /**
     * @brief Factory method for Timer.
     *
     * @details A Timer is used to run a callback function at a fixed rate in the
     * same thread that calls Node::spin().
     *
     * @param d The duration/period for the timer
     * @param callback The callback function to be invoked every duration
     * @return std::shared_ptr<Timer>
     */
    std::shared_ptr<Timer> create_timer(const rix::util::Duration &d, Timer::Callback callback);

    /**
     * @brief Returns true if the Node has not been shut down.
     *
     */
    virtual bool ok() const override;

    /**
     * @brief Stops the spin loop. ok() will return false after this function
     * is called.
     *
     */
    virtual void shutdown() override;

    /**
     * @brief This will invoke a single iteration of the node loop.
     *
     * @details A node contains a set of Spinner interface pointers, which is
     * the base class for the Publisher and Subscriber class. A single iteration
     * of the Node::spin_once function will invoke the spin_once function for
     * each of the Spinner interfaces in the components_ set. If at any point
     * one of the Spinner interfaces is shutdown, then this will remove that
     * Spinner from the components_ set. The idea here is that the lifetimes of
     * all Publisher and Subscriber objects are tied to the lifetime of the Node
     * that they are created from, unless they are shutdown by the user.
     *
     */
    virtual void spin_once() override;

   private:
    rix::msg::mediator::NodeInfo info_; /**< Info of this Node */
    ServerFactory server_factory_;      /**< Server factory used to create servers for Publishers and Subscribers */
    ClientFactory client_factory_;      /**< Client factory used to connect to the Mediator */
    std::vector<std::shared_ptr<interfaces::Spinner>> components_; /**< Set of Spinner interface pointers. */
    rix::ipc::Endpoint rixhub_endpoint_; /**< Endpoint of the rixhub instance (the Mediator's server). */
    std::atomic<bool> shutdown_flag_;

    /**
     * @brief Helper function used to generate random 64-bit ID numbers.
     *
     * @details Use this function for generating Node, Publisher, and Subscriber
     * IDs
     *
     * @return uint64_t A random 64-bit ID number.
     */
    static uint64_t generate_id();

    /**
     * @brief Factory method for Publisher.
     *
     * @details Reference the Publisher private constructor for the fields
     * necessary to create a new Publisher object. You must create a shared
     * pointer to a Publisher object. You MUST initialize the shared pointer in
     * the following manner:
     *
     *     std::shared_ptr<MyClass> c = std::shared_ptr<MyClass>(new MyClass(arg1, arg2));
     *
     * You CANNOT use std::make_shared. This will cause a compiler error because
     * the std::shared_ptr class does not have access to the private constructor.
     *
     * Ensure that you fill in all necessary fields of the PubInfo object (id,
     * node_id, topic_info, and endpoint).
     *
     * @param topic The topic information for the topic to publish on.
     * @param endpoint The endpoint that the publisher server will host on.
     * @return std::shared_ptr<Publisher> A shared pointer to a Publisher object.
     */
    std::shared_ptr<Publisher> create_publisher(const rix::msg::mediator::TopicInfo &topic_info,
                                                const rix::ipc::Endpoint &endpoint);

    /**
     * @brief Factory method for Subscriber.
     *
     * @details Reference the Subscriber private constructor for the fields
     * necessary to create a new Subscriber object. You must create a shared
     * pointer to a Subscriber object. You MUST initialize the shared pointer in
     * the following manner:
     *
     *     std::shared_ptr<MyClass> c = std::shared_ptr<MyClass>(new MyClass(arg1, arg2));
     *
     * You CANNOT use std::make_shared. This will cause a compiler error because
     * the std::shared_ptr class does not have access to the private constructor.
     *
     * Ensure that you fill in all necessary fields of the SubInfo object (id,
     * node_id, topic_info, and endpoint).
     * it.
     *
     * @tparam TMsg The message type of the topic.
     * @param topic_info The topic information for the topic to subscribe to.
     * @param callback The callback function to be invoked upon receiving a message from a publisher
     * @param endpoint The endpoint that the subscriber server will host on (used for notification of new publishers).
     * @return std::shared_ptr<Subscriber> A shared pointer to a Subscriber object.
     */
    std::shared_ptr<Subscriber> create_subscriber(const rix::msg::mediator::TopicInfo &topic_info,
                                                  const rix::ipc::Endpoint &endpoint);

   protected:
    static inline ServerFactory make_server_default{
        [](const rix::ipc::Endpoint &endpoint) { return std::make_shared<rix::ipc::ServerTCP>(endpoint); }};
    static inline ClientFactory make_client_default{[]() { return std::make_shared<rix::ipc::ClientTCP>(); }};
};

template <typename TMsg>
std::shared_ptr<Publisher> Node::create_publisher(const std::string &topic, const rix::ipc::Endpoint &endpoint) {
    static_assert(std::is_base_of<rix::msg::Message, TMsg>::value, "TMsg must be a subclass of rix::msg::Message.");
    // Get topic information
    rix::msg::mediator::TopicInfo topic_info;
    topic_info.name = topic;
    topic_info.message_hash = TMsg().hash();

    // Invoke private implementation
    return create_publisher(topic_info, endpoint);
}

template <typename TMsg>
std::shared_ptr<Subscriber> Node::create_subscriber(const std::string &topic,
                                                    std::function<void(const TMsg &)> callback,
                                                    const rix::ipc::Endpoint &endpoint) {
    static_assert(std::is_base_of<rix::msg::Message, TMsg>::value, "TMsg must be a subclass of rix::msg::Message.");
    // Get topic information
    rix::msg::mediator::TopicInfo topic_info;
    topic_info.name = topic;
    topic_info.message_hash = TMsg().hash();

    // Invoke private implementation
    auto sub = create_subscriber(topic_info, endpoint);
    
    // Set callback (need template info to do this)
    if (sub) sub->set_callback(callback);

    return sub;
}

}  // namespace core
}  // namespace rix