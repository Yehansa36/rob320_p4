#pragma once

#include <memory>
#include <mutex>
#include <set>

#include "rix/core/common.hpp"
#include "rix/core/interfaces/spinner.hpp"
#include "rix/ipc/interfaces/client.hpp"
#include "rix/ipc/interfaces/server.hpp"
#include "rix/msg/mediator/Operation.hpp"
#include "rix/msg/mediator/PubInfo.hpp"
#include "rix/msg/mediator/Status.hpp"
#include "rix/msg/mediator/SubInfo.hpp"
#include "rix/msg/standard/UInt32.hpp"
#include "rix/util/log.hpp"

namespace rix {
namespace core {

class Node;  // Forward declaration

class Publisher : public interfaces::Spinner {
    /**
     * @brief We declare the Node as a friend class so that its factory method
     * create_publisher has access to the private constructor.
     */
    friend class Node;

   public:
    Publisher(const Publisher &) = delete;
    Publisher &operator=(const Publisher &) = delete;
    ~Publisher();

    /**
     * @brief Returns true if the Publisher has not been shut down.
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
     * @brief Publish the message on the topic.
     *
     * @details First, this function will check that the input message hash
     * matches the message hash of the topic that this publisher is publishing
     * on. Then, if the hashes match, it will serialize the specified message
     * with the message size prefixed and send the data to each connection. If
     * a connection is not writable, erase it from the set.
     *
     * @param msg The message to be published
     */
    void publish(const rix::msg::Message &msg);

    /**
     * @brief Returns the number of subscribers that this publisher is 
     * currently connected to.
     *
     */
    size_t get_subscriber_count() const;

   private:
    rix::msg::mediator::PubInfo info_;
    std::shared_ptr<rix::ipc::interfaces::Server> server_;
    std::set<std::weak_ptr<rix::ipc::interfaces::Connection>, std::owner_less<>> connections_;
    mutable std::mutex connections_mutex_;
    ClientFactory factory_;
    rix::ipc::Endpoint rixhub_endpoint_;
    std::atomic<bool> shutdown_flag_;

    /**
     * @brief Private constructor to be used by Node::create_publisher. This
     * will register the publisher with the Mediator.
     *
     * @param info The info of the Node
     * @param server The server that will accept connections from subscribers.
     * @param factory The client factory used to create connections to the Mediator
     * @param rixhub_endpoint The endpoint of the rixhub instance (the Mediator's server)
     */
    Publisher(const rix::msg::mediator::PubInfo &info, std::shared_ptr<rix::ipc::interfaces::Server> server,
              ClientFactory factory, rix::ipc::Endpoint rixhub_endpoint);

    /**
     * @brief We do not want the user to call spin or spin_once for Publisher.
     * Only the Node should invoke these functions. We will declare them private
     * here for this reason.
     * 
     */
    using interfaces::Spinner::spin;

    /**
     * @brief This will invoke a single iteration of the publisher loop.
     * 
     * @details Here is a breakdown of the publisher loop:
     * 1. Check to see if a new connection has been made to the server. If not,
     *    return.
     * 2. Accept a new connection.
     * 3. Insert the connection into the connections_ set.
     * 
     */
    virtual void spin_once() override;
};

}  // namespace core
}  // namespace rix