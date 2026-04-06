#pragma once

#include <functional>
#include <memory>
#include <mutex>

#include "rix/core/common.hpp"
#include "rix/core/interfaces/spinner.hpp"
#include "rix/ipc/interfaces/client.hpp"
#include "rix/ipc/interfaces/server.hpp"
#include "rix/msg/mediator/Operation.hpp"
#include "rix/msg/mediator/PubInfo.hpp"
#include "rix/msg/mediator/Status.hpp"
#include "rix/msg/mediator/SubInfo.hpp"
#include "rix/msg/mediator/SubNotify.hpp"
#include "rix/msg/standard/UInt32.hpp"
#include "rix/util/log.hpp"

namespace rix {
namespace core {

class Node;  // Forward declaration

class Subscriber : public interfaces::Spinner {
    /**
     * @brief We declare the Node as a friend class so that its factory method
     * create_subscriber has access to the private constructor.
     */
    friend class Node;

   public:
    using SerializedCallback = std::function<void(const uint8_t *src, size_t len)>;

    Subscriber(const Subscriber &) = delete;
    Subscriber &operator=(const Subscriber &) = delete;
    ~Subscriber();

    /**
     * @brief Returns true if the Subscriber has not been shut down.
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
     * @brief Set the callback for the subscriber.
     *
     * @tparam TMsg The message type for the subscriber's topic.
     * @param callback The callback function to be invoked when a message is
     * received.
     */
    template <typename TMsg>
    void set_callback(std::function<void(const TMsg &)> callback);

    /**
     * @brief Returns the callback for this subscriber as a SerializedCallback
     * object.
     *
     * @details Subscriber is not a templated class, so we have to do some
     * type erasure techniques in order to store the callback. Reference the
     * set_callback implementation to see how this is done using lambda functions.
     *
     * @return SerializedCallback
     */
    SerializedCallback get_callback() const;

    /**
     * @brief Returns the number of publishers that this subscriber is 
     * currently connected to.
     *
     */
    size_t get_publisher_count() const;

   private:
    rix::msg::mediator::SubInfo info_;
    ClientFactory factory_;
    SerializedCallback callback_;
    mutable std::mutex callback_mutex_;
    std::shared_ptr<rix::ipc::interfaces::Server> server_;
    std::map<uint64_t, std::shared_ptr<rix::ipc::interfaces::Client>> clients_;
    rix::ipc::Endpoint rixhub_endpoint_;
    std::atomic<bool> shutdown_flag_;

    /**
     * @brief Private constructor to be used by Node::create_subscriber. This
     * will register the subscriber with the Mediator.
     *
     * @param info The info of the Subscriber
     * @param server The server that will accept connections from the Mediator (for notification of new publishers).
     * @param factory The client factory used to create connections to the Mediator
     * @param rixhub_endpoint The endpoint of the rixhub instance (the Mediator's server)
     */
    Subscriber(const rix::msg::mediator::SubInfo &info, std::shared_ptr<rix::ipc::interfaces::Server> server,
               ClientFactory factory, const rix::ipc::Endpoint &rixhub_endpoint);

    /**
     * @brief We do not want the user to call spin or spin_once for Subscriber.
     * Only the Node should invoke these functions. We will declare them private
     * here for this reason.
     *
     */
    using interfaces::Spinner::spin;

    /**
     * @brief This will invoke a single iteration of the subscriber loop.
     *
     * @details Here is a breakdown of the subscriber loop:
     * There are two parts to the subscriber loop, the first is to check if the
     * Mediator has notified the Susbcriber of any new publishers and connect to
     * them. The second is to check the existing connections for new messages.
     *
     * Part 1:
     * 1. Check to see if a new connection has been made to the server.
     * 2. Accept a new connection.
     * 3. Read a rix::msg::mediator::Operation message. This contains fields for
     *    the opcode of the message and the length of the following message.
     * 4. Read the number of bytes specified by the Operation message.
     * 5. Check that the opcode is SUB_NOTIFY. Other opcodes are invalid.
     * 6. Deserialize the data into a SubNotify message.
     * 7. For each publisher in the SubNotify message, create a client using the
     *    ClientFactory and connect to the publisher's server. Store this client
     *    in the clients_ set.
     * 
     * Important note: before calling connect, make sure that you set the client
     * to non-blocking mode. This is important because we cannot wait for the 
     * publisher to accept our connection in this loop. What if we want to 
     * subscribe to a publisher that belongs to our node? Would we be able to 
     * wait for that connection if the publisher's accept call is invoked during
     * its spin_once function? No, because the node cannot call the publisher's
     * spin_once if it is stuck waiting for the subscriber's spin_once to return.
     *
     * Part 2:
     * 1. Iterate through each client in the clients_ set.
     * 2. If the client is not ok(), erase it (this means that a hang up event
     *    has occured, meaning the publisher has closed the connection.)
     * 3. If the client is not connected or readable, skip it (this means that
     *    the publisher has not accepted our connection OR that it has not sent
     *    a message since the last iteration.)
     * 4. Read the prefixed size (4 bytes, use a rix::msg::standard::UInt32)
     * 5. Read the number of bytes specified by the size message.
     * 6. Invoke the callback on the byte array.
     *
     */
    virtual void spin_once() override;
};

template <typename TMsg>
void Subscriber::set_callback(std::function<void(const TMsg &)> callback) {
    static_assert(std::is_base_of<rix::msg::Message, TMsg>::value, "TMsg must be a subclass of rix::msg::Message.");

    /**
     * Ensure that the function's message type matches the message type of the
     * subscriber's topic.
     */
    if (TMsg().hash() != info_.topic_info.message_hash) {
        rix::util::Log::warn << "Message type mismatch in set_callback." << std::endl;
        return;
    }

    /**
     * Protect access to the callback (we do not want to change the callback if
     * clients are actively processing data)
     */
    std::lock_guard<std::mutex> guard(callback_mutex_);

    /**
     * We use a lambda function here to mask the templated callback type. We
     * know that we can express any class that derives from rix::msg::Message
     * in its serialized form as a byte array, which does not require knowledge
     * of the message type. We take advantage of this property so that the
     * subscriber is not a templated class.
     *
     * Why do we want to avoid Subscriber being a template?
     *
     * 1. If Subscriber had a template parameter, we could not store containers
     * of Subscriber objects that were subscribed to topics with varying message
     * types.
     * 2. It forces the implementation of the entire class to exist in a header
     * file, so we cannot hide solution code from you in future projects ;)
     *
     */
    callback_ = [callback](const uint8_t *msg, size_t len) {
        TMsg obj;
        size_t offset = 0;
        if (!obj.deserialize(msg, len, offset)) {
            rix::util::Log::warn << "Failed to deserialize message from publisher." << std::endl;
            return;
        }
        callback(obj);
    };
}

}  // namespace core
}  // namespace rix