#pragma once

#include <memory>
#include <mutex>
#include <set>

#include "rix/core/common.hpp"
#include "rix/core/interfaces/spinner.hpp"
#include "rix/ipc/client_tcp.hpp"
#include "rix/ipc/server_tcp.hpp"
#include "rix/msg/mediator/NodeInfo.hpp"
#include "rix/msg/mediator/Operation.hpp"
#include "rix/msg/mediator/PubInfo.hpp"
#include "rix/msg/mediator/Status.hpp"
#include "rix/msg/mediator/SubInfo.hpp"
#include "rix/msg/mediator/SubNotify.hpp"

namespace rix {
namespace core {

class Mediator : public interfaces::Spinner {
   public:
    /**
     * @brief Constructor
     *
     * @param server The server that the Mediator will host on.
     * @param client_factory The ClientFactory used to create connections to
     * components (used to notify Subscribers of new Publishers).
     */
    Mediator(
        const rix::ipc::Endpoint &rixhub_endpoint,
        ServerFactory server_factory =
            [](const rix::ipc::Endpoint &endpoint) { return std::make_shared<rix::ipc::ServerTCP>(endpoint); },
        ClientFactory client_factory = []() { return std::make_shared<rix::ipc::ClientTCP>(); });

    /**
     * @brief Destroy the Mediator object
     *
     */
    ~Mediator();

    /**
     * @brief Returns true if the Mediator has not been shut down.
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
     * @brief This will invoke a single iteration of the rixhub loop.
     *
     * @details Here is a breakdown of the rixhub loop:
     * 1. Check to see if a new connection has been made to the server. If not,
     *    return.
     * 2. Accept a new connection.
     * 3. Read a rix::msg::mediator::Operation message. This contains fields for
     *    the opcode of the message and the length of the following message.
     * 4. Read the number of bytes specified by the Operation message.
     * 5. Depdending on the opcode, deserialize and handle the message. Below is
     *    a table of valid opcodes, their corresponding message types, and the
     *    operation that needs to be performed upon reception.
     *        NODE_REGISTER: rix::msg::mediator::NodeInfo
     *            1. Insert the NodeInfo into the nodes_ map.
     *            2. Respond with a rix::msg::mediator::Status message. If an
     *               error occured during registration, set the 'error' field
     *               to a nonzero value.
     *        SUB_REGISTER: rix::msg::mediator::SubInfo
     *            1. Validate the TopicInfo contained in the SubInfo.
     *            2. If valid, insert the SubInfo into the subscribers_ map.
     *            3. Respond with a rix::msg::mediator::Status message. If an
     *               error occured during registration, set the 'error' field
     *               to a nonzero value.
     *            4. Use the 'endpoint' field in the SubInfo to send a
     *               rix::msg::mediator::SubNotify message to the subscriber
     *               that contains the PubInfo for each publisher on the topic.
     *        PUB_REGISTER: rix::msg::mediator::PubInfo
     *            1. Validate the TopicInfo contained in the SubInfo.
     *            2. If valid, insert the PubInfo into the publishers_ map.
     *            3. Respond with a rix::msg::mediator::Status message. If an
     *               error occured during registration, set the 'error' field
     *               to a nonzero value.
     *            4. For each subscriber on the publisher's topic, use the
     *               'endpoint' field in the SubInfo to send a
     *               rix::msg::mediator::SubNotify message to the subscriber
     *               that contains the PubInfo of the new publisher.
     *        NODE_DEREGISTER: rix::msg::mediator::NodeInfo
     *           1. Erase the NodeInfo from the nodes_ map.
     *        SUB_DEREGISTER: rix::msg::mediator::SubInfo
     *           1. Erase the SubInfo from the subscribers_ map.
     *        PUB_DEREGISTER: rix::msg::mediator::PubInfo
     *           1. Erase the PubInfo from the publishers_ map.
     */
    virtual void spin_once() override;

   private:
    std::shared_ptr<rix::ipc::interfaces::Server> server_;        /**< The server for registry of new components */
    ClientFactory client_factory_;                                /**< The factory method to create new clients */
    std::map<uint64_t, rix::msg::mediator::NodeInfo> nodes_;      /**< Active nodes (lookup by node ID) */
    std::map<uint64_t, rix::msg::mediator::PubInfo> publishers_;  /**< Active publishers (lookup by publisher ID) */
    std::map<uint64_t, rix::msg::mediator::SubInfo> subscribers_; /**< Active subscribers (lookup by subscriber ID) */
    std::map<std::string, std::array<uint64_t, 2>> topic_hashes_; /**< Message hashes (lookup by topic name) */
    std::atomic<bool> shutdown_flag_;

    /**
     * @brief Helper function to send the PubInfo to each subscriber. Each
     * subscriber in the vector should have the same topic as the publisher.
     *
     * @details Use this function when a new publisher is registered. It will
     * send the publisher info to each subscriber in the list.
     *
     * @param subscribers A list of subscribers on the same topic
     * @param publisher A publisher on the subscribers' topic
     */
    void notify_subscribers(const std::vector<rix::msg::mediator::SubInfo> &subscribers,
                            const rix::msg::mediator::PubInfo &publisher);

    /**
     * @brief Helper function to send each PubInfo to the subscriber. Each
     * publisher in the vector should have the same topic as the subscriber.
     *
     * @details Use this function when a new subscriber is registered. It will
     * send the publishers' info to the subscriber.
     *
     * @param subscriber A subscriber on the publishers' topic
     * @param publishers A list of publishers on the same topic
     */
    void notify_subscribers(const rix::msg::mediator::SubInfo &subscriber,
                            const std::vector<rix::msg::mediator::PubInfo> &publishers);

    /**
     * @brief Helper function to validate an incoming TopicInfo (from either a
     * publisher or subscriber).
     *
     * @details If the topic does not exist, add it to the topic_hashes_ map.
     * If the topic already exists, check that the new topic's message hash
     * matches the record.
     *
     * @param info The new TopicInfo
     * @return `true` if the topic is new or if the topic message hash matches
     * the existing message hash. `false` if the topic message hash does not
     * match the existing message hash.
     */
    bool validate_topic_info(const rix::msg::mediator::TopicInfo &info);

    /**
     * @brief Helper function to send a Status message to a Connection.
     *
     * @details Use this function to notify a registering component if there has
     * been an error during the registry process. Errors include:
     *     1. Deserialization failure
     *     2. Topic message hash mismatch
     *
     * @param conn The connection to which to send the Status message
     * @param status The Status message to send
     */
    void send_status_message(std::shared_ptr<rix::ipc::interfaces::Connection> conn, rix::msg::mediator::Status status);
};

}  // namespace core
}  // namespace rix