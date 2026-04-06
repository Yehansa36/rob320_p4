#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include "rix/ipc/endpoint.hpp"
#include "rix/ipc/interfaces/client.hpp"
#include "rix/ipc/interfaces/server.hpp"
#include "rix/msg/mediator/Operation.hpp"
#include "rix/msg/mediator/Status.hpp"
#include "rix/util/log.hpp"

namespace rix {
namespace core {

const uint16_t RIXHUB_PORT = 48104;

enum OPCODE {
    NODE_REGISTER = 80,
    SUB_REGISTER,
    PUB_REGISTER,

    SUB_NOTIFY = 90,

    NODE_DEREGISTER = 100,
    SUB_DEREGISTER,
    PUB_DEREGISTER,
};

/**
 * @brief Type definition for a ClientFactory. This is a function that returns
 * a shared pointer to a new rix::ipc::interfaces::Client object.
 *
 */
using ClientFactory = std::function<std::shared_ptr<rix::ipc::interfaces::Client>(void)>;

/**
 * @brief Type definition for a ServerFactory. This is a function that takes an
 * endpoint as input and returns a shared pointer to a new
 * rix::ipc::interfaces::Server object.
 *
 */
using ServerFactory = std::function<std::shared_ptr<rix::ipc::interfaces::Server>(const rix::ipc::Endpoint &)>;

/**
 * @brief Helper function to send a rix::msg::Message with the specified opcode 
 * to the specified endpoint using an unconnected client. This function does not
 * wait for a response from the server before returning.
 *
 * @param client An unconnected client pointer
 * @param msg The message to be sent
 * @param opcode The opcode corresponding to the message (see `OPCODE` enum)
 * @param endpoint The endpoint of the server to connect to
 * @return `true` if the client connects to the server and sends the message 
 * successfully
 */
static inline bool send_message_with_opcode_no_response(std::shared_ptr<rix::ipc::interfaces::Client> client,
                                          const rix::msg::Message &msg, OPCODE opcode,
                                          const rix::ipc::Endpoint &endpoint) {
    if (!client->connect(endpoint)) {
        rix::util::Log::warn << "Failed to connect to rixhub." << std::endl;
        return false;
    }

    if (!client->is_writable()) {
        rix::util::Log::warn << "Unable to write to rixhub." << std::endl;
        return false;
    }

    rix::msg::mediator::Operation op;
    op.len = msg.size();
    op.opcode = opcode;
    std::vector<uint8_t> buffer(op.size() + msg.size());
    size_t offset = 0;
    op.serialize(buffer.data(), offset);
    msg.serialize(buffer.data(), offset);

    ssize_t bytes = client->write(buffer.data(), buffer.size());
    if (bytes != buffer.size()) {
        rix::util::Log::warn << "Failed to write to rixhub." << std::endl;
        return false;
    }

    return true;
}

/**
 * @brief Helper function to send a rix::msg::Message with the specified opcode 
 * to the specified endpoint using an unconnected client. This function will
 * wait for a response from the server before returning.
 *
 * @param client An unconnected client pointer
 * @param msg The message to be sent
 * @param opcode The opcode corresponding to the message (see `OPCODE` enum)
 * @param endpoint The endpoint of the server to connect to
 * @return `true` if the client connects to the server, sends the message
 * successfully, and receives a status message with an error field of 0.
 */
static inline bool send_message_with_opcode(std::shared_ptr<rix::ipc::interfaces::Client> client, const rix::msg::Message &msg,
                              OPCODE opcode, const rix::ipc::Endpoint &endpoint) {
    if (!send_message_with_opcode_no_response(client, msg, opcode, endpoint)) return false;

    // Ensure that register was successful
    rix::msg::mediator::Status status;
    std::vector<uint8_t> buffer(status.size());
    ssize_t bytes = client->read(buffer.data(), buffer.size());
    size_t offset = 0;
    if (!status.deserialize(buffer.data(), bytes, offset)) {
        rix::util::Log::warn << "Failed to read from rixhub." << std::endl;
        return false;
    }
    if (status.error != 0) {
        rix::util::Log::warn << "Failed to register publisher: " << status.error << std::endl;
        return false;
    }

    return true;
}

}  // namespace core
}  // namespace rix