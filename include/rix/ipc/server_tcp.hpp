#pragma once

#include <atomic>
#include <unordered_set>

#include "rix/ipc/connection_tcp.hpp"
#include "rix/ipc/interfaces/server.hpp"
#include "rix/ipc/socket.hpp"

namespace rix {
namespace ipc {

class ServerTCP : public interfaces::Server {
   public:
    /**
     * @brief Construct a new TCP Server. 
     * 
     * @details This will bind the underlying socket to the specified endpoint
     * and listen for incoming connections. This should set the REUSEADDR socket
     * option before binding.
     * 
     * @param endpoint The endpoint that the server will be bound to.
     * @param backlog The maximum length for the queue of pending connections (maintained by the kernel)
     */
    ServerTCP(const Endpoint &endpoint, size_t backlog = INT16_MAX);

    ServerTCP();
    ServerTCP(const ServerTCP &other);
    ServerTCP &operator=(const ServerTCP &other);
    virtual ~ServerTCP();

    /**
     * @brief Extracts the first connection request on the queue of pending 
     * connections and creates a new Connection that will communicate directly
     * with the Client. 
     * 
     * @details If no pending connections are present on the queue and
     * the socket is not marked as non-blocking, this call will block until a 
     * connection is present. If the socket is marked as non-blocking and no 
     * pending connections are present on the queue, the call returns false.
     * 
     * @param connection A weak pointer to a Connection interface that will be
     * assigned to a ConnectionTCP object if the accept calls succeeds.
     * @return true if a connection was successfully accepted.
     */
    virtual bool accept(std::weak_ptr<interfaces::Connection> &connection) override;

    /**
     * @brief Closes the specified connection and destroys the Connection 
     * object.
     * 
     * @param connection The ConnectionTCP object to be closed and destroyed.
     */
    virtual void close(const std::weak_ptr<interfaces::Connection> &connection) override;

    /**
     * @brief Returns true if the underlying socket is ok and bound and 
     * listening.
     * 
     */
    virtual bool ok() const override;

    /**
     * @brief Returns the endpoint that the underlying socket is bound to.
     * 
     */
    virtual Endpoint local_endpoint() const override;

    /**
     * @brief Waits for a connection to be placed on the queue. If a connection
     * is already pending, this call will return immediately. Otherwise, it will
     * block until a connection is placed on the queue or until the duration 
     * elapses.
     * 
     * @param duration The maximum duration to wait for.
     * @return true if a connection is pending.
     */
    virtual bool wait_for_accept(rix::util::Duration duration) const override;

    /**
     * @brief Set the server into non-blocking mode.
     * 
     */
    virtual void set_nonblocking(bool status) override;
    
    /**
     * @brief Returns true if the server is in non-blocking mode
     * 
     */
    virtual bool is_nonblocking() const override;

   private:
    Socket socket;
    std::unordered_set<std::shared_ptr<interfaces::Connection>> connections;
};

}  // namespace ipc
}  // namespace rix