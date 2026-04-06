#pragma once

#include "rix/ipc/interfaces/connection.hpp"
#include "rix/ipc/socket.hpp"

namespace rix {
namespace ipc {

class ServerTCP;

class ConnectionTCP : public interfaces::Connection {
    friend class ServerTCP;

   public:
    ConnectionTCP();
    ConnectionTCP(const ConnectionTCP &other);
    ConnectionTCP &operator=(const ConnectionTCP &other);
    virtual ~ConnectionTCP();

    /**
     * @brief Read `size` bytes from the client and store them in `dst`.
     * 
     * @param dst The destination byte array
     * @param size The number of bytes to read from the client.
     * @return ssize_t The number of bytes actually read, or -1 on error.
     */
    virtual ssize_t read(uint8_t *buffer, size_t len) const override;

    /**
     * @brief Write `size` bytes from `src` to the client.
     * 
     * @param src The source byte array
     * @param size The number of bytes to write to the client.
     * @return ssize_t The number of bytes actually written, or -1 on error.
     */
    virtual ssize_t write(const uint8_t *buffer, size_t len) const override;

    /**
     * @brief Returns the endpoint of the remote client.
     * 
     */
    virtual Endpoint remote_endpoint() const override;

    /**
     * @brief Returns the endpoint of the local connection.
     * 
     */
    virtual Endpoint local_endpoint() const override;

    /**
     * @brief Returns true if the underlying socket is ok.
     * 
     */
    virtual bool ok() const override;

    /**
     * @brief Waits for the specified duration for the client to become writable.
     * 
     * @param duration The maximum duration to wait.
     * @return true if the client has become writable within the duration.
     */
    virtual bool wait_for_writable(const rix::util::Duration &duration) const override;

    /**
     * @brief Waits for the specified duration for the client to become readable.
     * 
     * @param duration The maximum duration to wait.
     * @return true if the client has become readable within the duration.
     */
    virtual bool wait_for_readable(const rix::util::Duration &duration) const override;

    /**
     * @brief Set the client into non-blocking mode.
     * 
     */
    virtual void set_nonblocking(bool status) override;

    /**
     * @brief Returns true if the client is in non-blocking mode
     * 
     */
    virtual bool is_nonblocking() const override;

   private:
    Socket socket;
    
    /**
     * @brief Private constructor used by Server to create a Connection object
     * during the accept call.
     * 
     * @param socket The socket used by the connection
     */
    ConnectionTCP(const Socket &socket);
};

}  // namespace ipc
}  // namespace rix