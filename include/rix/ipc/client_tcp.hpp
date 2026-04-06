#pragma once

#include "rix/ipc/interfaces/client.hpp"
#include "rix/ipc/socket.hpp"

namespace rix {
namespace ipc {

class ClientTCP : public interfaces::Client {
   public:
    ClientTCP();
    ClientTCP(const ClientTCP &other);
    ClientTCP &operator=(const ClientTCP &other);
    virtual ~ClientTCP();

    /**
     * @brief Attempt to connect to a server that is bound to the specified 
     * endpoint. 
     * 
     * @details If the client is in non-blocking mode, then this function will
     * always return false. In non-blocking mode, you only need to call this 
     * function once. It should be followed by a call to wait_for_connect to
     * determine if the connection was successful. If non-blocking mode is 
     * disabled, this call will block until the server accepts the connection, 
     * or until the connections fails.
     * 
     * @param endpoint The endpoint of the server to connect to.
     * @return true if the connection was successful. 
     */
    virtual bool connect(const Endpoint &endpoint) override;

    /**
     * @brief Read `size` bytes from the connection and store them in `dst`.
     * 
     * @details A connection must be established before this function can be 
     * successfully called.
     * 
     * @param dst The destination byte array
     * @param size The number of bytes to read from the connection.
     * @return ssize_t The number of bytes actually read, or -1 on error.
     */
    virtual ssize_t read(uint8_t *buffer, size_t len) const override;

    /**
     * @brief Write `size` bytes from `src` to the connection.
     * 
     * @details A connection must be established before this function can be 
     * successfully called.
     * 
     * @param src The source byte array
     * @param size The number of bytes to write to the connection.
     * @return ssize_t The number of bytes actually written, or -1 on error.
     */
    virtual ssize_t write(const uint8_t *buffer, size_t len) const override;

    /**
     * @brief Returns the endpoint of the remote connection.
     * 
     */
    virtual Endpoint remote_endpoint() const override;

    /**
     * @brief Returns the endpoint of the local client.
     * 
     */
    virtual Endpoint local_endpoint() const override;

    /**
     * @brief Returns true if the underlying socket is ok.
     * 
     */
    virtual bool ok() const override;

    /**
     * @brief Waits for a connection to be accepts by the server specified
     * by the endpoint passed to a preceding call to connect. This function
     * will fail if connect has not been called. This function is only necessary
     * if the client is in non-blocking mode.
     * 
     * @details To check if a connection has been accepted in non-blocking mode,
     * this function will wait for the underlying socket to become writable.
     * When the socket is writable, data transmission is possible, so we infer
     * that the connection has been made successfully.
     * 
     * @param duration The maximum duration to wait for.
     * @return true if the connection has been accepted by the server.
     */
    virtual bool wait_for_connect(const rix::util::Duration &duration) const override;

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

    /**
     * @brief Resets client. This has the same effect as destroying the client
     * and assigning a new one. It will disconnect from the server, destroy the
     * underlying socket, and create a new, uninitialized socket.
     * 
     */
    virtual void reset() override;

   private:
    Socket socket;
};

}  // namespace ipc
}  // namespace rix