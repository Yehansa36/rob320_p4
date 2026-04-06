#pragma once

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "rix/ipc/endpoint.hpp"
#include "rix/ipc/file.hpp"

namespace rix {
namespace ipc {

class Socket : public File {
   public:
    /**
     * @brief Construct a new Socket object. Sets fd_ to -1 by default.
     *
     */
    Socket();

    /**
     * @brief Construct a new Socket object. This will call `socket` to create
     * a new socket with the specified domain and type.
     *
     * @param domain The domain of the socket.
     * @param type The type of the socket.
     */
    Socket(int domain, int type);

    /**
     * @brief Copy constructor. This will duplicate the underlying file 
     * descriptor using `dup`.
     * 
     * @param src The source file
     */
    Socket(const Socket &src);

    /**
     * @brief Assignment operator. This will duplicate the underlying file 
     * descriptor using `dup`.
     * 
     * @param src The source file
     */
    Socket &operator=(const Socket &src);
    
    /**
     * @brief Move constructor. Moves the source file descriptor to the 
     * destination Socket and invalidates the source Socket.
     * 
     * @param src The source file
     */
    Socket(Socket &&src);

    /**
     * @brief Move assignment operator. If the destination Socket is valid, close
     * the destination. Moves the source file descriptor to the destination Socket
     * and invalidates the source Socket.
     * 
     * @param src The source file
     */
    Socket &operator=(Socket &&src);

    /**
     * @brief Destructor. Closes the underlying file descriptor.
     * 
     */
    virtual ~Socket();

    /**
     * @brief Bind the socket to the specified endpoint.
     *
     * @param endpoint The endpoint to bind to.
     * @return false on failuretrue 0 on success
     */
    bool bind(const Endpoint &endpoint);

    /**
     * @brief Enable a stream socket to accept incoming connections from other
     * sockets.
     *
     * @param backlog The maximum length to which the queue of pending connections
     * for the socket may grow.
     * @return false on failuretrue 0 on success
     */
    bool listen(int backlog);

    /**
     * @brief Establish a connection with another socket.
     *
     * @param endpoint The endpoint to connect to.
     * @return false on failuretrue 0 on success
     */
    bool connect(const Endpoint &endpoint);

    /**
     * @brief Accept a connection from a peer application on a listening stream
     * socket.
     *
     * @param sock The new socket created by accepting a connection. If
     * successful, `is_open` will return true.
     * @return false on failure, true on success
     */
    bool accept(Socket &sock);

    /**
     * @brief Get the value of a socket option.
     *
     * @param level The level at which the option is defined.
     * @param name The name of a specified socket option.
     * @param value The option data.
     * @return false on failure, true on success.
     */
    bool getsockopt(int level, int name, int &value);

    /**
     * @brief Set the value of a socket option.
     *
     * @param level The level for which the option is being set.
     * @param name The name of a specified socket option.
     * @param value The option data.
     * @return false on failure, true on success.
     */
    bool setsockopt(int level, int name, int value);

    /**
     * @brief Get the local endpoint of the socket.
     *
     * @param endpoint The local endpoint.
     * @return false on failure, true on success.
     */
    bool getsockname(Endpoint &endpoint) const;

    /**
     * @brief Get the remote endpoint of the socket.
     *
     * @param endpoint The remote endpoint.
     * @return false on failure, true on success.
     */
    bool getpeername(Endpoint &endpoint) const;

    bool is_bound() const;
    bool is_listening() const;

   private:
    int domain;
    int type;
    bool _bound;
    bool _listening;

    Socket(int fd, int domain, int type);
};

}  // namespace ipc
}  // namespace rix
