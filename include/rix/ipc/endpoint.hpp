#pragma once

#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <ifaddrs.h>

#include <iostream>
#include <vector>
#include <string>
#include <cstring>

namespace rix {
namespace ipc {

/**
 * @brief Class for representing Endpoints.
 * 
*/
class Endpoint {
  public:
    std::string address;
    int port;

    /**
     * @brief Default constructor. Initializes an empty Endpoint.
     */
    Endpoint();

    /**
     * @brief Constructs a Endpoint with the given IP address and port.
     * @param address The IP address.
     * @param port The port number.
     */
    Endpoint(const std::string &address, int port);

    /**
     * @brief Less-than operator for comparing Endpoints.
     * @param other The other Endpoint to compare with.
     * @return true if this Endpoint is less than the other Endpoint, false otherwise.
     */
    bool operator<(const Endpoint &other) const;

    /**
     * @brief Equality operator for comparing Endpoints.
     * @param other The other Endpoint to compare with.
     * @return true if the Endpoints are equal, false otherwise.
     */
    bool operator==(const Endpoint &other) const;

    /**
     * @brief Inequality operator for comparing Endpoints.
     * @param other The other Endpoint to compare with.
     * @return true if the Endpoints are not equal, false otherwise.
     */
    bool operator!=(const Endpoint &other) const;

    /**
     * @brief Converts the Endpoint to a string representation.
     * @return The string representation of the Endpoint.
     */
    std::string to_string() const;

    /**
     * @brief Stream insertion operator for printing Endpoints.
     * @param os The output stream.
     * @param uri The Endpoint to print.
     * @return The output stream.
     */
    friend std::ostream &operator<<(std::ostream &os, const Endpoint &uri) {
        os << uri.to_string();
        return os;
    }

    /**
     * @brief Hash functor for std::unordered_map and std::unordered_set
     * 
     */
    struct Hash {
        std::size_t operator()(const Endpoint &endpoint) const {
            return std::hash<std::string>{}(endpoint.to_string());
        }
    };
};

}  // namespace ipc
}  // namespace rix