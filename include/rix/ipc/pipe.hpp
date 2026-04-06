#pragma once

#include "rix/ipc/file.hpp"

namespace rix {
namespace ipc {

/**
 * 
 * @class Pipe
 * @brief Unidirectional, unnamed communication channel. Inherits from the 
 * `File` class. Use the factory method `Pipe::create` to instantiate a pair of 
 * Pipe objects. The first is the read-end and the second is the write-end.
 * 
 */
class Pipe : public File {
   public:
   /**
    * @brief Factory method to create a pair of Pipe objects. The first element
    * is the read-end and the second is the write-end.
    * 
    * @return std::array<Pipe, 2> The pipe pair
    */
    static std::array<Pipe, 2> create();

    /**
     * @brief Default constructor. This does not open a file descriptor. The 
     * only way to create a Pipe object is via the factory method `Pipe::create`
     * 
     */
    Pipe();

    /**
     * @brief Copy constructor. This will duplicate the underlying file 
     * descriptors using `dup`.
     * 
     * @param src The Pipe to be copied.
     */
    Pipe(const Pipe &src);

    /**
     * @brief Assignment operator. This will duplicate the underlying file 
     * descriptors using `dup`.
     * 
     * @param src The Pipe to be copied.
     */
    Pipe &operator=(const Pipe &src);

    /**
     * @brief Destructor. This will close the underlying file descriptors.
     * 
     */
    ~Pipe();

    /**
     * @brief Move constructor. Moves the source file descriptor to the 
     * destination Pipe and invalidates the source Pipe.
     * 
     * @param src The Pipe to be moved
     */
    Pipe(Pipe &&src);

    /**
     * @brief Move assignment operator. If the destination Pipe is valid, close
     * the destination. Moves the source file descriptor to the destination Pipe
     * and invalidates the source Pipe.
     * 
     * @param src The Pipe to be moved
     * @return Pipe& A reference to the destination Pipe
     */
    Pipe &operator=(Pipe &&src);

    /**
     * @brief Returns `true` if this Pipe is the read end.
     * 
     */
    bool is_read_end() const;
    
    /**
     * @brief Returns `true` if this Pipe is the read end.
     * 
     */
    bool is_write_end() const;

   private:
    /**
     * @brief Private constructor used by the `create` factory method.
     * 
     * @param fd The underlying file descriptor
     * @param read_end Flag to indicate if this is the read-end (true) or 
     * write-end (false).
     */
    Pipe(int fd, bool read_end);

    bool read_end_; 
};

}  // namespace ipc
}  // namespace rix