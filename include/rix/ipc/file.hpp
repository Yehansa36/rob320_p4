#pragma once

#include <fcntl.h>
#include <poll.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdint>
#include <string>
#include <vector>

#include "rix/ipc/interfaces/io.hpp"

namespace rix {
namespace ipc {

/**
 * @class File
 * @brief Object representing a standard file or I/O resource. This is the base
 * class for many objects, such as `Pipe` and `Fifo` in this project, and
 * `Socket` that is implemented in Project 2.
 *
 */
class File : public interfaces::IO {
   public:
   /**
    * @brief Removes the file specified by `pathname`.
    * 
    * @param pathname The path of the file to be removed
    * @return true if the file was successfully removed. 
    */
    static bool remove(const std::string &pathname);


    /**
     * @brief Default constructor. Does not open a file.
     *
     */
    File();

    /**
     * @brief Creates a File object with the corresponding file descriptor. This
     * will not duplicate the file descriptor, it will only assign its value to
     * an internal copy.
     *
     * @param fd The file descriptor
     */
    File(int fd);

    /**
     * @brief Creates a File object by opening the file specified by the path
     * name. 
     * 
     * @details The `creation_flags` parameter must include exactly one of the
     * following file access modes:
     *     O_RDONLY: open for reading only,
     *     O_WRONLY: open for writing only,
     *     O_RDWR:   open for reading and writing,
     *     O_EXEC:   open for execute only
     * In addition any combination of the following values can be bitwise or'ed:
     *     O_NONBLOCK: do not block on open or for data to become available,
     *     O_APPEND:   append on each write,
     *     O_CREAT:    create file if it does not exist,
     *     O_TRUNC:    truncate size to 0,
     *     O_EXCL:     error if O_CREAT and the file exists
     * If the O_CREAT flag is specified in the `creation_flags`, then the 
     * optional mode parameter must contain the file mode bits. These are 
     * commonly represented as octal numbers:
     *     0400: Allow read by owner.
     *     0200: Allow write by owner.
     *     0100: For files, allow execution by owner.
     *     0040: Allow read by group members.
     *     0020: Allow write by group members.
     *     0010: For files, allow execution by group members.
     *     0004: Allow read by others.
     *     0002: Allow write by others.
     *     0001: For files, allow execution by others.
     * 
     * For example, the following constructor will exclusively create a file 
     * named "my_file.txt" that can be written to by only the owner of the file
     * and read by anyone.
     * 
     *     File f("my_file.txt", O_RDWR | O_CREAT | O_EXCL, 0644);
     * 
     * In the file mode bits, we set the first value to 6 because 0400 | 0200 is
     * 0600, which enables reading and writing by the owner. We set the 
     * following two values to 4 because this enables reading only by group
     * members and others.
     * 
     * @param pathname The path name of the file to be opened.
     * @param creation_flags The creation flags.
     * @param mode The file mode bits (only necessary when O_CREAT included in
     * `creation_flags`).
     */
    File(std::string pathname, int creation_flags, mode_t mode = 0);

    /**
     * @brief Copy constructor. This will duplicate the underlying file 
     * descriptor using `dup`.
     * 
     * @param src The source file
     */
    File(const File &src);

    /**
     * @brief Assignment operator. This will duplicate the underlying file 
     * descriptor using `dup`.
     * 
     * @param src The source file
     */
    File &operator=(const File &src);
    
    /**
     * @brief Move constructor. Moves the source file descriptor to the 
     * destination Pipe and invalidates the source Pipe.
     * 
     * @param src The source file
     */
    File(File &&src);

    /**
     * @brief Move assignment operator. If the destination File is valid, close
     * the destination. Moves the source file descriptor to the destination File
     * and invalidates the source File.
     * 
     * @param src The source file
     */
    File &operator=(File &&src);

    /**
     * @brief Destructor. Closes the underlying file descriptor.
     * 
     */
    virtual ~File();

    /**
     * @brief Read `size` bytes from the file and store them in `dst`.
     * 
     * @param dst The destination byte array
     * @param size The number of bytes to read from the file.
     * @return ssize_t The number of bytes actually read, or -1 on error.
     */
    virtual ssize_t read(uint8_t *dst, size_t size) const override;
    
    /**
     * @brief Write `size` bytes from `src` to the file.
     * 
     * @param src The source byte array
     * @param size The number of bytes to write to the file.
     * @return ssize_t The number of bytes actually written, or -1 on error.
     */
    virtual ssize_t write(const uint8_t *src, size_t size) const override;

    /**
     * @brief Get the underlying file descriptor.
     * 
     * @return int The file descriptor
     */
    int fd() const;

    /**
     * @brief Returns `true` if the file is in a valid state, `false` otherwise.
     */
    bool ok() const;

    /**
     * @brief Waits for the specified duration for the file to become readable.
     * 
     * @param duration The maximum duration to wait.
     * @return true if the file has become readable within the duration.
     */
    virtual bool wait_for_readable(const util::Duration &duration) const override;

    /**
     * @brief Waits for the specified duration for the file to become writable.
     * 
     * @param duration The maximum duration to wait.
     * @return true if the file has become writable within the duration.
     */
    virtual bool wait_for_writable(const util::Duration &duration) const override;

    /**
     * @brief Toggles non-blocking IO operations.
     * 
     * @param status Flag to toggle non-blocking mode (true for non-blocking, 
     * false for blocking).
     */
    virtual void set_nonblocking(bool status) override;

    /**
     * @brief Returns true if the file is in non-blocking mode.
     *
     */
    virtual bool is_nonblocking() const override;

   protected:
    int fd_;
};

}  // namespace ipc
}  // namespace rix