#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "rix/msg/message.hpp"

namespace rix {
namespace msg {
namespace detail {

template <typename T>
inline uint32_t size_number(const T &src) {
    static_assert(std::is_arithmetic<T>::value, "T must be an arithmetic type");
    return sizeof(T);
}
inline uint32_t size_string(const std::string &src) { return 4 + src.size(); }
inline uint32_t size_message(const Message &src) { return src.size(); }
template <typename T, size_t N>
inline uint32_t size_number_array(const std::array<T, N> &src) {
    static_assert(std::is_arithmetic<T>::value, "T must be an arithmetic type");
    return N * sizeof(T);
}
template <size_t N>
inline uint32_t size_string_array(const std::array<std::string, N> &src) {
    uint32_t size = 0;
    for (const auto &s : src) size += size_string(s);
    return size;
}
template <typename T, size_t N>
inline uint32_t size_message_array(const std::array<T, N> &src) {
    static_assert(std::is_base_of<Message, T>::value, "T must derive from Message");
    uint32_t size = 0;
    for (const auto &m : src) size += size_message(m);
    return size;
}
template <typename T>
inline uint32_t size_number_vector(const std::vector<T> &src) {
    static_assert(std::is_arithmetic<T>::value, "T must be an arithmetic type");
    return 4 + src.size() * sizeof(T);
}
inline uint32_t size_string_vector(const std::vector<std::string> &src) {
    uint32_t size = 4;
    for (const auto &s : src) size += size_string(s);
    return size;
}
template <typename T>
inline uint32_t size_message_vector(const std::vector<T> &src) {
    static_assert(std::is_base_of<Message, T>::value, "T must derive from Message");
    uint32_t size = 4;
    for (const auto &m : src) size += size_message(m);
    return size;
}

/**
 * @brief Serializes a number `src` and stores it in the byte array `dst` at
 * `offset`. `offset` is incremented by the number of bytes written to `dst`.
 *
 * @tparam T The type of the source (must be an arithmetic type)
 * @param dst The destination byte array
 * @param offset The offset in the byte array at which to write (incremented by
 * number of bytes written)
 * @param src The source number to be serialized
 */
template <typename T>
inline void serialize_number(uint8_t *dst, size_t &offset, const T &src) {
    static_assert(std::is_arithmetic<T>::value, "T must be an arithmetic type");
    /**< TODO */
    std::memcpy(dst + offset, &src, sizeof(T));
    offset += sizeof(T);
}

/**
 * @brief Serializes a string `src` and stores it in the byte array `dst` at
 * `offset`. `offset` is incremented by the number of bytes written to `dst`.
 *
 * @param dst The destination byte array
 * @param offset The offset in the byte array at which to write (incremented by
 * number of bytes written)
 * @param src The source string to be serialized
 */
inline void serialize_string(uint8_t *dst, size_t &offset, const std::string &src) {
    /**< TODO */
    uint32_t size = src.size();
    std::memcpy(dst + offset, &size, 4);
    offset += 4;
    std::memcpy(dst + offset, src.data(), size);
    offset += size;
}

/**
 * @brief Serializes a message `src` and stores it in the byte array `dst` at
 * `offset`. `offset` is incremented by the number of bytes written to `dst`.
 *
 * @param dst The destination byte array
 * @param offset The offset in the byte array at which to write (incremented by
 * number of bytes written)
 * @param src The source message to be serialized
 */
inline void serialize_message(uint8_t *dst, size_t &offset, const Message &src) {
    /**< TODO */
    src.serialize(dst, offset);
}

/**
 * @brief Serializes a number array `src` and stores it in the byte array `dst`
 * at `offset`. `offset` is incremented by the number of bytes written to `dst`.
 *
 * @tparam T The type of the source array (must be an arithmetic type)
 * @tparam N The size of the source array
 * @param dst The destination byte array
 * @param offset The offset in the byte array at which to write (incremented by
 * number of bytes written)
 * @param src The source number array to be serialized
 */
template <typename T, size_t N>
inline void serialize_number_array(uint8_t *dst, size_t &offset,
                                   const std::array<T, N> &src) {
    /**< TODO */
    static_assert(std::is_arithmetic<T>::value, "T must be an arithmetic type");
    uint32_t size = N * sizeof(T);
    std::memcpy(dst + offset, src.data(), size);
    offset += size;
}

/**
 * @brief Serializes a string array `src` and stores it in the byte array `dst`
 * at `offset`. `offset` is incremented by the number of bytes written to `dst`.
 *
 * @tparam N The size of the source array
 * @param dst The destination byte array
 * @param offset The offset in the byte array at which to write (incremented by
 * number of bytes written)
 * @param src The source string array to be serialized
 */
template <size_t N>
inline void serialize_string_array(uint8_t *dst, size_t &offset,
                                   const std::array<std::string, N> &src) {
    /**< TODO */
    for (const auto &s : src) serialize_string(dst, offset, s);
}

/**
 * @brief Serializes a message array `src` and stores it in the byte array `dst`
 * at `offset`. `offset` is incremented by the number of bytes written to `dst`.
 *
 * @tparam T The type of the source array (must derive from Message)
 * @tparam N The size of the source array
 * @param dst The destination byte array
 * @param offset The offset in the byte array at which to write (incremented by
 * number of bytes written)
 * @param src The source message array to be serialized
 */
template <typename T, size_t N>
inline void serialize_message_array(uint8_t *dst, size_t &offset,
                                    const std::array<T, N> &src) {
    static_assert(std::is_base_of<Message, T>::value, "T must derive from Message");
    /**< TODO */
    for (const auto &m : src) serialize_message(dst, offset, m);
}

/**
 * @brief Serializes a number vector `src` and stores it in the byte array `dst`
 * at `offset`. `offset` is incremented by the number of bytes written to `dst`.
 *
 * @tparam T The type of the source array (must be an arithmetic type)
 * @param dst The destination byte array
 * @param offset The offset in the byte array at which to write (incremented by
 * number of bytes written)
 * @param src The source number vector to be serialized
 */
template <typename T>
inline void serialize_number_vector(uint8_t *dst, size_t &offset,
                                    const std::vector<T> &src) {
    static_assert(std::is_arithmetic<T>::value, "T must be an arithmetic type");
    /**< TODO */
    uint32_t size = src.size();
    serialize_number(dst, offset, size);
    size *= sizeof(T);
    std::memcpy(dst + offset, src.data(), size);
    offset += size;
}

/**
 * @brief Serializes a string vector `src` and stores it in the byte array `dst`
 * at `offset`. `offset` is incremented by the number of bytes written to `dst`.
 *
 * @param dst The destination byte array
 * @param offset The offset in the byte array at which to write (incremented by
 * number of bytes written)
 * @param src The source string vector to be serialized
 */
inline void serialize_string_vector(uint8_t *dst, size_t &offset,
                                    const std::vector<std::string> &src) {
    /**< TODO */
    uint32_t size = src.size();
    serialize_number(dst, offset, size);
    for (const auto &s : src) serialize_string(dst, offset, s);
}

/**
 * @brief Serializes a message vector `src` and stores it in the byte array
 * `dst` at `offset`. `offset` is incremented by the number of bytes written to
 * `dst`.
 *
 * @tparam T The type of the source array (must derive from Message)
 * @param dst The destination byte array
 * @param offset The offset in the byte array at which to write (incremented by
 * number of bytes written)
 * @param src The source message vector to be serialized
 */
template <typename T>
inline void serialize_message_vector(uint8_t *dst, size_t &offset,
                                     const std::vector<T> &src) {
    static_assert(std::is_base_of<Message, T>::value, "T must derive from Message");
    /**< TODO */
    uint32_t size = src.size();
    serialize_number(dst, offset, size);
    for (const auto &m : src) serialize_message(dst, offset, m);
}

/**
 * @brief Deserializes a number from the byte array `src` at `offset` and stores
 * it into `dst`. `src` must be at least `size` bytes long.
 *
 * @tparam T The type of the destination value (must be an arithmetic type)
 * @param dst The destination number
 * @param src The source byte array
 * @param size The size of the byte array
 * @param offset The position in the source byte array to deserialize data from
 * @return `false` if the number of bytes needed to deserialize the number is
 * greater than the number of bytes available in the source byte array. `true`
 * otherwise.
 */
template <typename T>
inline bool deserialize_number(T &dst, const uint8_t *src, size_t size, size_t &offset) {
    static_assert(std::is_arithmetic<T>::value, "T must be an arithmetic type");
    /**< TODO */
    if (offset + sizeof(T) > size) return false;
    dst = *reinterpret_cast<const T *>(src + offset);
    offset += sizeof(T);
    return true;
}

/**
 * @brief Deserializes a string from the byte array `src` at `offset` and stores
 * it into `dst`. `src` must be at least `size` bytes long.
 *
 * @param dst The destination string
 * @param src The source byte array
 * @param size The size of the byte array
 * @param offset The position in the source byte array to deserialize data from
 * @return `false` if the number of bytes needed to deserialize the string is
 * greater than the number of bytes available in the source byte array. `true`
 * otherwise.
 */
inline bool deserialize_string(std::string &dst, const uint8_t *src, size_t size,
                               size_t &offset) {
    /**< TODO */
    uint32_t str_size;
    if (!deserialize_number(str_size, src, size, offset)) return false;
    if (offset + str_size > size) return false;
    dst.resize(str_size);
    std::memcpy(dst.data(), src + offset, str_size);
    offset += str_size;
    return true;
}

/**
 * @brief Deserializes a message from the byte array `src` at `offset` and
 * stores it into `dst`. `src` must be at least `size` bytes long.
 *
 * @param dst The destination message
 * @param src The source byte array
 * @param size The size of the byte array
 * @param offset The position in the source byte array to deserialize data from
 * @return `false` if the number of bytes needed to deserialize the message is
 * greater than the number of bytes available in the source byte array. `true`
 * otherwise.
 */
inline bool deserialize_message(Message &dst, const uint8_t *src, size_t size,
                                size_t &offset) {
    /**< TODO */
    return dst.deserialize(src, size, offset);
}

/**
 * @brief Deserializes a string from the byte array `src` at `offset` and stores
 * it into `dst`. `src` must be at least `size` bytes long.
 *
 * @tparam T The type of the destination array (must be an arithmetic type)
 * @tparam N The size of the destination array
 * @param dst The destination string
 * @param src The source byte array
 * @param size The size of the byte array
 * @param offset The position in the source byte array to deserialize data from
 * @return `false` if the number of bytes needed to deserialize the string is
 * greater than the number of bytes available in the source byte array. `true`
 * otherwise.
 */
template <typename T, size_t N>
inline bool deserialize_number_array(std::array<T, N> &dst, const uint8_t *src,
                                     size_t size, size_t &offset) {
    static_assert(std::is_arithmetic<T>::value, "T must be an arithmetic type");
    /**< TODO */
    uint32_t arr_size = N * sizeof(T);
    if (offset + arr_size > size) return false;
    std::memcpy(dst.data(), src + offset, arr_size);
    offset += arr_size;
    return true;
}

/**
 * @brief Deserializes a string array from the byte array `src` at `offset` and
 * stores it into `dst`. `src` must be at least `size` bytes long.
 *
 * @tparam N The size of the destination array
 * @param dst The destination string array
 * @param src The source byte array
 * @param size The size of the byte array
 * @param offset The position in the source byte array to deserialize data from
 * @return `false` if the number of bytes needed to deserialize the string array
 * is greater than the number of bytes available in the source byte array.
 * `true` otherwise.
 */
template <size_t N>
inline bool deserialize_string_array(std::array<std::string, N> &dst, const uint8_t *src,
                                     size_t size, size_t &offset) {
    /**< TODO */
    for (auto &s : dst) {
        if (!deserialize_string(s, src, size, offset)) return false;
    }
    return true;
}

/**
 * @brief Deserializes a message array from the byte array `src` at `offset` and
 * stores it into `dst`. `src` must be at least `size` bytes long.
 *
 * @tparam T The type of the destination array (must derive from Message)
 * @tparam N The size of the destination array
 * @param dst The destination message array
 * @param src The source byte array
 * @param size The size of the byte array
 * @param offset The position in the source byte array to deserialize data from
 * @return `false` if the number of bytes needed to deserialize the message
 * array is greater than the number of bytes available in the source byte array.
 * `true` otherwise.
 */
template <typename T, size_t N>
inline bool deserialize_message_array(std::array<T, N> &dst, const uint8_t *src,
                                      size_t size, size_t &offset) {
    static_assert(std::is_base_of<Message, T>::value, "T must derive from Message");
    /**< TODO */
    for (auto &m : dst) {
        if (!deserialize_message(m, src, size, offset)) return false;
    }
    return true;
}

/**
 * @brief Deserializes a number vector from the byte array `src` at `offset` and
 * stores it into `dst`. `src` must be at least `size` bytes long.
 *
 * @tparam T The type of the destination array (must be an arithmetic type)
 * @param dst The destination number vector
 * @param src The source byte array
 * @param size The size of the byte array
 * @param offset The position in the source byte array to deserialize data from
 * @return `false` if the number of bytes needed to deserialize the number
 * vector is greater than the number of bytes available in the source byte
 * array. `true` otherwise.
 */
template <typename T>
inline bool deserialize_number_vector(std::vector<T> &dst, const uint8_t *src,
                                      size_t size, size_t &offset) {
    static_assert(std::is_arithmetic<T>::value, "T must be an arithmetic type");
    /**< TODO */
    uint32_t vec_size;
    if (!deserialize_number(vec_size, src, size, offset)) return false;
    if (offset + vec_size > size) return false;
    dst.resize(vec_size);
    size_t size_bytes = vec_size * sizeof(T);
    std::memcpy(dst.data(), src + offset, size_bytes);
    offset += size_bytes;
    return true;
}

/**
 * @brief Deserializes a string vector from the byte array `src` at `offset` and
 * stores it into `dst`. `src` must be at least `size` bytes long.
 *
 * @param dst The destination string vector
 * @param src The source byte array
 * @param size The size of the byte array
 * @param offset The position in the source byte array to deserialize data from
 * @return `false` if the number of bytes needed to deserialize the string
 * vector is greater than the number of bytes available in the source byte
 * array. `true` otherwise.
 */
inline bool deserialize_string_vector(std::vector<std::string> &dst, const uint8_t *src,
                                      size_t size, size_t &offset) {
    /**< TODO */
    uint32_t vec_size;
    if (!deserialize_number(vec_size, src, size, offset)) return false;
    dst.resize(vec_size);
    for (auto &s : dst) {
        if (!deserialize_string(s, src, size, offset)) return false;
    }
    return true;
}

/**
 * @brief Deserializes a message vector from the byte array `src` at `offset` and
 * stores it into `dst`. `src` must be at least `size` bytes long.
 *
 * @tparam T The type of the destination array (must derive from Message)
 * @param dst The destination message vector
 * @param src The source byte array
 * @param size The size of the byte array
 * @param offset The position in the source byte array to deserialize data from
 * @return `false` if the number of bytes needed to deserialize the message
 * vector is greater than the number of bytes available in the source byte
 * array. `true` otherwise.
 */
template <typename T>
inline bool deserialize_message_vector(std::vector<T> &dst, const uint8_t *src,
                                       size_t size, size_t &offset) {
    static_assert(std::is_base_of<Message, T>::value, "T must derive from Message");
    /**< TODO */
    uint32_t vec_size;
    if (!deserialize_number(vec_size, src, size, offset)) return false;
    dst.resize(vec_size);
    for (auto &m : dst) {
        if (!deserialize_message(m, src, size, offset)) return false;
    }
    return true;
}
}  // namespace detail
}  // namespace msg
}  // namespace rix