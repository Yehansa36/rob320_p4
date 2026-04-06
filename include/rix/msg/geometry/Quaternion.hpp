#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <cstring>

#include "rix/msg/serialization.hpp"
#include "rix/msg/message.hpp"

namespace rix {
namespace msg {
namespace geometry {

class Quaternion : public Message {
  public:
    float w;
    float x;
    float y;
    float z;

    Quaternion() = default;
    Quaternion(const Quaternion &other) = default;
    ~Quaternion() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_number(w);
        size += size_number(x);
        size += size_number(y);
        size += size_number(z);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x5cb28b688a9da3c7ULL, 0x36801d54a9a01ce1ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_number(dst, offset, w);
        serialize_number(dst, offset, x);
        serialize_number(dst, offset, y);
        serialize_number(dst, offset, z);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_number(w, src, size, offset)) { return false; };
        if (!deserialize_number(x, src, size, offset)) { return false; };
        if (!deserialize_number(y, src, size, offset)) { return false; };
        if (!deserialize_number(z, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix