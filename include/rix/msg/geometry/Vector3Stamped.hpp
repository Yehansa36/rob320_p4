#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <cstring>

#include "rix/msg/serialization.hpp"
#include "rix/msg/message.hpp"
#include "rix/msg/geometry/Vector3.hpp"
#include "rix/msg/standard/Header.hpp"

namespace rix {
namespace msg {
namespace geometry {

class Vector3Stamped : public Message {
  public:
    standard::Header header;
    geometry::Vector3 vector3;

    Vector3Stamped() = default;
    Vector3Stamped(const Vector3Stamped &other) = default;
    ~Vector3Stamped() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_message(header);
        size += size_message(vector3);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x9236c1ec83a06e9fULL, 0x19da5fd174aed535ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_message(dst, offset, header);
        serialize_message(dst, offset, vector3);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_message(header, src, size, offset)) { return false; };
        if (!deserialize_message(vector3, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix