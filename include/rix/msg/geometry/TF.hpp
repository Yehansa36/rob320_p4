#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <cstring>

#include "rix/msg/serialization.hpp"
#include "rix/msg/message.hpp"
#include "rix/msg/geometry/TransformStamped.hpp"

namespace rix {
namespace msg {
namespace geometry {

class TF : public Message {
  public:
    std::vector<geometry::TransformStamped> transforms;

    TF() = default;
    TF(const TF &other) = default;
    ~TF() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_message_vector(transforms);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0xa260c3d7ccd775ecULL, 0x176b837161441526ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_message_vector(dst, offset, transforms);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_message_vector(transforms, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix