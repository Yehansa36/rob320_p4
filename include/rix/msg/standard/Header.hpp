#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <cstring>

#include "rix/msg/serialization.hpp"
#include "rix/msg/message.hpp"
#include "rix/msg/standard/Time.hpp"

namespace rix {
namespace msg {
namespace standard {

class Header : public Message {
  public:
    uint32_t seq;
    standard::Time stamp;
    std::string frame_id;

    Header() = default;
    Header(const Header &other) = default;
    ~Header() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_number(seq);
        size += size_message(stamp);
        size += size_string(frame_id);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0xc843ae39514c0d3aULL, 0x13af07ef9144f643ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_number(dst, offset, seq);
        serialize_message(dst, offset, stamp);
        serialize_string(dst, offset, frame_id);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_number(seq, src, size, offset)) { return false; };
        if (!deserialize_message(stamp, src, size, offset)) { return false; };
        if (!deserialize_string(frame_id, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace standard
} // namespace msg
} // namespace rix