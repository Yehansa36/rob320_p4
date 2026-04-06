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
namespace standard {

class Color : public Message {
  public:
    float r;
    float g;
    float b;
    float a;

    Color() = default;
    Color(const Color &other) = default;
    ~Color() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_number(r);
        size += size_number(g);
        size += size_number(b);
        size += size_number(a);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0xf88b69207e8f108cULL, 0x3824545f5cb73032ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_number(dst, offset, r);
        serialize_number(dst, offset, g);
        serialize_number(dst, offset, b);
        serialize_number(dst, offset, a);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_number(r, src, size, offset)) { return false; };
        if (!deserialize_number(g, src, size, offset)) { return false; };
        if (!deserialize_number(b, src, size, offset)) { return false; };
        if (!deserialize_number(a, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace standard
} // namespace msg
} // namespace rix