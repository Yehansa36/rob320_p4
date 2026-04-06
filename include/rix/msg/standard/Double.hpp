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

class Double : public Message {
  public:
    double data;

    Double() = default;
    Double(const Double &other) = default;
    ~Double() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_number(data);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0xb2946cc965c09d4bULL, 0x2d6ad4c506beeb12ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_number(dst, offset, data);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_number(data, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace standard
} // namespace msg
} // namespace rix