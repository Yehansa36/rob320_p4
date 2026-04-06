#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace rix {
namespace msg {

class Message {
   public:
    virtual size_t size() const = 0;
    virtual std::array<uint64_t, 2> hash() const = 0;
    virtual void serialize(uint8_t *dst, size_t &offset) const = 0;
    virtual bool deserialize(const uint8_t *src, size_t size, size_t &offset) = 0;
};

}  // namespace msg
}  // namespace rix