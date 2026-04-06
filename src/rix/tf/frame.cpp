#include "rix/tf/frame.hpp"

namespace rix {
namespace tf {

Frame::Frame() {}

Frame::Frame(const std::string &name, const TransformBuffer &buffer) : name(name), buffer(buffer) {}

Frame::Frame(const std::string &name, const rix::util::Duration &duration)
    : name(name), buffer(duration) {}

Frame::Frame(const Frame &other) : name(other.name), buffer(other.buffer) {}

Frame &Frame::operator=(const Frame &other) {
    if (this != &other) {
        name = other.name;
        buffer = other.buffer;
    }
    return *this;
}

}  // namespace tf
}  // namespace rix