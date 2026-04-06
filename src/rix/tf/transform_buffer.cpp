#include "rix/tf/transform_buffer.hpp"

#include <Eigen/Geometry>
#include <algorithm>

#include "rix/robot/eigen_util.hpp"

namespace rix {
namespace tf {

TransformBuffer::TransformBuffer() {}

TransformBuffer::TransformBuffer(const rix::util::Duration &duration) : duration_(duration) {}

TransformBuffer::TransformBuffer(const TransformBuffer &other) : duration_(other.duration_), buffer_(other.buffer_) {}

TransformBuffer &TransformBuffer::operator=(const TransformBuffer &other) {
    if (this != &other) {
        duration_ = other.duration_;
        buffer_ = other.buffer_;
    }
    return *this;
}

size_t TransformBuffer::size() const { return buffer_.size(); }

bool TransformBuffer::empty() const { return buffer_.empty(); }

void TransformBuffer::clear() { buffer_.clear(); }

/*< TODO: Implement the insert method. */
void TransformBuffer::insert(const rix::util::Time &time, const rix::msg::geometry::Transform &transform) {
    return;
}

/*< TODO: Implement the get method. */
bool TransformBuffer::get(const rix::util::Time &time, rix::msg::geometry::Transform &transform) const {
    return false;
}

const std::deque<std::pair<rix::util::Time, rix::msg::geometry::Transform>> &TransformBuffer::data() const {
    return buffer_;
}

rix::util::Duration TransformBuffer::duration() const { return duration_; }

void TransformBuffer::set_duration(const rix::util::Duration &duration) { duration_ = duration; }

}  // namespace tf
}  // namespace rix