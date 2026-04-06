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
    // Step 1: Remove old transforms outside the duration window
    // This keeps memory usage bounded
    while (!buffer_.empty()) {
        rix::util::Time oldest_time = buffer_.front().first;
        if (time - oldest_time > duration_) {
            buffer_.pop_front();  // O(1) for deque
        } else {
            break;  // Rest are recent enough
        }
    }
    
    // Step 2: Binary search to find insertion position
    // We want to maintain time-sorted order (earliest at front, latest at back)
    auto it = std::lower_bound(
        buffer_.begin(), buffer_.end(), 
        std::make_pair(time, transform),
        // Comparison function: sort by time only
        [](const auto& a, const auto& b) { 
            return a.first < b.first; 
        }
    );
    
    // Step 3: Check for duplicate timestamp
    if (it != buffer_.end() && it->first == time) {
        // Duplicate timestamp → overwrite existing transform
        it->second = transform;
    } else {
        // New timestamp → insert at correct position
        buffer_.insert(it, std::make_pair(time, transform));
    }
}

/*< TODO: Implement the get method. */
bool TransformBuffer::get(const rix::util::Time &time, rix::msg::geometry::Transform &transform) const {
    // Return false if buffer is empty
    if (buffer_.empty()) {
        return false;
    }
    
    // SPECIAL CASE 1: time == 0 means return latest transform
    // This is a convention used in the system
    if (time == rix::util::Time(0)) {
        transform = buffer_.back().second;
        return true;
    }
    
    // SPECIAL CASE 2: find where this time would go in the buffer
    auto it = std::lower_bound(
        buffer_.begin(), buffer_.end(),
        std::make_pair(time, rix::msg::geometry::Transform()),
        [](const auto& a, const auto& b) { 
            return a.first < b.first; 
        }
    );
    
    // CASE 1: Exact match found
    if (it != buffer_.end() && it->first == time) {
        transform = it->second;
        return true;
    }
    
    // CASE 2: Time is before earliest entry in buffer
    // Return the earliest (oldest) transform we have
    if (it == buffer_.begin()) {
        transform = buffer_.front().second;
        return true;
    }
    
    // CASE 3: Time is after latest entry in buffer
    // Return the latest (most recent) transform we have
    if (it == buffer_.end()) {
        transform = buffer_.back().second;
        return true;
    }
    
    // CASE 4: Time is between two entries - INTERPOLATE
    // This is the most common case for continuous motion
    auto it_prev = std::prev(it);  // Transform at earlier time
    auto it_next = it;              // Transform at later time
    
    rix::util::Time time_prev = it_prev->first;
    rix::util::Time time_next = it_next->first;
    
    // Calculate interpolation ratio (0.0 to 1.0)
    // ratio = 0.0 means use prev transform
    // ratio = 1.0 means use next transform
    // ratio = 0.5 means halfway between
    double duration_total = static_cast<double>((time_next - time_prev).to_nanoseconds()); // seconds
    double duration_partial = static_cast<double>((time - time_prev).to_nanoseconds());     // seconds
    
    double ratio = duration_partial / duration_total;
    
    // Clamp to [0, 1] to be safe
    ratio = std::max(0.0, std::min(1.0, ratio));
    
    // Use the provided interpolation function
    // - Position: linear interpolation
    // - Rotation: SLERP (spherical linear interpolation)
    transform = rix::robot::interpolate(it_prev->second, it_next->second, ratio);
    
    return true;
}

const std::deque<std::pair<rix::util::Time, rix::msg::geometry::Transform>> &TransformBuffer::data() const {
    return buffer_;
}

rix::util::Duration TransformBuffer::duration() const { return duration_; }

void TransformBuffer::set_duration(const rix::util::Duration &duration) { duration_ = duration; }

}  // namespace tf
}  // namespace rix