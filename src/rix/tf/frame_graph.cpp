#include "rix/tf/frame_graph.hpp"

#include "rix/robot/eigen_util.hpp"

#include <set>
#include <algorithm>

namespace rix {
namespace tf {

FrameGraph::FrameGraph(const std::string &root, const rix::util::Duration &duration)
    : frames_(1, Frame(root, duration)), graph_(1), duration_(duration) {
    name_to_index_[root] = 0;
}

FrameGraph::FrameGraph(const FrameGraph &other)
    : graph_(other.graph_), frames_(other.frames_), name_to_index_(other.name_to_index_) {}

FrameGraph &FrameGraph::operator=(const FrameGraph &other) {
    if (this != &other) {
        graph_ = other.graph_;
        frames_ = other.frames_;
        name_to_index_ = other.name_to_index_;
        duration_ = other.duration_;
    }
    return *this;
}

bool FrameGraph::exists(const std::string &name) const { return name_to_index_.find(name) != name_to_index_.end(); }

FrameGraph::Iterator FrameGraph::get_root() const { return Iterator(*this, 0); }

/**< TODO: Implement the get_leaves method. */
std::vector<std::string> FrameGraph::get_leaves() const {
    std::vector<std::string> leaves;
    
    // Iterate through all frames (skip root at index 0)
    // Root is "world" and always has children (base link)
    for (size_t i = 1; i < frames_.size(); ++i) {
        // A leaf frame has no children
        // In graph_, frame[i] has structure: [parent_index, child1, child2, ...]
        // So if size <= 1, it only has a parent and no children
        if (graph_[i].size() <= 1) {
            leaves.push_back(frames_[i].name);
        }
    }
    
    return leaves;
}

bool FrameGraph::update(const rix::msg::geometry::TF &tf) {
    for (const auto &transform : tf.transforms) {
        if (!update(transform)) return false;
    }
    return true;
}

/**< TODO: Implement the update method. */
bool FrameGraph::update(const rix::msg::geometry::TransformStamped &transform) {
    const std::string& parent_frame = transform.header.frame_id;
    const std::string& child_frame = transform.child_frame_id;
    
    // REQUIREMENT 1: Parent frame MUST exist
    // This keeps the graph connected!
    if (!exists(parent_frame)) {
        return false;  // Can't add transform if parent doesn't exist
    }
    
    // REQUIREMENT 2: Get or create child frame
    if (!exists(child_frame)) {
        // Create new frame
        int child_index = frames_.size();
        
        // Create Frame with same duration as root frame
        frames_.push_back(Frame(child_frame, frames_[0].buffer.duration()));
        
        // Expand graph with empty vector for this new frame
        graph_.push_back(std::vector<int>());
        
        // Map name to index for fast lookup
        name_to_index_[child_frame] = child_index;
    }
    
    int child_index = name_to_index_[child_frame];
    int parent_index = name_to_index_[parent_frame];
    
    // REQUIREMENT 3: Frames can't have multiple parents!
    if (graph_[child_index].size() > 0) {
        // Child already exists with a parent
        int existing_parent = graph_[child_index][0];
        if (existing_parent != parent_index) {
            // Different parent than before - NOT ALLOWED
            return false;
        }
        // Otherwise, same parent - just update transform
    } else {
        // First time setting this child's parent
        // Add parent as first element of child's vector
        graph_[child_index].push_back(parent_index);
        
        // Add child to parent's children list
        graph_[parent_index].push_back(child_index);
    }
    
    // Insert the transform into child frame's buffer
    // This overwrites if same timestamp, or adds if new timestamp
    rix::util::Time msg_time(transform.header.stamp.sec, transform.header.stamp.nsec);
    frames_[child_index].buffer.insert(msg_time, transform.transform);    
    return true;
}

/**< TODO: Implement the get_transform method. */
bool FrameGraph::get_transform(const std::string &target_frame, const std::string &source_frame, rix::util::Time time,
                               rix::msg::geometry::TransformStamped &transform) const {
    // SPECIAL CASE: Source and target are the same
    if (target_frame == source_frame) {
        transform.header.frame_id = source_frame;
        transform.child_frame_id = target_frame;
        // Identity transform (no movement)
        transform.transform = rix::robot::eigen_to_msg(Eigen::Affine3d::Identity());
        //transform.header.stamp = time;
        return true;
    }
    
    // Find source and target frames in the graph
    auto it_source = find(source_frame);
    auto it_target = find(target_frame);
    
    if (it_source == end() || it_target == end()) {
        return false;  // One or both frames don't exist
    }
    
    // Find their nearest common ancestor
    auto it_ancestor = find_nearest_ancestor(it_source, it_target);
    if (it_ancestor == end()) {
        return false;  // No common ancestor (shouldn't happen)
    }
    
    // BUILD CHAIN 1: source → ancestor
    // We build this by traversing up from source to ancestor
    Eigen::Affine3d T_ancestor_source = Eigen::Affine3d::Identity();
    
    auto it = it_source;
    while (it != it_ancestor) {
        // Get transform from this frame to its parent
        rix::msg::geometry::Transform child_transform;
        if (!it->buffer.get(time, child_transform)) {
            return false;  // Couldn't get transform for this time
        }
        
        // Convert to Eigen for matrix multiplication
        Eigen::Affine3d T_child = rix::robot::msg_to_eigen(child_transform);
        
        // Accumulate: T_ancestor_source = T_child * T_ancestor_source
        // This builds the chain from child up to ancestor
        T_ancestor_source = T_child * T_ancestor_source;
        
        --it;  // Move to parent
    }
    
    // BUILD CHAIN 2: target → ancestor (then invert)
    Eigen::Affine3d T_target_ancestor = Eigen::Affine3d::Identity();
    
    it = it_target;
    while (it != it_ancestor) {
        // Get transform from this frame to its parent
        rix::msg::geometry::Transform child_transform;
        if (!it->buffer.get(time, child_transform)) {
            return false;
        }
        
        Eigen::Affine3d T_child = rix::robot::msg_to_eigen(child_transform);
        T_target_ancestor = T_child * T_target_ancestor;
        
        --it;  // Move to parent
    }
    
    // Invert: T_ancestor_target = (T_target_ancestor)^-1
    Eigen::Affine3d T_ancestor_target = T_target_ancestor.inverse();
    
    // COMBINE: T_target_source = T_ancestor_target * T_ancestor_source
    // This gives us the transform from source → target
    Eigen::Affine3d T_target_source = T_ancestor_target * T_ancestor_source;
    
    // Package result
    transform.header.frame_id = source_frame;
    transform.child_frame_id = target_frame;
    transform.transform = rix::robot::eigen_to_msg(T_target_source);
    //transform.header.stamp = time;
    
    return true;
}

FrameGraph::Iterator FrameGraph::find(const std::string &name) const {
    auto it = name_to_index_.find(name);
    if (it != name_to_index_.end()) {
        return Iterator(*this, it->second);
    }
    return end();
}

/**< TODO: Implement the find_nearest_ancestor method. */
FrameGraph::Iterator FrameGraph::find_nearest_ancestor(Iterator frame_a, Iterator frame_b) const {
    // Special case: if frames are identical, they're their own ancestor
    if (frame_a == frame_b) {
        return frame_a;
    }
    
    // Collect all ancestors of frame_a by traversing up
    std::set<int> ancestors_a;
    auto it = frame_a;
    while (it != end()) {
        ancestors_a.insert(it.index_);
        --it;  // Move to parent (operator-- is defined)
    }
    
    // Now traverse up from frame_b until we find a common ancestor
    auto it_b = frame_b;
    while (it_b != end()) {
        if (ancestors_a.find(it_b.index_) != ancestors_a.end()) {
            // Found a frame that's in both paths!
            return it_b;
        }
        --it_b;  // Move to parent
    }
    
    // No common ancestor found (shouldn't happen in a connected tree)
    return end();
}

FrameGraph::Iterator FrameGraph::find_nearest_ancestor(const std::string &frame_a, const std::string &frame_b) const {
    auto it_a = find(frame_a);
    if (it_a == end()) {
        return end();
    }
    auto it_b = find(frame_b);
    if (it_b == end()) {
        return end();
    }
    return find_nearest_ancestor(it_a, it_b);
}

FrameGraph::Iterator::Iterator(const Iterator &other) : graph_(other.graph_), index_(other.index_) {}

FrameGraph::Iterator &FrameGraph::Iterator::operator=(const Iterator &other) {
    if (this != &other) {
        index_ = other.index_;
    }
    return *this;
}

FrameGraph::Iterator FrameGraph::end() const { return Iterator(*this, -1); }

FrameGraph::Iterator::Iterator(const FrameGraph &graph, int index) : graph_(graph), index_(index) {}

FrameGraph::Iterator &FrameGraph::Iterator::operator--() {
    if (index_ > 0) {
        index_ = graph_.graph_[index_][0];
    }
    return *this;
}

FrameGraph::Iterator FrameGraph::Iterator::operator--(int) {
    FrameGraph::Iterator tmp = *this;
    --(*this);
    return tmp;
}

bool FrameGraph::Iterator::operator==(const Iterator &other) const {
    return index_ == other.index_ && &graph_ == &other.graph_;
}
bool FrameGraph::Iterator::operator!=(const Iterator &other) const { return !(*this == other); }
const Frame &FrameGraph::Iterator::operator*() const { return graph_.frames_[index_]; }
const Frame *FrameGraph::Iterator::operator->() const { return &graph_.frames_[index_]; }
bool FrameGraph::Iterator::operator<(const Iterator &other) const { return index_ < other.index_; }
bool FrameGraph::Iterator::operator>(const Iterator &other) const { return index_ > other.index_; }
bool FrameGraph::Iterator::operator<=(const Iterator &other) const { return index_ <= other.index_; }
bool FrameGraph::Iterator::operator>=(const Iterator &other) const { return index_ >= other.index_; }

}  // namespace tf
}  // namespace rix