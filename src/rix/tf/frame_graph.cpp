#include "rix/tf/frame_graph.hpp"

#include "rix/robot/eigen_util.hpp"

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
    return {};
}

bool FrameGraph::update(const rix::msg::geometry::TF &tf) {
    for (const auto &transform : tf.transforms) {
        if (!update(transform)) return false;
    }
    return true;
}

/**< TODO: Implement the update method. */
bool FrameGraph::update(const rix::msg::geometry::TransformStamped &transform) {
    return false;
}

/**< TODO: Implement the get_transform method. */
bool FrameGraph::get_transform(const std::string &target_frame, const std::string &source_frame, rix::util::Time time,
                               rix::msg::geometry::TransformStamped &transform) const {
    return false;
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