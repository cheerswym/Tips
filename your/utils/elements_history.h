#ifndef ONBOARD_UTILS_ELEMENTS_HISTORY_H_
#define ONBOARD_UTILS_ELEMENTS_HISTORY_H_

#include <memory>
#include <utility>

#include "absl/base/macros.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "boost/circular_buffer.hpp"
#include "onboard/async/async_util.h"
#include "onboard/utils/map_util.h"
namespace qcraft {
namespace elements_history {
template <class TimeType, class ElementType>
struct Node {
  TimeType time;
  ElementType val;
  template <typename VV>
  Node(const TimeType& time, VV&& val)
      : time(time), val(std::forward<VV>(val)) {}
  ~Node() {}
  bool operator()(const Node<TimeType, ElementType>& left,
                  const Node<TimeType, ElementType>& right) {
    return left.time < right.time;
  }
  bool operator()(const Node<TimeType, ElementType>& left, TimeType right) {
    return left.time < right;
  }
  bool operator()(TimeType left, const Node<TimeType, ElementType>& right) {
    return left < right.time;
  }
  friend bool operator<(const Node<TimeType, ElementType>& left,
                        const Node<TimeType, ElementType>& right) {
    return left.time < right.time;
  }
  friend bool operator<(const Node<TimeType, ElementType>& left,
                        const TimeType& right) {
    return left.time < right;
  }
  friend bool operator<(const TimeType& left,
                        const Node<TimeType, ElementType>& right) {
    return left < right.time;
  }
};

template <class NodeType>
class CircularSpan {
 public:
  using const_reference = const NodeType&;
  template <typename VV>
  CircularSpan(VV&& begin, size_t size)
      : begin_(std::forward<VV>(begin)),
        end_(std::next(begin, size)),
        size_(size) {}
  ~CircularSpan() {}
  constexpr typename boost::circular_buffer<NodeType>::const_iterator begin()
      const {
    return begin_;
  }
  constexpr typename boost::circular_buffer<NodeType>::const_iterator end()
      const {
    return end_;
  }
  constexpr const_reference operator[](size_t i) const noexcept {
    return ABSL_HARDENING_ASSERT(i < size()), *std::next(begin_, i);
  }
  size_t size() const noexcept { return size_; }

  // Span::front()
  //
  // Returns a reference to the first element of this span. The span must not
  // be empty.
  constexpr const_reference front() const noexcept {
    return ABSL_HARDENING_ASSERT(size() > 0), *begin();
  }

  // Span::back()
  //
  // Returns a reference to the last element of this span. The span must not
  // be empty.
  constexpr const_reference back() const noexcept {
    return ABSL_HARDENING_ASSERT(size() > 0), *std::prev(end(), 1);
  }

 private:
  typename boost::circular_buffer<NodeType>::const_iterator begin_;
  typename boost::circular_buffer<NodeType>::const_iterator end_;
  size_t size_;
};

template <class TimeType, class ElementType, class CircularSpan>
class ElementHistory {
 public:
  ElementHistory() = delete;

  explicit ElementHistory(const size_t history_capacity)
      : history_capacity_(history_capacity), buffer_(history_capacity) {
    Init();
  }

  const Node<TimeType, ElementType>& back() const { return buffer_.back(); }

  template <typename VV>
  bool Push(TimeType time, VV&& val) {
    if (!buffer_.empty() && buffer_.back().time >= time) {
      return false;
    }
    buffer_.push_back(Node<TimeType, ElementType>(time, std::forward<VV>(val)));
    return true;
  }

  boost::circular_buffer<Node<TimeType, ElementType>>* mutable_buffer() {
    return &buffer_;
  }

  size_t capacity() const { return history_capacity_; }
  size_t size() const { return buffer_.size(); }

  // Returns the span which start from the last element no greater than
  // start_t, or the first element greater than start_t.
  absl::StatusOr<CircularSpan> GetHistoryFrom(TimeType start_t) const {
    auto it = std::upper_bound(buffer_.begin(), buffer_.end(), start_t);
    if (it != buffer_.begin()) {
      std::advance(it, -1);
    }
    return CircularSpan(it, std::distance(it, buffer_.end()));
  }

  absl::StatusOr<CircularSpan> GetHistory() const {
    return CircularSpan(buffer_.begin(), buffer_.size());
  }

  auto begin() const noexcept { return buffer_.begin(); }
  auto end() const noexcept { return buffer_.end(); }
  auto begin() noexcept { return buffer_.begin(); }
  auto end() noexcept { return buffer_.end(); }

  // Remove items with time no greater than end_t.
  void PopBegin(TimeType end_t) {
    if (!buffer_.empty()) {
      auto it = std::upper_bound(buffer_.begin(), buffer_.end(), end_t);
      buffer_.erase_begin(std::distance(buffer_.begin(), it));
    }
  }

  bool empty() const { return buffer_.empty(); }

  void Clear() {
    buffer_.clear();
    Init();
  }

 private:
  const size_t history_capacity_;
  boost::circular_buffer<Node<TimeType, ElementType>> buffer_;

  void Init() {}
};

template <class ElementId, class TimeType, class ElementType,
          class ElementHistory>
class ElementsHistory {
 public:
  ElementsHistory() = delete;

  explicit ElementsHistory(size_t history_capacity)
      : history_capacity_(history_capacity) {
    Init();
  }

  bool Contains(const ElementId& key) const {
    return map_.find(key) != map_.end();
  }

  bool empty() const { return map_.empty(); }

  ElementHistory& operator[](const ElementId& key) {
    auto [iter, inserted] = map_.try_emplace(key, nullptr);
    if (inserted) {
      iter->second = std::make_unique<ElementHistory>(history_capacity_);
    }
    return *iter->second;
  }

  ElementHistory& at(const ElementId& key) const {
    return ABSL_HARDENING_ASSERT(Contains(key)), *(map_.at(key));
  }

  auto begin() const { return map_.begin(); }
  auto end() const { return map_.end(); }
  auto begin() { return map_.begin(); }
  auto end() { return map_.end(); }

  const ElementHistory* FindOrNull(const ElementId& key) const {
    const auto* res = ::qcraft::FindOrNull(map_, key);
    if (res == nullptr) {
      return nullptr;
    }
    return res->get();
  }

  void Erase(const ElementId& key) { map_.erase(key); }

  // Remove items with time no greater than end_t.
  void PopBegin(TimeType end_t) {
    for (auto it = map_.begin(); it != map_.end();) {
      auto history = it->second->GetHistory();
      if (it->second->empty() || !history.ok() ||
          history->back().time <= end_t) {
        map_.erase(it++);
      } else {
        it->second->PopBegin(end_t);
        it++;
      }
    }
  }

  void Clear() {
    map_.clear();
    Init();
  }

 private:
  const size_t history_capacity_;
  absl::flat_hash_map<ElementId, std::unique_ptr<ElementHistory>> map_;

  void Init() {}
};
}  // namespace elements_history
}  // namespace qcraft

#endif  // ONBOARD_UTILS_ELEMENTS_HISTORY_H_
