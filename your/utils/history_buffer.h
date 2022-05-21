#ifndef ONBOARD_UTILS_HISTORY_BUFFER_H_
#define ONBOARD_UTILS_HISTORY_BUFFER_H_

#include <algorithm>
#include <chrono>
#include <deque>
#include <utility>

#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "glog/logging.h"

namespace qcraft {

template <typename T, typename TimeType, typename DurationType>
class HistoryBufferT {
 public:
  HistoryBufferT() = default;
  HistoryBufferT(const HistoryBufferT &other) : data_(other.data_) {}
  HistoryBufferT(HistoryBufferT &&other) : data_(std::move(other.data_)) {}
  HistoryBufferT &operator=(const HistoryBufferT &other) = default;

  using ValueType = T;
  using ElementType = std::pair<TimeType, ValueType>;
  using ContainerType = std::deque<ElementType>;
  using IteratorType = typename ContainerType::iterator;
  using ConstIteratorType = typename ContainerType::const_iterator;
  using ReverseIteratorType = typename ContainerType::reverse_iterator;
  using ConstReverseIteratorType =
      typename ContainerType::const_reverse_iterator;

  // Direct random access.
  TimeType time(int index) const { return data_[index].first; }
  const ValueType &value(int index) const { return data_[index].second; }
  ValueType &value(int index) { return data_[index].second; }
  const ElementType &operator[](int index) const { return data_[index]; }
  ElementType &operator[](int index) { return data_[index]; }

  void clear() { data_.clear(); }
  int size() const { return data_.size(); }
  bool empty() const { return data_.empty(); }

  ElementType &front() { return data_.front(); }
  const ElementType &front() const { return data_.front(); }
  ElementType &back() { return data_.back(); }
  const ElementType &back() const { return data_.back(); }

  void push_back(TimeType time, ValueType data) {
    data_.emplace_back(time, std::move(data));
  }
  void pop_back() { data_.pop_back(); }
  TimeType back_time() const { return data_.back().first; }
  const ValueType &back_value() const { return data_.back().second; }
  ValueType &back_value() { return data_.back().second; }

  void push_front(TimeType time, ValueType data) {
    if (!data_.empty()) {
      DCHECK_LE(time, front_time());
    }
    data_.emplace_front(time, std::move(data));
  }
  void pop_front() { data_.pop_front(); }
  TimeType front_time() const { return data_.front().first; }
  const ValueType &front_value() const { return data_.front().second; }
  ValueType &front_value() { return data_.front().second; }

  void ClearOlderThan(DurationType duration) {
    while (!empty() && back_time() - front_time() > duration) {
      pop_front();
    }
  }

  void ClearOlderThanRefTime(TimeType ref_time, DurationType duration) {
    while (!empty() && ref_time - front_time() > duration) {
      pop_front();
    }
  }

  void PushBackAndClearStale(TimeType time, ValueType data,
                             DurationType duration) {
    push_back(time, std::move(data));
    ClearOlderThan(duration);
  }

  void PushBackAndClearOlderThanRefTime(TimeType time, ValueType data,
                                        TimeType ref_time,
                                        DurationType duration) {
    push_back(time, std::move(data));
    ClearOlderThanRefTime(ref_time, duration);
  }

  // Overall statistics.
  DurationType duration() const {
    return data_.empty() ? DurationType() : back_time() - front_time();
  }

  // Range-based iteration.
  // Iterating over (time, value) pairs.
  IteratorType begin() { return data_.begin(); }
  IteratorType end() { return data_.end(); }
  ConstIteratorType begin() const { return data_.cbegin(); }
  ConstIteratorType end() const { return data_.cend(); }

  ReverseIteratorType rbegin() { return data_.rbegin(); }
  ReverseIteratorType rend() { return data_.rend(); }
  ConstReverseIteratorType rbegin() const { return data_.rbegin(); }
  ConstReverseIteratorType rend() const { return data_.rend(); }

  // Iterating over timestamps.
  template <typename IteratorTypeT>
  class ConstTimeIteratorT {
   public:
    typedef std::ptrdiff_t difference_type;
    typedef TimeType value_type;
    typedef const TimeType *pointer;
    typedef const TimeType &reference;
    typedef size_t size_type;
    typedef std::bidirectional_iterator_tag iterator_category;

    explicit ConstTimeIteratorT(IteratorTypeT it) : it_(std::move(it)) {}
    ConstTimeIteratorT &operator++() {
      ++it_;
      return *this;
    }
    ConstTimeIteratorT &operator--() {
      --it_;
      return *this;
    }
    const TimeType &operator*() const { return it_->first; }
    const TimeType *operator->() const { return &(it_->first); }
    bool operator==(const ConstTimeIteratorT &other) const {
      return it_ == other.it_;
    }
    bool operator!=(const ConstTimeIteratorT &other) const {
      return !((*this) == other);
    }

   private:
    IteratorTypeT it_;
  };
  using ConstTimeIterator = class ConstTimeIteratorT<ConstIteratorType>;
  using ConstReverseTimeIterator =
      class ConstTimeIteratorT<ConstReverseIteratorType>;

  ConstTimeIterator time_begin() { return ConstTimeIterator(begin()); }
  ConstTimeIterator time_end() { return ConstTimeIterator(end()); }

  ConstReverseTimeIterator time_rbegin() {
    return ConstReverseTimeIterator(rbegin());
  }
  ConstReverseTimeIterator time_rend() {
    return ConstReverseTimeIterator(rend());
  }

  // Note that these ranges are invalidated by the same operations that would
  // invalidate iterators.
  class ConstTimeRange {
   public:
    ConstTimeRange(ConstTimeIterator begin, ConstTimeIterator end)
        : begin_(begin), end_(end) {}
    ConstTimeIterator begin() const { return begin_; }
    ConstTimeIterator end() const { return end_; }

   private:
    ConstTimeIterator begin_;
    ConstTimeIterator end_;
  };

  const ConstTimeRange time_range() {
    return ConstTimeRange(time_begin(), time_end());
  }

  // Iterating over values.
  template <typename IteratorTypeT, typename ValueReferenceT,
            typename ValuePointerT>
  class ValueIteratorT {
   public:
    typedef std::ptrdiff_t difference_type;
    typedef ValueType value_type;
    typedef ValuePointerT pointer;
    typedef ValueReferenceT reference;
    typedef size_t size_type;
    typedef std::bidirectional_iterator_tag iterator_category;

    explicit ValueIteratorT(IteratorTypeT it) : it_(std::move(it)) {}
    ValueIteratorT &operator++() {
      ++it_;
      return *this;
    }
    ValueIteratorT &operator--() {
      --it_;
      return *this;
    }
    ValueReferenceT operator*() const { return it_->second; }
    ValuePointerT operator->() const { return &(it_->second); }
    bool operator==(const ValueIteratorT<IteratorTypeT, ValueReferenceT,
                                         ValuePointerT> &other) const {
      return it_ == other.it_;
    }
    bool operator!=(const ValueIteratorT<IteratorTypeT, ValueReferenceT,
                                         ValuePointerT> &other) const {
      return !((*this) == other);
    }

   private:
    IteratorTypeT it_;
  };
  using ValueIterator =
      class ValueIteratorT<IteratorType, ValueType &, ValueType *>;
  using ConstValueIterator =
      class ValueIteratorT<ConstIteratorType, const ValueType &,
                           const ValueType *>;
  using ReverseValueIterator =
      class ValueIteratorT<ReverseIteratorType, ValueType &, ValueType *>;
  using ConstReverseValueIterator =
      class ValueIteratorT<ConstReverseIteratorType, const ValueType &,
                           const ValueType *>;

  ValueIterator value_begin() { return ValueIterator(begin()); }
  ValueIterator value_end() { return ValueIterator(end()); }
  ConstValueIterator value_begin() const { return ConstValueIterator(begin()); }
  ConstValueIterator value_end() const { return ConstValueIterator(end()); }

  ReverseValueIterator value_rbegin() { return ReverseValueIterator(rbegin()); }
  ReverseValueIterator value_rend() { return ReverseValueIterator(rend()); }
  ConstReverseValueIterator value_rbegin() const {
    return ConstReverseValueIterator(rbegin());
  }
  ConstReverseValueIterator value_rend() const {
    return ConstReverseValueIterator(rend());
  }

  template <typename ValueIteratorT>
  class ValueRangeT {
   public:
    ValueRangeT(ValueIteratorT begin, ValueIteratorT end)
        : begin_(begin), end_(end) {}
    ValueIteratorT begin() const { return begin_; }
    ValueIteratorT end() const { return end_; }

   private:
    ValueIteratorT begin_;
    ValueIteratorT end_;
  };
  using ValueRange = class ValueRangeT<ValueIterator>;
  using ConstValueRange = class ValueRangeT<ConstValueIterator>;

  const ValueRange value_range() {
    return ValueRange(value_begin(), value_end());
  }
  const ConstValueRange value_range() const {
    return ConstValueRange(value_begin(), value_end());
  }

  // Time-based access and search.
  // Returns TimeType() if no entry with time at least time is found.
  TimeType FindTimeAtLeast(TimeType time) const {
    const auto it = FindElementWithTimeAtLeast(time);
    if (it == end()) return TimeType();
    return it->first;
  }

  // If there are multiple entries with exactly the specified time, the first is
  // returned. Returns size() if no entry with time at least time is found.
  int GetIndexWithTimeAtLeast(TimeType time) const {
    const auto it = FindElementWithTimeAtLeast(time);
    return it - begin();
  }

  // If there are multiple entries with exactly the specified time, the first is
  // returned. Returns nullptr if no entry with time at least time is found.
  const ValueType *GetValueWithTimeAtLeast(TimeType time) const {
    const auto it = FindElementWithTimeAtLeast(time);
    if (it == end()) return nullptr;
    return &it->second;
  }

  // Returns TimeType() if no entry with time at most time is found.
  TimeType FindTimeAtMost(TimeType time) const {
    const auto it = FindElementWithTimeAtMost(time);
    if (it == rend()) return TimeType();
    return it->first;
  }

  // If there are multiple entries with exactly the specified time, the last is
  // returned. Returns -1 if no entry with time at most time is found.
  int GetIndexWithTimeAtMost(TimeType time) const {
    const auto it = FindElementWithTimeAtMost(time);
    return static_cast<int>(data_.size()) - 1 - (it - rbegin());
  }

  // If there are multiple entries with exactly the specified time, the last is
  // returned. Returns nullptr if no entry with time at most time is found.
  const ValueType *GetValueWithTimeAtMost(TimeType time) const {
    const auto it = FindElementWithTimeAtMost(time);
    if (it == rend()) return nullptr;
    return &it->second;
  }

  TimeType FindTimeClosest(TimeType time) const {
    const auto it1 = FindElementWithTimeAtLeast(time);
    const auto it2 = FindElementWithTimeAtMost(time);
    if (it1 == end()) return it2->first;
    if (it2 == rend()) return it1->first;
    return fabs(it1->first - time) < fabs(it2->first - time) ? it1->first
                                                             : it2->first;
  }

  int GetIndexClosestToTime(TimeType time) const {
    const auto it1 = FindElementWithTimeAtLeast(time);
    const auto it2 = FindElementWithTimeAtMost(time);
    if (it1 == end()) return rend() - it2 - 1;
    if (it2 == rend()) return it1 - begin();
    return fabs(it1->first - time) < fabs(it2->first - time) ? it1 - begin()
                                                             : rend() - it2 - 1;
  }

  // Get the value with time that is closeset to the input reference time.
  const ValueType *GetValueWithClosestTime(const TimeType time) const {
    if (size() == 0) return nullptr;
    const auto it1 = FindElementWithTimeAtLeast(time);
    const auto it2 = FindElementWithTimeAtMost(time);
    if (it1 == end()) return &it2->second;
    if (it2 == rend()) return &it1->second;
    return fabs(it1->first - time) < fabs(it2->first - time) ? &it1->second
                                                             : &it2->second;
  }

  int Insert(TimeType time, ValueType value) {
    const auto it = FindElementWithTimeAtLeast(time);
    const int index = it - data_.begin();
    data_.insert(it, std::make_pair(time, std::move(value)));
    return index;
  }

 protected:
  static bool Compare(const ElementType &element0,
                      const ElementType &element1) {
    return element0.first < element1.first;
  }

  static bool ElementLessThanTime(const ElementType &element, TimeType time) {
    return element.first < time;
  }

  static bool ElementGreaterThanTime(const ElementType &element,
                                     TimeType time) {
    return element.first > time;
  }

  // Return an iterator whose time is greater than or equal to time.
  ConstIteratorType FindElementWithTimeAtLeast(TimeType time) const {
    return std::lower_bound(begin(), end(), time,
                            &HistoryBufferT::ElementLessThanTime);
  }

  // Return a reverse iterator whose time is less than or equal to time.
  ConstReverseIteratorType FindElementWithTimeAtMost(TimeType time) const {
    return std::lower_bound(rbegin(), rend(), time,
                            &HistoryBufferT::ElementGreaterThanTime);
  }

 private:
  ContainerType data_;
};

template <typename T>
using HistoryBuffer = class HistoryBufferT<T, double, double>;

template <typename T>
using HistoryBufferAbslTime =
    class HistoryBufferT<T, absl::Time, absl::Duration>;

template <typename T, typename Clock>
using HistoryBufferStdTime =
    class HistoryBufferT<T, std::chrono::time_point<Clock>,  // NOLINT: lint bug
                         typename Clock::duration>;

}  // namespace qcraft

#endif  // ONBOARD_UTILS_HISTORY_BUFFER_H_
