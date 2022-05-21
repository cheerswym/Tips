#ifndef ONBOARD_PERCEPTION_OBSTACLE_GRID_H_
#define ONBOARD_PERCEPTION_OBSTACLE_GRID_H_

#include <string>
#include <utility>
#include <vector>

#include "onboard/async/async_util.h"
#include "onboard/async/thread_pool.h"
#include "onboard/global/spin_lock.h"
#include "onboard/global/trace.h"
#include "onboard/perception/obstacle.h"

namespace qcraft {
// This class is designed for coordinate convertation between rc and smooth
// coordinate. Need to use after initialized.
class ObstacleRCCoordConverter {
 public:
  ObstacleRCCoordConverter(int width, int height)
      : width_(width), height_(height), row_offset_(0), col_offset_(0) {}

  void InitializeWithPose(const VehiclePose& pose) {
    row_offset_ = RoundToInt(pose.y * (1.0f / Obstacle::kDiameter));
    col_offset_ = RoundToInt(pose.x * (1.0f / Obstacle::kDiameter));
  }

  std::pair<int, int> CoordToRC(Vec2f coord) const {
    const int col = width_ / 2 +
                    FloorToInt(coord.x() * (1.0f / Obstacle::kDiameter)) -
                    col_offset_;
    const int row = height_ / 2 +
                    FloorToInt(coord.y() * (1.0f / Obstacle::kDiameter)) -
                    row_offset_;
    return {row, col};
  }

  Vec2d RCToCoord(Vec2i rc) const {
    const double x =
        (rc.y() + col_offset_ - width_ * 0.5 + 0.5) * Obstacle::kDiameter;
    const double y =
        (rc.x() + row_offset_ - height_ * 0.5 + 0.5) * Obstacle::kDiameter;
    return {x, y};
  }

  int width() const { return width_; }
  int height() const { return height_; }
  int row_offset() const { return row_offset_; }
  int col_offset() const { return col_offset_; }

 private:
  int width_;
  int height_;
  int row_offset_;
  int col_offset_;
};

struct ObstacleInfo {
  // A slim representation of std::vector<int> with size 16B instead of 24B.
  class __attribute__((packed)) IndexVector {
   public:
    IndexVector() {}
    ~IndexVector() { delete buffer_; }

    void push_back(int val) {
      if (size_ == capacity_) {
        if (capacity_ == 0) capacity_ = 1;
        int* new_buffer = new int[capacity_ * 2];
        if (buffer_ != nullptr) {
          memcpy(new_buffer, buffer_, size_ * sizeof(int));
          delete buffer_;
        }
        buffer_ = new_buffer;
        capacity_ *= 2;
      }
      buffer_[size_] = val;
      size_++;
    }
    void clear() { size_ = 0; }

    bool empty() const { return size_ == 0; }
    int size() const { return size_; }
    const int* begin() const { return buffer_; }
    const int* end() const { return buffer_ + size_; }
    int* begin() { return buffer_; }
    int* end() { return buffer_ + size_; }
    int operator[](size_t i) const { return buffer_[i]; }

   private:
    int* buffer_ = nullptr;
    uint16_t size_ = 0;
    uint16_t capacity_ = 0;
  };

  IndexVector point_indices;
  int16_t dist_to_curb_cm = 0;

  enum Type : uint8_t {
    kUnknown = 0,
    kGround,
    kObstacle,
  };
  Type type = kUnknown;

  SpinLock spin_lock;

  void Clear() {
    point_indices.clear();
    dist_to_curb_cm = 0;
    type = kUnknown;
  }

  static std::string Type_Name(const Type type) {
    switch (type) {
      case kUnknown:
        return "kUnknown";
      case kGround:
        return "kGround";
      case kObstacle:
        return "kObstacle";
      default:
        QLOG(FATAL) << "Should not reach here.";
    }
  }
};
static_assert(sizeof(ObstacleInfo) == 16);

class ObstacleGrid {
 public:
  ObstacleGrid(int width, int height)
      : width_(width),
        height_(height),
        rc_coord_converter_(width, height),
        info_grid_(width * height),
        validity_grid_(width * height) {}

  ~ObstacleGrid() { reset_future_.Wait(); }

  void InitializeWithPose(const VehiclePose& pose) {
    // Wait until the reset is done. Reset is used to clear point indices for
    // each grid cell.
    reset_future_.Wait();
    rc_coord_converter_.InitializeWithPose(pose);
  }

  std::pair<int, int> CoordToRC(Vec2f coord) const {
    return rc_coord_converter_.CoordToRC(coord);
  }

  Vec2d RCToCoord(Vec2i rc) const { return rc_coord_converter_.RCToCoord(rc); }

  bool IsValid(int row, int col) const {
    return validity_grid_[row * width_ + col];
  }
  void SetIsValid(int row, int col) {
    validity_grid_[row * width_ + col] = true;
  }
  ObstacleInfo& operator()(int row, int col) {
    return info_grid_[row * width_ + col];
  }
  const ObstacleInfo& operator()(int row, int col) const {
    return info_grid_[row * width_ + col];
  }

  void ResetAsync() {
    reset_future_ = ScheduleFuture(ThreadPool::DisposalPool(), [=] {
      SCOPED_QTRACE("ObstacleGrid::ResetAsync");
      for (auto& info : info_grid_) {
        info.Clear();
      }
      validity_grid_.assign(validity_grid_.size(), 0);
    });
  }

  const ObstacleRCCoordConverter& rc_coord_converter() const {
    return rc_coord_converter_;
  }
  int width() const { return width_; }
  int height() const { return height_; }

 private:
  const int width_;
  const int height_;
  ObstacleRCCoordConverter rc_coord_converter_;
  std::vector<ObstacleInfo> info_grid_;
  // Use char instead of bool to guarantee thread safety.
  std::vector<char> validity_grid_;
  // A future used to reset info_grid.
  Future<void> reset_future_;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_OBSTACLE_GRID_H_
