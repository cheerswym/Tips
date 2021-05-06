#include "onboard/planner/speed/time_buffer_decider.h"

#include <algorithm>

#include "onboard/lite/logging.h"
#include "onboard/math/piecewise_linear_function.h"

namespace qcraft {
namespace planner {

namespace {

// TODO(renjie): Use more complicated logic.
double ComputePassTime(const StBoundary& st_boundary) {
  constexpr double kPassTimeForPedBase = 1.5;              // s.
  constexpr double kPassTimeForCyclistLowPriority = 1.2;   // s.
  constexpr double kPassTimeForCyclistHighPriority = 0.4;  // s.
  constexpr double kPassTimeForVehicleLowPriority = 1.0;   // s.
  constexpr double kPassTimeForVehicleHighPriority = 0.2;  // s.

  const auto& overlap_meta = *st_boundary.overlap_meta();
  const auto get_pass_time_by_overlap_priority =
      [](StOverlapMetaProto::OverlapPriority priority,
         double pass_time_for_high_priority,
         double pass_time_for_low_priority) {
        if (priority == StOverlapMetaProto::UNKNOWN_PRIORITY) {
          // Only happens when object ENTER/CROSS/INTERFERE the path from first
          // path point. Under such cases, the overlap should be ignored and no
          // pass time will be added.
          return 0.0;
        } else if (priority == StOverlapMetaProto::LOW ||
                   priority == StOverlapMetaProto::EQUAL) {
          return pass_time_for_low_priority;
        } else {
          return pass_time_for_high_priority;
        }
      };

  double res = 0.0;
  if (st_boundary.object_type() == StBoundaryProto::PEDESTRIAN) {
    // Prediction uncertainty is growing with distance and time.
    const double first_overlap_average_s =
        0.5 * (st_boundary.bottom_left_point().s() +
               st_boundary.upper_left_point().s());
    const PiecewiseLinearFunction<double> kPedPassTimeSFactor(
        {0.0, 30.0, 60.0, 90.0}, {1.0, 1.1, 1.2, 1.3});
    const PiecewiseLinearFunction<double> kPedPassTimeMinTFactor(
        {0.0, 1.0, 3.0, 5.0, 10.0}, {1.0, 1.1, 1.2, 1.3, 1.5});
    res = kPassTimeForPedBase * kPedPassTimeSFactor(first_overlap_average_s) *
          kPedPassTimeMinTFactor(st_boundary.min_t());
  } else if (st_boundary.object_type() == StBoundaryProto::CYCLIST) {
    res = get_pass_time_by_overlap_priority(overlap_meta.priority(),
                                            kPassTimeForCyclistHighPriority,
                                            kPassTimeForCyclistLowPriority);
  } else {
    // Vehicle type.
    res = get_pass_time_by_overlap_priority(overlap_meta.priority(),
                                            kPassTimeForVehicleHighPriority,
                                            kPassTimeForVehicleLowPriority);
  }

  // Pass time should be no larger than min_t.
  return std::min(st_boundary.min_t(), res);
}

// TODO(renjie): Use more complicated logic.
double ComputeYieldTime(const StBoundary& st_boundary) {
  constexpr double kYieldTimeForPedBase = 1.0;              // s.
  constexpr double kYieldTimeForCyclistLowPriority = 1.2;   // s.
  constexpr double kYieldTimeForCyclistHighPriority = 0.7;  // s.
  constexpr double kYieldTimeForVehicleLowPriority = 1.0;   // s.
  constexpr double kYieldTimeForVehicleHighPriority = 0.5;  // s.

  const auto& overlap_meta = *st_boundary.overlap_meta();
  const auto get_yield_time_by_overlap_priority_and_pattern =
      [](StOverlapMetaProto::OverlapPattern pattern,
         StOverlapMetaProto::OverlapPriority priority,
         double yield_time_for_high_priority,
         double yield_time_for_low_priority) {
        if (priority == StOverlapMetaProto::UNKNOWN_PRIORITY) {
          // Only happens when object CROSS/INTERFERE the path from first
          // path point, or the object LEAVE the path, or the attern is ENTER /
          // LEAVE but prediction horizon may be shorter than planning horizon.
          // For the former, the overlap should be ignored and no pass time will
          // be added; for the latter, we should deem the object as having high
          // priority.
          if (pattern == StOverlapMetaProto::LEAVE ||
              pattern == StOverlapMetaProto::ENTER ||
              pattern == StOverlapMetaProto::STAY) {
            return yield_time_for_low_priority;
          } else {
            return 0.0;
          }
        } else if (priority == StOverlapMetaProto::LOW ||
                   priority == StOverlapMetaProto::EQUAL) {
          return yield_time_for_low_priority;
        } else {
          return yield_time_for_high_priority;
        }
      };

  double res = 0.0;
  if (st_boundary.object_type() == StBoundaryProto::PEDESTRIAN) {
    // Prediction uncertainty is growing with distance and time.
    const double last_overlap_average_s =
        0.5 * (st_boundary.bottom_right_point().s() +
               st_boundary.upper_right_point().s());
    const PiecewiseLinearFunction<double> kPedYieldTimeSFactor(
        {0.0, 30.0, 60.0, 90.0}, {1.0, 1.1, 1.2, 1.3});
    const PiecewiseLinearFunction<double> kPedYieldTimeMinTFactor(
        {0.0, 1.0, 3.0, 5.0, 10.0}, {1.0, 1.1, 1.2, 1.3, 1.5});
    res = kYieldTimeForPedBase * kPedYieldTimeSFactor(last_overlap_average_s) *
          kPedYieldTimeMinTFactor(st_boundary.max_t());
  } else if (st_boundary.object_type() == StBoundaryProto::CYCLIST) {
    res = get_yield_time_by_overlap_priority_and_pattern(
        overlap_meta.pattern(), overlap_meta.priority(),
        kYieldTimeForCyclistHighPriority, kYieldTimeForCyclistLowPriority);
  } else {
    // Vehicle type.
    res = get_yield_time_by_overlap_priority_and_pattern(
        overlap_meta.pattern(), overlap_meta.priority(),
        kYieldTimeForVehicleHighPriority, kYieldTimeForVehicleLowPriority);
  }

  return res;
}

}  // namespace

void DecideTimeBuffersForStBoundary(StBoundaryWithDecision* st_boundary_wd) {
  QCHECK_NOTNULL(st_boundary_wd);

  const auto& st_boundary = *st_boundary_wd->raw_st_boundary();
  // Only add time buffers for st-boundaries having overlap meta.
  if (!st_boundary.overlap_meta().has_value()) return;
  const auto& overlap_meta = *st_boundary.overlap_meta();

  if (st_boundary_wd->decision_type() == StBoundaryProto::IGNORE) return;

  QCHECK(st_boundary.object_type() == StBoundaryProto::VEHICLE ||
         st_boundary.object_type() == StBoundaryProto::CYCLIST ||
         st_boundary.object_type() == StBoundaryProto::PEDESTRIAN)
      << StBoundaryProto::ObjectType_Name(st_boundary.object_type());

  double pass_time = 0.0;
  double yield_time = 0.0;

  // If the object is not on the path initially, we need to add a pass time
  // buffer for it.
  if (overlap_meta.pattern() == StOverlapMetaProto::ENTER ||
      overlap_meta.pattern() == StOverlapMetaProto::CROSS ||
      overlap_meta.pattern() == StOverlapMetaProto::INTERFERE) {
    pass_time = ComputePassTime(st_boundary);
  }

  // If the object leaves the path finally, we need to add a yield time buffer
  // for it.
  // NOTE(renjie): We also add ENTER and STAY pattern here because it may be
  // considered to disappear on the path after max_t by downstream speed
  // algorithms.
  if (overlap_meta.pattern() == StOverlapMetaProto::ENTER ||
      overlap_meta.pattern() == StOverlapMetaProto::STAY ||
      overlap_meta.pattern() == StOverlapMetaProto::LEAVE ||
      overlap_meta.pattern() == StOverlapMetaProto::CROSS ||
      overlap_meta.pattern() == StOverlapMetaProto::INTERFERE) {
    yield_time = ComputeYieldTime(st_boundary);
  }

  st_boundary_wd->SetTimeBuffer(pass_time, yield_time);
}

}  // namespace planner
}  // namespace qcraft
