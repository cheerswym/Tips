#ifndef ONBOARD_PLANNER_PLAN_PLAN_TASK_H_
#define ONBOARD_PLANNER_PLAN_PLAN_TASK_H_

#include <optional>
#include <utility>
#include <vector>

#include "common/proto/lane_point.pb.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/common/global_pose.h"
#include "onboard/planner/plan/proto/plan_task.pb.h"

namespace qcraft::planner {

constexpr double kMinCruiseLength = 5.0;  // m.

struct PlanTaskDestination {
  // One of
  std::optional<std::vector<mapping::ElementId>> parking_spots;
  std::optional<std::vector<mapping::LanePoint>> lane_points;
  std::optional<GlobalPose> global_pose;

  void FromProto(const PlanTaskDestinationProto &proto) {
    if (!proto.parking_spots_id().empty()) {
      parking_spots.emplace();
      parking_spots->reserve(proto.parking_spots_id().size());
      for (const auto id : proto.parking_spots_id()) {
        parking_spots->push_back(id);
      }
    }
    if (!proto.lane_points().empty()) {
      lane_points.emplace();
      lane_points->reserve(proto.lane_points().size());
      for (const auto &lp : proto.lane_points()) {
        lane_points->emplace_back(lp);
      }
    }
    if (proto.has_global_pose()) {
      global_pose = ConvertToGlobalPose(proto.global_pose());
    }
  }

  void ToProto(PlanTaskDestinationProto *proto) const {
    if (parking_spots.has_value()) {
      proto->mutable_parking_spots_id()->Reserve(parking_spots->size());
      for (const auto id : parking_spots.value()) {
        proto->add_parking_spots_id(id);
      }
    }
    if (lane_points.has_value()) {
      proto->mutable_lane_points()->Reserve(lane_points->size());
      for (const auto &lp : lane_points.value()) {
        lp.ToProto(proto->add_lane_points());
      }
    }
    if (global_pose.has_value()) {
      *proto->mutable_global_pose() =
          ConvertToGlobalPoseProto(global_pose.value());
    }
  }
};

struct PlanTaskDestinationInfo {
  PlanTaskDestination dest;
  double end_speed;  // 0 for stop.
  ReachDestinationCondition condition;

  // For uturn task.
  // NOTE: as lane path requires a semantic map manager, here we define a
  // LanePathProto instead.
  std::optional<mapping::LanePathProto> uturn_ref_lane_path;

  void FromProto(const PlanTaskDestinationInfoProto &proto) {
    dest.FromProto(proto.destination());
    end_speed = proto.end_speed();
    condition = proto.condition();

    if (proto.has_uturn_ref_lane_path()) {
      uturn_ref_lane_path = proto.uturn_ref_lane_path();
    }
  }

  void ToProto(PlanTaskDestinationInfoProto *proto) const {
    dest.ToProto(proto->mutable_destination());
    proto->set_end_speed(end_speed);
    *proto->mutable_condition() = condition;

    if (uturn_ref_lane_path.has_value()) {
      *proto->mutable_uturn_ref_lane_path() = uturn_ref_lane_path.value();
    }
  }
};

class PlanTask {
 public:
  PlanTask(PlanTaskType type, PlanTaskDestinationInfo dest_info)
      : type_(type), destination_info_(std::move(dest_info)) {}

  explicit PlanTask(const PlanTaskProto &proto) { FromProto(proto); }

  PlanTaskType type() const { return type_; }
  const PlanTaskDestinationInfo &destination_info() const {
    return destination_info_;
  }

  void FromProto(const PlanTaskProto &proto) {
    type_ = proto.type();
    destination_info_.FromProto(proto.destination_info());
  }

  void ToProto(PlanTaskProto *proto, int index) const {
    proto->set_index(index);
    proto->set_type(type_);
    destination_info_.ToProto(proto->mutable_destination_info());
  }

 protected:
  PlanTaskType type_;
  PlanTaskDestinationInfo destination_info_;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_PLAN_TASK_H_
