#include "onboard/prediction/container/objects_history.h"

#include <algorithm>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include "onboard/async/parallel_for.h"
#include "onboard/async/thread_pool.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace prediction {
void ObjectsHistory::Update(const ObjectsProto& objects_proto,
                            const LocalizationTransformProto& loc_transform,
                            ThreadPool* thread_pool) {
  FUNC_QTRACE();
  double end_t = MicroSecondsToSeconds(objects_proto.header().timestamp());
  CoordinateConverter target(loc_transform);
  std::map<ObjectIDType, bool> updated_objs;
  for (const auto& obj_proto : objects_proto.objects()) {
    updated_objs[obj_proto.id()] = false;
    const auto* hist = this->FindOrNull(obj_proto.id());
    if (hist != nullptr) {
      double last_t = hist->GetHistory()->back().time;
      // If the time difference between new obj and last observation is less
      // than min_dt_, do not add them.
      if (obj_proto.timestamp() - last_t < min_dt_) {
        continue;
      }
    }
    auto& history = (*this)[obj_proto.id()];
    history.Push(obj_proto.timestamp(), PredictionObject(obj_proto, target));
    updated_objs[obj_proto.id()] = true;
    end_t = std::min(obj_proto.timestamp(), end_t);
  }

  end_t -= max_hist_time_len_;

  const int num_objects = objects_proto.objects_size();
  ParallelFor(0, num_objects, thread_pool, [&](int i) {
    const auto& o = objects_proto.objects(i);
    if (updated_objs[o.id()]) {
      auto& history = (*this)[o.id()];
      auto& buffer = *history.mutable_buffer();
      history.PopBegin(end_t);
      // We don't have to transform the last object.
      for (int i = 0; i + 1 < buffer.size(); ++i) {
        buffer[i].val.TransformCoordinate(target);
      }
      history.UpdateStopTimeInfo();
    }
  });
  PopBegin(end_t);
}

}  // namespace prediction
}  // namespace qcraft
