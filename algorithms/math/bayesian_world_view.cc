#include "onboard/math/bayesian_world_view.h"
#include "absl/status/statusor.h"

namespace qcraft {
namespace bwv {

void InitializeBWVVec2d(BWVVec2dProto* bwv_vec, const Vec2dProto& vec) {
  auto mean = bwv_vec->mutable_mean();
  mean->set_x(vec.x());
  mean->set_y(vec.y());
  auto* cov = bwv_vec->mutable_cov();
  cov->clear_m();
  for (int i = 0; i < 4; ++i) {
    cov->add_m(0.0);
  }
}

void InitializeBWVScalar(BWVScalarProto* bwv_scalar, const double value) {
  bwv_scalar->set_mean(value);
  bwv_scalar->set_var(0);
}

std::vector<ObjectType> GetAllObjectTypes() {
  std::vector<ObjectType> types;
  const auto desc = ObjectType_descriptor();
  for (int i = 0; i < desc->value_count(); ++i) {
    const auto type_name = desc->value(i)->name();
    ObjectType type;
    ObjectType_Parse(type_name, &type);
    types.push_back(type);
  }
  return types;
}

void InitializeObjectBWVFields(ObjectProto* object, bool debug) {
  if (object->has_bwv()) return;
  ObjectProto::BWV* object_bwv = object->mutable_bwv();
  if (object->has_type()) {
    auto type_distr = object_bwv->mutable_type_distr();
    for (const auto type : GetAllObjectTypes()) {
      ObjectProto::BWV::CategoryDistr::ObjectTypeProb* entry =
          type_distr->add_entries();
      entry->set_type(type);
      entry->set_prob(object->type() == type ? 1.0 : 0.0);
    }
  }

  if (!object->has_id() && debug) {
    QLOG(WARNING) << absl::StrFormat(
        "Not sure if this is serious or not... But I found an object without "
        "any id!!!!");
  }

  if (object->has_pos()) {
    InitializeBWVVec2d(object_bwv->mutable_pos(), object->pos());
  } else if (debug) {
    QLOG(WARNING) << absl::StrFormat("I cannot find pos for %s!", object->id());
  }
  if (object->has_yaw()) {
    InitializeBWVScalar(object_bwv->mutable_yaw(), object->yaw());
  } else if (debug) {
    QLOG(WARNING) << absl::StrFormat("I cannot find yaw for %s!", object->id());
  }
  if (object->has_yaw_rate()) {
    InitializeBWVScalar(object_bwv->mutable_yaw_rate(), object->yaw_rate());
  } else if (debug) {
    QLOG(WARNING) << absl::StrFormat("I cannot find yaw_rate for %s!",
                                     object->id());
  }
  if (object->has_vel()) {
    InitializeBWVVec2d(object_bwv->mutable_vel(), object->vel());
  } else if (debug) {
    QLOG(WARNING) << absl::StrFormat("I cannot find vel for %s!", object->id());
  }
  if (object->has_accel()) {
    InitializeBWVVec2d(object_bwv->mutable_accel(), object->accel());
  } else if (debug) {
    QLOG(WARNING) << absl::StrFormat("I cannot find accel for %s!",
                                     object->id());
  }

  object_bwv->set_parked(object->has_parked() ? object->parked() : 0.0);
  object_bwv->set_offroad(object->has_offroad() ? object->offroad() : 0.0);
}

double GetObjectCategoryProb(const ObjectProto& object_proto,
                             const ObjectType type, bool debug) {
  QCHECK(object_proto.has_bwv());
  QCHECK(object_proto.bwv().has_type_distr());
  const auto& entries = object_proto.bwv().type_distr().entries();
  if (entries.size() <= type) {
    if (debug) {
      QLOG(WARNING) << absl::StrFormat(
          " I do not find data type for %s (%d)",
          ObjectType_descriptor()->value(type)->name(), type);
    }
    return 0.0;
  }
  return entries[type].prob();
}

ObjectType GetObjectMostLikelyCategory(const ObjectProto& object_proto,
                                       bool debug) {
  QCHECK(object_proto.has_bwv());
  QCHECK(object_proto.bwv().has_type_distr());
  ObjectType probable_type;
  double max_prob = 0.0;
  const auto desc = ObjectType_descriptor();
  for (int i = 0; i < desc->value_count(); ++i) {
    const auto& type_name = desc->value(i)->name();
    ObjectType type;
    ObjectType_Parse(type_name, &type);
    const auto& entries = object_proto.bwv().type_distr().entries();
    // Histrorical data may not contain latest object type.
    if (entries.size() <= type) {
      if (debug) {
        QLOG(WARNING) << absl::StrFormat(" I do not find data type for %s (%d)",
                                         type_name, type);
      }
      continue;
    }
    double prob = entries[type].prob();
    if (prob > max_prob) {
      probable_type = type;
    }
  }
  return probable_type;
}

const std::unordered_map<ObjectType, double> GetObjectCategoryDistr(
    const ObjectProto& object_proto) {
  QCHECK(object_proto.has_bwv() && object_proto.bwv().has_type_distr());
  std::unordered_map<ObjectType, double> object_type_distr;
  const auto desc = ObjectType_descriptor();
  const auto& entries = object_proto.bwv().type_distr().entries();
  for (int i = 0; i < desc->value_count(); ++i) {
    const auto type_name = desc->value(i)->name();
    ObjectType type;
    ObjectType_Parse(type_name, &type);
    object_type_distr[type] = entries[type].prob();
  }
  return object_type_distr;
}

}  // namespace bwv
}  // namespace qcraft.
