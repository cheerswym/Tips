#ifndef ONBOARD_MATH_BAYESIAN_WORLD_VIEW_H_
#define ONBOARD_MATH_BAYESIAN_WORLD_VIEW_H_

#include <unordered_map>
#include <vector>

#include "absl/strings/str_format.h"
#include "onboard/lite/logging.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace bwv {

void InitializeBWVVec2d(BWVVec2dProto *, const Vec2dProto &);
void InitializeBWVScalar(BWVScalarProto *, const double);

std::vector<ObjectType> GetAllObjectTypes();
void InitializeObjectBWVFields(ObjectProto *object, bool debug = false);

double GetObjectCategoryProb(const ObjectProto &object_proto,
                             const ObjectType type, bool debug = false);
ObjectType GetObjectMostLikelyCategory(const ObjectProto &object_proto,
                                       bool debug = false);

const std::unordered_map<ObjectType, double> GetObjectCategoryDistr(
    const ObjectProto &object_proto);

}  // namespace bwv
}  // namespace qcraft

#endif  // ONBOARD_MATH_BAYESIAN_WORLD_VIEW_H_
