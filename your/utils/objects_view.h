#ifndef ONBOARD_UTILS_OBJECTS_VIEW_H_
#define ONBOARD_UTILS_OBJECTS_VIEW_H_

#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "glog/logging.h"
#include "onboard/lite/check.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {

// Thread-safe
// Keep the view of objects proto by scope, same scope objects will be overrided
// by latest objects.
class ObjectsView {
 public:
  // Update objects by scope.
  void UpdateObjects(ObjectsProto::Scope scope,
                     std::shared_ptr<const ObjectsProto> objects);

  // Export all scopes' objects proto into one proto.
  std::shared_ptr<const ObjectsProto> ExportAllObjectsProto() const;

  // Export objects by scope.
  std::shared_ptr<const ObjectsProto> ExportObjectsProtoByScope(
      ObjectsProto::Scope scope) const;

  // Iterate over objects.
  void ForAllObjects(
      const std::function<void(const ObjectProto& object_proto)>& func) const;

  void ForObjectsInScopes(
      const std::set<ObjectsProto::Scope>& scopes,
      const std::function<void(const ObjectProto& object_proto)>& func) const;

  void ForEachScope(const std::function<void(ObjectsProto::Scope,
                                             const ObjectsProto&)>& func) const;

 private:
  mutable absl::Mutex mu_;
  std::map<ObjectsProto::Scope, std::shared_ptr<const ObjectsProto>>
      scope_to_objects_ GUARDED_BY(mu_);
};

}  // namespace qcraft

#endif  // ONBOARD_UTILS_OBJECTS_VIEW_H_
