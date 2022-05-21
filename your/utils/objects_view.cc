#include "onboard/utils/objects_view.h"

#include <utility>

#include "onboard/utils/map_util.h"

namespace qcraft {

void ObjectsView::UpdateObjects(ObjectsProto::Scope scope,
                                std::shared_ptr<const ObjectsProto> objects) {
  absl::MutexLock l(&mu_);
  scope_to_objects_[scope] = std::move(objects);
}

std::shared_ptr<const ObjectsProto> ObjectsView::ExportAllObjectsProto() const {
  absl::ReaderMutexLock l(&mu_);
  if (scope_to_objects_.size() == 1) {
    // Short-cut.
    return scope_to_objects_.begin()->second;
  }
  auto objects = std::make_shared<ObjectsProto>();
  for (const auto &[_, objs] : scope_to_objects_) {
    for (const auto &obj : objs->objects()) {
      *objects->add_objects() = obj;
    }
  }
  return objects;
}

std::shared_ptr<const ObjectsProto> ObjectsView::ExportObjectsProtoByScope(
    ObjectsProto::Scope scope) const {
  absl::ReaderMutexLock l(&mu_);
  const auto *objects = FindOrNull(scope_to_objects_, scope);
  if (objects == nullptr) return nullptr;
  return *objects;
}

void ObjectsView::ForAllObjects(
    const std::function<void(const ObjectProto &object_proto)> &func) const {
  ForObjectsInScopes({ObjectsProto::SCOPE_REAL, ObjectsProto::SCOPE_VIRTUAL,
                      ObjectsProto::SCOPE_AV},
                     func);
}

void ObjectsView::ForObjectsInScopes(
    const std::set<ObjectsProto::Scope> &scopes,
    const std::function<void(const ObjectProto &object_proto)> &func) const {
  absl::ReaderMutexLock l(&mu_);
  for (const ObjectsProto::Scope scope : scopes) {
    const auto *objects = FindOrNull(scope_to_objects_, scope);
    if (objects == nullptr) continue;
    for (const ObjectProto &object : (*objects)->objects()) {
      func(object);
    }
  }
}

void ObjectsView::ForEachScope(
    const std::function<void(ObjectsProto::Scope, const ObjectsProto &)> &func)
    const {
  absl::ReaderMutexLock l(&mu_);
  for (const auto &kv : scope_to_objects_) {
    func(kv.first, *kv.second);
  }
}

}  // namespace qcraft
