#ifndef ONBOARD_LITE_MODIFIER_REMOVE_OBJECTS_H_
#define ONBOARD_LITE_MODIFIER_REMOVE_OBJECTS_H_

#include <set>
#include <string>

#include "onboard/lite/modifier/message_modifier_base.h"

namespace qcraft {

// A message modifier that removes specified perception objects.
class RemoveObjects : public MessageModifier {
 public:
  explicit RemoveObjects(const MessageModifierContext& context)
      : MessageModifier(context) {
    const auto modifier_or =
        GetModifierProto<RemoveObjectsModifier>(context.execution);
    if (modifier_or.has_value()) {
      for (const auto& id : modifier_or->object_ids()) {
        ids_to_remove_.insert(id);
      }
    }
  }

  void MaybeModify(const std::string& channel,
                   google::protobuf::Message* lite_msg,
                   bool* drop_msg = nullptr) override {
    if (ids_to_remove_.empty() || channel != "objects_proto") return;

    auto* objects = static_cast<ObjectsProto*>(lite_msg);

    google::protobuf::RepeatedPtrField<ObjectProto> objects_to_keep;
    for (auto& object : *objects->mutable_objects()) {
      if (ids_to_remove_.count(object.id()) == 0) {
        object.Swap(objects_to_keep.Add());
      }
    }
    objects->mutable_objects()->Swap(&objects_to_keep);
  }

 private:
  std::set<std::string> ids_to_remove_;
};
REGISTER_MESSAGE_MODIFIER(RemoveObjects, remove_objects_modifier);

}  // namespace qcraft

#endif  // ONBOARD_LITE_MODIFIER_REMOVE_OBJECTS_H_
