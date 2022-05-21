#ifndef ONBOARD_LITE_MODIFIER_KEEP_OBJECTS_H_
#define ONBOARD_LITE_MODIFIER_KEEP_OBJECTS_H_

#include <set>
#include <string>

#include "onboard/lite/modifier/message_modifier_base.h"

namespace qcraft {

// A message modifier that keeps specified perception objects.
class KeepObjects : public MessageModifier {
 public:
  explicit KeepObjects(const MessageModifierContext& context)
      : MessageModifier(context) {
    const auto modifier_or =
        GetModifierProto<KeepObjectsModifier>(context.execution);
    if (modifier_or.has_value()) {
      for (const auto& id : modifier_or->object_ids()) {
        ids_to_keep_.insert(id);
      }
    }
  }

  void MaybeModify(const std::string& channel,
                   google::protobuf::Message* lite_msg,
                   bool* drop_msg = nullptr) override {
    if (ids_to_keep_.empty() || channel != "objects_proto") return;

    auto* objects = static_cast<ObjectsProto*>(lite_msg);

    google::protobuf::RepeatedPtrField<ObjectProto> objects_to_keep;
    for (auto& object : *objects->mutable_objects()) {
      if (ContainsKey(ids_to_keep_, object.id())) {
        object.Swap(objects_to_keep.Add());
      }
    }
    objects->mutable_objects()->Swap(&objects_to_keep);
  }

 private:
  std::set<std::string> ids_to_keep_;
};
REGISTER_MESSAGE_MODIFIER(KeepObjects, keep_objects_modifier);

}  // namespace qcraft

#endif  // ONBOARD_LITE_MODIFIER_KEEP_OBJECTS_H_
