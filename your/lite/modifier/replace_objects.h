#ifndef ONBOARD_LITE_MODIFIER_REPLACE_OBJECTS_H_
#define ONBOARD_LITE_MODIFIER_REPLACE_OBJECTS_H_

#include <set>
#include <string>
#include <unordered_map>

#include "google/protobuf/message.h"
#include "offboard/simulation/proto/scenario.pb.h"
#include "onboard/global/registry.h"
#include "onboard/lite/modifier/message_modifier_base.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {

class ReplaceObjects : public MessageModifier {
 public:
  explicit ReplaceObjects(const MessageModifierContext& context);

  void MaybeModify(const std::string& channel,
                   google::protobuf::Message* lite_msg,
                   bool* drop_msg = nullptr) override;

 private:
  // A map from object id to desired modification.
  std::unordered_map<std::string, ReplaceObjectsModifier::Mod> id_mods_;
  std::unordered_map<ObjectType, ReplaceObjectsModifier::Mod> type_mods_;
  std::set<int> target_object_types_;
  std::set<std::string> replaced_obj_ids_;
  ReplaceObjectsModifier modifier_conf_;
};
REGISTER_MESSAGE_MODIFIER(ReplaceObjects, replace_objects_modifier);

}  // namespace qcraft

#endif  // ONBOARD_LITE_MODIFIER_REPLACE_OBJECTS_H_
