#ifndef ONBOARD_LITE_MODIFIER_TELEPORT_SDC_H_
#define ONBOARD_LITE_MODIFIER_TELEPORT_SDC_H_

#include <map>
#include <optional>
#include <string>

#include "onboard/lite/modifier/message_modifier_base.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft {

class TeleportSDC : public MessageModifier {
 public:
  explicit TeleportSDC(const MessageModifierContext& context)
      : MessageModifier(context) {
    const auto& modifier_or =
        GetModifierProto<TeleportSdcModifier>(context.execution);
    if (modifier_or.has_value() && !modifier_or->mod().object_id().empty()) {
      mod_ = modifier_or->mod();
    }
  }

  void MaybeModify(const std::string& channel,
                   google::protobuf::Message* lite_msg,
                   bool* drop_msg = nullptr) override {
    if (!mod_.has_value()) return;
    const auto& object_id = mod_->object_id();

    if (channel == "sensor_pose") {
      if (!last_replaced_object_.has_value() && drop_msg) {
        *drop_msg = true;
        return;
      } else if (last_replaced_object_.has_value()) {
        auto* sensor_pose = static_cast<PoseProto*>(lite_msg);
        sensor_pose->mutable_pos_smooth()->set_x(
            last_replaced_object_->pos().x());
        sensor_pose->mutable_pos_smooth()->set_y(
            last_replaced_object_->pos().y());
        sensor_pose->set_yaw(last_replaced_object_->yaw());
        sensor_pose->mutable_vel_smooth()->set_x(
            last_replaced_object_->vel().x());
        sensor_pose->mutable_vel_smooth()->set_y(
            last_replaced_object_->vel().y());
        sensor_pose->mutable_accel_smooth()->set_x(
            last_replaced_object_->accel().x());
        sensor_pose->mutable_accel_smooth()->set_y(
            last_replaced_object_->accel().y());
      }
    } else if (channel == "objects_proto") {
      auto* objects = static_cast<ObjectsProto*>(lite_msg);

      std::map<std::string, ObjectProto> removed_objs;
      google::protobuf::RepeatedPtrField<ObjectProto> objects_to_keep;
      for (auto& object : *objects->mutable_objects()) {
        if (object_id == object.id()) {
          last_replaced_object_ = object;
        } else {
          object.Swap(objects_to_keep.Add());
        }
      }
      objects->mutable_objects()->Swap(&objects_to_keep);
    }
  }

 private:
  std::optional<TeleportSdcModifier::Mod> mod_;
  std::optional<ObjectProto> last_replaced_object_;
};
REGISTER_MESSAGE_MODIFIER(TeleportSDC, teleport_sdc_modifier);

}  // namespace qcraft

#endif  // ONBOARD_LITE_MODIFIER_TELEPORT_SDC_H_
