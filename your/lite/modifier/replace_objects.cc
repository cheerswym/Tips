#include "onboard/lite/modifier/replace_objects.h"

#include <algorithm>
#include <optional>

#include "absl/status/status.h"
#include "absl/time/time.h"
#include "glog/logging.h"
#include "google/protobuf/descriptor.h"
#include "offboard/agents/proto/sim_agent.pb.h"
#include "offboard/simulation/proto/virtual_object.pb.h"
#include "onboard/lite/lite_message_util.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/math/geometry/proto/box2d.pb.h"
#include "onboard/proto/lite_msg.pb.h"
#include "onboard/proto/sim_augmentation.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {

VirtualObjectConfProto CreateVirtualObjectConfProto(
    const ObjectProto& object, absl::Duration cur_offset,
    const agents::SimAgentConfigProto& sim_agent_config) {
  VirtualObjectConfProto vo;
  vo.set_start_sec(absl::ToDoubleSeconds(cur_offset));
  auto* perception_obj = vo.mutable_virtual_perception_object();
  perception_obj->set_id(object.id());
  // TODO(mike): make these configurable.
  perception_obj->set_length(std::max(4.0, object.bounding_box().length()));
  perception_obj->set_width(std::max(2.0, object.bounding_box().width()));
  perception_obj->set_type(object.type());

  auto* agent_conf = vo.mutable_agent_virtual_object()->mutable_config();
  agent_conf->CopyFrom(sim_agent_config);
  switch (agent_conf->AgentType_case()) {
    case agents::SimAgentConfigProto::kShyAgent:
      agent_conf->mutable_shy_agent()->set_object_id(object.id());
      break;
    case agents::SimAgentConfigProto::kIccAgent:
      agent_conf->mutable_icc_agent()->set_object_id(object.id());
      break;
    default:
      agent_conf->mutable_shy_agent()->set_object_id(object.id());
      break;
  }
  return vo;
}

bool NeedObject2Agent(const ReplaceObjectsModifier::Mod& mod,
                      absl::Duration cur_offset) {
  // Skip if we haven't hit the trigger time.
  const auto& start_offset = absl::Seconds(mod.start_offset_secs());
  const auto& end_offset = mod.has_end_offset_secs()
                               ? absl::Seconds(mod.end_offset_secs())
                               : absl::InfiniteDuration();
  if (cur_offset < start_offset || cur_offset > end_offset) return false;
  return true;
}

ReplaceObjects::ReplaceObjects(const MessageModifierContext& context)
    : MessageModifier(context) {
  const auto modifier_or =
      GetModifierProto<ReplaceObjectsModifier>(context.execution);
  CHECK(modifier_or.has_value());
  CHECK_GT(modifier_or->mods_size(), 0);
  for (const auto& mod : modifier_or->mods()) {
    switch (mod.ModType_case()) {
      case ReplaceObjectsModifier::Mod::kObjectId:
        CHECK(!ContainsKey(id_mods_, mod.object_id()));
        id_mods_[mod.object_id()] = mod;
        break;
      case ReplaceObjectsModifier::Mod::kObjectType:
        CHECK(!ContainsKey(type_mods_, mod.object_type()));
        type_mods_[mod.object_type()] = mod;
        break;
      default:
        break;
    }
  }
}

void ReplaceObjects::MaybeModify(const std::string& channel,
                                 google::protobuf::Message* lite_msg,
                                 bool* drop_msg) {
  if (channel != "objects_proto") return;

  auto* objects = static_cast<ObjectsProto*>(lite_msg);

  if (objects->scope() == ObjectsProto::SCOPE_VIRTUAL) return;

  CreateVirtualObjectsRequestProto create_vo_request;
  google::protobuf::RepeatedPtrField<ObjectProto> objects_to_keep;
  for (auto& object : *objects->mutable_objects()) {
    if (ContainsKey(replaced_obj_ids_, object.id())) continue;
    auto cur_offset = GetOffset(*lite_msg);
    auto* mod = FindOrNull(type_mods_, object.type());
    if (!mod) mod = FindOrNull(id_mods_, object.id());
    if (mod && NeedObject2Agent(*mod, cur_offset)) {
      replaced_obj_ids_.insert(object.id());
      create_vo_request.add_virtual_objects()->CopyFrom(
          CreateVirtualObjectConfProto(object, cur_offset,
                                       mod->sim_agent_config()));
    } else {
      object.Swap(objects_to_keep.Add());
    }
  }
  objects->mutable_objects()->Swap(&objects_to_keep);

  if (create_vo_request.virtual_objects_size() > 0) {
    auto* header = objects->mutable_header();
    auto* sim_header =
        header->MutableExtension(SimLiteHeaderProto::sim_lite_header);
    LiteMsgWrapper lite_msg_warpper;
    const auto& field_name =
        ConvertToLowerUnderscore(create_vo_request.GetDescriptor()->name());
    CHECK(
        LiteMsgConverter::Get()
            .SetLiteMsgByName(create_vo_request, field_name, &lite_msg_warpper)
            .ok());
    auto* request = sim_header->add_create_message_requests();
    request->set_channel(field_name);
    lite_msg_warpper.Swap(request->mutable_msg());
  }
}
}  // namespace qcraft
