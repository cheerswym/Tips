#include "onboard/global/conf/conf.h"

#include <memory>
#include <mutex>
#include <utility>

#include "absl/synchronization/mutex.h"
#include "glog/logging.h"
#include "google/protobuf/message.h"
#include "onboard/lite/check.h"
#include "onboard/utils/file_util.h"

namespace qcraft {
namespace conf {

namespace {

ConfigurationProto g_configuration_;
std::unique_ptr<std::string> g_loaded_configuration_name_;
absl::Mutex g_configuration_mutex_;

constexpr char kConfigurationRootPath[] = "onboard/conf/";

// This function searches through all of the oneof fields in the message, and
// processes those that consists of exactly two fields, a message field named
// XXX and a string field named XXX_string. If the string field is assigned and
// not the corresponding message field in the oneof, then treat that string as
// a filename of a text proto file (.pb.txt) and load that file as the message
// type of the corresponding message field, clearing the string field in the
// meantime (because they are part of a oneof). Also recurse into all message
// fields (not just the loaded messages) in this message to potentially load
// their files.
//
// At the end of this function, message will have all their message fields
// populated (whether loaded from the string counterpart in their oneofs or they
// come already loaded).
void RecursivelyLoadFilesIntoProtos(google::protobuf::Message *message) {
  const google::protobuf::Reflection *reflection = message->GetReflection();
  const google::protobuf::Descriptor *descriptor = message->GetDescriptor();
  for (int i = 0; i < descriptor->oneof_decl_count(); ++i) {
    const google::protobuf::OneofDescriptor *oneof_descriptor =
        descriptor->oneof_decl(i);
    if (oneof_descriptor->field_count() == 2) {
      const google::protobuf::FieldDescriptor *field0 =
          oneof_descriptor->field(0);
      const google::protobuf::FieldDescriptor *field1 =
          oneof_descriptor->field(1);
      if (field0->name().size() > field1->name().size()) {
        std::swap(field0, field1);
      }

      // If this is a field named XXX, then field0 should be a message field
      // named XXX and field1 should be a string field named XXX_file.
      if (field1->name() == field0->name() + "_file" &&
          field0->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE &&
          field1->type() == google::protobuf::FieldDescriptor::TYPE_STRING) {
        // If the string field is populated instead of the message field, load
        // the file specified by the string field into the message field and
        // clear the string field now.
        if (reflection->HasField(*message, field1)) {
          QCHECK(!reflection->HasField(*message, field0));
          const std::string filename = reflection->GetString(*message, field1);
          reflection->ClearField(message, field1);
          google::protobuf::Message *field0_message =
              reflection->MutableMessage(message, field0);
          const bool success = file_util::TextFileToProto(
              kConfigurationRootPath + filename, field0_message);
          QCHECK(success) << kConfigurationRootPath + filename;
        }
      }
    }
  }

  for (int i = 0; i < descriptor->field_count(); ++i) {
    const google::protobuf::FieldDescriptor *field = descriptor->field(i);
    if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      RecursivelyLoadFilesIntoProtos(
          reflection->MutableMessage(message, field));
    }
  }
}

void LoadConfigurationThreadUnsafe(const std::string &conf_name) {
  g_loaded_configuration_name_ = std::make_unique<std::string>(conf_name);
  const std::string filename = kConfigurationRootPath + conf_name + ".pb.txt";
  const bool success = file_util::TextFileToProto(filename, &g_configuration_);
  QCHECK(success);

  RecursivelyLoadFilesIntoProtos(&g_configuration_);
}

}  // namespace

const ConfigurationProto &conf(const std::string &conf_name) {
  absl::MutexLock l(&g_configuration_mutex_);
  if (g_loaded_configuration_name_ == nullptr ||
      *g_loaded_configuration_name_ != conf_name) {
    LoadConfigurationThreadUnsafe(conf_name);
    VLOG(2) << "Loaded configuration for " << conf_name << ":" << std::endl
            << g_configuration_.DebugString();
  }

  // Returning the reference is thread-safe here because of the one-time loading
  // guarantee.
  return g_configuration_;
}

}  // namespace conf
}  // namespace qcraft
