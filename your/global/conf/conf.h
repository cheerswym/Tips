#ifndef ONBOARD_GLOBAL_CONF_CONF_H_
#define ONBOARD_GLOBAL_CONF_CONF_H_

#include <string>

#include "onboard/global/conf/proto/configuration.pb.h"

namespace qcraft {
namespace conf {

constexpr char kDefaultConfigurationName[] = "alpaca";

// Gets the configuration proto (with all message fields populated instead of
// the filename one-of counterparts).
//
// If the configuration with the given name was loaded before (the last
// conf() call was with the same name), no file loading will happen
// again. If conf() is called with a different name or is called for
// the first time, it will load the configuration with the given name from file
// //onboard/conf/[conf_name].pb.txt.
//
// The loader also processes certain oneof fields specially: if the
// ConfigurationProto (or any of its contained messages) has a oneof field
// consisting of exactly two fields, where one of the two fields is a message
// field named XXX and the other is a string field named XXX_file, then the
// string field is treated as the path (relative to //onboard/conf/ as well) to
// a text proto file (.pb.txt) containing the message field's proto type. If the
// directly loaded ConfigurationProto has the string field filled in, the loader
// will load that file and populate the message field automatically (clearing
// the string field as they are part of a same oneof). This proceeds
// recursively into all messages contained inside ConfigurationProto.
const ConfigurationProto &conf(
    const std::string &conf_name = kDefaultConfigurationName);

}  // namespace conf
}  // namespace qcraft

#endif  // ONBOARD_GLOBAL_CONF_CONF_H_
