#ifndef ONBOARD_NODE_NODE_QUERY_STRUCT_H_
#define ONBOARD_NODE_NODE_QUERY_STRUCT_H_

#include <string>
#include <vector>

namespace qcraft {

enum NodeQueryCmdType : int16_t {
  QUERY_NODE_INPUTS = 0,
};

struct NodeQeuryRequest {
  std::string full_node_name;  // requesting node name
  NodeQueryCmdType query_cmd;
};

struct NodeInputMsgDecl {
  std::string field_name;
  std::string channel;
  std::string domain;
};

struct NodeQeuryResponse {
  std::vector<NodeInputMsgDecl> node_inputs;
};

}  // namespace qcraft

#endif  // ONBOARD_NODE_NODE_QUERY_STRUCT_H_
