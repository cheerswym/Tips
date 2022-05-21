#ifndef ONBOARD_LITE_SERVICE_SERVER_BASE_H_
#define ONBOARD_LITE_SERVICE_SERVER_BASE_H_

#include <string>

namespace qcraft {

class ServerBase {
 public:
  explicit ServerBase(const std::string& server_name)
      : server_name_(server_name) {}

  virtual ~ServerBase() {}

  const std::string& ServerName() const { return server_name_; }

 protected:
  std::string server_name_;
};

}  // namespace qcraft

#endif  // ONBOARD_LITE_SERVICE_SERVER_BASE_H_
