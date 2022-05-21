#ifndef ONBOARD_LITE_SERVICE_CLIENT_BASE_H_
#define ONBOARD_LITE_SERVICE_CLIENT_BASE_H_

#include <string>

namespace qcraft {
class ClientBase {
 public:
  explicit ClientBase(const std::string& server_name)
      : server_name_(server_name) {}

  virtual ~ClientBase() {}

  const std::string& ServerName() const { return server_name_; }

  virtual bool ServerIsReady() const { return true; }

 protected:
  std::string server_name_;
};

}  // namespace qcraft

#endif  // ONBOARD_LITE_SERVICE_CLIENT_BASE_H_
