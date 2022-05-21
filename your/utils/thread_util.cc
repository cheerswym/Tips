#include "onboard/utils/thread_util.h"

namespace qcraft {

void QSetThreadName(const std::string& name) {
  pthread_setname_np(pthread_self(), name.c_str());
}

}  // namespace qcraft
