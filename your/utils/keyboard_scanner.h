#ifndef ONBOARD_UTILS_KEYBOARD_SCANNER_H_
#define ONBOARD_UTILS_KEYBOARD_SCANNER_H_

/// USAGE EXAMPLE:

// #include "onboard/utils/keyboard_scanner.h"
// #include <iostream>
// int main() {
//   KeyboardScanner ks;
//   std::atomic_bool go_on = false, next = false;
//   ks.RegisterKeyForTrue(Key::RIGHT, &next);
//   ks.RegisterKeyForToggle(Key::SPACE, &go_on);
//   int counter = 0;
//   while (true) {
//     ks.WaitForAnyTrue(go_on, next);
//     std::cout << "Happy day: " << counter++ << std::endl;
//     usleep(1e5);
//     next.store(false);
//   }
//   return 0;
// }

#include <unistd.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <thread>
#include <vector>

namespace qcraft {
class Key {
 public:
  enum {
    UP = 65,
    DOWN = 66,
    LEFT = 68,
    RIGHT = 67,
    SPACE = 32,
    ENTER = 10,
    DEL = 51,
    HOME = 72,
    END = 70,
    INS = 50,
    PAGEUP = 53,
    PAGEDOWN = 54,
    BACKSPACE = 127,
  };
};

class KeyboardScanner {
 public:
  KeyboardScanner();

  ~KeyboardScanner() { thread_.reset(); }

  void RegisterKeyCallback(char key, std::function<void(void)> cb) {
    std::lock_guard<std::mutex> lck(cb_mutex_);
    callbacks_[key].push_back(cb);
  }

  void ClearKeyCallbacks(char key) {
    std::lock_guard<std::mutex> lck(cb_mutex_);
    if (callbacks_.count(key)) callbacks_.erase(key);
  }

  bool WaitForAnyKey(int delay = 0) {
    std::unique_lock<std::mutex> lck(cv_mutex_);
    if (delay)
      return cv_.wait_for(lck, std::chrono::microseconds(delay)) ==
             std::cv_status::no_timeout;
    else
      cv_.wait(lck);
    return true;
  }

 private:
  std::map<char, std::vector<std::function<void(void)>>> callbacks_;
  std::mutex cb_mutex_;
  std::unique_ptr<std::thread> thread_;
  std::condition_variable cv_;
  std::mutex cv_mutex_;
};
}  // namespace qcraft

#endif  // ONBOARD_UTILS_KEYBOARD_SCANNER_H_
