#include "onboard/utils/keyboard_scanner.h"

#include <termios.h>
#include <unistd.h>

#include "onboard/utils/thread_util.h"
namespace {
char getch() {
  char buf = 0;
  struct termios old = {0};
  if (tcgetattr(0, &old) < 0) perror("tcsetattr()");
  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;
  if (tcsetattr(0, TCSANOW, &old) < 0) perror("tcsetattr ICANON");
  if (read(0, &buf, 1) < 0) perror("read()");
  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  if (tcsetattr(0, TCSADRAIN, &old) < 0) perror("tcsetattr ~ICANON");
  return (buf);
}
}  // namespace

namespace qcraft {
KeyboardScanner::KeyboardScanner()
    : thread_(new std::thread([this]() {
        QSetThreadName("KeyboardScanner");
        while (true) {
          char ch = getch();
          {
            std::lock_guard<std::mutex> lck(cb_mutex_);
            if (callbacks_.count(ch)) {
              auto &cb_vec = callbacks_[ch];
              for (auto &cb : cb_vec) cb();
            }
          }
          {
            std::lock_guard<std::mutex> lck(cv_mutex_);
            cv_.notify_all();
          }
        }
      })) {}
}  // namespace qcraft
