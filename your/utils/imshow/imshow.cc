#include "onboard/utils/imshow/imshow.h"

#include <QtCore/QThread>
#include <QtCore/QTimer>
#include <QtGui/QKeyEvent>
#include <QtGui/QPixmap>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGraphicsPixmapItem>
#include <QtWidgets/QHBoxLayout>
#include <map>
#include <mutex>
#include <vector>

#include "gflags/gflags.h"
#include "offboard/calibration/calibrator/zoom_view.h"
#include "onboard/base/macros.h"
#include "onboard/lite/logging.h"
#include "onboard/utils/imshow/asm_opencv.h"
#include "onboard/utils/keyboard_scanner.h"

DEFINE_bool(enable_keyboard, false,
            "If enabled, keystrokes would be captured by our program. This "
            "feature changes the terminal behavior. You have to Ctrl-C to stop "
            "your program, or the terminal window would be messed up and "
            "become unusable");

namespace {

using qcraft::KeyboardScanner;
using qcraft::calibration::ZoomView;

class ImageWindow : public QDialog {
 public:
  explicit ImageWindow(const char *title, int width = 0, int height = 0)
      : QDialog(nullptr), view_(new ZoomView), scene_(new QGraphicsScene) {
    setWindowFlags(Qt::Window);
    view_->setScene(scene_);
    pixmap_ = scene_->addPixmap(QPixmap());
    auto layout = new QHBoxLayout;
    layout->addWidget(view_);
    setLayout(layout);
    setWindowTitle(title);
    if (width || height) resize(width, height);
  }

  ~ImageWindow() { delete view_; }

  void imshow(const cv::Mat &mat) {
    pixmap_->setPixmap(ASM::cvMatToQPixmap(mat));
    show();
  }

  void keyPressEvent(QKeyEvent *e) override {
    switch (e->key()) {
      default:
        if (!key_press_cb_.empty()) {
          for (auto &cb : key_press_cb_) cb(e);
        }
    }
  }

  void registerKeyPressEvent(std::function<void(QKeyEvent *)> cb) {
    key_press_cb_.push_back(cb);
  }

 private:
  ZoomView *view_;
  QGraphicsScene *scene_;
  QGraphicsPixmapItem *pixmap_;
  std::vector<std::function<void(QKeyEvent *)>> key_press_cb_;
};

class WindowManager : public QObject {
 public:
  static WindowManager *Instance(bool create_if_needed = true) {
    static WindowManager *instance = nullptr;
    if (!instance && create_if_needed) {
      static std::once_flag flag;
      std::call_once(flag,
                     [&] { instance = new (std::nothrow) WindowManager(); });
    }
    return instance;
  }

  void createWindow(const std::string &name, int width, int height) {
    std::lock_guard<std::mutex> lck(wnd_mutex_);
    if (!windows_.count(name)) {
      auto wnd = new ImageWindow(name.c_str(), width, height);
      wnd->registerKeyPressEvent(std::bind(&WindowManager::keyPressEvent, this,
                                           std::placeholders::_1));
      windows_.emplace(name, wnd);
    }
  }
  void imshow(const std::string &name, const cv::Mat &mat) {
    std::lock_guard<std::mutex> lck(wnd_mutex_);
    if (!windows_.count(name)) return;
    windows_[name]->imshow(mat);
  }
  void destroyWindow(const std::string &winname) {
    std::lock_guard<std::mutex> lck(wnd_mutex_);
    if (windows_.count(winname)) {
      delete windows_[winname];
      windows_.erase(winname);
    }
  }
  void waitKey(int delay_us) {
    if (delay_us > 0) {
      // naive delay, might be a little longer than expected
      while ((delay_us -= 10) > 0 &&
             !(cmd_line_kbd_ && cmd_line_kbd_->WaitForAnyKey(100)) &&
             !guiKeyPressed())
        qApp->processEvents(QEventLoop::AllEvents);
    } else {
      while (!(cmd_line_kbd_ && cmd_line_kbd_->WaitForAnyKey(1000)) &&
             !guiKeyPressed()) {
        qApp->processEvents(QEventLoop::AllEvents);
        if (!pause_.load()) break;
      }
    }
  }

  void keyPressEvent(QKeyEvent *e) {
    switch (e->key()) {
      // more condition can be added
      case Qt::Key_Tab:  // avoid Alt+Tab to switch window
      case Qt::Key_Alt:  // avoid Alt+Tab to switch window
        break;
      case Qt::Key_Enter:   // Hit 'Enter' will toggle pause/play
      case Qt::Key_Return:  // Hit 'Enter' will toggle pause/play
        pause_.store(!pause_.load());
        // no break here
      default:
        key_pressed_.store(true);
    }
  }

  bool guiKeyPressed() {
    if (key_pressed_.load()) {
      key_pressed_.store(false);
      return true;
    }
    return false;
  }

 private:
  WindowManager() {
    // register kayboard 'Enter' as continue
    if (FLAGS_enable_keyboard) {
      cmd_line_kbd_ = new KeyboardScanner;
      cmd_line_kbd_->RegisterKeyCallback(qcraft::Key::ENTER, [this]() {
        QKeyEvent e{QEvent::KeyPress, Qt::Key_Enter, Qt::NoModifier};
        this->keyPressEvent(&e);
      });
    }

    if (!QApplication::instance()) {
      static int fake_argc = 1;
      char fake_argv1[] = "ImageWindowInternalQApp";
      char *fake_argv[] = {fake_argv1};
      new QApplication(fake_argc, fake_argv);
      moveToThread(QApplication::instance()->thread());
    }
  }

  std::map<std::string, ImageWindow *> windows_;
  std::mutex wnd_mutex_;
  KeyboardScanner *cmd_line_kbd_ = nullptr;
  std::atomic_bool key_pressed_{false};
  std::atomic_bool pause_{true};

  DISALLOW_COPY_AND_ASSIGN(WindowManager);
};

}  // namespace

namespace qcraft {
void imshow(const std::string &winname, const cv::Mat &mat) {
  auto wm = WindowManager::Instance();
  if (QThread::currentThread() != QApplication::instance()->thread()) {
    QMetaObject::invokeMethod(wm, "createWindow", Qt::BlockingQueuedConnection,
                              Q_ARG(const std::string &, winname));
  } else {
    wm->createWindow(winname, mat.cols + 24, mat.rows + 24);
  }
  wm->imshow(winname, mat);
}

void destroyWindow(const std::string &winname) {
  WindowManager::Instance()->destroyWindow(winname);
}

void waitKey(int delay_us) { WindowManager::Instance()->waitKey(delay_us); }
}  // namespace qcraft
