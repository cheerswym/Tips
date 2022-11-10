#include <chrono>
#include <iostream>
#include <thread>

void thread_function(const std::string &msg) {
  std::cout << msg << " thread function\n";
}

void bar() {
  // simulate expensive operation
  // std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "thread bar\n";
}

class MyFunctor {
public:
  void operator()() { std::cout << "functor\n"; }
};

int main() {
  std::cout << "main thread\n";
  std::cout << std::boolalpha;

  std::thread t(thread_function, "hello");
  std::thread t1(bar);
  // t.detach();
  // t1.detach();
  // Observers.
  std::cout << t.get_id() << "\n";
  std::cout << t.joinable() << "\n";
  std::cout << t.native_handle() << "\n";
  std::cout << t.hardware_concurrency() << "\n";
  // t.swap(t1);
  if (t.joinable()) {
    t.join();
  }
  if (t1.joinable()) {
    t1.join();
  }

  MyFunctor functor;
  std::thread t2(functor);
  // Use () to warp.
  std::thread t3((MyFunctor()));
  if (t2.joinable()) {
    t2.join();
  }
  if (t3.joinable()) {
    t3.join();
  }

  std::cout << "done\n";
  return 0;
}
