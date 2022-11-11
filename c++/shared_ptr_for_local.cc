#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

struct Base {
  Base() { std::cout << "  Base::Base()\n"; }
  int a = 0;
  // Note: non-virtual destructor is OK here
  ~Base() { std::cout << "  Base::~Base()\n"; }
};

std::shared_ptr<Base> test = nullptr;
void SetPtr(const Base& base) { test = std::make_shared<Base>(base); }
void BuildObject() {
  Base base;
  base.a = 3;
  SetPtr(base);
}
int main() {
  BuildObject();
  std::cout << "a: " << test->a << "\n";
  return 0;
}
