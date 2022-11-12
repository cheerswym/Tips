#include <iostream>

// Template Function with a Type T
// During instantiation, this T will be replaced by the data type of argument.
template <class T>
T maxNum(T a, T b) {
  return (a > b ? a : b);
}

int main() {
  int x = 5, y = 2;
  float a = 4.5, b = 1.3;

  std::cout << maxNum<int>(x, y) << "\n";
  std::cout << maxNum<float>(a, b) << "\n";
  std::cout << maxNum(a, b) << "\n";
  std::cout << maxNum(x, y) << "\n";

  return 0;
}
