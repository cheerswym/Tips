#include <iostream>
#include <vector>
using namespace std;

int main() {
  vector<int> v;
  v.reserve(200);
  int last = 0;
  for (int i = 1; i <= 1e5; ++i) {
    v.push_back(1);
    if (last != (int)v.capacity()) {
      cout << v.capacity() << "    ";
      last = v.capacity();
    }
  }
  return 0;
}
