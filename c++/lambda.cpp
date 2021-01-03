#include <algorithm>
#include <iostream>
#include <vector>
#include <limits> 
using namespace std;

int main() {

  vector<std::pair<int, int>> vec;
  vec.push_back({1,2});
  vec.push_back({2,2});
  vec.push_back({3,2});
  vec.push_back({4,2});
  vec.push_back({5,2});
  vec.push_back({6,2});
  
  auto it_low = std::upper_bound(vec.begin(), vec.end(), 4, 
		[](const int b, std::pair<int, int> a) { return a.first < b ; });

  cout << it_low - vec.begin() << endl;
  double a = std::numeric_limits<double>::infinity();
  double b = std::numeric_limits<double>::infinity()+123;
 cout <<  a - b << endl;
}
