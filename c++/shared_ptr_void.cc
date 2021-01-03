#include <memory>
#include <iostream>
#include <vector>

class test {
public:
  test() {
    std::cout << "Test created" << std::endl;
  }
  ~test() {
    std::cout << "Test destroyed" << std::endl;
  }

};

int main() {
  std::cout << "At begin of main.\ncreating std::vector<std::shared_ptr<void>>" 
            << std::endl;
  std::vector<std::shared_ptr<void>> v;
  {
    std::cout << "Creating test" << std::endl;
    v.push_back( std::shared_ptr<test>( new test() ) );
    std::cout << "Leaving scope" << std::endl;
  }
  std::cout << "Leaving main" << std::endl;
  return 0;

}
