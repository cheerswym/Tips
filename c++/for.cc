#include <iostream>
#include <variant>
#include <vector>

class A {
	public:
	
	A a;

};
int main()
{
  int foo= 42;
	
  int&& baz = std::move(foo); 
 
  std::cout << baz<< std::endl;
  std::cout << foo<< std::endl;
  
  
  //std::string source;
  std::string des= "12345";
  std::string source = std::move(des);
  std::cout << source<< std::endl;
  std::cout << des<< std::endl;
  
  return 0;
}
