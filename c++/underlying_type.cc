#include <iostream>
#include <type_traits>
 
enum e1 {};
enum class e2 {};
enum class e3: unsigned {};
enum class e4: int {};
 
int main() {
 
std::underlying_type_t<e4>;
 

}
