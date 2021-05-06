#include <algorithm>
#include <iostream>
#include <vector>
#include <list>
int f()
{ 
    static int i = 1;
    return i++;
}
 
int main()
{
    std::vector<double> v(5);
    std::generate(v.begin(), v.end(), f);
 
    std::cout << "v: ";
    for (auto iv: v) {
        std::cout << iv << " ";
    }
    std::cout << "\n";
 
    // Initialize with default values 0,1,2,3,4 from a lambda function
    // Equivalent to std::iota(v.begin(), v.end(), 0);
    std::generate(v.begin(), v.end(), [n = 3.0] () mutable { return n+=0.2; });
 std::cout << v.size()<< "\n";
    std::cout << "v: ";
    
    v.erase(v.begin() + 1);
    for (auto iv: v) {
        std::cout << iv << " ";
    }
    std::cout << "\n";
     std::cout << v.size()<< "\n";
   // vec.erase(vec.begin() + index);
   
   
   std::list<double> test;
   test.push_back(1);
   test.push_back(2);
   test.push_back(3);
   std::cout << *(std::prev(std::prev(test.end()))) << "\n";
}
