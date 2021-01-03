#include <iostream>
#include <utility>
#include <string>
#include <map>
 
int main()
{
    std::map<std::pair<int,int>, std::string> m;
    
    m.emplace(std::make_pair(std::make_pair(2,4), std::string("a")));
	
	m.emplace(std::make_pair(std::make_pair(1,4), std::string("b")));
    for (const auto &p : m) {
        std::cout << p.first.first << " => " << p.second << '\n';
    }
   std::cout <<  std::stoi("a214") << "\n";
}
