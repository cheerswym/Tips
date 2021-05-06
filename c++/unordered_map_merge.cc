#include <iostream>
#include <string>
#include <utility>
#include <unordered_map>
#include <vector>
 
// print out a std::pair
template <class Os, class U, class V>
Os& operator<<(Os& os, const std::pair<U,V>& p) {
    return os << '{' << p.first << ", " << p.second << '}';
}
 
// print out an associative container
template <class Os, class K, class V>
Os& operator<<(Os& os, const std::unordered_map<K, V>& v) {
    os << '[' << v.size() << "] { ";
    bool o{};
    for (const auto& e : v)
        os << (o ? ", " : (o = 1, "")) << e;
    return os << " }\n";
}
 
int main()
{
    std::unordered_map<std::string, int>
        p{ {"321", 6},{"213", 3}, {"123", 2}, {"111", 1}, {"222", 0}, {"222", 3} },
        q{ {"E", 6}, {"E", 7}, {"D", 5} }; // , {"A", 4}
 

    std::cout << "p: " << p << "q: " << q;

std::unordered_map<int, std::string>
        pp{ { 6,"E"},{ 3,"C"}, {2,"B"}, {1,"A"}, {0,"A"} };
 std::cout << "pp: " << pp;
    //p.merge(q);
 
    std::cout << "p.merge(q);\n" << "p: " << p << "q: " << q;
    if (q.empty()) {
		std::cout << "empty\n";
	}
    
    
    std::vector<int> test(10,9);
    for (auto t : test) {
		std::cout << t << ", ";
		}
		std::cout << "\n";
     int a = test.at(110);
     std::cout << a<< "\n";

}
