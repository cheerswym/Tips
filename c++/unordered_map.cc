// unordered_map::at
#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>

class Node {
	public:
	Node() = default;
	~Node(){};
};

//Node::Node() { a = 9; }
int main ()
{
  Node node;
  std::unordered_map<std::string,int> mymap = {
                { "Mars", 3000},
                { "Saturn", 60000},
                { "Jupiter", 70000 } };

  mymap.at("Mars") = 3396;
  mymap.at("Saturn") += 272;
  mymap.at("Jupiter") = mymap.at("Saturn") + 9638;
  mymap["asdf"]=23;

  for (auto& x: mymap) {
    std::cout << x.first << ": " << x.second << std::endl;
  }
  std::string time_stamp = "1234567.9776";
int a = std::stoi(time_stamp);
std::cout << a << "\n";


int i=10; 
while(i-- > 0);
std::cout << "i=" << i << "\n";


std::unordered_set<int> decisions;
decisions.insert(1);
decisions.insert(2);
decisions.insert(3);
decisions.insert(6);
for (auto it = decisions.begin(); it != decisions.end(); it++) {
//std::cout << "distance: " <<std::distance(decisions.begin(), it)   << "\n";
std::cout << *it << "\n";
}

  return 0;
}
