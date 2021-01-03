#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
#include <map>
#include <set>
#include <unordered_set>

void test(const int& input, int * const output) {
std::cout << "input="<<input<< std::endl;
*output = 5;
std::cout << "input2="<<input<< std::endl;
}


int main(){
	int a = 3;
	test(a,&a);
	std::cout << "a="<<a<< std::endl;





	
std::vector<std::tuple<double, double, double>> workspaceBoundary; 

for (const auto &bound : workspaceBoundary) {
    if (std::get<0>(bound) > 10.0) {

    }
}
int64_t header_time = 123123456;

double ret = static_cast<double>(static_cast<int64_t>(std::round(header_time / 10000))/10.0);
std::cout << ret << std::endl;

 std::string pi = "pi is " + std::to_string(3.100000);
 std::cout<< "fdfds\t"<< pi << '\n';
 
 
 std::map<int,int> mymap;
 
  mymap[1]=202;
  if(mymap.count(1)==0)
  mymap[1]=101;
  mymap[3]=302;
  
  for (std::map<int,int>::iterator it=mymap.begin(); it!=mymap.end(); ++it)
    std::cout << it->first << " => " << it->second << '\n';
    
  
  std::unordered_set<int64_t> objects_set;
  objects_set.emplace(4);
  objects_set.emplace(2);
  objects_set.emplace(4);
  objects_set.emplace(1);
  for (std::unordered_set<int64_t>::iterator it=objects_set.begin(); it!=objects_set.end(); ++it)
    std::cout << *it<<  '\n';
    
}
