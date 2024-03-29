// unordered_multimap::erase
#include <iostream>
#include <unordered_map>

int main ()
{
  std::unordered_multimap<std::string,std::string> myumm = {
         {"strawberry","red"},
         {"banana","yellow"},
         {"orange","orange"},
         {"lemon","yellow"},
         {"apple","red"},
         {"apple","green"},
         {"pear","green"},
  };


  // erase examples:
 // myumm.erase ( myumm.begin() );     // erasing by iterator
 // myumm.erase ("apple");             // erasing by key (erases 2 elements)
 // myumm.erase ( myumm.find("orange"), myumm.end() ); // erasing by range

  // show content:
  for ( auto& x: myumm )
    std::cout << x.first << ": " << x.second << std::endl;
    
    
	int a = 10;
	bool b = true;
	if (b && a++) {
	std::cout << "qwer\n";
	}
std::cout << a;
  return 0;
}
