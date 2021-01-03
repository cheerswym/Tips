#include <iostream>
#include <variant>
 
struct NoDefConstr
{
    NoDefConstr(int i)
    {
        std::cout << "NoDefConstr::NoDefConstr(int) called\n";
    }
};
 
int main()
{
	
	
	std::variant<int, std::string> var{"hi"}; // initialized with string alternative
	std::cout << var.index(); // prints 1
	var = 42; // now holds int alternative
	std::cout << var.index(); // prints 0

	try {
	std::string s = std::get<std::string>(var); // access by type
	int i = std::get<0>(var); // access by index
	}
	catch (const std::bad_variant_access& e) { // in case a wrong type/index is used
		}

	
    //std::variant<NoDefConstr, int> v1; // ERROR: can’t default construct first type
 
    return 0;
}
