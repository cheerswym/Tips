#include <set>
#include <iostream>

class GeometryForm {
 public:
  virtual double length() const = 0;

  virtual ~GeometryForm() {}
};

class StraightLine : GeometryForm {

};

int main()
{
    std::set<int> c = {1, 2, 3,3, 4, 5, 6, 7, 8, 9};
 
    // erase all odd numbers from c
    for(auto it = c.begin(); it != c.end(); ++it) {
        if(*it % 2 == 1)
            it = c.erase(it);
       // else
           // ++it;
    }
 
    for(int n : c) {
        std::cout << n << ' ';
    }
}
