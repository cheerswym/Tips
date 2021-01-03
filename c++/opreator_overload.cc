#include <iostream>
using std::cout;
using std::endl;
using std::ostream;

class Tree {
  int height;
public:
  Tree(int treeHeight) : height(treeHeight) {
    cout << __func__ << "(), this = " << this << endl;
  }
  ~Tree() { cout << "~Tree()\n"; }
#if 0
  friend ostream&
  operator<<(ostream& os, const Tree* t) {
    return os << "Tree height is: "
              << t->height << endl;
  }
#else
  friend ostream&
  operator<<(ostream& os, const Tree& t) {
    return os << "Tree height is: "
              << t.height << endl;
  }
#endif
}; 


class Vector2 {
	public:
    int x,y;
 
    friend std::ostream& operator<<(std::ostream& os, const Vector2& rhs) {
        os<<"Vector2( "<<rhs.x<<" , "<<rhs.y<<" )";
        return os;
    }
};

int main() {
  Tree* t = new Tree(40);
  delete t;
  t = nullptr;
  delete t;

	Vector2 v1,v2;
	v1.x = 10;
	v1.y = 39;
	std::cout<<v1<<std::endl;
	std::cout<<v2<<std::endl;
}
