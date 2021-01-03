#include <iostream>
using namespace std;
class Base
{
public:
    Base(){};
    virtual void Show(){cout<<"This is Base calss\n";}
};
class Derived:public Base
{
public:
    Derived(){};
    void Show(){cout<<"This is Derived class\n";}
    //void print(){cout<<"This is Derived class print\n";}
};
class Derived2:public Base
{
public:
    Derived2(){};
    void Show(){cout<<"This is Derived class\n";}
    void print(){cout<<"This is Derived class print\n";}
};
int main()
{
    //Base *base ;
   // Derived *der = new Derived;
    //base = dynamic_cast<Base*>(der); //正确，但不必要。
    //base = der; //先上转换总是安全的
    //base->Show();
    //reinterpret_cast
    Base *base = new Derived();
    Derived *der;
    der = reinterpret_cast<Derived*>(base);
    if(der == nullptr) {
	cout<< "failed\n";
	return 1;
	}
    der->Show();
	 double a = 2.333;
	 char b;
	 b = static_cast<char>(char)(a);
cout<< sizeof(b);
}
