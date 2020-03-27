#if 0
#include<iostream>
#include <memory>

using namespace std;
void func(shared_ptr<int>)
{
    ;
}
int main()
{
    int a = 5;
    //auto p = new int(5);
    shared_ptr<int> p = make_shared<int>(6);
    func(shared_ptr<int>(p));
    cout << *p << endl;
    return 0;
}
#endif

#include<iostream>
#include <memory>
using namespace std;
void func(shared_ptr<int>)
{
    ;
}
int main()
{
	#if 0
    int a = 5;
    auto p = &a;
    func(shared_ptr<int>(p));  // 释放保存在栈上的变量 core
    cout << *p << endl;
    #endif
    
    auto p = make_shared<int>(45);
	int* iPtr = p.get();
	
	{
	   shared_ptr<int>(iPtr);
	}
//delete iPtr;
	*p = 7;
	int value = *p; // Error! 内存已经被释放
	
	cout << value << endl;
    return 0;
}
