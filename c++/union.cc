#include <iostream>
using namespace std;
 
union test
{
     char mark;
     long num;
     float score;
}a;

void Get(char c) {
	cout<< "test:"<<c<<endl;
}
int main()
{
     // cout<<a<<endl; // wrong
     a.mark = 'b';
     cout<<a.mark<<endl; // 输出'b'
     cout<<a.num<<endl; // 98 字符'b'的ACSII值
     cout<<a.score<<endl; // 输出错误值
      Get(a.mark);
 
     a.num = 10;
     cout<<a.mark<<endl; // 输出换行 非常感谢suxin同学的指正
     cout<<a.num<<endl; // 输出10
     cout<<a.score<<endl; // 输出错误值
 
     a.score = 10.0;
     cout<<a.mark<<endl; // 输出空
     cout<<a.num<<endl; // 输出错误值
     cout<<a.score<<endl; // 输出10
 
     return 0;
}
