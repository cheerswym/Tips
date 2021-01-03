#include <iostream>
 
//负责打印最后一个实参
template <typename T>
void print(const T& arg){
    std::cout<<"print1"<<std::endl;
    std::cout<<arg<<std::endl;
}
 
//template <typename ...Types>
//void print(const Types&... args){
//    std::cout<<"print3"<<std::endl;
//    std::cout<<sizeof...(Types)<<std::endl;//得出传入参数包的大小
   // print(args...);//这样的话会一直调用自己，出现死循环
//}
 
template <typename T,typename...Types>
void print(const T& arg,const Types&...args){
    std::cout<<"print2"<<std::endl;
    std::cout<<arg<<std::endl;
    print(args...);
}
 
using namespace std;
 
int main()
{
    print(5.6,"hello",33,22);
    return 0;
}
