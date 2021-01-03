#include <string>
#include <iostream>
#include <string_view>

void* operator new(std::size_t count){
  std::cout<<"分配了堆内存"<<count<<"字节."<<std::endl;
  return malloc(count);
}

void operator delete(void* p){
   std::cout<<"释放堆内存:"<<p<<std::endl;
   free(p);
}

void show_str(const std::string &str){
   std::cout<<std::endl;
   std::cout<<"show_str()临时变量tmp初始化"<<std::endl;
   std::string tmp=str;
   printf("str副本的地址:%p\n",str.c_str());
   printf("tmp副本的地址:%p\n",tmp.c_str());
}

void show_stv(std::string_view stv){
   printf("string_view对象str的内部地址:%p\n",stv.data());
}

int main(void){
  std::cout<<std::endl;
  std::cout<<"-------初始化string对象-------"<<std::endl;
  std::string you="How do you do~,My name is peter!";
  std::cout<<std::endl;
  
  std::cout<<"-------初始化string_view对象-------"<<std::endl;
  std::string_view stv(you.c_str(),you.size());
  std::cout<<std::endl;
  std::cout<<"---------show_str---------"<<std::endl;
  printf("main函数:you副本中的字符串地址:%p\n",you.c_str());
  show_str(you);

  std::cout<<std::endl;
  std::cout<<"字符串字面量直接传参方式"<<std::endl;
  show_str("How do you do~,My name is peter!");

  std::cout<<std::endl;
  std::cout<<"---------show_stv----------"<<std::endl;
  std::cout<<std::endl;
  std::cout<<"字符串字面量直接传参方式"<<std::endl;
  show_stv(stv);

  std::cout<<std::endl;
  std::cout<<"数组字符串传参方式"<<std::endl;
  show_stv("How do you do~,My name is peter!");

  std::cout<<std::endl;

  return 0; 
}
