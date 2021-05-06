#include <iostream>
#include<mutex>


  
#define DECLARE_SINGLETON(classname)                                      \
 public:                                                                  \
  static classname* Instance() {              \
    static classname* instance = nullptr;                                 \
    if (!instance) {                                  \
      instance = new (std::nothrow) classname();                     \
    }                                                                     \
    return instance;                                                      \
  }                                                                       \
                                                                          \
                                                                          \
 private:                                                                 \
  classname();                                                            \
  
  
class PlannerParams {
 public:
  const int &planner_params() const { return a; }
  static const int &Get() {
    return Instance()->planner_params();
  }
  
 private:
 int a;
 
  DECLARE_SINGLETON(PlannerParams);
};

PlannerParams::PlannerParams(){
	std::cout<< "test...." << std::endl;
 a = 10;
}
	//PlannerParams PlannerParams::instance=nullptr;
int main() {
	
	
	std::cout<< PlannerParams::Get() << std::endl;
	std::cout<< PlannerParams::Get() << std::endl;
	std::cout<< PlannerParams::Get() << std::endl;
}
