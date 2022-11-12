#include <iostream>

class Person {
public:
  Person(const std::string &name) : name_(name) {}
  const std::string &GetName() const { return name_; }

private:
  const std::string &name_;
};

class Paper {
public:
  Paper(const std::string *name) : name_(name) {}
  const std::string &GetName() const { return *name_; }

private:
  const std::string *name_;
};
int main() {
  Person person("Ox333");
  std::cout << person.GetName() << "\n"; // Error!
  std::operator<<(std::operator<<(std::cout, person.GetName()), "\n");
  std::string msg = "vfd";
  Paper paper(&msg);
  std::cout << paper.GetName() << "\n";
}
