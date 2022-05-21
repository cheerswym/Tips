// Usage:
// class Base {
//  public:
//   explicit Base(std::ostream& os) : os_(os) {}
//   virtual void Print() = 0;
//  protected:
//   std::ostream& os_;
// };
//
// Then, derived subclasses should call the REGISTER_SUBCLASS macro after their
// class definition. For example:
//
// class Derived1 : public Base {
//  public:
//   explicit Derived1(std::ostream& os) : Base(os) {}
//   virtual void Print() { os_ << "Derived1" << std::endl; }
// };
// REGISTER_SUBCLASS(Base, Derived1, std::ostream&)
//
// Then, when an instance of one of the subclasses is desired, it can be
// constructed as:
//
// Registry<Base, std::ostream&>::Create("Derived1", std::cout);
//
// If instead, Derived1 should be constructed when the string "foo" is passed to
// Create(), then the REGISTER_SUBCLASS_NAME macro should be called. For
// example,
//
// REGISTER_SUBCLASS_NAME(Derived1, Base, foo, std::ostream&)

#ifndef ONBOARD_GLOBAL_REGISTRY_H_
#define ONBOARD_GLOBAL_REGISTRY_H_

#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>

#include "onboard/lite/check.h"

namespace qcraft {

#define PASTER(x, y, z) x##_##y##_##z
#define PASTER1(x, y, z) PASTER(x, y, z)
#define UNIQUE(X) PASTER1(X, __COUNTER__, __LINE__)

// Utility macros
#define REGISTRY_SPACE
#define REGISTRY_CONCAT(...) __VA_ARGS__
#define REGISTRY_GET_MACRO(_0, _1, _2, _3, _4, NAME, ...) NAME

#define REGISTRY_CTOR(Base, Derived, params, param_names) \
  [](params) -> Base* { return new Derived(param_names); }

#define REGISTER_SUBCLASS0(Base, Derived, Identifier)                \
  static bool UNIQUE(_registered_##Base) = Registry<Base>::Register( \
      Identifier, REGISTRY_CTOR(Base, Derived, , ));  // NOLINT

#define REGISTER_SUBCLASS1(Base, Derived, Identifier, dtype1)                \
  static bool UNIQUE(_registered_##Base) = Registry<Base, dtype1>::Register( \
      Identifier,                                                            \
      REGISTRY_CTOR(Base, Derived, dtype1 REGISTRY_SPACE arg1, arg1));

#define REGISTER_SUBCLASS2(Base, Derived, Identifier, dtype1, dtype2) \
  static bool UNIQUE(_registered_##Base) =                            \
      Registry<Base, dtype1, dtype2>::Register(                       \
          Identifier,                                                 \
          REGISTRY_CTOR(Base, Derived,                                \
                        REGISTRY_CONCAT(dtype1 REGISTRY_SPACE arg1,   \
                                        dtype2 REGISTRY_SPACE arg2),  \
                        REGISTRY_CONCAT(arg1, arg2)));

#define REGISTER_SUBCLASS3(Base, Derived, Identifier, dtype1, dtype2, dtype3) \
  static bool UNIQUE(_registered_##Base) =                                    \
      Registry<Base, dtype1, dtype2, dtype3>::Register(                       \
          Identifier,                                                         \
          REGISTRY_CTOR(Base, Derived,                                        \
                        REGISTRY_CONCAT(dtype1 REGISTRY_SPACE arg1,           \
                                        dtype2 REGISTRY_SPACE arg2,           \
                                        dtype3 REGISTRY_SPACE arg3),          \
                        REGISTRY_CONCAT(arg1, arg2, arg3)));

#define REGISTER_SUBCLASS4(Base, Derived, Identifier, dtype1, dtype2, dtype3, \
                           dtype4)                                            \
  static bool UNIQUE(_registered_##Base) =                                    \
      Registry<Base, dtype1, dtype2, dtype3, dtype4>::Register(               \
          Identifier,                                                         \
          REGISTRY_CTOR(Base, Derived,                                        \
                        REGISTRY_CONCAT(dtype1 REGISTRY_SPACE arg1,           \
                                        dtype2 REGISTRY_SPACE arg2,           \
                                        dtype3 REGISTRY_SPACE arg3,           \
                                        dtype4 REGISTRY_SPACE arg4),          \
                        REGISTRY_CONCAT(arg1, arg2, arg3, arg4)));

#define REGISTER_SUBCLASS_NAME(Base, Derived, Identifier, ...) \
  REGISTRY_GET_MACRO(_0, ##__VA_ARGS__, REGISTER_SUBCLASS4,    \
                     REGISTER_SUBCLASS3, REGISTER_SUBCLASS2,   \
                     REGISTER_SUBCLASS1, REGISTER_SUBCLASS0)   \
  (Base, Derived, #Identifier, ##__VA_ARGS__)

#define REGISTER_SUBCLASS(Base, Derived, ...) \
  REGISTER_SUBCLASS_NAME(Base, Derived, Derived, ##__VA_ARGS__)

template <typename T, typename... Pack>
class Registry {
 public:
  using ConstructorT = std::function<T*(Pack...)>;
  using MapCtorsT = std::unordered_map<std::string, ConstructorT>;

  template <typename... Args>
  static T* CreateOrDie(const std::string& class_name, Args&&... pack) {
    if (Constructors().find(class_name) != Constructors().end()) {
      return Constructors()[class_name](std::forward<Args>(pack)...);
    }
    QCHECK(false) << "Class " << class_name << " not registered.";
    return nullptr;
  }

  static bool Register(const std::string& class_name, ConstructorT ctor) {
    Constructors()[class_name] = std::move(ctor);
    return true;
  }

  static bool IsRegistered(const std::string& class_name) {
    return Constructors().find(class_name) != Constructors().end();
  }

  static void Unregister(const std::string& class_name) {
    Constructors().erase(class_name);
  }

 private:
  static MapCtorsT& Constructors() {
    static MapCtorsT* ctor_map = new MapCtorsT();
    return *ctor_map;
  }

  Registry();
  Registry(const Registry& other) = delete;
  Registry(const Registry&& other) = delete;
  Registry& operator=(const Registry& other) = delete;
};

};  // namespace qcraft

#endif  // ONBOARD_GLOBAL_REGISTRY_H_
