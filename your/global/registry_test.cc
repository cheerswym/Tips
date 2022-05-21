#include "onboard/global/registry.h"

#include <string>
#include <thread>
#include <utility>

#include "absl/strings/str_cat.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace qcraft {
namespace {

class Base1 {
 public:
  virtual ~Base1() {}

  explicit Base1(const std::string& name) : name_(name) {}

  virtual std::string GetFullName() {
    return absl::StrCat("Base1", "::", name_);
  }

 protected:
  std::string name_;
};

class Derived1 : public Base1 {
 public:
  explicit Derived1(const std::string& name) : Base1(name) {}
  std::string GetFullName() override {
    return absl::StrCat("Derived1", "::", name_);
  }
};

REGISTER_SUBCLASS(Base1, Derived1, const std::string&);

class Derived2 : public Base1 {
 public:
  explicit Derived2(const std::string& name) : Base1(name) {}
  std::string GetFullName() override {
    return absl::StrCat("Derived2", "::", name_);
  }
};
REGISTER_SUBCLASS_NAME(Base1, Derived2, DiffName, const std::string&);

TEST(RegistryTest, RegisterAndCreate) {
  std::unique_ptr<Base1> base;
  base.reset(
      Registry<Base1, const std::string&>::CreateOrDie("Derived1", "Test1"));
  EXPECT_EQ("Derived1::Test1", base->GetFullName());

  base.reset(
      Registry<Base1, const std::string&>::CreateOrDie("DiffName", "Test2"));
  EXPECT_EQ("Derived2::Test2", base->GetFullName());
}

}  // namespace
}  // namespace qcraft
