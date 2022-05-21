#include "onboard/utils/elements_history.h"

#include <algorithm>
#include <utility>

#include "gtest/gtest.h"

namespace qcraft {
namespace elements_history {

TEST(ElementsHistoryTest, Test_ElementHistory_Push) {
  ElementHistory<double, int, CircularSpan<Node<double, int>>> element_history(
      3);
  EXPECT_EQ(element_history.empty(), true);
  EXPECT_EQ(element_history.Push<int>(1.0, 1), true);
  EXPECT_EQ(element_history.Push<int>(1.0, 1), false);
}

TEST(ElementsHistoryTest, Test_ElementHistory_Push_overwrite) {
  ElementHistory<double, std::string, CircularSpan<Node<double, std::string>>>
      element_history(3);
  element_history.Push<std::string>(1.0, "1");
  element_history.Push<std::string>(2.0, "2");
  element_history.Push<std::string>(3.0, "3");
  element_history.Push<std::string>(4.0, "4");

  auto actual_res = element_history.GetHistory();

  EXPECT_EQ(actual_res.ok(), true);
  EXPECT_EQ(actual_res->size(), 3);
}

TEST(ElementsHistoryTest, Test_ElementHistory_GetHistory_FromStartTime) {
  ElementHistory<double, std::string, CircularSpan<Node<double, std::string>>>
      element_history(3);
  element_history.Push<std::string>(1.0, "1");
  element_history.Push<std::string>(2.0, "2");
  element_history.Push<std::string>(3.0, "3");

  auto actual_res = element_history.GetHistoryFrom(0.5);
  EXPECT_EQ(actual_res.ok(), true);
  EXPECT_EQ((*actual_res)[0].val, "1");
  EXPECT_EQ(actual_res->size(), 3);

  actual_res = element_history.GetHistoryFrom(1.0);
  EXPECT_EQ(actual_res.ok(), true);
  EXPECT_EQ((*actual_res)[0].val, "1");
  EXPECT_EQ(actual_res->size(), 3);

  actual_res = element_history.GetHistoryFrom(1.5);
  EXPECT_EQ(actual_res.ok(), true);
  EXPECT_EQ((*actual_res)[0].val, "1");
  EXPECT_EQ(actual_res->size(), 3);

  actual_res = element_history.GetHistoryFrom(2.5);
  EXPECT_EQ(actual_res.ok(), true);
  EXPECT_EQ((*actual_res)[0].val, "2");
  EXPECT_EQ(actual_res->size(), 2);
}

TEST(ElementsHistoryTest, Test_ElementHistory_GetHistory_All) {
  ElementHistory<double, std::string, CircularSpan<Node<double, std::string>>>
      element_history(3);
  element_history.Push<std::string>(1.0, "1");
  element_history.Push<std::string>(2.0, "2");

  auto actual_res = element_history.GetHistory();

  EXPECT_EQ(actual_res.ok(), true);
  EXPECT_EQ(actual_res->size(), 2);
  EXPECT_EQ((*actual_res)[0].time, 1.0);
  EXPECT_EQ((*actual_res)[0].val, "1");
}

TEST(ElementsHistoryTest, Test_ElementHistory_PopBegin) {
  ElementHistory<double, std::string, CircularSpan<Node<double, std::string>>>
      element_history(3);
  element_history.Push<std::string>(1.0, "1");
  element_history.Push<std::string>(2.0, "2");

  element_history.PopBegin(1.0);

  auto actual_res = element_history.GetHistory();
  EXPECT_EQ(actual_res.ok(), true);
  EXPECT_EQ(actual_res->size(), 1);
}

TEST(ElementsHistoryTest, Test_ElementsHistory_PopBegin) {
  ElementsHistory<int, double, std::string,
                  ElementHistory<double, std::string,
                                 CircularSpan<Node<double, std::string>>>>
      elements_history(3);
  elements_history[1].Push<std::string>(1.0, "1");
  elements_history[1].Push<std::string>(2.0, "2");

  elements_history.PopBegin(1.0);

  auto actual_res = elements_history[1].GetHistory();
  EXPECT_EQ(actual_res.ok(), true);
  EXPECT_EQ(actual_res->size(), 1);
}

TEST(ElementsHistoryTest, Test_ElementsHistory_Contains) {
  ElementsHistory<int, double, std::string,
                  ElementHistory<double, std::string,
                                 CircularSpan<Node<double, std::string>>>>
      elements_history(3);
  elements_history[1].Push<std::string>(1.0, "1");

  EXPECT_EQ(elements_history.Contains(1), true);
  EXPECT_EQ(elements_history.Contains(2), false);
}

TEST(ElementsHistoryTest, Test_CircularSpan_ForLoop) {
  ElementHistory<double, std::string, CircularSpan<Node<double, std::string>>>
      element_history(3);
  element_history.Push<std::string>(1.0, "1");
  element_history.Push<std::string>(2.0, "2");

  auto actual_res = element_history.GetHistory();
  EXPECT_EQ(actual_res.ok(), true);
  EXPECT_NO_THROW(for (const auto& node
                       : (*actual_res)) {
    (void)node.val;
    (void)node.time;
  });
}

TEST(ElementsHistoryTest, Test_ElementHistory_iterator) {
  ElementHistory<int64_t, int64_t, CircularSpan<Node<int64_t, int64_t>>>
      element_history(10);
  std::vector<int64_t> input_vec = {0, 1, 2, 3, 4};
  for (auto e : input_vec) {
    element_history.Push(e, 2 * e);
  }
  auto elem_cap = element_history.capacity();
  EXPECT_EQ(elem_cap, 10);
  auto elem_size = element_history.size();
  EXPECT_EQ(elem_size, input_vec.size());
  std::vector<int64_t> out_vec(element_history.size());
  std::transform(element_history.begin(), element_history.end(),
                 out_vec.begin(), [](const auto& node) { return node.time; });
  EXPECT_EQ(input_vec, out_vec);
}

}  // namespace elements_history
}  // namespace qcraft
