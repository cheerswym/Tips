#include "onboard/perception/sensor_fov/line_iterator.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace qcraft::sensor_fov {

TEST(LineIteratorTest, BuildLine) {
  LineIterator lit0({-100, 0}, {100, 0});
  EXPECT_EQ(lit0.count(), 201);

  LineIterator lit1({-100, -100}, {100, 100}, 8);
  EXPECT_EQ(lit1.count(), 201);

  LineIterator lit2({-100, -100}, {100, 100}, 4);
  EXPECT_EQ(lit2.count(), 401);

  std::vector<Vec2i> gt{{5, 10}, {5, 9}, {6, 8}, {6, 7},
                        {7, 6},  {7, 5}, {8, 4}, {8, 3}};
  LineIterator lit3({5, 10}, {8, 3});
  for (int i = 0; i < lit3.count(); ++i, ++lit3) {
    EXPECT_EQ(lit3.pos(), gt[i]);
  }
}

}  // namespace qcraft::sensor_fov
