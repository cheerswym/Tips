#include "onboard/vis/common/colormap.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace vis {

TEST(ColormapTest, DISABLED_Default) {
  Colormap map;
  EXPECT_EQ(map.Map(0.0).ToInt32(), 0xffffff);
  EXPECT_EQ(map.Map(1.0).ToInt32(), 0xffffff);
  EXPECT_EQ(map.Map(0.0).rgba(), Colormap::MapWithDefaultScheme(0.0).rgba());
  EXPECT_EQ(map.Map(1.0).rgba(), Colormap::MapWithDefaultScheme(1.0).rgba());
}

TEST(ColormapTest, Scaled) {
  Colormap map(Colormap::kDefaultScheme, -1.0, 2.0);
  EXPECT_EQ(map.Map(-1.0).rgba(), Colormap::MapWithDefaultScheme(0.0).rgba());
  EXPECT_EQ(map.Map(0.5).rgba(), Colormap::MapWithDefaultScheme(0.5).rgba());
  EXPECT_EQ(map.Map(2.0).rgba(), Colormap::MapWithDefaultScheme(1.0).rgba());
}

TEST(ColormapTest, DISABLED_Magma) {
  Colormap map("magma");
  EXPECT_EQ(map.Map(0.0).ToInt32(), 0xffffff);
  EXPECT_EQ(map.Map(0.5).ToInt32(), 0xffffff);
  EXPECT_EQ(map.Map(1.0).ToInt32(), 0xffffff);
}

TEST(ColormapTest, CustomData) {
  std::vector<Color> data;
  data.push_back(Color(0x0000ff));
  data.push_back(Color(0x00ffff));
  data.push_back(Color(0xffffff));
  Colormap map(data);
  EXPECT_EQ(map.Map(0.0).ToInt32(), 0x0000ff);
  EXPECT_EQ(map.Map(0.5).ToInt32(), 0x00ffff);
  EXPECT_EQ(map.Map(1.0).ToInt32(), 0xffffff);
}

}  // namespace vis
}  // namespace qcraft
