#include "onboard/perception/tracker/association/association_util.h"

#include <iostream>

#include "absl/strings/str_format.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace qcraft::tracker {

TEST(AssociationUtil, TestIoU) {
  // Bev IoU
  Box2d box1({0., 0.}, 0., 5.0, 3.0);
  Box2d box2({0., 0.}, M_PI / 2, 5.0, 3.0);
  EXPECT_DOUBLE_EQ(association_util::IoU(box1, box2), 9.0 / 21.0);
  EXPECT_DOUBLE_EQ(association_util::IoU(Polygon2d(box1), Polygon2d(box2)),
                   9.0 / 21.0);
  // Image IoU
  // Has overlap
  BoundingBox2dProto img_box1;
  img_box1.set_x(2);
  img_box1.set_y(0);
  img_box1.set_width(4);
  img_box1.set_height(4);
  BoundingBox2dProto img_box2;
  img_box2.set_x(0);
  img_box2.set_y(2);
  img_box2.set_width(4);
  img_box2.set_height(4);
  EXPECT_DOUBLE_EQ(association_util::IoU(img_box1, img_box2), 4.0 / 28.0);
  // No overlap
  BoundingBox2dProto img_box3;
  img_box3.set_x(0);
  img_box3.set_y(0);
  img_box3.set_width(4);
  img_box3.set_height(4);
  BoundingBox2dProto img_box4;
  img_box4.set_x(10);
  img_box4.set_y(10);
  img_box4.set_width(4);
  img_box4.set_height(4);
  EXPECT_DOUBLE_EQ(association_util::IoU(img_box3, img_box4), 0.0);
}

}  // namespace qcraft::tracker

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
