#include "onboard/math/frenet_frame.h"

#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "onboard/lite/check.h"

namespace qcraft {
namespace {

const std::vector<Vec2d> points1 = {{1.0, 0.0}, {2.0, 1.0}, {3.0, 2.0}};
const std::vector<Vec2d> points2 = {{1.0, 0.0}, {2.0, 1.0}, {3.0, 0.0}};
const std::vector<Vec2d> points3 = {{1.0, 0.0}, {1.01, 0.01}, {1.02, 0.02},
                                    {1.5, 0.5}, {1.51, 0.51}, {1.99, 0.99},
                                    {2.0, 1.0}};

TEST(FrenetFrameTest, BuildTest) { FrenetFrame frenet(points3); }

TEST(FrenetFrameTest, InterpolateTangentTest) {
  FrenetFrame frenet(points1);
  Vec2d tangent;
  const Vec2d xy1(0.0, 0.0);
  tangent = frenet.InterpolateTangentByXY(xy1);
  EXPECT_NEAR(tangent.x(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(tangent.y(), std::sqrt(2) / 2.0, 1e-5);

  const Vec2d xy2(1.0, 1.0);
  tangent = frenet.InterpolateTangentByXY(xy2);
  EXPECT_NEAR(tangent.x(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(tangent.y(), std::sqrt(2) / 2.0, 1e-5);

  const Vec2d xy3(3.0, 3.0);
  tangent = frenet.InterpolateTangentByXY(xy3);
  EXPECT_NEAR(tangent.x(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(tangent.y(), std::sqrt(2) / 2.0, 1e-5);
}

TEST(FrenetFrameTest, InterpolateTangentTest2) {
  FrenetFrame frenet(points2);
  Vec2d tangent;
  const Vec2d xy1(0.0, 0.0);
  tangent = frenet.InterpolateTangentByXY(xy1);
  EXPECT_NEAR(tangent.x(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(tangent.y(), std::sqrt(2) / 2.0, 1e-5);

  const Vec2d xy2(1.0, 1.0);
  tangent = frenet.InterpolateTangentByXY(xy2);
  EXPECT_NEAR(tangent.x(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(tangent.y(), std::sqrt(2) / 2.0, 1e-5);

  const Vec2d xy3(2.0, 2.0);
  tangent = frenet.InterpolateTangentByXY(xy3);
  EXPECT_NEAR(tangent.x(), 1.0, 1e-5);
  EXPECT_NEAR(tangent.y(), 0.0, 1e-5);

  const Vec2d xy4(3.0, 2.0);
  tangent = frenet.InterpolateTangentByXY(xy4);
  EXPECT_NEAR(tangent.x(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(tangent.y(), -std::sqrt(2) / 2.0, 1e-5);

  const Vec2d xy5(3.0, -1.0);
  tangent = frenet.InterpolateTangentByXY(xy5);
  EXPECT_NEAR(tangent.x(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(tangent.y(), -std::sqrt(2) / 2.0, 1e-5);
}

TEST(FrenetFrameTest, InterpolateTangentTest3) {
  FrenetFrame frenet(points2);
  Vec2d tangent;
  tangent = frenet.InterpolateTangentByS(0.0);
  EXPECT_NEAR(tangent.x(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(tangent.y(), std::sqrt(2) / 2.0, 1e-5);

  tangent = frenet.InterpolateTangentByS(std::sqrt(2) / 2);
  EXPECT_NEAR(tangent.x(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(tangent.y(), std::sqrt(2) / 2.0, 1e-5);

  tangent = frenet.InterpolateTangentByS(std::sqrt(2));
  EXPECT_NEAR(tangent.x(), 1.0, 1e-5);
  EXPECT_NEAR(tangent.y(), 0.0, 1e-5);

  tangent = frenet.InterpolateTangentByS(std::sqrt(2) * 3 / 2);
  EXPECT_NEAR(tangent.x(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(tangent.y(), -std::sqrt(2) / 2.0, 1e-5);

  tangent = frenet.InterpolateTangentByS(std::sqrt(2) * 2);
  EXPECT_NEAR(tangent.x(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(tangent.y(), -std::sqrt(2) / 2.0, 1e-5);
}

TEST(FrenetFrameTest, XYToSLTest) {
  FrenetFrame frenet(points1);
  Vec2d sl;
  const Vec2d xy1(0.0, 0.0);
  sl = frenet.XYToSL(xy1);
  EXPECT_NEAR(sl.x(), -std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(sl.y(), std::sqrt(2) / 2.0, 1e-5);

  const Vec2d xy2(1.0, 1.0);
  sl = frenet.XYToSL(xy2);
  EXPECT_NEAR(sl.x(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(sl.y(), std::sqrt(2) / 2.0, 1e-5);

  const Vec2d xy3(3.0, 3.0);
  sl = frenet.XYToSL(xy3);
  EXPECT_NEAR(sl.x(), 2.5 * std::sqrt(2), 1e-5);
  EXPECT_NEAR(sl.y(), std::sqrt(2) / 2.0, 1e-5);
}

TEST(FrenetFrameTest, XYToSLTest2) {
  FrenetFrame frenet(points2);
  Vec2d sl;
  const Vec2d xy1(0.0, 0.0);
  sl = frenet.XYToSL(xy1);
  EXPECT_NEAR(sl.x(), -std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(sl.y(), std::sqrt(2) / 2.0, 1e-5);

  const Vec2d xy2(1.0, 1.0);
  sl = frenet.XYToSL(xy2);
  EXPECT_NEAR(sl.x(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(sl.y(), std::sqrt(2) / 2.0, 1e-5);

  const Vec2d xy3(2.0, 2.0);
  sl = frenet.XYToSL(xy3);
  EXPECT_NEAR(sl.x(), std::sqrt(2), 1e-5);
  EXPECT_NEAR(sl.y(), 1.0, 1e-5);

  const Vec2d xy4(3.0, 2.0);
  sl = frenet.XYToSL(xy4);
  EXPECT_NEAR(sl.x(), std::sqrt(2), 1e-5);
  EXPECT_NEAR(sl.y(), std::sqrt(2), 1e-5);

  const Vec2d xy5(3.0, -1.0);
  sl = frenet.XYToSL(xy5);
  EXPECT_NEAR(sl.x(), 2.5 * std::sqrt(2), 1e-5);
  EXPECT_NEAR(sl.y(), -std::sqrt(2) / 2.0, 1e-5);
}

TEST(FrenetFrameTest, XYToSLTest3) {
  FrenetFrame frenet(points3);
  Vec2d sl;
  Vec2d normal;
  std::pair<int, int> index_pair;
  double alpha;
  const Vec2d xy(1.0, 0.5);
  frenet.XYToSL(xy, &sl, &normal, &index_pair, &alpha);
  EXPECT_NEAR(sl.x(), std::sqrt(2) / 4.0, 1e-5);
  EXPECT_NEAR(sl.y(), std::sqrt(2) / 4.0, 1e-5);
  EXPECT_NEAR(normal.x(), -std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal.y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_EQ(index_pair.first, 0);
  EXPECT_EQ(index_pair.second, 3);
}

TEST(FrenetFrameTest, XYToSLToXYTest) {
  const std::vector<Vec2d> points = {
      {-5.58306, -224.036}, {7.48225, -224.026}, {32.5322, -224.007},
      {57.5822, -223.988},  {82.6322, -223.969}, {94.4169, -223.96},
      {119.451, -223.984},  {144.484, -224.008}, {164.539, -224.027},
      {189.565, -224.009},  {194.417, -224.005}};
  FrenetFrame frenet(points);
  const Vec2d xy(83.5264, -230.783);
  const Vec2d sl = frenet.XYToSL(xy);
  const Vec2d rev_xy = frenet.SLToXY(sl);
  EXPECT_NEAR(xy.DistanceTo(rev_xy), 0.0, 1e-5);
}

TEST(FrenetFrameTest, XYToSLBunchTest1) {
  FrenetFrame frenet(points1);
  const std::vector<Vec2d> xy = {
      {0.0, 0.0}, {1.0, 1.0}, {3.0, 3.0}, {1.0, 1.0}};
  std::vector<Vec2d> sl(xy.size());
  std::vector<Vec2d> normal(xy.size());
  std::vector<std::pair<int, int>> index_pairs(xy.size());
  std::vector<double> alpha(xy.size());

  frenet.XYToSL(xy, absl::MakeSpan(sl), absl::MakeSpan(normal),
                absl::MakeSpan(index_pairs), absl::MakeSpan(alpha));

  EXPECT_NEAR(sl[0].x(), -std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(sl[0].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[0].x(), -std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[0].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_EQ(index_pairs[0].first, 0);
  EXPECT_EQ(index_pairs[0].second, 1);
  EXPECT_NEAR(alpha[0], -0.5, 1e-5);

  EXPECT_NEAR(sl[1].x(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(sl[1].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[1].x(), -std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[1].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_EQ(index_pairs[1].first, 0);
  EXPECT_EQ(index_pairs[1].second, 1);
  EXPECT_NEAR(alpha[1], 0.5, 1e-5);

  EXPECT_NEAR(sl[2].x(), 2.5 * std::sqrt(2), 1e-5);
  EXPECT_NEAR(sl[2].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[2].x(), -std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[2].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_EQ(index_pairs[2].first, 1);
  EXPECT_EQ(index_pairs[2].second, 2);
  EXPECT_NEAR(alpha[2], 1.5, 1e-5);

  EXPECT_NEAR(sl[3].x(), std::sqrt(2), 1e-5);
  EXPECT_NEAR(sl[3].y(), 1.0, 1e-5);
  EXPECT_NEAR(normal[3].x(), -std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[3].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_EQ(index_pairs[3].first, 1);
  EXPECT_EQ(index_pairs[3].second, 2);
  EXPECT_NEAR(alpha[3], -0.5, 1e-5);

  for (int i = 0; i < xy.size(); ++i) {
    VLOG(1) << "s[" << i << "] " << sl[i].x() << " l[" << i << "] "
            << sl[i].y();
    VLOG(1) << "normal[" << i << "] (" << normal[i].x() << " " << normal[i].y()
            << ")";
    VLOG(1) << "prev index[" << i << "] " << index_pairs[i].first;
    VLOG(1) << "succ index[" << i << "] " << index_pairs[i].second;
    VLOG(1) << "alpha[" << i << "] " << alpha[i];
  }
}

TEST(FrenetFrameTest, XYToSLBunchTest2) {
  FrenetFrame frenet(points2);
  const std::vector<Vec2d> xy = {{-1.0, -1.0}, {0.0, 0.0}, {1.0, 1.0},
                                 {2.0, 2.0},   {3.0, 1.0}, {4.0, 0.0},
                                 {5.0, -1.0}};
  std::vector<Vec2d> sl(xy.size());
  std::vector<Vec2d> normal(xy.size());
  std::vector<std::pair<int, int>> index_pairs(xy.size());
  std::vector<double> alpha(xy.size());

  frenet.XYToSL(xy, absl::MakeSpan(sl), absl::MakeSpan(normal),
                absl::MakeSpan(index_pairs), absl::MakeSpan(alpha));

  EXPECT_NEAR(sl[0].x(), -1.5 * std::sqrt(2), 1e-5);
  EXPECT_NEAR(sl[0].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[0].x(), -std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[0].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_EQ(index_pairs[0].first, 0);
  EXPECT_EQ(index_pairs[0].second, 1);
  EXPECT_NEAR(alpha[0], -1.5, 1e-5);

  EXPECT_NEAR(sl[1].x(), -std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(sl[1].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[1].x(), -std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[1].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_EQ(index_pairs[1].first, 0);
  EXPECT_EQ(index_pairs[1].second, 1);
  EXPECT_NEAR(alpha[1], -0.5, 1e-5);

  EXPECT_NEAR(sl[2].x(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(sl[2].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[2].x(), -std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[2].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_EQ(index_pairs[2].first, 0);
  EXPECT_EQ(index_pairs[2].second, 1);
  EXPECT_NEAR(alpha[2], 0.5, 1e-5);

  EXPECT_NEAR(sl[3].x(), std::sqrt(2), 1e-5);
  EXPECT_NEAR(sl[3].y(), 1.0, 1e-5);
  EXPECT_NEAR(normal[3].x(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[3].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_EQ(index_pairs[3].first, 1);
  EXPECT_EQ(index_pairs[3].second, 2);
  EXPECT_NEAR(alpha[3], -0.5, 1e-5);

  EXPECT_NEAR(sl[4].x(), 1.5 * std::sqrt(2), 1e-5);
  EXPECT_NEAR(sl[4].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[4].x(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[4].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_EQ(index_pairs[4].first, 1);
  EXPECT_EQ(index_pairs[4].second, 2);
  EXPECT_NEAR(alpha[4], 0.5, 1e-5);

  EXPECT_NEAR(sl[5].x(), 2.5 * std::sqrt(2), 1e-5);
  EXPECT_NEAR(sl[5].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[5].x(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[5].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_EQ(index_pairs[5].first, 1);
  EXPECT_EQ(index_pairs[5].second, 2);
  EXPECT_NEAR(alpha[5], 1.5, 1e-5);

  EXPECT_NEAR(sl[6].x(), 3.5 * std::sqrt(2), 1e-5);
  EXPECT_NEAR(sl[6].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[6].x(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_NEAR(normal[6].y(), std::sqrt(2) / 2.0, 1e-5);
  EXPECT_EQ(index_pairs[6].first, 1);
  EXPECT_EQ(index_pairs[6].second, 2);
  EXPECT_NEAR(alpha[6], 2.5, 1e-5);

  for (int i = 0; i < xy.size(); ++i) {
    VLOG(1) << "s[" << i << "] " << sl[i].x() << " l[" << i << "] "
            << sl[i].y();
    VLOG(1) << "normal[" << i << "] (" << normal[i].x() << " " << normal[i].y()
            << ")";
    VLOG(1) << "prev index[" << i << "] " << index_pairs[i].first;
    VLOG(1) << "succ index[" << i << "] " << index_pairs[i].second;
    VLOG(1) << "alpha[" << i << "] " << alpha[i];
  }
}

}  // namespace
}  // namespace qcraft
