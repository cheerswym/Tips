#include "onboard/vis/canvas/canvas.h"

#include <sstream>

#include "gtest/gtest.h"

namespace qcraft {
namespace vis {

TEST(Canvas, BufferDraw) {
  Canvas canvas(Canvas::RenderDestination::kBuffer);
  canvas.DrawPoint({1.0, 2.0, 3.0}, Color::kRed, 5);
  ASSERT_EQ(canvas.buffer_.ops_size(), 1);
  EXPECT_TRUE(canvas.buffer_.ops(0).has_draw_call());
  EXPECT_TRUE(canvas.buffer_.ops(0).draw_call().has_point());
  EXPECT_EQ(canvas.buffer_.ops(0).draw_call().point().point_size(), 5);
  BoxBuilder(&canvas, {1.0, 2.0, 3.0}, 0.0, {1.0, 1.0}, Color::kRed)
      .SetFillColor(Color::kBlue)
      .Draw();
  ASSERT_EQ(canvas.buffer_.ops_size(), 2);
  EXPECT_TRUE(canvas.buffer_.ops(1).has_draw_call());
  EXPECT_TRUE(canvas.buffer_.ops(1).draw_call().has_box());
  EXPECT_TRUE(canvas.buffer_.ops(1).draw_call().box().has_border_style());
  EXPECT_TRUE(canvas.buffer_.ops(1).draw_call().box().has_fill_style());

  canvas.Translate(1.0, 2.0, 3.0);
  ASSERT_EQ(canvas.buffer_.ops_size(), 3);
  EXPECT_TRUE(canvas.buffer_.ops(2).has_affine_transformation());

  canvas.DrawCircle({1.0, 2.0, 3.0}, 2.0, Color::kGreen);
  ASSERT_EQ(canvas.buffer_.ops_size(), 4);
  EXPECT_TRUE(canvas.buffer_.ops(1).has_draw_call());
  EXPECT_TRUE(canvas.buffer_.ops(1).draw_call().has_box());
  EXPECT_FALSE(canvas.buffer_.ops(1).draw_call().circle().has_fill_style());
}

TEST(Canvas, Playback) {
  Canvas canvas0(Canvas::RenderDestination::kBuffer);
  canvas0.DrawPoint({1.0, 2.0, 3.0}, Color::kRed, 5);
  BoxBuilder(&canvas0, {1.0, 2.0, 3.0}, 0.0, {1.0, 1.0}, Color::kRed)
      .SetFillColor(Color::kBlue)
      .Draw();
  canvas0.Translate(1.0, 2.0, 3.0);
  canvas0.DrawCircle({1.0, 2.0, 3.0}, 2.0, Color::kGreen);

  Canvas canvas1(Canvas::RenderDestination::kBuffer);
  canvas1.PlaybackBuffer(canvas0.buffer_);

  EXPECT_EQ(canvas0.buffer_.DebugString(), canvas1.buffer_.DebugString());
}

TEST(Canvas, Serdes) {
  Canvas canvas0(Canvas::RenderDestination::kBuffer);
  canvas0.DrawPoint({1.0, 2.0, 3.0}, Color::kRed, 5);
  BoxBuilder(&canvas0, {1.0, 2.0, 3.0}, 0.0, {1.0, 1.0}, Color::kRed)
      .SetFillColor(Color::kBlue)
      .Draw();
  canvas0.Translate(1.0, 2.0, 3.0);
  canvas0.DrawCircle({1.0, 2.0, 3.0}, 2.0, Color::kGreen);

  std::stringstream s;
  canvas0.SerializeToStream(s);

  Canvas canvas1(Canvas::RenderDestination::kBuffer);
  canvas1.DeserializeFromStream(s);

  EXPECT_EQ(canvas0.buffer_.DebugString(), canvas1.buffer_.DebugString());
}

}  // namespace vis
}  // namespace qcraft
