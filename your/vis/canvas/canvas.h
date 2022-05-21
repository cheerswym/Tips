#ifndef ONBOARD_VIS_CANVAS_CANVAS_H_
#define ONBOARD_VIS_CANVAS_CANVAS_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "onboard/math/geometry/affine_transformation.h"
#include "onboard/math/geometry/util.h"
#include "onboard/vis/canvas/proto/canvas_buffer.pb.h"
#include "onboard/vis/common/color.h"

namespace qcraft {
namespace vis {

// Canvas is an interface for rendering basic primitives. Compared to using
// OpenGL directly, it has the following advantages:
// - It supports both direct rendering (forwarding draw calls to OpenGL) and
//   serialization (saving all the draw calls and data for later).
// - It uses a uniform interface for both modes, allowing flexible switches.
// - It makes common draw calls convenient to use (drawing a circle is a single
//   call, unlike using OpenGL directly).
// However using it for the direct rendering only has the following drawbacks:
// - It does not support many OpenGL operations such as shading, texturing
//   or FBO.
// - It is very inefficient with many DrawXXX calls, as it cannot exploit the
//   batch processing support from OpenGL (each DrawXXX call must be a separate
//   pair of glBegin() and glEnd() calls).
//
// Canvas is thread safe.
//
// TODO(Chris): Add a deferred rendering mode (serializing, deserializing and
// then immediately into direct rendering) to enabling batching.
class Canvas {
 public:
  enum class RenderDestination { kOpenGL, kBuffer };

  explicit Canvas(RenderDestination dest = RenderDestination::kOpenGL);
  explicit Canvas(int64_t flag);

  // Returns the render destination of this canvas.
  RenderDestination render_destination() const { return render_destination_; }

  // Load a canvas buffer and playback its content. If this canvas is to
  // kOpenGL, the content of the canvas buffer will be rendered to OpenGL. If
  // this canvas is to kBuffer, the content of the canvas buffer will be
  // effectively transferred to this canvas.
  // The istream version simply reads the buffer from the input stream according
  // to a predefined encoding. Prefer using the stream version to the protobuf
  // version to allow this class to handle the encoding.
  void PlaybackBuffer(const CanvasBufferProto &buffer);
  bool DeserializeFromStream(std::istream &is);

  // For kBuffer destination only.
  // Similar to above, prefer using the stream version to allow this class to
  // handle the encoding.
  void SwapBuffer(CanvasBufferProto *buffer);
  bool SerializeToStream(std::ostream &os) const;

  void Clear() {
    absl::WriterMutexLock l(&buffer_mutex_);
    buffer_.Clear();
  }

  void SetGroundZero(int64_t flag);

  // Draw calls. Each primitive type has a basic call that accepts the full spec
  // in the form of a proto, and a few convenient overloads.
  void DrawPoint(const PointProto &proto);
  void DrawPoint(Vec3d pos, const Color &color);
  void DrawPoint(Vec3d pos, const Color &color, int point_size);

  void DrawPoints(const std::vector<Vec2d> &points, double z,
                  const Color &color, int point_size);
  void DrawPoints(const PointsProto &proto);
  void DrawPoints(const std::vector<Vec3d> &points, const Color &color);
  void DrawPoints(const std::vector<Vec3d> &points, const Color &color,
                  int point_size);
  void DrawPoints(const std::vector<Vec3d> &points,
                  const std::vector<Color> &colors);
  void DrawPoints(const std::vector<Vec3d> &points,
                  const std::vector<Color> &colors, int point_size);
  void DrawPoints(const std::vector<Vec3d> &points, const Color &color,
                  bool translucent, bool depth_sort);
  void DrawPoints(const std::vector<Vec3d> &points, const Color &color,
                  int point_size, bool translucent, bool depth_sort);
  void DrawPoints(const std::vector<Vec3d> &points,
                  const std::vector<Color> &colors, bool translucent,
                  bool depth_sort);
  void DrawPoints(const std::vector<Vec3d> &points,
                  const std::vector<Color> &colors, int point_size,
                  bool translucent, bool depth_sort);

  void DrawLine(const LineProto &proto);
  void DrawLine(Vec3d start, Vec3d end, const Color &color);
  void DrawLine(Vec3d start, Vec3d end, const Color &color, int line_width);
  void DrawLine(Vec3d start, Vec3d end, const Color &color, int line_width,
                BorderStyleProto::LineStyle line_style);

  void DrawLineStrip(const LineStripProto &proto);
  void DrawLineStrip(const std::vector<Vec3d> &vertices, const Color &color);
  void DrawLineStrip(const std::vector<Vec3d> &vertices, const Color &color,
                     int line_width);
  void DrawLineStrip(const std::vector<Vec3d> &vertices, const Color &color,
                     int line_width, BorderStyleProto::LineStyle line_style);

  void DrawCircle(const CircleProto &proto);
  void DrawCircle(Vec3d center, double radius, const Color &border_color);
  void DrawCircle(Vec3d center, double radius, const Color &border_color,
                  const Color &fill_color);
  void DrawCircle(Vec3d center, double radius, const Color &border_color,
                  int border_line_width);
  void DrawCircle(Vec3d center, double radius, const Color &border_color,
                  const Color &fill_color, int border_line_width);
  void DrawCircle(Vec3d center, double radius, const Color &border_color,
                  int border_line_width,
                  BorderStyleProto::LineStyle border_line_style);
  void DrawCircle(Vec3d center, double radius, const Color &border_color,
                  const Color &fill_color, int border_line_width,
                  BorderStyleProto::LineStyle border_line_style);

  void DrawBox(const BoxProto &proto);
  void DrawBox(Vec3d center, double heading, Vec2d size,
               const Color &border_color);
  void DrawBox(Vec3d center, double heading, Vec2d size,
               const Color &border_color, const Color &fill_color);
  void DrawBox(Vec3d center, double heading, Vec2d size,
               const Color &border_color, int border_line_width);
  void DrawBox(Vec3d center, double heading, Vec2d size,
               const Color &border_color, const Color &fill_color,
               int border_line_width);
  void DrawBox(Vec3d center, double heading, Vec2d size,
               const Color &border_color, int border_line_width,
               BorderStyleProto::LineStyle border_line_style);
  void DrawBox(Vec3d center, double heading, Vec2d size,
               const Color &border_color, const Color &fill_color,
               int border_line_width,
               BorderStyleProto::LineStyle border_line_style);

  void DrawPolygon(const PolygonProto &proto);
  void DrawPolygon(const std::vector<Vec3d> &vertices,
                   const Color &border_color);
  void DrawPolygon(const std::vector<Vec3d> &vertices,
                   const Color &border_color, const Color &fill_color);
  void DrawPolygon(const std::vector<Vec3d> &vertices,
                   const Color &border_color, int border_line_width);
  void DrawPolygon(const std::vector<Vec3d> &vertices,
                   const Color &border_color, const Color &fill_color,
                   int border_line_width);
  void DrawPolygon(const std::vector<Vec3d> &vertices,
                   const Color &border_color, int border_line_width,
                   BorderStyleProto::LineStyle border_line_style);
  void DrawPolygon(const std::vector<Vec3d> &vertices,
                   const Color &border_color, const Color &fill_color,
                   int border_line_width,
                   BorderStyleProto::LineStyle border_line_style);
  void DrawPolygon(const std::vector<Vec2d> &vertices, double z,
                   const Color &border_color);
  void DrawPolygon(const Polygon2d &polygon, double z,
                   const Color &border_color);

  void DrawText(const TextProto &proto);
  void DrawText(const std::string &text, Vec3d center, double heading,
                double size, const Color &color,
                TextProto::Align align = TextProto::LEFT);

  void DrawImage(ImageProto proto);
  // Pixels: row major, bottom to top, three bytes per pixel (r, g, b). Does not
  // take ownership of pixels.
  void DrawImage(const unsigned char *pixels, int width, int height,
                 Vec3d center, double heading, double size);

  // Transformation operations. In the OpenGL context, these transformations
  // only apply to the MODELVIEW mode because Canvas does not handle cameras.
  void PushTransformation();
  void PopTransformation();

  void Translate(double x, double y, double z);
  void Translate(Vec3d t);
  void Rotate(double angle, double x, double y, double z);
  void Rotate(double angle, Vec3d axis);
  void Scale(double sx, double sy, double sz);
  void Scale(Vec3d s);

  void SetTransformation(const AffineTransformation &t);
  void ApplyTransformation(const AffineTransformation &t);

  const AffineTransformation &GetCurrentTransformation() const {
    absl::WriterMutexLock l(&t_stack_mutex_);
    return t_stack_.back();
  }

 protected:
  bool buffered() const {
    return render_destination_ == RenderDestination::kBuffer;
  }

 private:
  const RenderDestination render_destination_;
  CanvasBufferProto buffer_ GUARDED_BY(buffer_mutex_);
  mutable absl::Mutex buffer_mutex_;
  // Transformation stack. Never empty; back() is the current active
  // transformation.
  std::vector<AffineTransformation> t_stack_ GUARDED_BY(t_stack_mutex_);
  mutable absl::Mutex t_stack_mutex_;
};

class BoxBuilder {
 public:
  BoxBuilder(Canvas *const canvas, const Vec3d &center, const double heading,
             const Vec2d &size, const Color &border_color);

  BoxBuilder &SetBorderLineWidth(int border_line_width);
  BoxBuilder &SetBorderColor(const Color &border_color);
  BoxBuilder &SetBorderLineStyle(
      const BorderStyleProto::LineStyle &border_line_style);
  BoxBuilder &SetFillColor(const Color &fill_color);
  BoxBuilder &SetRenderHeading(bool render_heading);
  void Draw();

 private:
  Canvas *const canvas_;
  const Vec3d center_;
  const double heading_;
  const Vec2d size_;

  int border_line_width_;
  Color border_color_;
  BorderStyleProto::LineStyle border_line_style_;
  std::optional<Color> fill_color_;
  bool render_heading_ = false;
};

}  // namespace vis
}  // namespace qcraft

#endif  // ONBOARD_VIS_CANVAS_CANVAS_H_
