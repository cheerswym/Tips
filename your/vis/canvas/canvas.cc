#include "onboard/vis/canvas/canvas.h"

#include <algorithm>
#include <numeric>
#include <utility>

#include "glog/logging.h"
#include "onboard/vis/gl/gl_common.h"
#include "onboard/vis/gl/text_renderer.h"

namespace qcraft {
namespace vis {
namespace {

constexpr int kDefaultPointSize = 1;
constexpr int kDefaultLineWidth = 1;
constexpr BorderStyleProto::LineStyle kDefaultLineStyle =
    BorderStyleProto::SOLID;

void SetGLLineStyle(BorderStyleProto::LineStyle line_style) {
  switch (line_style) {
    case BorderStyleProto::SOLID:
    default:
      glLineStipple(1, 0xffff);  // 1111111111111111
      break;
    case BorderStyleProto::DOTTED:
      glLineStipple(1, 0x8888);  // 1000100010001000
      break;
    case BorderStyleProto::DASHED:
      glLineStipple(1, 0xfff8);  // 1111111111111000
      break;
    case BorderStyleProto::DOT_AND_DASH:
      glLineStipple(1, 0xff88);  // 1111111110001000
      break;
  }
  glEnable(GL_LINE_STIPPLE);
}

void ClearGLLineStyle() {
  glLineStipple(1, 0xffff);
  glDisable(GL_LINE_STIPPLE);
}

void PopulateBorderStyleProto(const Color &color, int line_width,
                              BorderStyleProto::LineStyle line_style,
                              BorderStyleProto *proto) {
  color.ToProto(proto->mutable_color());
  proto->set_line_width(line_width);
  proto->set_line_style(line_style);
}

void PopulateBorderStyleProto(const Color &color, int line_width,
                              BorderStyleProto *proto) {
  PopulateBorderStyleProto(color, line_width, kDefaultLineStyle, proto);
}

void PopulateBorderStyleProto(const Color &color, BorderStyleProto *proto) {
  PopulateBorderStyleProto(color, kDefaultLineWidth, proto);
}

void PopulateFillStyleProto(const Color &color, FillStyleProto *proto) {
  color.ToProto(proto->mutable_color());
}

}  // namespace

Canvas::Canvas(RenderDestination dest) : render_destination_(dest) {
  absl::MutexLock l(&t_stack_mutex_);
  t_stack_.emplace_back();
}

Canvas::Canvas(int64_t flag) : render_destination_(RenderDestination::kBuffer) {
  buffer_.set_flag(flag);
  absl::MutexLock l(&t_stack_mutex_);
  t_stack_.emplace_back();
}

void Canvas::SetGroundZero(int64_t flag) {
  absl::MutexLock l(&buffer_mutex_);
  buffer_.set_flag(flag);
}

void Canvas::PlaybackBuffer(const CanvasBufferProto &buffer) {
  for (const auto &op : buffer.ops()) {
    if (op.has_draw_call()) {
      const auto &draw_call = op.draw_call();
      if (draw_call.has_point()) {
        DrawPoint(draw_call.point());
      } else if (draw_call.has_points()) {
        DrawPoints(draw_call.points());
      } else if (draw_call.has_line()) {
        DrawLine(draw_call.line());
      } else if (draw_call.has_line_strip()) {
        DrawLineStrip(draw_call.line_strip());
      } else if (draw_call.has_circle()) {
        DrawCircle(draw_call.circle());
      } else if (draw_call.has_box()) {
        DrawBox(draw_call.box());
      } else if (draw_call.has_polygon()) {
        DrawPolygon(draw_call.polygon());
      } else if (draw_call.has_text()) {
        DrawText(draw_call.text());
      } else if (draw_call.has_image()) {
        DrawImage(draw_call.image());
      }
    } else if (op.has_affine_transformation()) {
      const auto &transformation = op.affine_transformation();
      const AffineTransformation t(transformation);
      SetTransformation(t);
    }
  }
}

bool Canvas::DeserializeFromStream(std::istream &is) {
  CanvasBufferProto buffer;
  buffer.ParseFromIstream(&is);
  if (is.bad()) return false;

  PlaybackBuffer(buffer);
  return true;
}

void Canvas::SwapBuffer(CanvasBufferProto *buffer) {
  CHECK(buffer != nullptr);
  absl::MutexLock l(&buffer_mutex_);
  *buffer = std::move(buffer_);
}

bool Canvas::SerializeToStream(std::ostream &os) const {
  absl::MutexLock l(&buffer_mutex_);
  return buffer_.SerializeToOstream(&os);
}

void Canvas::DrawPoint(const PointProto &proto) {
  // Lock not only for buffer_ write but for OpenGL draw calls as well.
  absl::MutexLock l(&buffer_mutex_);
  if (buffered()) {
    *buffer_.add_ops()->mutable_draw_call()->mutable_point() = proto;
  } else {
    if (proto.has_border_style()) {
      glColorInt32(proto.border_style().color().rgba());
      if (proto.point_size() != 0) glPointSize(proto.point_size());
      glBegin(GL_POINTS);
      glVertexVec3d(proto.pos());
      glEnd();
    }
  }
}

void Canvas::DrawPoints(const PointsProto &proto) {
  // Lock not only for buffer_ write but for OpenGL draw calls as well.
  absl::MutexLock l(&buffer_mutex_);
  if (buffered()) {
    *buffer_.add_ops()->mutable_draw_call()->mutable_points() = proto;
  } else {
    CHECK_EQ(proto.xs_size(), proto.ys_size());
    CHECK_EQ(proto.xs_size(), proto.zs_size());

    // Single color if there is only one element in the colors repeated field.
    if (proto.colors_size() == 1) {
      glColorInt32(proto.colors(0));
    } else {
      CHECK_EQ(proto.xs_size(), proto.colors_size());
    }

    // Enable blending if rendering translucent.
    std::vector<int> pindex(proto.xs_size());
    std::iota(pindex.begin(), pindex.end(), 0);
    if (proto.translucent()) {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

      // Enable sorting by depth if needed.
      if (proto.depth_sort()) {
        Mat4d modelview;
        glGetDoublev(GL_MODELVIEW_MATRIX, modelview.data());
        AffineTransformation t(modelview);

        std::vector<double> pdepth(proto.xs_size());
        for (int i = 0; i < proto.xs_size(); ++i) {
          const Vec3d pworld(proto.xs(i), proto.ys(i), proto.zs(i));
          const Vec3d pcamera = t.TransformPoint(pworld);
          pdepth[i] = pcamera.z();
        }

        std::sort(pindex.begin(), pindex.end(), [&pdepth](int i0, int i1) {
          // We want the farthest object first.
          return pdepth[i0] < pdepth[i1];
        });
      }
    }

    // Render all the points.
    if (proto.point_size() != 0) glPointSize(proto.point_size());
    // TODO(Fang): consider using vertex buffer.
    glBegin(GL_POINTS);
    for (int i = 0; i < proto.xs_size(); ++i) {
      if (proto.colors_size() > 1) {
        glColorInt32(proto.colors(i));
      }
      glVertex3f(proto.xs(pindex[i]), proto.ys(pindex[i]), proto.zs(pindex[i]));
    }
    glEnd();

    if (proto.translucent()) {
      glDisable(GL_BLEND);
    }
  }
}

void Canvas::DrawLine(const LineProto &proto) {
  // Lock not only for buffer_ write but for OpenGL draw calls as well.
  absl::MutexLock l(&buffer_mutex_);
  if (buffered()) {
    *buffer_.add_ops()->mutable_draw_call()->mutable_line() = proto;
  } else {
    if (proto.has_border_style()) {
      glColorInt32(proto.border_style().color().rgba());
      glLineWidth(proto.border_style().line_width());
      SetGLLineStyle(proto.border_style().line_style());
      glBegin(GL_LINES);
      glVertexVec3d(proto.start());
      glVertexVec3d(proto.end());
      glEnd();
      ClearGLLineStyle();
    }
  }
}

void Canvas::DrawLineStrip(const LineStripProto &proto) {
  // Lock not only for buffer_ write but for OpenGL draw calls as well.
  absl::MutexLock l(&buffer_mutex_);
  if (buffered()) {
    *buffer_.add_ops()->mutable_draw_call()->mutable_line_strip() = proto;
  } else {
    if (proto.has_border_style()) {
      glColorInt32(proto.border_style().color().rgba());
      glLineWidth(proto.border_style().line_width());
      SetGLLineStyle(proto.border_style().line_style());
      glBegin(GL_LINE_STRIP);
      for (const Vec3dProto &v : proto.vertices()) {
        glVertexVec3d(v);
      }
      glEnd();
      ClearGLLineStyle();
    }
  }
}

void Canvas::DrawCircle(const CircleProto &proto) {
  // Lock not only for buffer_ write but for OpenGL draw calls as well.
  absl::MutexLock l(&buffer_mutex_);
  constexpr int kN = 60;
  if (buffered()) {
    *buffer_.add_ops()->mutable_draw_call()->mutable_circle() = proto;
  } else {
    const Vec3d center = Vec3dFromProto(proto.center());
    const double radius = proto.radius();
    if (proto.has_border_style()) {
      glColorInt32(proto.border_style().color().rgba());
      glLineWidth(proto.border_style().line_width());
      SetGLLineStyle(proto.border_style().line_style());
      glBegin(GL_LINE_LOOP);
      for (int i = 0; i < kN; ++i) {
        const double theta = i * M_PI * 2 / kN;
        glVertexVec3d(center +
                      radius * Vec3d(std::cos(theta), std::sin(theta), 0.0));
      }
      glEnd();
      ClearGLLineStyle();
    }
    if (proto.has_fill_style()) {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glColorInt32(proto.fill_style().color().rgba());
      glBegin(GL_TRIANGLE_FAN);
      glVertexVec3d(center);
      for (int i = 0; i <= kN; ++i) {
        const double theta = i * M_PI * 2 / kN;
        glVertexVec3d(center +
                      radius * Vec3d(std::cos(theta), std::sin(theta), 0.0));
      }
      glEnd();
      glDisable(GL_BLEND);
    }
  }
}

void Canvas::DrawBox(const BoxProto &proto) {
  // Lock not only for buffer_ write but for OpenGL draw calls as well.
  absl::MutexLock l(&buffer_mutex_);
  if (buffered()) {
    *buffer_.add_ops()->mutable_draw_call()->mutable_box() = proto;
  } else {
    const Vec3d center = Vec3dFromProto(proto.center());
    const double heading = proto.heading();
    const Vec3d tangent(std::cos(heading), std::sin(heading), 0.0);
    const Vec3d normal(-tangent.y(), tangent.x(), 0.0);
    const Vec2d size = Vec2dFromProto(proto.size());
    const double half_length = size.x() * 0.5;
    const double half_width = size.y() * 0.5;

    if (proto.has_border_style()) {
      glColorInt32(proto.border_style().color().rgba());
      glLineWidth(proto.border_style().line_width());
      SetGLLineStyle(proto.border_style().line_style());
      glBegin(GL_LINE_LOOP);
      glVertexVec3d(center + half_length * tangent + half_width * normal);
      glVertexVec3d(center - half_length * tangent + half_width * normal);
      glVertexVec3d(center - half_length * tangent - half_width * normal);
      glVertexVec3d(center + half_length * tangent - half_width * normal);
      glEnd();
      ClearGLLineStyle();
    }
    if (proto.has_fill_style()) {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glColorInt32(proto.fill_style().color().rgba());
      glBegin(GL_QUADS);
      glVertexVec3d(center + half_length * tangent + half_width * normal);
      glVertexVec3d(center - half_length * tangent + half_width * normal);
      glVertexVec3d(center - half_length * tangent - half_width * normal);
      glVertexVec3d(center + half_length * tangent - half_width * normal);
      glEnd();
      glDisable(GL_BLEND);
    }
    if (proto.render_heading()) {
      glBegin(GL_LINES);
      const Vec3d heading_vec =
          Vec3d(Vec2d::FastUnitFromAngle(heading) * 0.5, center.z());
      const Vec3d first_point =
          center + half_length * tangent + half_width * normal;
      const Vec3d last_point =
          center + half_length * tangent - half_width * normal;
      const Vec3d mid_point = (first_point + last_point) * 0.5;
      glVertexVec3d(mid_point);
      glVertexVec3d(mid_point + heading_vec);
      glEnd();
    }
  }
}

void Canvas::DrawPolygon(const PolygonProto &proto) {
  // Lock not only for buffer_ write but for OpenGL draw calls as well.
  absl::MutexLock l(&buffer_mutex_);
  if (buffered()) {
    *buffer_.add_ops()->mutable_draw_call()->mutable_polygon() = proto;
  } else {
    if (proto.has_border_style()) {
      glColorInt32(proto.border_style().color().rgba());
      glLineWidth(proto.border_style().line_width());
      SetGLLineStyle(proto.border_style().line_style());
      glBegin(GL_LINE_LOOP);
      for (const Vec3dProto &v : proto.vertices()) {
        glVertexVec3d(v);
      }
      glEnd();
      ClearGLLineStyle();
    }
    if (proto.has_fill_style()) {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glColorInt32(proto.fill_style().color().rgba());
      glBegin(GL_TRIANGLE_FAN);
      for (const Vec3dProto &v : proto.vertices()) {
        glVertexVec3d(v);
      }
      glEnd();
      glDisable(GL_BLEND);
    }
  }
}

void Canvas::DrawText(const TextProto &proto) {
  // Lock not only for buffer_ write but for OpenGL draw calls as well.
  absl::MutexLock l(&buffer_mutex_);
  if (buffered()) {
    *buffer_.add_ops()->mutable_draw_call()->mutable_text() = proto;
  } else {
    text::Options options;
    if (proto.has_fill_style()) {
      options.color.Set(proto.fill_style().color());
    }
    switch (proto.align()) {
      default:
      case TextProto::LEFT:
        options.align = text::Options::Align::kLeft;
        break;
      case TextProto::CENTER:
        options.align = text::Options::Align::kCenter;
        break;
      case TextProto::RIGHT:
        options.align = text::Options::Align::kRight;
        break;
    }
    text::RenderTextInWorld(
        proto.text(), Vec3dFromProto(proto.center()), proto.size(), options,
        Quaternion(AngleAxis(proto.heading(), Vec3d::UnitZ())));
  }
}

void Canvas::DrawImage(ImageProto proto) {
  // Lock not only for buffer_ write but for OpenGL draw calls as well.
  absl::MutexLock l(&buffer_mutex_);
  if (buffered()) {
    buffer_.add_ops()->mutable_draw_call()->mutable_image()->Swap(&proto);
  } else {
    // TODO(Fang) consider rendering as a single textured quad. But who's going
    // to own the texture and manage its memory?
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslated(proto.center().x(), proto.center().y(), proto.center().z());
    glRotated(r2d(proto.heading()), 0.0, 0.0, 1.0);
    glScaled(proto.size(), proto.size(), 1.0);
    glTranslated(-proto.width() * 0.5, -proto.height() * 0.5, 0.0);
    glBegin(GL_QUADS);
    for (int i = 0; i < proto.height(); ++i) {
      for (int j = 0; j < proto.width(); ++j) {
        glColor3ub(proto.pixels()[(i * proto.width() + j) * 3 + 0],
                   proto.pixels()[(i * proto.width() + j) * 3 + 1],
                   proto.pixels()[(i * proto.width() + j) * 3 + 2]);
        glVertex3i(j, i, 0);
        glVertex3i(j + 1, i, 0);
        glVertex3i(j + 1, i + 1, 0);
        glVertex3i(j, i + 1, 0);
      }
    }
    glEnd();
    glPopMatrix();
  }
}

void Canvas::DrawPoint(Vec3d pos, const Color &color) {
  DrawPoint(pos, color, kDefaultPointSize);
}

void Canvas::DrawPoint(Vec3d pos, const Color &color, int point_size) {
  PointProto proto;
  PopulateBorderStyleProto(color, proto.mutable_border_style());
  Vec3dToProto(pos, proto.mutable_pos());
  proto.set_point_size(point_size);
  DrawPoint(proto);
}

void Canvas::DrawPoints(const std::vector<Vec3d> &points, const Color &color) {
  DrawPoints(points, color, false, false);
}

void Canvas::DrawPoints(const std::vector<Vec3d> &points, const Color &color,
                        int point_size) {
  DrawPoints(points, color, point_size, false, false);
}

void Canvas::DrawPoints(const std::vector<Vec2d> &points, double z,
                        const Color &color, int point_size) {
  std::vector<Vec3d> points_3d;
  points_3d.reserve(points.size());
  for (const auto &pt : points) {
    points_3d.push_back(Vec3d(pt, z));
  }
  DrawPoints(points_3d, color, point_size, false, false);
}

void Canvas::DrawPoints(const std::vector<Vec3d> &points,
                        const std::vector<Color> &colors) {
  DrawPoints(points, colors, false, false);
}

void Canvas::DrawPoints(const std::vector<Vec3d> &points,
                        const std::vector<Color> &colors, int point_size) {
  DrawPoints(points, colors, point_size, false, false);
}

void Canvas::DrawPoints(const std::vector<Vec3d> &points, const Color &color,
                        bool translucent, bool depth_sort) {
  DrawPoints(points, color, kDefaultPointSize, translucent, depth_sort);
}

void Canvas::DrawPoints(const std::vector<Vec3d> &points, const Color &color,
                        int point_size, bool translucent, bool depth_sort) {
  PointsProto proto;
  proto.add_colors(color.ToInt32());
  proto.mutable_xs()->Reserve(points.size());
  proto.mutable_ys()->Reserve(points.size());
  proto.mutable_zs()->Reserve(points.size());
  for (int i = 0; i < points.size(); ++i) {
    proto.add_xs(points[i].x());
    proto.add_ys(points[i].y());
    proto.add_zs(points[i].z());
  }
  proto.set_point_size(point_size);
  proto.set_translucent(translucent);
  proto.set_depth_sort(depth_sort);
  DrawPoints(proto);
}

void Canvas::DrawPoints(const std::vector<Vec3d> &points,
                        const std::vector<Color> &colors, bool translucent,
                        bool depth_sort) {
  DrawPoints(points, colors, kDefaultPointSize, translucent, depth_sort);
}

void Canvas::DrawPoints(const std::vector<Vec3d> &points,
                        const std::vector<Color> &colors, int point_size,
                        bool translucent, bool depth_sort) {
  CHECK_EQ(points.size(), colors.size());
  PointsProto proto;
  proto.mutable_colors()->Reserve(points.size());
  proto.mutable_xs()->Reserve(points.size());
  proto.mutable_ys()->Reserve(points.size());
  proto.mutable_zs()->Reserve(points.size());
  for (int i = 0; i < points.size(); ++i) {
    proto.add_colors(colors[i].ToInt32());
    proto.add_xs(points[i].x());
    proto.add_ys(points[i].y());
    proto.add_zs(points[i].z());
  }
  proto.set_point_size(point_size);
  proto.set_translucent(translucent);
  proto.set_depth_sort(depth_sort);
  DrawPoints(proto);
}

void Canvas::DrawLine(Vec3d start, Vec3d end, const Color &color) {
  DrawLine(start, end, color, kDefaultLineWidth);
}

void Canvas::DrawLine(Vec3d start, Vec3d end, const Color &color,
                      int line_width) {
  DrawLine(start, end, color, line_width, kDefaultLineStyle);
}

void Canvas::DrawLine(Vec3d start, Vec3d end, const Color &color,
                      int line_width, BorderStyleProto::LineStyle line_style) {
  LineProto proto;
  PopulateBorderStyleProto(color, line_width, line_style,
                           proto.mutable_border_style());
  Vec3dToProto(start, proto.mutable_start());
  Vec3dToProto(end, proto.mutable_end());
  DrawLine(proto);
}

void Canvas::DrawLineStrip(const std::vector<Vec3d> &vertices,
                           const Color &color) {
  DrawLineStrip(vertices, color, kDefaultLineWidth);
}

void Canvas::DrawLineStrip(const std::vector<Vec3d> &vertices,
                           const Color &color, int line_width) {
  DrawLineStrip(vertices, color, line_width, kDefaultLineStyle);
}

void Canvas::DrawLineStrip(const std::vector<Vec3d> &vertices,
                           const Color &color, int line_width,
                           BorderStyleProto::LineStyle line_style) {
  LineStripProto proto;
  PopulateBorderStyleProto(color, line_width, line_style,
                           proto.mutable_border_style());
  for (const Vec3d &v : vertices) {
    Vec3dToProto(v, proto.add_vertices());
  }
  DrawLineStrip(proto);
}

void Canvas::DrawCircle(Vec3d center, double radius,
                        const Color &border_color) {
  DrawCircle(center, radius, border_color, kDefaultLineWidth);
}

void Canvas::DrawCircle(Vec3d center, double radius, const Color &border_color,
                        const Color &fill_color) {
  DrawCircle(center, radius, border_color, fill_color, kDefaultLineWidth);
}

void Canvas::DrawCircle(Vec3d center, double radius, const Color &border_color,
                        int border_line_width) {
  DrawCircle(center, radius, border_color, border_line_width,
             kDefaultLineStyle);
}

void Canvas::DrawCircle(Vec3d center, double radius, const Color &border_color,
                        const Color &fill_color, int border_line_width) {
  DrawCircle(center, radius, border_color, fill_color, border_line_width,
             kDefaultLineStyle);
}

void Canvas::DrawCircle(Vec3d center, double radius, const Color &border_color,
                        int border_line_width,
                        BorderStyleProto::LineStyle border_line_style) {
  CircleProto proto;
  PopulateBorderStyleProto(border_color, border_line_width, border_line_style,
                           proto.mutable_border_style());
  // No fill.
  Vec3dToProto(center, proto.mutable_center());
  proto.set_radius(radius);
  DrawCircle(proto);
}

void Canvas::DrawCircle(Vec3d center, double radius, const Color &border_color,
                        const Color &fill_color, int border_line_width,
                        BorderStyleProto::LineStyle border_line_style) {
  CircleProto proto;
  PopulateBorderStyleProto(border_color, border_line_width, border_line_style,
                           proto.mutable_border_style());
  PopulateFillStyleProto(fill_color, proto.mutable_fill_style());
  Vec3dToProto(center, proto.mutable_center());
  proto.set_radius(radius);
  DrawCircle(proto);
}

void Canvas::DrawBox(Vec3d center, double heading, Vec2d size,
                     const Color &border_color) {
  DrawBox(center, heading, size, border_color, kDefaultLineWidth);
}

void Canvas::DrawBox(Vec3d center, double heading, Vec2d size,
                     const Color &border_color, const Color &fill_color) {
  DrawBox(center, heading, size, border_color, fill_color, kDefaultLineWidth);
}

void Canvas::DrawBox(Vec3d center, double heading, Vec2d size,
                     const Color &border_color, int border_line_width) {
  DrawBox(center, heading, size, border_color, border_line_width,
          kDefaultLineStyle);
}

void Canvas::DrawBox(Vec3d center, double heading, Vec2d size,
                     const Color &border_color, const Color &fill_color,
                     int border_line_width) {
  DrawBox(center, heading, size, border_color, fill_color, border_line_width,
          kDefaultLineStyle);
}

void Canvas::DrawBox(Vec3d center, double heading, Vec2d size,
                     const Color &border_color, int border_line_width,
                     BorderStyleProto::LineStyle border_line_style) {
  BoxProto proto;
  PopulateBorderStyleProto(border_color, border_line_width, border_line_style,
                           proto.mutable_border_style());
  // No fill.
  Vec3dToProto(center, proto.mutable_center());
  proto.set_heading(heading);
  Vec2dToProto(size, proto.mutable_size());
  DrawBox(proto);
}

void Canvas::DrawBox(Vec3d center, double heading, Vec2d size,
                     const Color &border_color, const Color &fill_color,
                     int border_line_width,
                     BorderStyleProto::LineStyle border_line_style) {
  BoxProto proto;
  PopulateBorderStyleProto(border_color, border_line_width, border_line_style,
                           proto.mutable_border_style());
  PopulateFillStyleProto(fill_color, proto.mutable_fill_style());
  Vec3dToProto(center, proto.mutable_center());
  proto.set_heading(heading);
  Vec2dToProto(size, proto.mutable_size());
  DrawBox(proto);
}

void Canvas::DrawPolygon(const std::vector<Vec3d> &vertices,
                         const Color &border_color) {
  DrawPolygon(vertices, border_color, kDefaultLineWidth);
}

void Canvas::DrawPolygon(const std::vector<Vec3d> &vertices,
                         const Color &border_color, const Color &fill_color) {
  DrawPolygon(vertices, border_color, fill_color, kDefaultLineWidth);
}

void Canvas::DrawPolygon(const std::vector<Vec3d> &vertices,
                         const Color &border_color, int border_line_width) {
  DrawPolygon(vertices, border_color, border_line_width, kDefaultLineStyle);
}

void Canvas::DrawPolygon(const std::vector<Vec3d> &vertices,
                         const Color &border_color, const Color &fill_color,
                         int border_line_width) {
  DrawPolygon(vertices, border_color, fill_color, border_line_width,
              kDefaultLineStyle);
}

void Canvas::DrawPolygon(const std::vector<Vec3d> &vertices,
                         const Color &border_color, int border_line_width,
                         BorderStyleProto::LineStyle border_line_style) {
  PolygonProto proto;
  PopulateBorderStyleProto(border_color, border_line_width, border_line_style,
                           proto.mutable_border_style());
  // No fill.
  for (const Vec3d &v : vertices) {
    Vec3dToProto(v, proto.add_vertices());
  }
  DrawPolygon(proto);
}

void Canvas::DrawPolygon(const std::vector<Vec3d> &vertices,
                         const Color &border_color, const Color &fill_color,
                         int border_line_width,
                         BorderStyleProto::LineStyle border_line_style) {
  PolygonProto proto;
  PopulateBorderStyleProto(border_color, border_line_width, border_line_style,
                           proto.mutable_border_style());
  PopulateFillStyleProto(fill_color, proto.mutable_fill_style());
  for (const Vec3d &v : vertices) {
    Vec3dToProto(v, proto.add_vertices());
  }
  DrawPolygon(proto);
}

void Canvas::DrawPolygon(const std::vector<Vec2d> &vertices, double z,
                         const Color &border_color) {
  PolygonProto proto;
  PopulateBorderStyleProto(border_color, kDefaultLineWidth, kDefaultLineStyle,
                           proto.mutable_border_style());
  // No fill.
  for (const Vec2d &v : vertices) {
    Vec3dToProto(Vec3d(v, z), proto.add_vertices());
  }
  DrawPolygon(proto);
}

void Canvas::DrawPolygon(const Polygon2d &polygon, double z,
                         const Color &border_color) {
  DrawPolygon(polygon.points(), z, border_color);
}

void Canvas::DrawText(const std::string &text, Vec3d center, double heading,
                      double size, const Color &color, TextProto::Align align) {
  TextProto proto;
  PopulateFillStyleProto(color, proto.mutable_fill_style());
  proto.set_text(text);
  Vec3dToProto(center, proto.mutable_center());
  proto.set_heading(heading);
  proto.set_size(size);
  proto.set_align(align);
  DrawText(proto);
}

void Canvas::DrawImage(const unsigned char *pixels, int width, int height,
                       Vec3d center, double heading, double size) {
  ImageProto proto;
  proto.set_pixels(pixels, width * height * 3);
  proto.set_width(width);
  proto.set_height(height);
  Vec3dToProto(center, proto.mutable_center());
  proto.set_heading(heading);
  proto.set_size(size);
  DrawImage(std::move(proto));
}

void Canvas::PushTransformation() {
  if (buffered()) {
    absl::MutexLock l(&t_stack_mutex_);
    t_stack_.push_back(t_stack_.back());
  } else {
    glPushMatrix();
  }
}

void Canvas::PopTransformation() {
  if (buffered()) {
    absl::MutexLock l(&t_stack_mutex_);
    if (t_stack_.size() > 1) {
      absl::MutexLock l(&buffer_mutex_);
      t_stack_.pop_back();
      auto *transformation = buffer_.add_ops()->mutable_affine_transformation();
      t_stack_.back().ToProto(transformation);
    }
  } else {
    glPopMatrix();
  }
}

void Canvas::Translate(double x, double y, double z) {
  if (buffered()) {
    absl::MutexLock l(&t_stack_mutex_);
    absl::MutexLock l2(&buffer_mutex_);
    t_stack_.back().ApplyTranslation(x, y, z);
    auto *transformation = buffer_.add_ops()->mutable_affine_transformation();
    t_stack_.back().ToProto(transformation);
  } else {
    glTranslated(x, y, z);
  }
}

void Canvas::Translate(Vec3d t) {
  if (buffered()) {
    absl::MutexLock l(&t_stack_mutex_);
    absl::MutexLock l2(&buffer_mutex_);
    t_stack_.back().ApplyTranslation(t);
    auto *transformation = buffer_.add_ops()->mutable_affine_transformation();
    t_stack_.back().ToProto(transformation);
  } else {
    glTranslated(t.x(), t.y(), t.z());
  }
}

void Canvas::Rotate(double angle, double x, double y, double z) {
  if (buffered()) {
    absl::MutexLock l(&t_stack_mutex_);
    absl::MutexLock l2(&buffer_mutex_);
    t_stack_.back().ApplyRotation(angle, x, y, z);
    auto *transformation = buffer_.add_ops()->mutable_affine_transformation();
    t_stack_.back().ToProto(transformation);
  } else {
    glRotated(angle, x, y, z);
  }
}

void Canvas::Rotate(double angle, Vec3d axis) {
  if (buffered()) {
    absl::MutexLock l(&t_stack_mutex_);
    absl::MutexLock l2(&buffer_mutex_);
    t_stack_.back().ApplyRotation(angle, axis);
    auto *transformation = buffer_.add_ops()->mutable_affine_transformation();
    t_stack_.back().ToProto(transformation);
  } else {
    glRotated(angle, axis.x(), axis.y(), axis.z());
  }
}

void Canvas::Scale(double sx, double sy, double sz) {
  if (buffered()) {
    absl::MutexLock l(&t_stack_mutex_);
    absl::MutexLock l2(&buffer_mutex_);
    t_stack_.back().ApplyScaling(sx, sy, sz);
    auto *transformation = buffer_.add_ops()->mutable_affine_transformation();
    t_stack_.back().ToProto(transformation);
  } else {
    glScaled(sx, sy, sz);
  }
}

void Canvas::Scale(Vec3d s) {
  if (buffered()) {
    absl::MutexLock l(&t_stack_mutex_);
    absl::MutexLock l2(&buffer_mutex_);
    t_stack_.back().ApplyScaling(s);
    auto *transformation = buffer_.add_ops()->mutable_affine_transformation();
    t_stack_.back().ToProto(transformation);
  } else {
    glScaled(s.x(), s.y(), s.z());
  }
}

void Canvas::SetTransformation(const AffineTransformation &t) {
  if (buffered()) {
    absl::MutexLock l(&t_stack_mutex_);
    absl::MutexLock l2(&buffer_mutex_);
    t_stack_.back().Set(t);
    auto *transformation = buffer_.add_ops()->mutable_affine_transformation();
    t_stack_.back().ToProto(transformation);
  } else {
    glLoadMatrixd(t.mat().data());
  }
}

void Canvas::ApplyTransformation(const AffineTransformation &t) {
  if (buffered()) {
    absl::MutexLock l(&t_stack_mutex_);
    absl::MutexLock l2(&buffer_mutex_);
    t_stack_.back().Apply(t);
    auto *transformation = buffer_.add_ops()->mutable_affine_transformation();
    t_stack_.back().ToProto(transformation);
  } else {
    glMultMatrixd(t.mat().data());
  }
}

BoxBuilder::BoxBuilder(Canvas *const canvas, const Vec3d &center,
                       const double heading, const Vec2d &size,
                       const Color &border_color)
    : canvas_(canvas),
      center_(center),
      heading_(heading),
      size_(size),
      border_line_width_(kDefaultLineWidth),
      border_color_(border_color),
      border_line_style_(kDefaultLineStyle) {}

BoxBuilder &BoxBuilder::SetBorderLineWidth(int border_line_width) {
  border_line_width_ = border_line_width;
  return *this;
}

BoxBuilder &BoxBuilder::SetBorderColor(const Color &border_color) {
  border_color_ = border_color;
  return *this;
}

BoxBuilder &BoxBuilder::SetFillColor(const Color &fill_color) {
  fill_color_ = fill_color;
  return *this;
}

BoxBuilder &BoxBuilder::SetBorderLineStyle(
    const BorderStyleProto::LineStyle &border_line_style) {
  border_line_style_ = border_line_style;
  return *this;
}

BoxBuilder &BoxBuilder::SetRenderHeading(bool render_heading) {
  render_heading_ = render_heading;
  return *this;
}

void BoxBuilder::Draw() {
  BoxProto proto;
  proto.set_heading(heading_);
  Vec2dToProto(size_, proto.mutable_size());

  PopulateBorderStyleProto(border_color_, border_line_width_,
                           border_line_style_, proto.mutable_border_style());
  if (fill_color_) {
    PopulateFillStyleProto(fill_color_.value(), proto.mutable_fill_style());
  }
  proto.set_render_heading(render_heading_);
  Vec3dToProto(center_, proto.mutable_center());

  canvas_->DrawBox(proto);
}

}  // namespace vis
}  // namespace qcraft
