#include "onboard/vis/gl/text_renderer.h"

#include <QtGui/QImage>
#include <map>

#include "glog/logging.h"
#include "onboard/math/util.h"
#include "onboard/vis/gl/gl_common.h"

namespace qcraft {
namespace vis {
namespace text {

namespace {

std::map<std::string, GLuint> font_texture_map_;

GLuint GetFontTextureOrDie(const std::string &font_face) {
  if (font_texture_map_.empty()) {
    LOG(FATAL)
        << "Font textures not loaded. Don't forget to call "
           "qcraft::vis::text::InitializeGL() before starting to render.";
  }

  const auto it = font_texture_map_.find(font_face);
  if (it == font_texture_map_.end()) {
    LOG(FATAL) << "Font " << font_face << " not found.";
  }

  const GLuint font_texture = it->second;
  if (font_texture == 0) {
    LOG(FATAL) << "Font texture for font " << font_face
               << " not correctly created.";
  }

  return font_texture;
}

}  // namespace

void InitializeGl() {
  for (const std::string &font_face : {"go_mono", "courier", "monospace"}) {
    const std::string filename = "onboard/vis/gl/data/" + font_face + "32.png";
    const QImage image(filename.c_str());
    CHECK_EQ(image.width(), 512);
    CHECK_EQ(image.height(), 512);
    CHECK(image.hasAlphaChannel());
    CHECK_EQ(image.format(), QImage::Format_ARGB32);

    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 512, 512, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, image.bits());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    font_texture_map_[font_face] = texture;
  }
}

void RenderTextInWorld(const std::string &str, const Vec3d &pos, double size,
                       const Quaternion &rotation) {
  RenderTextInWorld(str, pos, size, Options(), rotation);
}

void RenderTextInWorld(const std::string &str, const Vec3d &pos, double size,
                       const Options &options, const Quaternion &rotation) {
  const Vec3d x_plus = rotation * Vec3d::UnitX();
  const Vec3d y_plus = rotation * Vec3d::UnitY();

  const GLuint font_texture = GetFontTextureOrDie(options.font_face);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, font_texture);
  glColor(options.color);
  glBegin(GL_QUADS);
  constexpr double kTcSize = 1.0 / 16.0;
  const double kerning = options.kerning;
  const double dx = size * (1.0 - 2.0 * kerning);
  const double dy = size;
  const double len = str.size();
  const double offset =
      options.align == Options::Align::kLeft
          ? 0.0
          : (options.align == Options::Align::kCenter ? -len * 0.5 : -len);
  for (int i = 0; i < str.size(); ++i) {
    const int c = static_cast<int>(str[i]);
    const int row = c / 16;
    const int col = c % 16;

    glTexCoord2d((col + kerning) * kTcSize, 1.0 - row * kTcSize);
    glVertexVec3d(pos + (offset + i) * dx * x_plus);

    glTexCoord2d((col + 1 - kerning) * kTcSize, 1.0 - row * kTcSize);
    glVertexVec3d(pos + (offset + i + 1) * dx * x_plus);

    glTexCoord2d((col + 1 - kerning) * kTcSize, 1.0 - (row + 1) * kTcSize);
    glVertexVec3d(pos + (offset + i + 1) * dx * x_plus + dy * y_plus);

    glTexCoord2d((col + kerning) * kTcSize, 1.0 - (row + 1) * kTcSize);
    glVertexVec3d(pos + (offset + i) * dx * x_plus + dy * y_plus);
  }
  glEnd();
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_BLEND);
}

void RenderTextOnScreen(const std::string &str, const Vec2d &pos, double size) {
  RenderTextOnScreen(str, pos, size, Options());
}

void RenderTextOnScreen(const std::string &str, const Vec2d &pos, double size,
                        const Options &options) {
  glDisable(GL_DEPTH_TEST);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();

  int viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);  // x, y, w, h.
  double w = static_cast<double>(viewport[2]);
  double h = static_cast<double>(viewport[3]);
  glOrtho(0.0, w, 0.0, h, -1.0, 1.0);

  RenderTextInWorld(
      str, {WrapToRange(pos.x(), 0.0, w), WrapToRange(pos.y(), 0.0, h), 0.0},
      size, options);

  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glEnable(GL_DEPTH_TEST);
}

}  // namespace text
}  // namespace vis
}  // namespace qcraft
