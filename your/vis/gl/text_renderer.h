#ifndef ONBOARD_VIS_GL_TEXT_RENDERER_H_
#define ONBOARD_VIS_GL_TEXT_RENDERER_H_

#include <string>

#include "onboard/math/eigen.h"
#include "onboard/math/vec.h"
#include "onboard/vis/common/color.h"

namespace qcraft {
namespace vis {
namespace text {

constexpr char kDefaultFontFace[] = "go_mono";
constexpr double kDefaultKerning = 0.25;

// Initialize GL.
void InitializeGl();

struct Options {
  std::string font_face = kDefaultFontFace;
  double kerning = kDefaultKerning;
  Color color = Color::kWhite;
  enum class Align { kLeft = 0, kCenter, kRight };
  Align align = Align::kLeft;
};

// Draws text at the given position with the given size and rotation. Default
// orientation is in the x-y plane going in the x+ direction.
void RenderTextInWorld(const std::string &str, const Vec3d &pos, double size,
                       const Quaternion &rotation = Quaternion::Identity());
void RenderTextInWorld(const std::string &str, const Vec3d &pos, double size,
                       const Options &options,
                       const Quaternion &rotation = Quaternion::Identity());

// Draws text in the screen coordinate frame (positions are in pixels). Negative
// coordinates are wrapped around. For example, coordinates (0, -20) are
// equivalent to (0, <viewport_height> - 20)).
void RenderTextOnScreen(const std::string &str, const Vec2d &pos, double size);
void RenderTextOnScreen(const std::string &str, const Vec2d &pos, double size,
                        const Options &options);

}  // namespace text
}  // namespace vis
}  // namespace qcraft

#endif  // ONBOARD_VIS_GL_TEXT_RENDERER_H_
