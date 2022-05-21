#ifndef ONBOARD_VIS_GL_GL_COMMON_H_
#define ONBOARD_VIS_GL_GL_COMMON_H_

// clang-format off
#include "glad/glad.h"  // NOLINT
// clang-format on

#include <GL/glu.h>  // NOLINT

#include <string>  // NOLINT

#include "glog/logging.h"

namespace qcraft {
namespace vis {

template <typename T>
void glVertexVec3d(const T &v) {
  glVertex3d(v.x(), v.y(), v.z());
}

template <typename T>
void glColor(const T &color) {
  glColor4d(color.r(), color.g(), color.b(), color.a());
}

template <typename T>
void glColorWithAlpha(const T &color, const double alpha) {
  glColor4d(color.r(), color.g(), color.b(), alpha);
}

template <typename T>
void glColorVec4d(const T &color) {
  glColor4d(color.x(), color.y(), color.z(), color.w());
}

template <typename T>
void glColorVec3d(const T &color) {
  glColor3d(color.x(), color.y(), color.z());
}

// Int32 bytes high to low: red, green, blue, alpha.
inline void glColorInt32(int color) {
  // This assumes the system is little-endian.
  const unsigned char *bytes = reinterpret_cast<unsigned char *>(&color);
  glColor4ub(bytes[3], bytes[2], bytes[1], bytes[0]);
}

inline void CheckGLError() {
  const GLenum error = glGetError();
  if (error == GL_NO_ERROR) return;
  LOG(FATAL) << "OpenGL error " << static_cast<int>(error) << ": "
             << gluErrorString(error);
}

}  // namespace vis
}  // namespace qcraft

#endif  // ONBOARD_VIS_GL_GL_COMMON_H_
