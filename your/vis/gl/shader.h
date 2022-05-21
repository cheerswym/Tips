#ifndef ONBOARD_VIS_GL_SHADER_H_
#define ONBOARD_VIS_GL_SHADER_H_

#include <iostream>
#include <string>
#include <unordered_map>

#include "onboard/math/eigen.h"
#include "onboard/math/vec.h"
#include "onboard/vis/gl/gl_common.h"

namespace qcraft {
namespace vis {

class Shader {
 public:
  Shader(const char *vert_shader_src, const char *frag_shader_src);
  Shader(const std::string &vert_shader_src,
         const std::string &frag_shader_src);
  ~Shader();

  // Program name, as in glUseProgram().
  GLuint program() const { return program_; }

  // Sets the uniforms by name.
  void SetUniformInt(const std::string &name, int value) const;
  void SetUniformFloat(const std::string &name, float value) const;
  void SetUniformVec4d(const std::string &name, const Vec4d &value) const;
  void SetUniformVec3d(const std::string &name, const Vec3d &value) const;
  void SetUniformMat4d(const std::string &name, const Mat4d &value) const;
  void SetUniformMat3d(const std::string &name, const Mat3d &value) const;

 protected:
  // Read the uniform location from the cache if known, or perform the lookup
  // and add to cache if not known.
  GLint GetUniformLocation(const std::string &name) const;

 private:
  GLuint program_ = 0;
  mutable std::unordered_map<std::string, GLint> uniform_locations_;
};

}  // namespace vis
}  // namespace qcraft

#endif  // ONBOARD_VIS_GL_SHADER_H_
